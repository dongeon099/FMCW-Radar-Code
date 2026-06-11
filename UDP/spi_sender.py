# -*- coding: utf-8 -*-
"""SPI sender/receiver for Raspberry Pi <-> STM32.

Raspberry Pi setup notes:
    sudo raspi-config
    Interface Options -> SPI -> Enable
    sudo reboot
    ls /dev/spidev*
    pip3 install spidev

If permission is denied, add the user to the spi group when available, or test with:
    sudo python3 main.py

The module keeps mock mode so development PCs can run the full pipeline without SPI
hardware. Real SPI uses spidev xfer2(), so every transfer sends the Raspberry Pi
ACC recommendation packet and reads the STM32 ego-speed/steering-angle packet.
"""

from __future__ import annotations

import logging
import math
import os
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from acc_controller import ACCRecommendation, AdaptiveCruiseController
from config import (
    EGO_SPEED_DEFAULT_MPS,
    SPI_BITS_PER_WORD,
    SPI_BUS,
    SPI_DEVICE,
    SPI_EGO_SPEED_SCALE_MPS,
    SPI_MAX_OBJECTS,
    SPI_MOCK_EGO_SPEED_MPS,
    SPI_MOCK_STEERING_ANGLE_DEG,
    SPI_MOCK_TURN_SIGNAL,
    SPI_MODE,
    SPI_PACKET_PERIOD_SEC,
    SPI_SAFE_FALLBACK_TO_MOCK,
    SPI_SPEED_HZ,
    SPI_STEERING_ANGLE_SCALE_DEG,
    SPI_USE_MOCK,
    TURN_SIGNAL_HAZARD,
    TURN_SIGNAL_INVALID,
    TURN_SIGNAL_LEFT,
    TURN_SIGNAL_NONE,
    TURN_SIGNAL_RIGHT,
    USE_MOCK_SPI,
)


logger = logging.getLogger(__name__)


HEADER1 = 0xAA
HEADER2 = 0x55
VERSION = 0x01

LANE_ID_NONE = 0
LANE_ID_LEFT = 1
LANE_ID_CENTER = 2
LANE_ID_RIGHT = 3

RISK_NONE = 0
RISK_CAUTION = 1
RISK_WARNING = 2
RISK_DANGER = 3

OBJECT_FIELD_BYTES = 8
RADAR_PACKET_BASE_BYTES = 8
RADAR_PACKET_LEN = RADAR_PACKET_BASE_BYTES + SPI_MAX_OBJECTS * OBJECT_FIELD_BYTES + 1
STM32_SPI_PACKET_LEN = 33
STM32_TURN_SIGNAL_INDEX = 7
STM32_CHECKSUM_INDEX = STM32_SPI_PACKET_LEN - 1


def _byte(value: int) -> int:
    """Clamp any integer-like value to one byte."""
    return max(0, min(255, int(value))) & 0xFF


def _as_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _to_uint16_bytes(value: int) -> Tuple[int, int]:
    value = max(0, min(0xFFFF, int(value)))
    return (value >> 8) & 0xFF, value & 0xFF


def _to_int16_bytes(value: int) -> Tuple[int, int]:
    value = max(-32768, min(32767, int(value)))
    if value < 0:
        value = (1 << 16) + value
    return (value >> 8) & 0xFF, value & 0xFF


def compute_checksum(packet_without_checksum: Sequence[int]) -> int:
    """Simple STM32-friendly checksum: sum(all bytes before checksum) & 0xFF."""
    return sum(_byte(value) for value in packet_without_checksum) & 0xFF


def _turn_signal_to_stm32_byte(turn_signal: int) -> int:
    if turn_signal == TURN_SIGNAL_RIGHT:
        return 1
    if turn_signal == TURN_SIGNAL_LEFT:
        return 2
    return 0


def build_miso_packet(
    ego_speed_mps: float,
    steering_angle_deg: float,
    turn_signal: int = TURN_SIGNAL_NONE,
) -> List[int]:
    """Build the 33-byte STM32 MISO packet, mainly for mock/tests."""
    speed_raw = round(_as_float(ego_speed_mps) / SPI_EGO_SPEED_SCALE_MPS)
    angle_raw = round(_as_float(steering_angle_deg) / SPI_STEERING_ANGLE_SCALE_DEG)
    speed_hi, speed_lo = _to_int16_bytes(speed_raw)
    angle_hi, angle_lo = _to_int16_bytes(angle_raw)

    packet = [0] * STM32_SPI_PACKET_LEN
    packet[0:7] = [
        HEADER1,
        HEADER2,
        VERSION,
        speed_lo,
        speed_hi,
        angle_lo,
        angle_hi,
    ]
    packet[STM32_TURN_SIGNAL_INDEX] = _turn_signal_to_stm32_byte(turn_signal)
    packet[STM32_CHECKSUM_INDEX] = compute_checksum(packet[:STM32_CHECKSUM_INDEX])
    return packet


def parse_miso_packet(
    rx_bytes: Optional[Sequence[int]],
) -> Optional[Tuple[float, float, int]]:
    """Parse and validate the 33-byte STM32 vehicle-state packet."""
    if rx_bytes is None or len(rx_bytes) < STM32_SPI_PACKET_LEN:
        return None

    packet = [_byte(value) for value in rx_bytes[:STM32_SPI_PACKET_LEN]]
    if packet[:3] != [HEADER1, HEADER2, VERSION]:
        return None
    if compute_checksum(packet[:STM32_CHECKSUM_INDEX]) != packet[STM32_CHECKSUM_INDEX]:
        return None

    speed_raw = packet[3] | (packet[4] << 8)
    if speed_raw & 0x8000:
        speed_raw -= 1 << 16
    steering_raw = packet[5] | (packet[6] << 8)
    if steering_raw & 0x8000:
        steering_raw -= 1 << 16

    return (
        speed_raw * SPI_EGO_SPEED_SCALE_MPS,
        steering_raw * SPI_STEERING_ANGLE_SCALE_DEG,
        read_turn_signal_from_rx(packet),
    )


def read_turn_signal_from_rx(rx_bytes: Optional[Sequence[int]]) -> int:
    """Extract STM32 blink status byte from SPI RX bytes.

    main (1).c sends:
        0x00: no turn signal
        0x01: right turn signal on
        0x02: left turn signal on
    """
    if not rx_bytes or len(rx_bytes) <= STM32_TURN_SIGNAL_INDEX:
        return TURN_SIGNAL_INVALID

    blink_state = int(rx_bytes[STM32_TURN_SIGNAL_INDEX]) & 0xFF
    if blink_state == 0:
        return TURN_SIGNAL_NONE
    if blink_state == 1:
        return TURN_SIGNAL_RIGHT
    if blink_state == 2:
        return TURN_SIGNAL_LEFT
    return TURN_SIGNAL_INVALID


def turn_signal_name(turn_signal: int) -> str:
    names = {
        TURN_SIGNAL_NONE: "NONE",
        TURN_SIGNAL_LEFT: "LEFT",
        TURN_SIGNAL_RIGHT: "RIGHT",
        TURN_SIGNAL_HAZARD: "HAZARD",
        TURN_SIGNAL_INVALID: "INVALID",
    }
    return names.get(int(turn_signal), f"UNKNOWN({turn_signal})")


def _front_distance_m(obj: Dict) -> float:
    y = _as_float(obj.get("y"))
    if y > 0.0:
        return y
    distance = _as_float(obj.get("distance"))
    if distance > 0.0:
        return distance
    x = _as_float(obj.get("x"))
    return math.hypot(x, y)


def _closing_speed_mps(obj: Dict) -> float:
    vy = _as_float(obj.get("vy"))
    if vy < 0.0:
        return abs(vy)
    radial = _as_float(obj.get("radial_velocity", obj.get("v")))
    if radial < 0.0:
        return abs(radial)
    return 0.0


def _ttc_seconds(obj: Dict) -> Optional[float]:
    distance = _front_distance_m(obj)
    closing_speed = _closing_speed_mps(obj)
    if distance <= 0.0 or closing_speed <= 0.0:
        return None
    return distance / closing_speed


def _priority_key(obj: Dict) -> Tuple[int, float, float, float]:
    """Higher-risk objects come first, then lower TTC and shorter distance."""
    risk = int(obj.get("risk_level", RISK_NONE))
    ttc = obj.get("ttc")
    if ttc is None:
        ttc = _ttc_seconds(obj)
    distance = _front_distance_m(obj)
    closing = _closing_speed_mps(obj)
    return (-risk, float(ttc) if ttc is not None else 9999.0, distance, -closing)


def select_right_lane_objects(lane_3_objects: Iterable[Dict]) -> List[Dict]:
    objects = [dict(obj) for obj in lane_3_objects or []]
    objects.sort(key=_priority_key)
    return objects[:SPI_MAX_OBJECTS]


def select_left_lane_objects(lane_1_objects: Iterable[Dict]) -> List[Dict]:
    objects = [dict(obj) for obj in lane_1_objects or []]
    objects.sort(key=_priority_key)
    return objects[:SPI_MAX_OBJECTS]


def select_objects_by_turn_signal(
    turn_signal: int,
    lane_1_objects: Iterable[Dict],
    lane_2_objects: Iterable[Dict],
    lane_3_objects: Iterable[Dict],
) -> Tuple[List[Dict], int]:
    """Select objects for STM32 based on the current turn-signal request.

    lane_1_objects: left lane objects
    lane_2_objects: center/current lane objects
    lane_3_objects: right lane objects
    """
    if turn_signal == TURN_SIGNAL_RIGHT:
        return select_right_lane_objects(lane_3_objects), LANE_ID_RIGHT
    if turn_signal == TURN_SIGNAL_LEFT:
        return select_left_lane_objects(lane_1_objects), LANE_ID_LEFT
    if turn_signal == TURN_SIGNAL_HAZARD:
        objects = list(lane_1_objects or []) + list(lane_3_objects or [])
        objects.sort(key=_priority_key)
        return objects[:SPI_MAX_OBJECTS], LANE_ID_NONE
    return [], LANE_ID_NONE


def calculate_risk_level(objects: Iterable[Dict]) -> int:
    """Convert selected objects into one packet-level risk byte."""
    risk = RISK_NONE
    for obj in objects or []:
        obj_risk = int(obj.get("risk_level", RISK_NONE))
        distance = _front_distance_m(obj)
        ttc = obj.get("ttc")
        if ttc is None:
            ttc = _ttc_seconds(obj)

        if distance > 0.0 and distance <= 1.5:
            obj_risk = max(obj_risk, RISK_DANGER)
        elif ttc is not None and ttc <= 1.0:
            obj_risk = max(obj_risk, RISK_DANGER)

        risk = max(risk, obj_risk)
    return max(RISK_NONE, min(RISK_DANGER, risk))


def _object_to_packet_fields(obj: Dict) -> List[int]:
    distance_cm = int(round(max(0.0, _front_distance_m(obj)) * 100.0))
    velocity_mps = _as_float(obj.get("radial_velocity", obj.get("v", obj.get("vy", 0.0))))
    velocity_cms = int(round(velocity_mps * 100.0))
    x_cm = int(round(_as_float(obj.get("x")) * 100.0))
    y_cm = int(round(_as_float(obj.get("y")) * 100.0))

    fields: List[int] = []
    fields.extend(_to_uint16_bytes(distance_cm))
    fields.extend(_to_int16_bytes(velocity_cms))
    fields.extend(_to_int16_bytes(x_cm))
    fields.extend(_to_int16_bytes(y_cm))
    return fields


def build_radar_packet(
    turn_signal: int,
    selected_objects: Iterable[Dict],
    seq: int,
    lane_id: int = LANE_ID_NONE,
) -> List[int]:
    """Build a fixed-length STM32 radar packet.

    Packet length is 33 bytes when SPI_MAX_OBJECTS=3:
        [0]  0xAA
        [1]  0x55
        [2]  protocol version
        [3]  sequence number
        [4]  turn request/status from STM32
        [5]  object count, max 3
        [6]  risk level, 0:none 1:caution 2:warning 3:danger
        [7]  lane id, 0:none 1:left 2:center 3:right
        [8..31]  up to 3 objects, each:
            distance_cm uint16, velocity_cms int16, x_cm int16, y_cm int16
        [32] checksum = sum(bytes[0..31]) & 0xFF
    """
    objects = list(selected_objects or [])[:SPI_MAX_OBJECTS]
    packet = [
        HEADER1,
        HEADER2,
        VERSION,
        _byte(seq),
        _byte(turn_signal),
        _byte(len(objects)),
        _byte(calculate_risk_level(objects)),
        _byte(lane_id),
    ]

    for obj in objects:
        packet.extend(_object_to_packet_fields(obj))

    while len(packet) < RADAR_PACKET_LEN - 1:
        packet.append(0)

    packet.append(compute_checksum(packet))
    return packet


def build_acc_packet(
    turn_signal: int,
    selected_objects: Iterable[Dict],
    seq: int,
    lane_id: int,
    recommendation: Optional[ACCRecommendation],
) -> List[int]:
    """Build the 33-byte MOSI packet while preserving the legacy bytes 0..7.

    Layout:
        [0..7]   legacy header/selection fields
        [8..9]   recommended speed, uint16 little-endian, x100 m/s
        [10..11] safe distance, uint16 little-endian, x100 m
        [12..13] TTC, uint16 little-endian, x100 s; 0xFFFF means unavailable
        [14..31] reserved
        [32]     checksum of bytes 0..31
    """
    objects = list(selected_objects or [])[:SPI_MAX_OBJECTS]
    packet = [
        HEADER1,
        HEADER2,
        VERSION,
        _byte(seq),
        _byte(turn_signal),
        _byte(len(objects)),
        _byte(calculate_risk_level(objects)),
        _byte(lane_id),
    ]
    packet.extend([0] * (RADAR_PACKET_LEN - len(packet)))

    if recommendation is None:
        recommended_speed_x100 = 0
        safe_distance_x100 = 0
        ttc_x100 = 0xFFFF
    else:
        recommended_speed_x100 = round(
            max(0.0, recommendation.recommended_speed_mps) * 100.0
        )
        safe_distance_x100 = round(max(0.0, recommendation.safe_distance_m) * 100.0)
        ttc_x100 = (
            0xFFFF
            if recommendation.ttc_sec is None
            else round(max(0.0, recommendation.ttc_sec) * 100.0)
        )

    speed_hi, speed_lo = _to_uint16_bytes(recommended_speed_x100)
    distance_hi, distance_lo = _to_uint16_bytes(safe_distance_x100)
    ttc_hi, ttc_lo = _to_uint16_bytes(ttc_x100)
    packet[8:14] = [
        speed_lo,
        speed_hi,
        distance_lo,
        distance_hi,
        ttc_lo,
        ttc_hi,
    ]
    packet[STM32_CHECKSUM_INDEX] = compute_checksum(packet[:STM32_CHECKSUM_INDEX])
    return packet


def build_stm32_permission_packet(lane_change_allowed: int) -> List[int]:
    """MOSI packet: byte0 bit0 is permission; remaining bytes provide RX clocks."""
    return [_byte(lane_change_allowed) & 0x01] + [0x00] * (STM32_SPI_PACKET_LEN - 1)


class SPISender:
    """SPI master used by Raspberry Pi to communicate with STM32."""

    def __init__(
        self,
        bus: int = SPI_BUS,
        device: int = SPI_DEVICE,
        speed_hz: int = SPI_SPEED_HZ,
        mode: int = SPI_MODE,
        bits_per_word: int = SPI_BITS_PER_WORD,
        use_mock: Optional[bool] = None,
        packet_period_sec: float = SPI_PACKET_PERIOD_SEC,
        safe_fallback_to_mock: bool = SPI_SAFE_FALLBACK_TO_MOCK,
    ):
        if use_mock is None:
            use_mock = SPI_USE_MOCK if SPI_USE_MOCK is not None else USE_MOCK_SPI

        self.bus = int(bus)
        self.device = int(device)
        self.speed_hz = int(speed_hz)
        self.mode = int(mode)
        self.bits_per_word = int(bits_per_word)
        self.use_mock = bool(use_mock)
        self.packet_period_sec = float(packet_period_sec)
        self.safe_fallback_to_mock = bool(safe_fallback_to_mock)
        self.frame_id = 0
        self.last_turn_signal = TURN_SIGNAL_NONE
        self.last_encoder_speed = 0
        self.last_ego_speed_mps = 0.0
        self.last_steering_angle_deg = 0.0
        self.last_miso_valid = False
        self.last_lane_change_allowed = 0
        self.last_selected_lane = LANE_ID_NONE
        self.last_acc_recommendation: Optional[ACCRecommendation] = None
        self.last_packet: List[int] = []
        self.last_rx: List[int] = []
        self._spi: Optional[object] = None
        self._last_transfer_time = 0.0
        self._acc_controller = AdaptiveCruiseController()

        if self.use_mock:
            logger.info("SPI mock mode enabled")
            return

        self.open_spi()

    def open_spi(self) -> bool:
        """Open /dev/spidevX.Y through spidev."""
        if self.use_mock:
            return False

        dev_path = f"/dev/spidev{self.bus}.{self.device}"
        if os.name != "nt" and not os.path.exists(dev_path):
            message = (
                f"{dev_path} not found. Enable SPI with: sudo raspi-config -> "
                "Interface Options -> SPI -> Enable, then sudo reboot. "
                "Also check: ls /dev/spidev*"
            )
            logger.warning(message)
            if self.safe_fallback_to_mock:
                self.use_mock = True
                logger.warning("falling back to SPI mock mode")
                return False
            raise FileNotFoundError(message)

        try:
            import spidev  # type: ignore

            self._spi = spidev.SpiDev()
            self._spi.open(self.bus, self.device)
            self._spi.max_speed_hz = self.speed_hz
            self._spi.mode = self.mode
            self._spi.bits_per_word = self.bits_per_word
            logger.info(
                "SPI opened bus=%s device=%s speed_hz=%s mode=%s bits=%s",
                self.bus,
                self.device,
                self.speed_hz,
                self.mode,
                self.bits_per_word,
            )
            return True
        except ImportError as exc:
            message = (
                "spidev is not installed. On Raspberry Pi run: pip3 install spidev"
            )
            logger.warning("%s (%s)", message, exc)
        except PermissionError as exc:
            message = (
                "SPI permission denied. Try adding the user to the spi group, "
                "or test with: sudo python3 main.py"
            )
            logger.warning("%s (%s)", message, exc)
        except Exception as exc:
            logger.warning("SPI open failed: %s", exc)

        if self.safe_fallback_to_mock:
            self.use_mock = True
            self._spi = None
            logger.warning("falling back to SPI mock mode")
            return False
        raise RuntimeError("SPI open failed and fallback is disabled")

    def close_spi(self) -> None:
        if self._spi is None:
            return
        try:
            self._spi.close()
            logger.info("SPI closed")
        finally:
            self._spi = None

    def close(self) -> None:
        self.close_spi()

    def should_transfer(self) -> bool:
        if self.packet_period_sec <= 0.0:
            return True
        return (time.monotonic() - self._last_transfer_time) >= self.packet_period_sec

    def spi_transfer_packet(self, tx_packet: Iterable[int], force: bool = False) -> List[int]:
        """Send tx_packet and return RX bytes from STM32.

        Mock mode keeps the old log style: "SPI mock send packet=..."
        Real mode logs: "SPI real transfer tx=..., rx=..."
        """
        packet = [_byte(value) for value in tx_packet]
        if len(packet) < STM32_SPI_PACKET_LEN:
            packet.extend([0] * (STM32_SPI_PACKET_LEN - len(packet)))
        elif len(packet) > STM32_SPI_PACKET_LEN:
            packet = packet[:STM32_SPI_PACKET_LEN]
        self.last_packet = packet

        if not force and not self.should_transfer():
            return self.last_rx

        self._last_transfer_time = time.monotonic()

        if self.use_mock or self._spi is None:
            rx = build_miso_packet(
                SPI_MOCK_EGO_SPEED_MPS,
                SPI_MOCK_STEERING_ANGLE_DEG,
                SPI_MOCK_TURN_SIGNAL,
            )
            self.last_rx = rx
            self._update_miso_state(rx)
            logger.info("SPI mock send packet=%s", packet)
            return rx

        try:
            # 중요: xfer2()는 packet을 IN-PLACE로 RX 데이터로 수정한다.
            # 따라서 TX 데이터를 로깅하려면 xfer2() 호출 전에 복사해야 한다.
            tx_to_log = list(packet)
            rx = list(self._spi.xfer2(packet)) # 패킷은 rpi가 stm32로 보내는 데이터, rx는 stm32가 rpi로 동시에 보내 데이터
            self.last_rx = rx
            self._update_miso_state(rx)
            logger.info("SPI real transfer tx=%s, rx=%s", tx_to_log, rx)
            return rx
        except Exception as exc:
            logger.warning("SPI transfer failed: %s", exc)
            self.last_miso_valid = False
            if self.safe_fallback_to_mock:
                self.use_mock = True
                logger.warning("falling back to SPI mock mode after transfer failure")
                return self.spi_transfer_packet(packet, force=True)
            return []

    def _update_miso_state(self, rx: Sequence[int]) -> bool:
        parsed = parse_miso_packet(rx)
        self.last_miso_valid = parsed is not None
        if parsed is None:
            logger.warning("Invalid STM32 MISO packet: %s", list(rx))
            return False

        (
            self.last_ego_speed_mps,
            self.last_steering_angle_deg,
            self.last_turn_signal,
        ) = parsed
        logger.info(
            "STM32 ego_speed=%.2f m/s steering_angle=%.1f deg turn_signal=%s",
            self.last_ego_speed_mps,
            self.last_steering_angle_deg,
            turn_signal_name(self.last_turn_signal),
        )
        return True

    def poll_turn_signal(self) -> int:
        """Legacy API: perform an exchange and return the last known turn signal."""
        heartbeat = self.last_packet or build_acc_packet(
            turn_signal=self.last_turn_signal,
            selected_objects=[],
            seq=self.frame_id,
            lane_id=self.last_selected_lane,
            recommendation=self.last_acc_recommendation,
        )
        self.spi_transfer_packet(heartbeat, force=True)
        return self.last_turn_signal

    def transfer_lane_result(self, lane_result, tracks: Optional[Iterable[Dict]] = None) -> List[int]:
        """Exchange lane permission and the 33-byte STM32 vehicle-state packet.

        STM32 sends speed x100, steering x10, turn signal, and checksum.
        Raspberry Pi sends the previous cycle's 33-byte ACC recommendation packet.
        """
        if self.last_packet and not self.should_transfer():
            return self.last_packet

        tx_packet = self.last_packet or build_acc_packet(
            turn_signal=self.last_turn_signal,
            selected_objects=[],
            seq=self.frame_id,
            lane_id=self.last_selected_lane,
            recommendation=None,
        )
        self.spi_transfer_packet(tx_packet, force=True)
        turn_signal = self.last_turn_signal

        left_objects = list(getattr(lane_result, "left_objects", []) or [])
        right_objects = list(getattr(lane_result, "right_objects", []) or [])
        center_objects = [
            dict(track)
            for track in (tracks or [])
            if track.get("lane_label", "unknown") not in ("left", "right")
        ]

        selected_objects, lane_id = select_objects_by_turn_signal(
            turn_signal,
            lane_1_objects=left_objects,
            lane_2_objects=center_objects,
            lane_3_objects=right_objects,
        )

        risk = calculate_risk_level(selected_objects)
        lane_change_allowed = 1 if risk == RISK_NONE else 0
        self.frame_id = (self.frame_id + 1) & 0xFF
        self.last_lane_change_allowed = lane_change_allowed
        self.last_selected_lane = lane_id
        self.last_acc_recommendation = self._acc_controller.update(
            tracks=tracks or [],
            ego_speed_mps=EGO_SPEED_DEFAULT_MPS,
            steering_angle_deg=0.0,
        )
        self.last_packet = build_acc_packet(
            turn_signal=turn_signal,
            selected_objects=selected_objects,
            seq=self.frame_id,
            lane_id=lane_id,
            recommendation=self.last_acc_recommendation,
        )

        logger.info(
            "SPI selection turn_signal=%s lane_id=%s risk=%s next_lane_change_allowed=%s selected_count=%s objects=%s",
            turn_signal_name(turn_signal),
            lane_id,
            risk,
            lane_change_allowed,
            len(selected_objects),
            [
                {
                    "track_id": obj.get("track_id"),
                    "x": round(_as_float(obj.get("x")), 2),
                    "y": round(_as_float(obj.get("y")), 2),
                    "risk": obj.get("risk_level", RISK_NONE),
                }
                for obj in selected_objects
            ],
        )
        return self.last_packet

    # Backward-compatible packet builder from the previous mock-only version.
    def build_packet(
        self,
        left_risk: int,
        right_risk: int,
        left_count: int,
        right_count: int,
    ) -> List[int]:
        packet = [
            HEADER1,
            HEADER2,
            VERSION,
            _byte(self.frame_id),
            _byte(left_risk),
            _byte(right_risk),
            _byte(left_count),
            _byte(right_count),
        ]
        packet.append(compute_checksum(packet))
        self.frame_id = (self.frame_id + 1) & 0xFF
        return packet

    # Backward-compatible send function. New code should prefer spi_transfer_packet().
    def send_packet(self, packet: Iterable[int]) -> None:
        self.spi_transfer_packet(packet, force=True)


def open_spi() -> SPISender:
    sender = SPISender(use_mock=False)
    return sender


def close_spi(sender: Optional[SPISender]) -> None:
    if sender is not None:
        sender.close_spi()


def spi_transfer_packet(sender: SPISender, tx_packet: Iterable[int]) -> List[int]:
    return sender.spi_transfer_packet(tx_packet)
