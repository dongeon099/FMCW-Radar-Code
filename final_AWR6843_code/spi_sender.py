# -*- coding: utf-8 -*-
"""SPI sender/receiver for Raspberry Pi <-> STM32.

Raspberry Pi(Master)와 STM32(Slave) 간에 헤더, sequence, CRC8을 포함한
고정 8바이트 프레임을 교환합니다.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from typing import Dict, Iterable, Optional, Sequence, Tuple

from config import (
    SPI_BUS,
    SPI_DEVICE,
    SPI_SPEED_HZ,
    SPI_MODE,
    SPI_AUTO_DETECT_MODE,
    SPI_PACKET_PERIOD_SEC,
    SPI_FRAME_RETRY_COUNT,
    SPI_FRAME_RETRY_DELAY_SEC,
    SPI_USE_MOCK,
    USE_MOCK_SPI,
    TURN_SIGNAL_LEFT,
    TURN_SIGNAL_NONE,
    TURN_SIGNAL_RIGHT,
)

logger = logging.getLogger(__name__)

# 고정 SPI 프레임: AA 55 | sequence | data[4] | CRC8
SPI_FRAME_SIZE = 8
SPI_SYNC = (0xAA, 0x55)
SPI_CRC8_POLYNOMIAL = 0x07
TTC_UNAVAILABLE_X100 = 0xFFFF
STM_SPEED_MIN_KMH = -50
STM_SPEED_MAX_KMH = 50
STM_STEERING_MIN_DEG = -45
STM_STEERING_MAX_DEG = 45


def _byte(value: int) -> int:
    """값을 안전하게 1바이트(0~255) 범위로 제한합니다."""
    return max(0, min(255, int(value))) & 0xFF


def _int8(value: int) -> int:
    """STM32가 보낸 uint8 표현을 signed int8 값으로 복원합니다."""
    value = int(value) & 0xFF
    return value - 256 if value >= 128 else value


def _crc8(data: Sequence[int]) -> int:
    """CRC-8/ATM(poly=0x07, init=0x00)을 계산합니다."""
    crc = 0
    for value in data:
        crc ^= int(value) & 0xFF
        for _ in range(8):
            crc = ((crc << 1) ^ SPI_CRC8_POLYNOMIAL) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _build_frame(sequence: int, data: Sequence[int]) -> list[int]:
    if len(data) != 4:
        raise ValueError("SPI frame data must contain exactly 4 bytes")
    frame = [SPI_SYNC[0], SPI_SYNC[1], int(sequence) & 0xFF]
    frame.extend(int(value) & 0xFF for value in data)
    frame.append(_crc8(frame))
    return frame


def calculate_closest_ttc(lane_result, tracks: Optional[Iterable[Dict]]) -> float:
    """비전 파이프라인 데이터에서 현재 가장 위험한(짧은) TTC 값을 계산합니다.
    장애물이 없거나 안전하면 무한대(inf)를 반환합니다.
    """
    min_ttc = float('inf')
    
    # 모든 트랙 객체를 통합하여 탐색
    all_objects = []
    if lane_result:
        all_objects.extend(getattr(lane_result, "left_objects", []) or [])
        all_objects.extend(getattr(lane_result, "right_objects", []) or [])
    if tracks:
        all_objects.extend(tracks)

    for obj in all_objects:
        try:
            # 거리(y)와 상대속도(vy) 추출
            y = float(obj.get("y", obj.get("distance", 0.0)))
            vy = float(obj.get("vy", obj.get("radial_velocity", obj.get("v", 0.0))))
            
            # 접근 중인 장애물에 대해서만 TTC 연산 (vy가 음수일 때 다가오는 경우 처리)
            closing_speed = abs(vy) if vy < 0 else 0.0
            if y > 0.0 and closing_speed > 0.0:
                ttc = y / closing_speed
                if ttc < min_ttc:
                    min_ttc = ttc
        except (TypeError, ValueError):
            continue

    return min_ttc


def encode_ttc_x100(ttc: float) -> int:
    """TTC를 STM32 프로토콜의 uint16, 0.01초 단위로 인코딩합니다."""
    if not math.isfinite(ttc) or ttc < 0.0:
        return TTC_UNAVAILABLE_X100
    return min(TTC_UNAVAILABLE_X100 - 1, max(0, int(round(ttc * 100.0))))


class SPISender:
    """헤더/sequence/CRC가 포함된 고정 8바이트 SPI 마스터 클래스"""

    def __init__(self, bus: int = SPI_BUS, device: int = SPI_DEVICE, speed_hz: int = SPI_SPEED_HZ):
        self.bus = int(bus)
        self.device = int(device)
        self.speed_hz = int(speed_hz)
        self.use_mock = SPI_USE_MOCK if SPI_USE_MOCK is not None else USE_MOCK_SPI
        
        # 최신 수신 상태 변수들
        self.last_steering_angle_raw = 0
        self.last_ego_speed_raw = 0
        self.last_steering_angle_deg = 0.0
        self.last_ego_speed_mps = 0.0
        self.last_turn_signal = TURN_SIGNAL_NONE
        self.last_rx_raw = tuple(0 for _ in range(SPI_FRAME_SIZE))
        self.last_miso_valid = False
        self.invalid_miso_count = 0
        self.last_miso_sequence = None
        self.last_stm_status = 0
        self.last_valid_miso_time = None
        self.valid_miso_count = 0
        self._tx_sequence = 0
        self._rx_stream = bytearray()
        self._target_ttc_x100 = TTC_UNAVAILABLE_X100
        self._target_flags = 0
        
        self._spi = None
        self._last_transfer_time = 0.0
        self._transfer_lock = threading.Lock()
        self._target_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker = None

        if not self.use_mock:
            self.open_spi()
        self.start()

    def open_spi(self) -> bool:
        dev_path = f"/dev/spidev{self.bus}.{self.device}"
        if not os.path.exists(dev_path):
            logger.warning(f"SPI 디바이스 노드({dev_path})가 없어 Mock 모드로 전환합니다.")
            self.use_mock = True
            return False
        try:
            import spidev
            self._spi = spidev.SpiDev()
            self._spi.open(self.bus, self.device)
            self._spi.max_speed_hz = self.speed_hz
            self._spi.mode = SPI_MODE
            if SPI_AUTO_DETECT_MODE:
                self._detect_spi_mode()
            return True
        except Exception as e:
            logger.error(f"SPI 오픈 실패, Mock 전환: {e}")
            self.use_mock = True
            return False

    @staticmethod
    def _extract_probe_frames(stream: bytearray) -> int:
        count = 0
        while len(stream) >= 2:
            sync_index = stream.find(bytes(SPI_SYNC))
            if sync_index < 0:
                stream[:] = stream[-1:] if stream[-1] == SPI_SYNC[0] else b""
                break
            if sync_index > 0:
                del stream[:sync_index]
            if len(stream) < SPI_FRAME_SIZE:
                break
            candidate = tuple(stream[:SPI_FRAME_SIZE])
            if candidate[7] == _crc8(candidate[:7]):
                count += 1
                del stream[:SPI_FRAME_SIZE]
            else:
                del stream[0]
        return count

    def _detect_spi_mode(self) -> None:
        """실제 선로에서 정상 CRC 프레임이 가장 많이 잡히는 SPI mode를 선택합니다."""
        scores = {}
        probe_packet = _build_frame(0, [0xFF, 0xFF, 0, 0])

        for mode in range(4):
            self._spi.mode = mode
            stream = bytearray()
            score = 0
            for _ in range(16):
                rx = self._spi.xfer2(probe_packet)
                stream.extend(int(value) & 0xFF for value in rx)
                score += self._extract_probe_frames(stream)
                time.sleep(0.003)
            scores[mode] = score

        best_mode = max(scores, key=lambda mode: (scores[mode], mode == SPI_MODE))
        self._spi.mode = best_mode
        logger.warning(
            "SPI mode auto-detect scores=%s selected_mode=%d",
            scores,
            best_mode,
        )

    def close_spi(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None

    def start(self) -> None:
        """레이더 처리와 독립적인 SPI 폴링 스레드를 시작합니다."""
        if self._worker is not None and self._worker.is_alive():
            return
        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._poll_loop,
            name="spi-poll",
            daemon=True,
        )
        self._worker.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        self._worker = None
        self.close_spi()

    def _poll_loop(self) -> None:
        period = max(0.005, float(SPI_PACKET_PERIOD_SEC))
        next_deadline = time.monotonic()

        while not self._stop_event.is_set():
            with self._target_lock:
                ttc_x100 = self._target_ttc_x100
                flags = self._target_flags

            try:
                self.transfer_frame(ttc_x100, flags)
            except Exception:
                logger.exception("SPI polling loop failed")

            next_deadline += period
            wait_time = next_deadline - time.monotonic()
            if wait_time <= 0.0:
                next_deadline = time.monotonic()
                continue
            self._stop_event.wait(wait_time)

    def _consume_rx_chunk(self, rx: Sequence[int]):
        self.last_rx_raw = tuple(int(value) & 0xFF for value in rx)
        self._rx_stream.extend(self.last_rx_raw)

        while len(self._rx_stream) >= 2:
            sync_index = self._rx_stream.find(bytes(SPI_SYNC))
            if sync_index < 0:
                # 다음 chunk와 이어질 수 있는 마지막 0xAA만 보존한다.
                self._rx_stream[:] = self._rx_stream[-1:] if self._rx_stream[-1] == SPI_SYNC[0] else b""
                return None
            if sync_index > 0:
                del self._rx_stream[:sync_index]
            if len(self._rx_stream) < SPI_FRAME_SIZE:
                return None

            candidate = tuple(self._rx_stream[:SPI_FRAME_SIZE])
            if candidate[7] == _crc8(candidate[:7]):
                del self._rx_stream[:SPI_FRAME_SIZE]
                return candidate

            # 가짜 헤더 또는 손상 프레임: 한 바이트만 버리고 다음 AA55 탐색
            del self._rx_stream[0]

        return None

    def transfer_frame(self, ttc_x100: int, flags: int = 0) -> Tuple[int, int, int]:
        with self._transfer_lock:
            return self._transfer_frame_unlocked(ttc_x100, flags)

    def _transfer_frame_unlocked(self, ttc_x100: int, flags: int = 0) -> Tuple[int, int, int]:
        """고정 8바이트 프레임으로 STM32와 상태를 교환합니다.

        MOSI: AA 55 | seq | TTC LSB | TTC MSB | flags | reserved | CRC8
        MISO: AA 55 | seq | speed | steering | blink | status | CRC8
        """
        ttc_x100 = max(0, min(TTC_UNAVAILABLE_X100, int(ttc_x100)))
        request_sequence = self._tx_sequence
        self._tx_sequence = (self._tx_sequence + 1) & 0xFF
        tx_packet = _build_frame(
            request_sequence,
            [ttc_x100 & 0xFF, (ttc_x100 >> 8) & 0xFF, int(flags) & 0xFF, 0],
        )
        
        if self.use_mock or self._spi is None:
            valid_frame = tuple(_build_frame(request_sequence, [45, 0, 0, 0]))
        else:
            valid_frame = None
            for attempt in range(max(1, int(SPI_FRAME_RETRY_COUNT))):
                try:
                    rx = self._spi.xfer2(tx_packet)
                except Exception as e:
                    logger.warning(f"SPI 전송 오류: {e}")
                    return (self.last_steering_angle_raw, self.last_ego_speed_raw, self.last_turn_signal)

                valid_frame = self._consume_rx_chunk(rx)
                if valid_frame is not None:
                    break
                if attempt + 1 < SPI_FRAME_RETRY_COUNT:
                    # NSS를 올린 동안 STM 완료 콜백이 다음 프레임을 re-arm할 시간을 준다.
                    time.sleep(max(0.0, float(SPI_FRAME_RETRY_DELAY_SEC)))

        if valid_frame is not None:
            sequence = valid_frame[2]
            speed_kmh = _int8(valid_frame[3])
            steering_deg = _int8(valid_frame[4])
            blink_raw = valid_frame[5]
            frame_valid = (
                sequence != self.last_miso_sequence
                and STM_SPEED_MIN_KMH <= speed_kmh <= STM_SPEED_MAX_KMH
                and STM_STEERING_MIN_DEG <= steering_deg <= STM_STEERING_MAX_DEG
                and blink_raw in (0, 1, 2)
            )
        else:
            frame_valid = False

        if not frame_valid:
            self.invalid_miso_count += 1
            if self.invalid_miso_count == 1 or self.invalid_miso_count % 20 == 0:
                logger.warning(
                    "No valid SPI frame yet: raw=%s buffered=%d invalid_count=%d",
                    list(self.last_rx_raw),
                    len(self._rx_stream),
                    self.invalid_miso_count,
                )
            return (
                self.last_steering_angle_raw,
                self.last_ego_speed_raw,
                self.last_turn_signal,
            )

        self.last_miso_sequence = sequence
        self.last_stm_status = valid_frame[6]
        self.last_valid_miso_time = time.monotonic()
        self.valid_miso_count += 1
        self.last_ego_speed_raw = speed_kmh
        self.last_steering_angle_raw = steering_deg
        self.last_ego_speed_mps = self.last_ego_speed_raw / 3.6
        self.last_steering_angle_deg = float(self.last_steering_angle_raw)

        # STM32 wire value: 1=RIGHT, 2=LEFT
        if blink_raw == 1:
            self.last_turn_signal = TURN_SIGNAL_RIGHT
        elif blink_raw == 2:
            self.last_turn_signal = TURN_SIGNAL_LEFT
        else:
            self.last_turn_signal = TURN_SIGNAL_NONE
        self.last_miso_valid = True

        return (self.last_steering_angle_raw, self.last_ego_speed_raw, self.last_turn_signal)

    def transfer_3bytes(self, ttc_x100: int, flags: int = 0) -> Tuple[int, int, int]:
        """기존 호출부 호환용 별칭입니다. 실제 전송은 8바이트 프레임입니다."""
        return self.transfer_frame(ttc_x100, flags)

    def transfer_lane_result(self, lane_result, tracks: Optional[Iterable[Dict]] = None) -> Tuple[int, int, int]:
        """레이더 결과로 다음 SPI 송신값을 갱신하고 최신 수신 상태를 반환합니다."""
        # 1. 최신 객체들로부터 충돌 위협 시간(TTC) 산출
        current_ttc = calculate_closest_ttc(lane_result, tracks)
        
        # 2. uint16 형태로 변환 (0.01초 단위, little endian)
        ttc_x100 = encode_ttc_x100(current_ttc)
        
        # SPI worker가 50Hz로 사용할 최신 목표값만 갱신한다.
        with self._target_lock:
            self._target_ttc_x100 = ttc_x100

        rx_steering = self.last_steering_angle_raw
        rx_speed = self.last_ego_speed_raw
        rx_turn = self.last_turn_signal
        
        logger.info(
            "SPI [8B] -> TX_TTC_x100: %d | RX_SEQ: %s | "
            "RX_Speed: %d | RX_Steer: %d | RX_Turn: %d | Valid: %d Invalid: %d",
            ttc_x100,
            self.last_miso_sequence,
            rx_speed,
            rx_steering,
            rx_turn,
            self.valid_miso_count,
            self.invalid_miso_count,
        )
        return (rx_steering, rx_speed, rx_turn)


# 하위 호환성 및 편의성을 위한 전역 래퍼 함수
def open_spi() -> SPISender:
    return SPISender()

def close_spi(sender: Optional[SPISender]) -> None:
    if sender is not None:
        sender.close()
