# -*- coding: utf-8 -*-
"""Laptop entrypoint: receive Raspberry Pi UDP radar results and visualize them."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import config as app_config
from advanced_visualizer import create_visualizer
from network_receiver import RadarUDPReceiver


logger = logging.getLogger(__name__)


@dataclass
class LaptopLaneResult:
    left_objects: List[Dict]
    right_objects: List[Dict]
    left_risk: int
    right_risk: int


def _as_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _as_int(value, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _lane_label(lane_id) -> str:
    try:
        lane_id = int(lane_id)
    except (TypeError, ValueError):
        return "unknown"
    if lane_id == 1:
        return "left"
    if lane_id == 2:
        return "center"
    if lane_id == 3:
        return "right"
    return "unknown"


def _turn_signal_name(value) -> str:
    try:
        value = int(value)
    except (TypeError, ValueError):
        return str(value or "UNKNOWN").upper()
    return {
        0: "NONE",
        1: "RIGHT",
        2: "LEFT",
        3: "HAZARD",
    }.get(value, f"UNKNOWN({value})")


def advice_from_packet(packet: Optional[Dict], fallback_timestamp: float) -> Dict:
    if not packet:
        return {"reason": f"UDP timestamp {fallback_timestamp:.3f}"}

    raw_advice = packet.get("advice")
    advice = dict(raw_advice) if isinstance(raw_advice, dict) else {}
    raw_state = packet.get("vehicle_state")
    state = raw_state if isinstance(raw_state, dict) else {}

    turn_signal = state.get("turn_signal", packet.get("turn_signal"))
    advice.update(
        {
            "ego_speed_kmh": state.get("ego_speed_kmh"),
            "ego_current_speed_mps": state.get(
                "ego_speed_mps", advice.get("ego_current_speed_mps")
            ),
            "current_steering_angle_deg": state.get(
                "steering_angle_deg", advice.get("current_steering_angle_deg")
            ),
            "turn_signal": _turn_signal_name(turn_signal),
            "miso_valid": state.get("miso_valid"),
            "spi_sequence": state.get("spi_sequence"),
            "spi_valid_count": state.get("spi_valid_count"),
            "spi_invalid_count": state.get("spi_invalid_count"),
        }
    )
    advice.setdefault("reason", f"UDP timestamp {fallback_timestamp:.3f}")
    return advice


def _track_from_udp_object(obj: Dict) -> Dict:
    risk = _as_int(obj.get("risk"), 0)
    lane_label = _lane_label(obj.get("lane"))
    return {
        "track_id": _as_int(obj.get("track_id"), 0),
        "lane": _as_int(obj.get("lane"), 0),
        "lane_label": lane_label,
        "x": _as_float(obj.get("x")),
        "y": _as_float(obj.get("y")),
        "v": _as_float(obj.get("v")),
        "radial_velocity": _as_float(obj.get("v")),
        "vx": 0.0,
        "vy": _as_float(obj.get("v")),
        "ttc": obj.get("ttc"),
        "risk": risk,
        "risk_level": risk,
        "status": "confirmed",
        "age": "-",
        "hits": "-",
        "missed_count": "-",
    }


def tracks_from_packet(packet: Optional[Dict]) -> List[Dict]:
    if not packet:
        return []
    return [
        _track_from_udp_object(obj)
        for obj in packet.get("objects", []) or []
        if isinstance(obj, dict)
    ]


def lane_result_from_tracks(tracks: Iterable[Dict]) -> LaptopLaneResult:
    left_objects = [dict(track) for track in tracks if track.get("lane_label") == "left"]
    right_objects = [dict(track) for track in tracks if track.get("lane_label") == "right"]
    return LaptopLaneResult(
        left_objects=left_objects,
        right_objects=right_objects,
        left_risk=max((int(obj.get("risk_level", 0)) for obj in left_objects), default=0),
        right_risk=max((int(obj.get("risk_level", 0)) for obj in right_objects), default=0),
    )


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    receiver = RadarUDPReceiver(port=app_config.UDP_PORT, buffer_size=app_config.UDP_RECV_BUFFER)
    visualizer = create_visualizer(app_config)
    last_tracks: List[Dict] = []
    last_advice: Dict = {}
    last_frame = -1
    last_timestamp = time.monotonic()

    try:
        while True:
            packet = receiver.receive()
            if packet is not None:
                last_tracks = tracks_from_packet(packet)
                last_frame = int(packet.get("frame", last_frame))
                last_timestamp = _as_float(packet.get("timestamp"), time.monotonic())
                last_advice = advice_from_packet(packet, last_timestamp)

            lane_result = lane_result_from_tracks(last_tracks)
            if visualizer is not None:
                visualizer.update(
                    detections=[],
                    clusters=[],
                    tracks=last_tracks,
                    lane_result=lane_result,
                    spi_packet=None,
                    frame_id=last_frame,
                    dt=0.0,
                    processing_time_ms=0.0,
                    advice=last_advice,
                )

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n사용자 중지")

    finally:
        receiver.close()
        if visualizer is not None:
            visualizer.close()
        print("UDP 수신/시각화 종료")


if __name__ == "__main__":
    main()
