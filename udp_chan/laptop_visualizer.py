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
    if isinstance(lane_id, str):
        text = lane_id.strip().lower()
        if text in ("left", "l", "lane_1", "1"):
            return "left"
        if text in ("center", "centre", "middle", "c", "lane_2", "2"):
            return "center"
        if text in ("right", "r", "lane_3", "3"):
            return "right"
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


def _lane_from_x(x_value: float) -> str:
    left_low, left_high = getattr(app_config, "LEFT_LANE_X_RANGE", (-0.09, -0.03))
    center_low, center_high = getattr(app_config, "CENTER_LANE_X_RANGE", (-0.03, 0.03))
    right_low, right_high = getattr(app_config, "RIGHT_LANE_X_RANGE", (0.03, 0.09))
    if left_low <= x_value <= left_high:
        return "left"
    if right_low <= x_value <= right_high:
        return "right"
    if center_low <= x_value <= center_high:
        return "center"
    return "unknown"


def _lane_id_from_label(label: str) -> int:
    if label == "left":
        return 1
    if label == "center":
        return 2
    if label == "right":
        return 3
    return 0


def _risk_from_distance_velocity(distance: float, velocity: float) -> int:
    closing_speed = abs(velocity) if velocity < 0.0 else 0.0
    if distance <= 2.0 or (distance <= 4.0 and closing_speed >= 1.5):
        return 2
    if distance <= 5.0 or (distance <= 8.0 and closing_speed >= 0.8):
        return 1
    return 0


def _packet_value(packet: Optional[Dict], names, default=None):
    if not packet:
        return default
    for name in names:
        if name in packet:
            return packet.get(name)
    return default


def _track_from_udp_object(obj: Dict) -> Dict:
    x_value = _as_float(obj.get("x"))
    distance = _as_float(obj.get("distance", obj.get("y")))
    velocity = _as_float(obj.get("velocity", obj.get("v", obj.get("radial_velocity"))))
    lane_label = _lane_label(obj.get("lane", obj.get("lane_label")))
    if lane_label == "unknown":
        lane_label = _lane_from_x(x_value)
    risk = _as_int(
        obj.get("risk", obj.get("risk_level")),
        _risk_from_distance_velocity(distance, velocity),
    )
    ttc = obj.get("ttc")
    if ttc is None and velocity < 0.0 and distance > 0.0:
        ttc = distance / abs(velocity)
    return {
        "track_id": _as_int(obj.get("track_id"), 0),
        "lane": _as_int(obj.get("lane"), _lane_id_from_label(lane_label)),
        "lane_label": lane_label,
        "x": x_value,
        "y": distance,
        "distance": distance,
        "v": velocity,
        "velocity": velocity,
        "radial_velocity": velocity,
        "vx": 0.0,
        "vy": velocity,
        "ttc": ttc,
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
    last_frame = -1
    last_timestamp = time.monotonic()
    last_receive_time = 0.0
    last_turn_signal = app_config.TURN_SIGNAL_NONE
    last_selected_lane = 0
    last_recommended_speed_mps = None

    try:
        while True:
            packet = receiver.receive()
            if packet is not None:
                last_tracks = tracks_from_packet(packet)
                last_frame = int(packet.get("frame", last_frame))
                last_timestamp = _as_float(packet.get("timestamp"), time.monotonic())
                last_receive_time = time.monotonic()
                last_turn_signal = _packet_value(
                    packet,
                    ("turn_signal", "turnSignal", "turn_request", "indicator"),
                    app_config.TURN_SIGNAL_NONE,
                )
                last_selected_lane = _packet_value(
                    packet,
                    ("selected_lane", "selectedLane", "lane_id"),
                    0,
                )
                last_recommended_speed_mps = _packet_value(
                    packet, ("recommended_speed_mps",), None
                )

            lane_result = lane_result_from_tracks(last_tracks)
            now = time.monotonic()
            waiting_for_data = last_receive_time <= 0.0 or now - last_receive_time > 1.5
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
                    advice={
                        "reason": f"UDP timestamp {last_timestamp:.3f}",
                        "turn_signal": last_turn_signal,
                        "selected_lane": last_selected_lane,
                        "waiting_for_data": waiting_for_data,
                        "last_data_age_sec": None if last_receive_time <= 0.0 else now - last_receive_time,
                        "recommended_speed_mps": last_recommended_speed_mps,
                    },
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
