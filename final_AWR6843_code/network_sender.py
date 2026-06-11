# -*- coding: utf-8 -*-
"""Raspberry Pi UDP sender for processed AWR6843 radar results.

This module sends only object-level results after TLV parsing, DBSCAN,
EKF tracking, lane classification, and TTC/risk decision. Raw point clouds
are intentionally not included in the UDP payload.
"""

from __future__ import annotations

import json
import logging
import math
import socket
from dataclasses import asdict, is_dataclass
from typing import Dict, Iterable, List, Optional

import config as app_config


logger = logging.getLogger(__name__)

LANE_ID_UNKNOWN = 0
LANE_ID_LEFT = 1
LANE_ID_CENTER = 2
LANE_ID_RIGHT = 3


def _as_float_or_none(value) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _as_float(value, default: float = 0.0) -> float:
    result = _as_float_or_none(value)
    return default if result is None else result


def _as_int(value, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_json_scalar(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    try:
        return int(value)
    except (TypeError, ValueError):
        return str(value)


def _as_json_mapping(value) -> Dict:
    if value is None:
        return {}
    if is_dataclass(value):
        value = asdict(value)
    elif not isinstance(value, dict):
        value = vars(value) if hasattr(value, "__dict__") else {}
    return {str(key): _as_json_scalar(item) for key, item in value.items()}


def _lane_to_id(value) -> int:
    if value in (LANE_ID_LEFT, LANE_ID_CENTER, LANE_ID_RIGHT):
        return int(value)
    text = str(value or "").strip().lower()
    if text in ("left", "lane_1", "1", "l"):
        return LANE_ID_LEFT
    if text in ("center", "middle", "lane_2", "2", "c"):
        return LANE_ID_CENTER
    if text in ("right", "lane_3", "3", "r"):
        return LANE_ID_RIGHT
    return LANE_ID_UNKNOWN


def _selected_lane_from_value(value) -> int:
    return _lane_to_id(value)


def _merge_lane_objects(tracks: Iterable[Dict], lane_result) -> List[Dict]:
    lane_objects = []
    lane_objects.extend(getattr(lane_result, "left_objects", []) or [])
    lane_objects.extend(getattr(lane_result, "right_objects", []) or [])
    lane_by_id = {obj.get("track_id"): obj for obj in lane_objects}

    merged = []
    for track in tracks or []:
        copied = dict(track)
        lane_obj = lane_by_id.get(copied.get("track_id"))
        if lane_obj:
            copied["lane"] = _lane_to_id(lane_obj.get("lane_label"))
            copied["lane_label"] = lane_obj.get("lane_label", copied.get("lane_label", "unknown"))
            copied["ttc"] = lane_obj.get("ttc", copied.get("ttc"))
            copied["risk"] = lane_obj.get("risk_level", copied.get("risk", copied.get("risk_level", 0)))
            copied["risk_level"] = copied["risk"]
        else:
            copied["lane"] = _lane_to_id(copied.get("lane_label"))
            copied["ttc"] = copied.get("ttc")
            copied["risk"] = copied.get("risk", copied.get("risk_level", 0))
        merged.append(copied)
    return merged


def build_radar_result_payload(
    frame_id: int,
    timestamp: float,
    objects: Iterable[Dict],
    turn_signal,
    selected_lane,
    vehicle_state=None,
    advice=None,
) -> Dict:
    """Build the JSON-serializable UDP payload requested by the laptop."""
    payload_objects = []
    for obj in objects or []:
        payload_objects.append(
            {
                "track_id": _as_int(obj.get("track_id"), 0),
                "lane": _lane_to_id(obj.get("lane", obj.get("lane_label"))),
                "x": _as_float(obj.get("x")),
                "y": _as_float(obj.get("y")),
                "v": _as_float(obj.get("v", obj.get("radial_velocity", obj.get("speed", 0.0)))),
                "ttc": _as_float_or_none(obj.get("ttc")),
                "risk": _as_int(obj.get("risk", obj.get("risk_level", 0)), 0),
            }
        )

    return {
        "frame": int(frame_id),
        "timestamp": _as_float(timestamp),
        "turn_signal": _as_json_scalar(turn_signal),
        "selected_lane": _selected_lane_from_value(selected_lane),
        "vehicle_state": _as_json_mapping(vehicle_state),
        "advice": _as_json_mapping(advice),
        "objects": payload_objects,
    }


class RadarUDPSender:
    """Small UDP sender wrapper so the socket can be reused across frames."""

    def __init__(
        self,
        ip: str = app_config.LAPTOP_IP,
        port: int = app_config.UDP_PORT,
        enabled: bool = app_config.UDP_ENABLED,
    ):
        self.enabled = bool(enabled)
        self.address = (str(ip), int(port))
        self._socket: Optional[socket.socket] = None
        if self.enabled:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            logger.info("UDP sender enabled target=%s:%s", self.address[0], self.address[1])
        else:
            logger.info("UDP sender disabled")

    def send_payload(self, payload: Dict) -> bool:
        if not self.enabled or self._socket is None:
            return False
        try:
            encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self._socket.sendto(encoded, self.address)
            return True
        except Exception as exc:
            logger.warning("UDP send failed: %s", exc)
            return False

    def send_radar_result(
        self,
        frame_id: int,
        timestamp: float,
        objects: Iterable[Dict],
        turn_signal,
        selected_lane,
        vehicle_state=None,
        advice=None,
    ) -> bool:
        payload = build_radar_result_payload(
            frame_id=frame_id,
            timestamp=timestamp,
            objects=objects,
            turn_signal=turn_signal,
            selected_lane=selected_lane,
            vehicle_state=vehicle_state,
            advice=advice,
        )
        return self.send_payload(payload)

    def close(self) -> None:
        if self._socket is None:
            return
        try:
            self._socket.close()
        finally:
            self._socket = None


_default_sender: Optional[RadarUDPSender] = None


def get_default_sender() -> RadarUDPSender:
    global _default_sender
    if _default_sender is None:
        _default_sender = RadarUDPSender()
    return _default_sender


def prepare_udp_objects(tracks: Iterable[Dict], lane_result) -> List[Dict]:
    """Merge EKF tracks with lane/TTC/risk fields for UDP transmission."""
    return _merge_lane_objects(tracks, lane_result)


def send_radar_result(
    frame_id: int,
    timestamp: float,
    objects: Iterable[Dict],
    turn_signal,
    selected_lane,
    vehicle_state=None,
    advice=None,
) -> bool:
    """Convenience function used by Raspberry Pi main loops."""
    return get_default_sender().send_radar_result(
        frame_id=frame_id,
        timestamp=timestamp,
        objects=objects,
        turn_signal=turn_signal,
        selected_lane=selected_lane,
        vehicle_state=vehicle_state,
        advice=advice,
    )


def close_default_sender() -> None:
    global _default_sender
    if _default_sender is not None:
        _default_sender.close()
        _default_sender = None
