# -*- coding: utf-8 -*-
"""Adaptive cruise control recommendation using radar tracks and MISO state."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional

from config import (
    ACC_CRUISE_SPEED_KP,
    ACC_CRUISE_SPEED_MPS,
    ACC_DISTANCE_KP,
    ACC_MAX_ACCEL_MPS2,
    ACC_MAX_DECEL_MPS2,
    ACC_MAX_LOOKAHEAD_M,
    ACC_MAX_PATH_OFFSET_M,
    ACC_PATH_HALF_WIDTH_M,
    ACC_RECOMMENDATION_HORIZON_SEC,
    ACC_RELATIVE_SPEED_KD,
    ACC_STANDSTILL_GAP_M,
    ACC_STEERING_SIGN,
    ACC_TIME_HEADWAY_SEC,
    ACC_TTC_CAUTION_SEC,
    ACC_TTC_EMERGENCY_SEC,
    ACC_USE_CONFIRMED_TRACKS_ONLY,
    ACC_WHEELBASE_M,
    EGO_MAX_REASONABLE_SPEED_MPS,
    RADIAL_VELOCITY_NEGATIVE_IS_CLOSING,
)


@dataclass
class ACCRecommendation:
    current_speed_mps: float  
    current_steering_angle_deg: float
    recommended_speed_mps: float
    recommended_accel_mps2: float
    lead_object_id: Optional[int]
    lead_distance_m: Optional[float]
    lead_relative_speed_mps: Optional[float]
    safe_distance_m: float
    ttc_sec: Optional[float]
    reason: str


def _finite(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _track_id(track: Dict) -> Optional[int]:
    try:
        return int(track.get("track_id", track.get("id")))
    except (TypeError, ValueError):
        return None


def _relative_gap_speed(track: Dict) -> float:
    """Return gap-rate velocity: negative means that the object is approaching."""
    if "vy" in track:
        vy = _finite(track.get("vy"))
        radial = track.get("radial_velocity", track.get("v"))
        if abs(vy) > 1e-3 or radial is None:
            return vy

    radial = _finite(track.get("radial_velocity", track.get("v", 0.0)))
    return radial if RADIAL_VELOCITY_NEGATIVE_IS_CLOSING else -radial


class AdaptiveCruiseController:
    """Calculate a longitudinal speed recommendation without actuating the car."""

    def _path_center_x(self, distance_y: float, steering_angle_deg: float) -> float:
        steering_rad = math.radians(steering_angle_deg * ACC_STEERING_SIGN)
        curvature = math.tan(steering_rad) / max(ACC_WHEELBASE_M, 1e-3)
        path_x = 0.5 * curvature * distance_y * distance_y
        return _clamp(path_x, -ACC_MAX_PATH_OFFSET_M, ACC_MAX_PATH_OFFSET_M)

    def _select_lead(
        self,
        tracks: Iterable[Dict],
        steering_angle_deg: float,
    ) -> Optional[Dict]:
        candidates = []
        for source in tracks or []:
            track = dict(source)
            if ACC_USE_CONFIRMED_TRACKS_ONLY and track.get("status") != "confirmed":
                continue

            y = _finite(track.get("y"))
            if y <= 0.0 or y > ACC_MAX_LOOKAHEAD_M:
                continue

            x = _finite(track.get("x"))
            path_x = self._path_center_x(y, steering_angle_deg)
            if abs(x - path_x) > ACC_PATH_HALF_WIDTH_M:
                continue

            track["_acc_distance_m"] = y
            track["_acc_path_error_m"] = abs(x - path_x)
            candidates.append(track)

        if not candidates:
            return None
        return min(candidates, key=lambda item: (item["_acc_distance_m"], item["_acc_path_error_m"]))

    def update(
        self,
        tracks: Iterable[Dict],
        ego_speed_mps: float,
        steering_angle_deg: float,
    ) -> ACCRecommendation:
        ego_speed = _clamp(_finite(ego_speed_mps), 0.0, EGO_MAX_REASONABLE_SPEED_MPS)
        steering_angle = _finite(steering_angle_deg)
        # acc_standstill_gap : 정차 시 앞차와의 최소 간격 (m)
        # acc_time_headway : 주행 시 앞차와의 안전 시간 간격 (s)
        safe_distance = ACC_STANDSTILL_GAP_M + ACC_TIME_HEADWAY_SEC * ego_speed
        lead = self._select_lead(tracks, steering_angle)

        if lead is None:
            accel = ACC_CRUISE_SPEED_KP * (ACC_CRUISE_SPEED_MPS - ego_speed)
            reason = "cruise_speed_control"
            lead_id = None
            distance = None
            relative_speed = None
            ttc = None
        else:
            lead_id = _track_id(lead)
            distance = _finite(lead.get("_acc_distance_m"))
            relative_speed = _relative_gap_speed(lead)
            distance_error = distance - safe_distance
            accel = (
                ACC_DISTANCE_KP * distance_error
                + ACC_RELATIVE_SPEED_KD * relative_speed
            )
            closing_speed = max(0.0, -relative_speed)
            ttc = distance / closing_speed if closing_speed > 1e-3 else None
            reason = "lead_vehicle_following"

            if ttc is not None and ttc <= ACC_TTC_EMERGENCY_SEC:
                accel = -ACC_MAX_DECEL_MPS2
                reason = "emergency_ttc"
            elif ttc is not None and ttc <= ACC_TTC_CAUTION_SEC:
                accel = min(accel, -0.5 * ACC_MAX_DECEL_MPS2)
                reason = "caution_ttc"

        accel = _clamp(accel, -ACC_MAX_DECEL_MPS2, ACC_MAX_ACCEL_MPS2)
        recommended_speed = ego_speed + accel * ACC_RECOMMENDATION_HORIZON_SEC
        recommended_speed = _clamp(
            recommended_speed,
            0.0,
            EGO_MAX_REASONABLE_SPEED_MPS,
        )

        return ACCRecommendation(
            current_speed_mps=ego_speed,
            current_steering_angle_deg=steering_angle,
            recommended_speed_mps=recommended_speed,
            recommended_accel_mps2=accel,
            lead_object_id=lead_id,
            lead_distance_m=distance,
            lead_relative_speed_mps=relative_speed,
            safe_distance_m=safe_distance,
            ttc_sec=ttc,
            reason=reason,
        )
