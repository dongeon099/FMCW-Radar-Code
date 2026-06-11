# -*- coding: utf-8 -*-
"""Lane-level risk decision for tracked objects.

기존 코드는 x 위치로 lane 1/2/3을 나눈 뒤 속도 필터를 적용한다.
이 모듈은 그 좌/우 차선 개념을 유지하면서, EKF로 유지된 track을 기준으로
left/right 위험도를 CLEAR/CAUTION/WARNING으로 계산한다.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from config import (
    DISTANCE_CAUTION_THRESHOLD,
    DISTANCE_WARNING_THRESHOLD,
    LEFT_LANE_X_RANGE,
    LANE_USE_CONFIRMED_TRACKS_ONLY,
    RADIAL_VELOCITY_NEGATIVE_IS_CLOSING,
    RIGHT_LANE_X_RANGE,
    TTC_CAUTION_THRESHOLD,
    TTC_WARNING_THRESHOLD,
)


logger = logging.getLogger(__name__)


RISK_CLEAR = 0
RISK_CAUTION = 1
RISK_WARNING = 2


@dataclass
class LaneDecisionResult:
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


def _in_range(value: float, range_pair: Tuple[float, float]) -> bool:
    low, high = range_pair
    return float(low) <= value <= float(high)


def _is_confirmed(track: Dict) -> bool:
    return track.get("status") == "confirmed"


def _front_distance(track: Dict) -> float:
    """위험 판단에 사용할 전방 거리.

    현재 좌표계에서는 y가 전방 방향으로 쓰이고 있으므로 y를 우선 사용한다.
    y가 없거나 0에 가까운 특수 데이터에서는 distance 필드로 fallback한다.
    """
    y_distance = _as_float(track.get("y"), 0.0)
    if y_distance > 0.0:
        return y_distance
    return _as_float(track.get("distance"), 0.0)


def _closing_speed(track: Dict) -> float:
    """상대 접근 속도를 계산한다.

    EKF의 vy가 음수이면 y 거리가 줄어드는 상황으로 보고 접근 중으로 판단한다.
    radial velocity만 있는 경우에는 config의 부호 설정을 따르되, EKF 상태로
    억지 변환하지 않고 TTC 계산에만 보조적으로 사용한다.
    """
    vy = _as_float(track.get("vy"), 0.0)
    if vy < 0.0:
        return abs(vy)

    radial_velocity = _as_float(track.get("radial_velocity", track.get("v", 0.0)))
    if RADIAL_VELOCITY_NEGATIVE_IS_CLOSING and radial_velocity < 0.0:
        return abs(radial_velocity)
    if not RADIAL_VELOCITY_NEGATIVE_IS_CLOSING and radial_velocity > 0.0:
        return radial_velocity
    return 0.0


def _ttc(track: Dict) -> Optional[float]:
    distance = _front_distance(track)
    closing_speed = _closing_speed(track)
    if distance <= 0.0 or closing_speed <= 0.0:
        return None
    return distance / closing_speed


def _risk_for_track(track: Dict) -> int:
    distance = _front_distance(track)
    ttc = _ttc(track)

    if distance <= DISTANCE_WARNING_THRESHOLD:
        return RISK_WARNING
    if ttc is not None and ttc <= TTC_WARNING_THRESHOLD:
        return RISK_WARNING

    if distance <= DISTANCE_CAUTION_THRESHOLD:
        return RISK_CAUTION
    if ttc is not None and ttc <= TTC_CAUTION_THRESHOLD:
        return RISK_CAUTION

    return RISK_CLEAR


class LaneRiskDecision:
    """좌측/우측 차선 객체 목록과 위험도를 계산한다."""

    def __init__(self):
        self._last_signature = None

    def update(self, tracks: Iterable[Dict]) -> LaneDecisionResult:
        left_objects: List[Dict] = []
        right_objects: List[Dict] = []

        for track in tracks: # track은 EKF로 유지되는 객체 상태를 가정한다.
            if LANE_USE_CONFIRMED_TRACKS_ONLY and not _is_confirmed(track):
                continue

            enriched = dict(track)
            x = _as_float(enriched.get("x"), 0.0)

            if _in_range(x, LEFT_LANE_X_RANGE):
                enriched["lane_label"] = "left"
                enriched["ttc"] = _ttc(enriched)
                enriched["risk_level"] = _risk_for_track(enriched)
                left_objects.append(enriched)
            elif _in_range(x, RIGHT_LANE_X_RANGE):
                enriched["lane_label"] = "right"
                enriched["ttc"] = _ttc(enriched)
                enriched["risk_level"] = _risk_for_track(enriched)
                right_objects.append(enriched)
            else:
                enriched["lane_label"] = "unknown"
                enriched["ttc"] = _ttc(enriched)
                enriched["risk_level"] = RISK_CLEAR

        left_risk = max((obj["risk_level"] for obj in left_objects), default=RISK_CLEAR)
        right_risk = max((obj["risk_level"] for obj in right_objects), default=RISK_CLEAR)

        result = LaneDecisionResult(
            left_objects=left_objects,
            right_objects=right_objects,
            left_risk=left_risk,
            right_risk=right_risk,
        )
        self._log_if_changed(result)
        return result

    def _log_if_changed(self, result: LaneDecisionResult) -> None:
        signature = (
            result.left_risk,
            result.right_risk,
            len(result.left_objects),
            len(result.right_objects),
        )
        if signature == self._last_signature:
            return
        self._last_signature = signature
        logger.info(
            "lane risk left=%s right=%s left_count=%s right_count=%s",
            result.left_risk,
            result.right_risk,
            len(result.left_objects),
            len(result.right_objects),
        )
