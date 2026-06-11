# -*- coding: utf-8 -*-
"""Lane-change advisory layer built on top of lane risk decisions.

이 모듈은 기존 DBSCAN -> tracking -> lane risk -> SPI 흐름을 바꾸지 않고,
LaneDecisionResult와 track dict를 입력으로 받아 차선 변경 가능성을 별도 출력으로 만든다.
"""

from __future__ import annotations

import logging
import math
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Union

from acc_controller import ACCRecommendation, AdaptiveCruiseController
from config import (
    DT_DEFAULT,
    DT_MIN,
    EGO_MAX_REASONABLE_ACCEL_MPS2,
    EGO_MAX_REASONABLE_SPEED_MPS,
    EGO_SPEED_DEFAULT_MPS,
    LANE_CHANGE_ACCEL_ALPHA,
    LANE_CHANGE_MIN_REQUIRED_GAP_M,
    LANE_CHANGE_SAFE_GAP_M,
    LANE_CHANGE_TIME_SEC,
    RADIAL_VELOCITY_NEGATIVE_IS_CLOSING,
    TURN_SIGNAL_HAZARD,
    TURN_SIGNAL_INVALID,
    TURN_SIGNAL_LEFT,
    TURN_SIGNAL_NONE,
    TURN_SIGNAL_RIGHT,
)


logger = logging.getLogger(__name__)


RISK_CLEAR = 0
RISK_CAUTION = 1
RISK_WARNING = 2


@dataclass
class LaneChangeAdvice:
    turn_signal: Union[int, str]
    target_lane: str
    lane_change_possible: bool
    risk_level: int
    reason: str
    ego_current_speed_mps: float
    ego_required_speed_mps: float
    ego_required_accel_mps2: float
    target_object_id: Optional[int]
    target_object_distance_m: Optional[float]
    target_object_velocity_mps: Optional[float]
    target_object_accel_mps2: Optional[float]
    predicted_gap_after_lane_change_m: Optional[float]
    required_safe_gap_m: float
    current_steering_angle_deg: float = 0.0
    acc_recommended_speed_mps: float = 0.0
    acc_recommended_accel_mps2: float = 0.0
    acc_lead_object_id: Optional[int] = None
    acc_lead_distance_m: Optional[float] = None
    acc_lead_relative_speed_mps: Optional[float] = None
    acc_safe_distance_m: float = 0.0
    acc_ttc_sec: Optional[float] = None
    acc_reason: str = "not_calculated"

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class _TrackKinematicsHistory:
    previous_vy: Optional[float] = None
    previous_radial_velocity: Optional[float] = None
    smoothed_accel: float = 0.0
    last_timestamp: Optional[float] = None


def _as_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _finite_or_none(value) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(result):
        return None
    return result


def _safe_dt(dt: Optional[float]) -> float:
    value = _as_float(dt, DT_DEFAULT)
    if value <= 0.0:
        return DT_DEFAULT
    return max(DT_MIN, value)


def _track_id(obj: Dict) -> Optional[int]:
    value = obj.get("track_id", obj.get("id"))
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _front_distance(obj: Dict) -> float:
    y_distance = _as_float(obj.get("y"), 0.0)
    if y_distance > 0.0:
        return y_distance

    distance = _as_float(obj.get("distance"), 0.0)
    if distance > 0.0:
        return distance

    x = _as_float(obj.get("x"), 0.0)
    return math.hypot(x, y_distance)


def _radial_to_gap_velocity(radial_velocity: Optional[float]) -> Optional[float]:
    if radial_velocity is None:
        return None
    if RADIAL_VELOCITY_NEGATIVE_IS_CLOSING:
        return radial_velocity
    return -radial_velocity


def _raw_radial_velocity(obj: Dict) -> Optional[float]:
    return _finite_or_none(obj.get("radial_velocity", obj.get("v")))


def _gap_velocity(obj: Dict) -> float:
    """현재 좌표계에서 gap이 줄어드는 속도는 음수로 맞춘다."""
    vy = _finite_or_none(obj.get("vy")) if "vy" in obj else None
    radial = _raw_radial_velocity(obj)

    # EKF 초기 구간의 vy=0보다 radar radial velocity가 더 의미 있을 수 있어 fallback한다.
    if vy is not None and (abs(vy) > 1e-3 or radial is None or abs(radial) <= 1e-3):
        return vy

    radial_gap_velocity = _radial_to_gap_velocity(radial)
    if radial_gap_velocity is not None:
        return radial_gap_velocity
    if vy is not None:
        return vy
    return 0.0


def _closing_speed(obj: Dict) -> float:
    return max(0.0, -_gap_velocity(obj))


def _ttc(obj: Dict) -> Optional[float]:
    distance = _front_distance(obj)
    closing = _closing_speed(obj)
    if distance <= 0.0 or closing <= 0.0:
        return None
    return distance / closing


def _risk_level(obj: Dict) -> int:
    try:
        return max(RISK_CLEAR, min(RISK_WARNING, int(obj.get("risk_level", RISK_CLEAR))))
    except (TypeError, ValueError):
        return RISK_CLEAR


def _signal_name(turn_signal: Union[int, str]) -> str:
    if isinstance(turn_signal, str):
        text = turn_signal.strip().upper()
        if text in ("L", "LEFT", "TURN_LEFT"):
            return "LEFT"
        if text in ("R", "RIGHT", "TURN_RIGHT"):
            return "RIGHT"
        if text in ("H", "HAZARD", "BOTH"):
            return "HAZARD"
        if text in ("NONE", "NO", "OFF", "0", ""):
            return "NONE"
        return text

    try:
        value = int(turn_signal)
    except (TypeError, ValueError):
        return "INVALID"

    names = {
        TURN_SIGNAL_NONE: "NONE",
        TURN_SIGNAL_LEFT: "LEFT",
        TURN_SIGNAL_RIGHT: "RIGHT",
        TURN_SIGNAL_HAZARD: "HAZARD",
        TURN_SIGNAL_INVALID: "INVALID",
    }
    return names.get(value, f"UNKNOWN({value})")


def _target_lane_from_signal(signal_name: str) -> str:
    if signal_name == "LEFT":
        return "left"
    if signal_name == "RIGHT":
        return "right"
    if signal_name == "HAZARD":
        return "hazard"
    return "none"


class LaneChangeAdvisor:
    """Turn signal 기반 차선 변경 가능성 판단 레이어."""

    def __init__(
        self,
        lane_change_time_sec: float = LANE_CHANGE_TIME_SEC,
        safe_gap_m: float = LANE_CHANGE_SAFE_GAP_M,
        min_required_gap_m: float = LANE_CHANGE_MIN_REQUIRED_GAP_M,
        accel_alpha: float = LANE_CHANGE_ACCEL_ALPHA,
        max_reasonable_accel_mps2: float = EGO_MAX_REASONABLE_ACCEL_MPS2,
        max_reasonable_speed_mps: float = EGO_MAX_REASONABLE_SPEED_MPS,
    ):
        self.lane_change_time_sec = max(DT_MIN, _as_float(lane_change_time_sec, LANE_CHANGE_TIME_SEC))
        self.safe_gap_m = max(0.0, _as_float(safe_gap_m, LANE_CHANGE_SAFE_GAP_M))
        self.min_required_gap_m = max(0.0, _as_float(min_required_gap_m, LANE_CHANGE_MIN_REQUIRED_GAP_M))
        self.accel_alpha = max(0.0, min(1.0, _as_float(accel_alpha, LANE_CHANGE_ACCEL_ALPHA)))
        self.max_reasonable_accel_mps2 = max(
            0.0, _as_float(max_reasonable_accel_mps2, EGO_MAX_REASONABLE_ACCEL_MPS2)
        )
        self.max_reasonable_speed_mps = max(
            0.0, _as_float(max_reasonable_speed_mps, EGO_MAX_REASONABLE_SPEED_MPS)
        )
        self._history_by_track_id: Dict[int, _TrackKinematicsHistory] = {}
        self._last_log_signature: Optional[Tuple] = None
        self.acc_controller = AdaptiveCruiseController()

    def update(
        self,
        turn_signal: Union[int, str],
        lane_result,
        tracks: Optional[Iterable[Dict]],
        ego_current_speed_mps: float = EGO_SPEED_DEFAULT_MPS,
        current_steering_angle_deg: float = 0.0,
        dt: Optional[float] = None,
        timestamp: Optional[float] = None,
    ) -> LaneChangeAdvice:
        """차선 변경 advice를 계산한다.

        입력은 기존 pipeline에서 이미 생성한 lane_result/tracks dict를 그대로 사용한다.
        """
        safe_dt = _safe_dt(dt)
        ego_speed = max(0.0, _as_float(ego_current_speed_mps, EGO_SPEED_DEFAULT_MPS))
        required_safe_gap = max(self.safe_gap_m, self.min_required_gap_m)
        track_list = [dict(track) for track in (tracks or [])]
        acc_recommendation = self.acc_controller.update(
            tracks=track_list,
            ego_speed_mps=ego_speed,
            steering_angle_deg=current_steering_angle_deg,
        )
        track_by_id = {
            track_id: track
            for track in track_list
            for track_id in [_track_id(track)]
            if track_id is not None
        }

        signal_name = _signal_name(turn_signal)
        target_lane = _target_lane_from_signal(signal_name)

        if signal_name == "NONE":
            self._touch_histories(track_list, safe_dt, timestamp, already_touched=set())
            advice = self._base_advice(
                turn_signal=signal_name,
                target_lane=target_lane,
                ego_speed=ego_speed,
                required_safe_gap=required_safe_gap,
                possible=False,
                reason="not_requested",
            )
            self._apply_acc(advice, acc_recommendation)
            self._log_advice(advice)
            return advice

        if signal_name == "HAZARD":
            self._touch_histories(track_list, safe_dt, timestamp, already_touched=set())
            advice = self._base_advice(
                turn_signal=signal_name,
                target_lane=target_lane,
                ego_speed=ego_speed,
                required_safe_gap=required_safe_gap,
                possible=False,
                reason="hazard_not_supported",
            )
            self._apply_acc(advice, acc_recommendation)
            self._log_advice(advice)
            return advice

        if signal_name not in ("LEFT", "RIGHT"):
            self._touch_histories(track_list, safe_dt, timestamp, already_touched=set())
            advice = self._base_advice(
                turn_signal=signal_name,
                target_lane=target_lane,
                ego_speed=ego_speed,
                required_safe_gap=required_safe_gap,
                possible=False,
                reason="invalid_turn_signal",
            )
            self._apply_acc(advice, acc_recommendation)
            self._log_advice(advice)
            return advice

        lane_objects = self._objects_for_lane(lane_result, target_lane, track_by_id)
        if not lane_objects:
            self._touch_histories(track_list, safe_dt, timestamp, already_touched=set())
            advice = self._base_advice(
                turn_signal=signal_name,
                target_lane=target_lane,
                ego_speed=ego_speed,
                required_safe_gap=required_safe_gap,
                possible=True,
                reason="no_target_objects",
            )
            self._apply_acc(advice, acc_recommendation)
            self._log_advice(advice)
            return advice

        touched_ids = set()
        enriched_objects: List[Dict] = []
        for obj in lane_objects:
            copied = dict(obj)
            accel, touched_id = self._estimate_acceleration(copied, safe_dt, timestamp)
            copied["target_object_accel_mps2"] = accel
            if touched_id is not None:
                touched_ids.add(touched_id)
            enriched_objects.append(copied)

        self._touch_histories(track_list, safe_dt, timestamp, already_touched=touched_ids)
        target = self._select_target_object(enriched_objects)
        advice = self._build_advice(
            turn_signal=signal_name,
            target_lane=target_lane,
            target=target,
            ego_speed=ego_speed,
            required_safe_gap=required_safe_gap,
        )
        self._apply_acc(advice, acc_recommendation)
        self._log_advice(advice)
        return advice

    def _apply_acc(
        self,
        advice: LaneChangeAdvice,
        recommendation: ACCRecommendation,
    ) -> None:
        advice.current_steering_angle_deg = recommendation.current_steering_angle_deg
        advice.acc_recommended_speed_mps = recommendation.recommended_speed_mps
        advice.acc_recommended_accel_mps2 = recommendation.recommended_accel_mps2
        advice.acc_lead_object_id = recommendation.lead_object_id
        advice.acc_lead_distance_m = recommendation.lead_distance_m
        advice.acc_lead_relative_speed_mps = recommendation.lead_relative_speed_mps
        advice.acc_safe_distance_m = recommendation.safe_distance_m
        advice.acc_ttc_sec = recommendation.ttc_sec
        advice.acc_reason = recommendation.reason

        # ACC is the longitudinal safety limit even during a lane-change request.
        advice.ego_required_speed_mps = min(
            advice.ego_required_speed_mps,
            recommendation.recommended_speed_mps,
        )
        advice.ego_required_accel_mps2 = recommendation.recommended_accel_mps2

    def _base_advice(
        self,
        turn_signal: Union[int, str],
        target_lane: str,
        ego_speed: float,
        required_safe_gap: float,
        possible: bool,
        reason: str,
    ) -> LaneChangeAdvice:
        return LaneChangeAdvice(
            turn_signal=turn_signal,
            target_lane=target_lane,
            lane_change_possible=bool(possible),
            risk_level=RISK_CLEAR,
            reason=reason,
            ego_current_speed_mps=ego_speed,
            ego_required_speed_mps=ego_speed,
            ego_required_accel_mps2=0.0,
            target_object_id=None,
            target_object_distance_m=None,
            target_object_velocity_mps=None,
            target_object_accel_mps2=None,
            predicted_gap_after_lane_change_m=None,
            required_safe_gap_m=required_safe_gap,
        )

    def _objects_for_lane(self, lane_result, target_lane: str, track_by_id: Dict[int, Dict]) -> List[Dict]:
        if target_lane == "left":
            objects = getattr(lane_result, "left_objects", []) or []
        elif target_lane == "right":
            objects = getattr(lane_result, "right_objects", []) or []
        else:
            objects = []

        merged_objects: List[Dict] = []
        for obj in objects:
            copied = dict(obj)
            track_id = _track_id(copied)
            if track_id is not None and track_id in track_by_id:
                merged = dict(track_by_id[track_id])
                merged.update(copied)
                copied = merged
            merged_objects.append(copied)
        return merged_objects

    def _select_target_object(self, objects: List[Dict]) -> Dict:
        def priority(obj: Dict) -> Tuple[int, float, float, float]:
            risk = _risk_level(obj)
            ttc = obj.get("ttc")
            if ttc is None or not math.isfinite(_as_float(ttc, float("inf"))):
                ttc = _ttc(obj)
            ttc_value = float(ttc) if ttc is not None and math.isfinite(float(ttc)) else 9999.0
            distance = _front_distance(obj)
            closing = _closing_speed(obj)
            return (-risk, ttc_value, distance, -closing)

        return sorted(objects, key=priority)[0]

    def _estimate_acceleration(
        self,
        obj: Dict,
        dt: float,
        timestamp: Optional[float],
    ) -> Tuple[float, Optional[int]]:
        track_id = _track_id(obj)
        if track_id is None:
            return 0.0, None

        history = self._history_by_track_id.get(track_id)
        if history is None:
            history = _TrackKinematicsHistory()
            self._history_by_track_id[track_id] = history

        if timestamp is not None and history.last_timestamp is not None:
            effective_dt = _safe_dt(timestamp - history.last_timestamp)
        else:
            effective_dt = _safe_dt(dt)

        current_vy = _finite_or_none(obj.get("vy")) if "vy" in obj else None
        current_radial = _radial_to_gap_velocity(_raw_radial_velocity(obj))

        measured_accel: Optional[float] = None
        use_vy = (
            current_vy is not None
            and history.previous_vy is not None
            and (
                abs(current_vy) > 1e-3
                or abs(history.previous_vy) > 1e-3
                or current_radial is None
                or abs(current_radial) <= 1e-3
            )
        )
        if use_vy:
            measured_accel = (current_vy - history.previous_vy) / effective_dt
        elif current_radial is not None and history.previous_radial_velocity is not None:
            measured_accel = (current_radial - history.previous_radial_velocity) / effective_dt

        if measured_accel is None or not math.isfinite(measured_accel):
            measured_accel = history.smoothed_accel

        smoothed_accel = (
            self.accel_alpha * measured_accel
            + (1.0 - self.accel_alpha) * history.smoothed_accel
        )
        if not math.isfinite(smoothed_accel):
            smoothed_accel = 0.0

        history.previous_vy = current_vy
        history.previous_radial_velocity = current_radial
        history.smoothed_accel = smoothed_accel
        history.last_timestamp = timestamp
        return smoothed_accel, track_id

    def _touch_histories(
        self,
        tracks: Iterable[Dict],
        dt: float,
        timestamp: Optional[float],
        already_touched: set,
    ) -> None:
        active_ids = set(already_touched)
        for track in tracks:
            track_id = _track_id(track)
            if track_id is None:
                continue
            active_ids.add(track_id)
            if track_id in already_touched:
                continue
            self._estimate_acceleration(track, dt, timestamp)

        stale_ids = [track_id for track_id in self._history_by_track_id if track_id not in active_ids]
        for track_id in stale_ids:
            self._history_by_track_id.pop(track_id, None)

    def _build_advice(
        self,
        turn_signal: str,
        target_lane: str,
        target: Dict,
        ego_speed: float,
        required_safe_gap: float,
    ) -> LaneChangeAdvice:
        target_id = _track_id(target)
        distance = _front_distance(target)
        target_velocity = _gap_velocity(target)
        target_accel = _as_float(target.get("target_object_accel_mps2"), 0.0)
        risk = _risk_level(target)
        time_sec = self.lane_change_time_sec

        # 발표용 1차 모델: ego speed로 확보되는 longitudinal gap 증가분과
        # 상대 차량의 gap 방향 속도/가속도를 같은 축에서 합산한다.
        predicted_gap = (
            distance
            + (ego_speed + target_velocity) * time_sec
            + 0.5 * target_accel * time_sec * time_sec
        )
        if not math.isfinite(predicted_gap):
            predicted_gap = 0.0

        if predicted_gap >= required_safe_gap:
            return LaneChangeAdvice(
                turn_signal=turn_signal,
                target_lane=target_lane,
                lane_change_possible=True,
                risk_level=risk,
                reason="safe_gap_available",
                ego_current_speed_mps=ego_speed,
                ego_required_speed_mps=ego_speed,
                ego_required_accel_mps2=0.0,
                target_object_id=target_id,
                target_object_distance_m=distance,
                target_object_velocity_mps=target_velocity,
                target_object_accel_mps2=target_accel,
                predicted_gap_after_lane_change_m=predicted_gap,
                required_safe_gap_m=required_safe_gap,
            )

        required_speed = (
            required_safe_gap
            - distance
            - target_velocity * time_sec
            - 0.5 * target_accel * time_sec * time_sec
        ) / time_sec
        required_speed = max(ego_speed, 0.0, required_speed)
        if not math.isfinite(required_speed):
            required_speed = self.max_reasonable_speed_mps + 1.0

        required_accel = (required_speed - ego_speed) / time_sec
        if not math.isfinite(required_accel):
            required_accel = self.max_reasonable_accel_mps2 + 1.0

        possible = True
        reason = "speed_adjustment_required"
        risk = max(risk, RISK_CAUTION)

        if predicted_gap < self.min_required_gap_m:
            risk = max(risk, RISK_WARNING)
        if required_accel > self.max_reasonable_accel_mps2:
            possible = False
            reason = "required_acceleration_too_high"
            risk = max(risk, RISK_WARNING)
        elif required_speed > self.max_reasonable_speed_mps:
            possible = False
            reason = "required_speed_too_high"
            risk = max(risk, RISK_WARNING)

        return LaneChangeAdvice(
            turn_signal=turn_signal,
            target_lane=target_lane,
            lane_change_possible=possible,
            risk_level=risk,
            reason=reason,
            ego_current_speed_mps=ego_speed,
            ego_required_speed_mps=required_speed,
            ego_required_accel_mps2=required_accel,
            target_object_id=target_id,
            target_object_distance_m=distance,
            target_object_velocity_mps=target_velocity,
            target_object_accel_mps2=target_accel,
            predicted_gap_after_lane_change_m=predicted_gap,
            required_safe_gap_m=required_safe_gap,
        )

    def _log_advice(self, advice: LaneChangeAdvice) -> None:
        signature = (
            advice.turn_signal,
            advice.lane_change_possible,
            advice.reason,
            advice.target_object_id,
            round(_as_float(advice.predicted_gap_after_lane_change_m), 2)
            if advice.predicted_gap_after_lane_change_m is not None
            else None,
        )
        if signature == self._last_log_signature:
            return
        self._last_log_signature = signature

        logger.info(
            "[ADVICE] turn=%s possible=%s reason=%s ego=%.2f req_speed=%.2f "
            "req_accel=%.2f target_id=%s gap_future=%s safe_gap=%.2f",
            advice.turn_signal,
            advice.lane_change_possible,
            advice.reason,
            advice.ego_current_speed_mps,
            advice.ego_required_speed_mps,
            advice.ego_required_accel_mps2,
            advice.target_object_id,
            f"{advice.predicted_gap_after_lane_change_m:.2f}"
            if advice.predicted_gap_after_lane_change_m is not None
            else "-",
            advice.required_safe_gap_m,
        )
