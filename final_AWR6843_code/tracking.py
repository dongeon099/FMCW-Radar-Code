# -*- coding: utf-8 -*-
"""Lightweight tracking-by-detection pipeline.

이 모듈은 기존 DBSCAN/객체 추출 결과를 그대로 입력으로 받아서
현업에서 많이 쓰는 "Detection -> Association -> Track 관리" 흐름을 추가한다.
센서/클러스터링 코드는 건드리지 않고, 매 프레임 생성된 detection dict만 넣으면 된다.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment as _linear_sum_assignment
except Exception:  # scipy가 없는 임베디드/경량 환경을 위한 fallback.
    _linear_sum_assignment = None

from config import (
    ASSOCIATION_DISTANCE_THRESHOLD,
    ASSOCIATION_USE_MAHALANOBIS,
    DT_DEFAULT,
    DT_MAX,
    DT_MIN,
    EKF_INITIAL_POSITION_VARIANCE,
    EKF_INITIAL_VELOCITY_VARIANCE,
    EKF_MEASUREMENT_NOISE_POSITION,
    EKF_MEASUREMENT_NOISE_VELOCITY,
    EKF_PROCESS_NOISE_POSITION,
    EKF_PROCESS_NOISE_VELOCITY,
    MAX_MISSED_FRAMES,
    MIN_HITS_TO_CONFIRM,
)


logger = logging.getLogger(__name__)


TRACK_TENTATIVE = "tentative"
TRACK_CONFIRMED = "confirmed"
TRACK_DELETED = "deleted"


def _as_float(value, default: float = 0.0) -> float:
    """None/문자열/NaN이 섞여도 추적기가 죽지 않도록 float로 안전 변환한다."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _clamp_dt(dt: Optional[float]) -> float:
    """프레임 시간이 비정상적으로 튀면 EKF 예측이 흔들리므로 config 범위로 제한한다."""
    if dt is None:
        return DT_DEFAULT
    dt = _as_float(dt, DT_DEFAULT)
    if dt <= 0.0:
        return DT_DEFAULT
    return max(DT_MIN, min(dt, DT_MAX))


def _safe_detection(detection: Dict) -> Dict:
    """기존 코드의 dict 구조를 유지하면서 추적에 필요한 필드를 보정한다."""
    x = _as_float(detection.get("x"))
    y = _as_float(detection.get("y"))
    z = _as_float(detection.get("z"))
    distance = detection.get("distance")
    if distance is None:
        distance = math.sqrt(x * x + y * y + z * z)

    copied = dict(detection)
    copied["x"] = x
    copied["y"] = y
    copied["z"] = z
    copied["distance"] = _as_float(distance)
    copied["v"] = _as_float(detection.get("v", detection.get("velocity", 0.0)))
    return copied


def _measurement_from_detection(detection: Dict) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """detection에서 EKF 측정 벡터 z, 측정 행렬 H, 측정 노이즈 R을 만든다.

    센서가 x/y 위치만 주면 [x, y]만 업데이트한다.
    vx/vy가 명시적으로 들어온 경우에만 [x, y, vx, vy] 업데이트를 사용한다.
    radial velocity(v)는 방향 정보가 부족하므로 EKF의 vx/vy 측정값으로 억지 변환하지 않는다.
    """
    x = _as_float(detection.get("x"))
    y = _as_float(detection.get("y"))

    has_vx_vy = detection.get("vx") is not None and detection.get("vy") is not None
    if has_vx_vy:
        vx = _as_float(detection.get("vx"))
        vy = _as_float(detection.get("vy"))
        z = np.array([x, y, vx, vy], dtype=float)
        h = np.eye(4, dtype=float)
        r = np.diag(
            [
                EKF_MEASUREMENT_NOISE_POSITION,
                EKF_MEASUREMENT_NOISE_POSITION,
                EKF_MEASUREMENT_NOISE_VELOCITY,
                EKF_MEASUREMENT_NOISE_VELOCITY,
            ]
        )
        return z, h, r

    z = np.array([x, y], dtype=float)
    h = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    r = np.diag([EKF_MEASUREMENT_NOISE_POSITION, EKF_MEASUREMENT_NOISE_POSITION])
    return z, h, r


def _fallback_linear_sum_assignment(cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """scipy가 없을 때 쓰는 단순 greedy 매칭.

    Hungarian Algorithm처럼 전역 최적을 보장하지는 않지만, 임베디드 환경에서
    dependency 없이 동작해야 할 때를 위한 안전장치다.
    """
    if cost_matrix.size == 0:
        return np.array([], dtype=int), np.array([], dtype=int)

    pairs = []
    for row in range(cost_matrix.shape[0]):
        for col in range(cost_matrix.shape[1]):
            cost = cost_matrix[row, col]
            if math.isfinite(float(cost)):
                pairs.append((float(cost), row, col))

    pairs.sort(key=lambda item: item[0])
    used_rows = set()
    used_cols = set()
    selected_rows = []
    selected_cols = []

    for _, row, col in pairs:
        if row in used_rows or col in used_cols:
            continue
        used_rows.add(row)
        used_cols.add(col)
        selected_rows.append(row)
        selected_cols.append(col)

    return np.array(selected_rows, dtype=int), np.array(selected_cols, dtype=int)


def solve_assignment(cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Hungarian Algorithm을 우선 사용하고, scipy가 없으면 fallback을 사용한다."""
    if cost_matrix.size == 0:
        return np.array([], dtype=int), np.array([], dtype=int)
    if not np.isfinite(cost_matrix).any():
        return np.array([], dtype=int), np.array([], dtype=int)
    if _linear_sum_assignment is None:
        logger.debug("scipy is not available; using greedy association fallback")
        return _fallback_linear_sum_assignment(cost_matrix)

    # scipy는 inf만 있는 행/열이 섞이면 infeasible 오류가 날 수 있다.
    # 원본 cost_matrix는 유지하고, solver에만 큰 숫자로 바꾼 행렬을 전달한다.
    finite_costs = cost_matrix[np.isfinite(cost_matrix)]
    large_cost = float(finite_costs.max() + 1_000_000.0)
    solver_matrix = np.where(np.isfinite(cost_matrix), cost_matrix, large_cost)
    try:
        return _linear_sum_assignment(solver_matrix)
    except ValueError:
        logger.debug("Hungarian solver failed; using greedy association fallback")
        return _fallback_linear_sum_assignment(cost_matrix)


@dataclass
class AssociationResult:
    matches: List[Tuple[int, int]]
    unmatched_track_indices: List[int]
    unmatched_detection_indices: List[int]


class EKFTrack:
    """한 개 물체에 대한 EKF 상태와 track metadata를 보관한다."""

    def __init__(
        self,
        track_id: int,
        detection: Dict,
        timestamp: Optional[float] = None,
        min_hits: int = MIN_HITS_TO_CONFIRM,
        max_missed: int = MAX_MISSED_FRAMES,
    ):
        detection = _safe_detection(detection)

        self.track_id = int(track_id)
        self.min_hits = int(min_hits)
        self.max_missed = int(max_missed)
        self.state = np.array(
            [
                detection["x"],
                detection["y"],
                0.0,
                0.0,
            ],
            dtype=float,
        )
        self.covariance = np.diag(
            [
                EKF_INITIAL_POSITION_VARIANCE,
                EKF_INITIAL_POSITION_VARIANCE,
                EKF_INITIAL_VELOCITY_VARIANCE,
                EKF_INITIAL_VELOCITY_VARIANCE,
            ]
        ).astype(float)

        self.age = 1
        self.hits = 1
        self.missed_count = 0
        self.status = TRACK_TENTATIVE
        self.last_update_time = timestamp
        self.lane_label = "unknown"
        self.risk_level = 0
        self.last_detection = detection
        self.radial_velocity = detection.get("v", 0.0)
        self.predicted_state = self.state.copy()
        self.updated_state = self.state.copy()
        self.updated_this_frame = True

    def predict(self, dt: float) -> None:
        """등속 운동 모델로 다음 프레임 위치를 예측한다."""
        dt = _clamp_dt(dt)
        f = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        # Q는 "모델이 얼마나 틀릴 수 있는지"를 나타낸다.
        # 위치/속도 노이즈를 분리해 두면 센서 특성에 맞게 쉽게 조정할 수 있다.
        q = np.diag(
            [
                EKF_PROCESS_NOISE_POSITION * dt,
                EKF_PROCESS_NOISE_POSITION * dt,
                EKF_PROCESS_NOISE_VELOCITY * dt,
                EKF_PROCESS_NOISE_VELOCITY * dt,
            ]
        )

        self.state = f @ self.state
        self.covariance = f @ self.covariance @ f.T + q
        self.predicted_state = self.state.copy()
        self.updated_this_frame = False
        self.age += 1

    def update(self, detection: Dict, timestamp: Optional[float] = None) -> None:
        """현재 detection으로 EKF 상태를 보정한다."""
        detection = _safe_detection(detection)
        z, h, r = _measurement_from_detection(detection)

        innovation = z - h @ self.state
        s = h @ self.covariance @ h.T + r
        k = self.covariance @ h.T @ np.linalg.pinv(s)

        self.state = self.state + k @ innovation
        identity = np.eye(4, dtype=float)
        self.covariance = (identity - k @ h) @ self.covariance
        self.updated_state = self.state.copy()
        self.updated_this_frame = True

        self.hits += 1
        self.missed_count = 0
        self.last_update_time = timestamp
        self.last_detection = detection
        self.radial_velocity = detection.get("v", self.radial_velocity)

        if self.hits >= self.min_hits:
            self.status = TRACK_CONFIRMED

    def mark_missed(self) -> None:
        """이번 프레임에서 매칭되지 않은 track의 missed count를 증가시킨다."""
        self.missed_count += 1
        if self.missed_count > self.max_missed:
            self.status = TRACK_DELETED

    def position_distance(self, detection: Dict) -> float:
        detection = _safe_detection(detection)
        return float(np.hypot(self.state[0] - detection["x"], self.state[1] - detection["y"]))

    def mahalanobis_distance(self, detection: Dict) -> float:
        """예측 위치와 detection 사이의 Mahalanobis distance를 계산한다."""
        detection = _safe_detection(detection)
        h = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        r = np.diag([EKF_MEASUREMENT_NOISE_POSITION, EKF_MEASUREMENT_NOISE_POSITION])
        innovation = np.array([detection["x"], detection["y"]], dtype=float) - h @ self.state
        s = h @ self.covariance @ h.T + r
        return float(math.sqrt(max(0.0, innovation.T @ np.linalg.pinv(s) @ innovation)))

    def to_dict(self) -> Dict:
        """기존 코드와 연결하기 쉽게 dict 형태로 track을 내보낸다."""
        x, y, vx, vy = [float(value) for value in self.state]
        z = _as_float(self.last_detection.get("z", 0.0))
        distance = _as_float(self.last_detection.get("distance"), math.sqrt(x * x + y * y + z * z))
        speed = float(math.hypot(vx, vy))

        result = {
            "track_id": self.track_id,
            "x": x,
            "y": y,
            "z": z,
            "vx": vx,
            "vy": vy,
            "v": _as_float(self.radial_velocity),
            "radial_velocity": _as_float(self.radial_velocity),
            "speed": speed,
            "distance": distance,
            "age": self.age,
            "hits": self.hits,
            "missed_count": self.missed_count,
            "status": self.status,
            "lane_label": self.lane_label,
            "risk_level": self.risk_level,
            "last_update_time": self.last_update_time,
            "covariance": self.covariance.copy(),
        }
        if self.predicted_state is not None:
            result["predicted_x"] = float(self.predicted_state[0])
            result["predicted_y"] = float(self.predicted_state[1])
        if self.updated_this_frame and self.updated_state is not None:
            result["updated_x"] = float(self.updated_state[0])
            result["updated_y"] = float(self.updated_state[1])
        return result


class ObjectTracker:
    """여러 EKFTrack을 관리하는 Tracking-by-Detection 관리자."""

    def __init__(
        self,
        min_hits: int = MIN_HITS_TO_CONFIRM,
        max_missed: int = MAX_MISSED_FRAMES,
        association_distance_threshold: float = ASSOCIATION_DISTANCE_THRESHOLD,
        use_mahalanobis: bool = ASSOCIATION_USE_MAHALANOBIS,
    ):
        self.min_hits = int(min_hits)
        self.max_missed = int(max_missed)
        self.association_distance_threshold = float(association_distance_threshold)
        self.use_mahalanobis = bool(use_mahalanobis)
        self.tracks: List[EKFTrack] = []
        self.next_track_id = 1

    def _build_cost_matrix(self, detections: Sequence[Dict]) -> np.ndarray:
        if not self.tracks or not detections:
            return np.empty((len(self.tracks), len(detections)), dtype=float)

        cost_matrix = np.full((len(self.tracks), len(detections)), np.inf, dtype=float)
        for track_index, track in enumerate(self.tracks):
            for detection_index, detection in enumerate(detections):
                if self.use_mahalanobis:
                    cost = track.mahalanobis_distance(detection)
                else:
                    cost = track.position_distance(detection)

                # gating: 너무 멀리 떨어진 detection은 같은 객체 후보에서 제외한다.
                if cost <= self.association_distance_threshold:
                    cost_matrix[track_index, detection_index] = cost

        return cost_matrix

    def _associate(self, detections: Sequence[Dict]) -> AssociationResult:
        cost_matrix = self._build_cost_matrix(detections)
        row_indices, col_indices = solve_assignment(cost_matrix)

        matches: List[Tuple[int, int]] = []
        matched_track_indices = set()
        matched_detection_indices = set()

        for row, col in zip(row_indices, col_indices):
            cost = cost_matrix[row, col]
            if not math.isfinite(float(cost)):
                continue
            matches.append((int(row), int(col)))
            matched_track_indices.add(int(row))
            matched_detection_indices.add(int(col))

        unmatched_tracks = [
            index for index in range(len(self.tracks)) if index not in matched_track_indices
        ]
        unmatched_detections = [
            index for index in range(len(detections)) if index not in matched_detection_indices
        ]

        logger.debug(
            "association matches=%s unmatched_tracks=%s unmatched_detections=%s",
            matches,
            unmatched_tracks,
            unmatched_detections,
        )
        return AssociationResult(matches, unmatched_tracks, unmatched_detections)

    def update(
        self,
        detections: Optional[Iterable[Dict]],
        dt: Optional[float] = None,
        timestamp: Optional[float] = None,
    ) -> List[Dict]:
        """매 프레임 호출하는 메인 함수.

        Args:
            detections: 기존 DBSCAN/클러스터링에서 나온 객체 dict 리스트.
            dt: 이전 프레임과 현재 프레임 사이 시간.
            timestamp: 현재 프레임 시각. 없으면 None으로 기록한다.
        """
        dt = _clamp_dt(dt)
        safe_detections = [_safe_detection(detection) for detection in (detections or [])]

        for track in self.tracks:
            track.predict(dt)

        association = self._associate(safe_detections)

        for track_index, detection_index in association.matches:
            track = self.tracks[track_index]
            detection = safe_detections[detection_index]
            track.update(detection, timestamp=timestamp)

        for track_index in association.unmatched_track_indices:
            self.tracks[track_index].mark_missed()

        for detection_index in association.unmatched_detection_indices:
            self._create_track(safe_detections[detection_index], timestamp)

        self._remove_deleted_tracks()
        return [track.to_dict() for track in self.tracks if track.status != TRACK_DELETED]

    def _create_track(self, detection: Dict, timestamp: Optional[float]) -> None:
        track = EKFTrack(
            self.next_track_id,
            detection,
            timestamp=timestamp,
            min_hits=self.min_hits,
            max_missed=self.max_missed,
        )
        self.tracks.append(track)
        logger.info(
            "created track_id=%s x=%.2f y=%.2f",
            track.track_id,
            track.state[0],
            track.state[1],
        )
        self.next_track_id += 1

    def _remove_deleted_tracks(self) -> None:
        alive_tracks: List[EKFTrack] = []
        for track in self.tracks:
            if track.status == TRACK_DELETED:
                logger.info(
                    "deleted track_id=%s age=%s hits=%s missed=%s",
                    track.track_id,
                    track.age,
                    track.hits,
                    track.missed_count,
                )
                continue
            alive_tracks.append(track)
        self.tracks = alive_tracks
