# -*- coding: utf-8 -*-
"""Sensor-free demo for LaneChangeAdvisor.

좌측 깜빡이가 켜진 상황을 가정하고, LaneRiskDecision 결과와 track dict만으로
차선 변경 advice가 어떻게 변하는지 확인한다.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Sequence, Tuple

from config import TURN_SIGNAL_LEFT
from lane_change_advisor import LaneChangeAdvice, LaneChangeAdvisor
from lane_decision import LaneRiskDecision


TrackFrame = Dict[str, float]


def _make_track(track_id: int, x: float, y: float, velocity_mps: float) -> Dict:
    return {
        "track_id": track_id,
        "x": x,
        "y": y,
        "z": 0.0,
        "vx": 0.0,
        "vy": velocity_mps,
        "v": velocity_mps,
        "radial_velocity": velocity_mps,
        "distance": max(0.0, y),
        "status": "confirmed",
        "age": 10,
        "hits": 10,
        "missed_count": 0,
    }


def _build_sequence(
    track_id: int,
    initial_y: float,
    velocities_mps: Sequence[float],
    dt: float,
    x: float = -0.55,
) -> List[Dict]:
    y = float(initial_y)
    frames: List[Dict] = []
    for index, velocity in enumerate(velocities_mps):
        if index > 0:
            y += float(velocities_mps[index - 1]) * dt
        frames.append(_make_track(track_id, x=x, y=y, velocity_mps=float(velocity)))
    return frames


def _fmt(value, digits: int = 2) -> str:
    if value is None:
        return "-"
    return f"{float(value):.{digits}f}"


def _print_frame(frame_id: int, advice: LaneChangeAdvice) -> None:
    print(
        "frame={frame:02d} target_id={target} distance={distance}m "
        "velocity={velocity}m/s accel={accel}m/s^2 future_gap={gap}m "
        "required_ego_speed={req_speed}m/s possible={possible} reason={reason}".format(
            frame=frame_id,
            target=advice.target_object_id,
            distance=_fmt(advice.target_object_distance_m),
            velocity=_fmt(advice.target_object_velocity_mps),
            accel=_fmt(advice.target_object_accel_mps2),
            gap=_fmt(advice.predicted_gap_after_lane_change_m),
            req_speed=_fmt(advice.ego_required_speed_mps),
            possible=advice.lane_change_possible,
            reason=advice.reason,
        )
    )


def run_scenario(name: str, frames: Iterable[Dict], ego_speed_mps: float, dt: float) -> None:
    print(f"\n=== {name} | ego_current_speed={ego_speed_mps:.2f} m/s ===")
    lane_decision = LaneRiskDecision()
    advisor = LaneChangeAdvisor()

    for frame_id, track in enumerate(frames):
        tracks = [track]
        lane_result = lane_decision.update(tracks)
        advice = advisor.update(
            turn_signal=TURN_SIGNAL_LEFT,
            lane_result=lane_result,
            tracks=tracks,
            ego_current_speed_mps=ego_speed_mps,
            dt=dt,
            timestamp=frame_id * dt,
        )
        _print_frame(frame_id, advice)


def build_demo_scenarios() -> List[Tuple[str, List[Dict], float, float]]:
    return [
        (
            "1. left lane object approaches at constant speed",
            _build_sequence(track_id=1, initial_y=18.0, velocities_mps=[-3.0] * 6, dt=0.5),
            6.0,
            0.5,
        ),
        (
            "2. left lane object accelerates while approaching",
            _build_sequence(
                track_id=2,
                initial_y=22.0,
                velocities_mps=[-2.0, -3.0, -4.0, -5.0, -6.0, -7.0],
                dt=0.5,
            ),
            6.0,
            0.5,
        ),
        (
            "3. ego speed too low, lane change not reasonable",
            _build_sequence(
                track_id=3,
                initial_y=8.0,
                velocities_mps=[-12.0, -13.0, -14.0, -15.0],
                dt=0.2,
            ),
            2.0,
            0.2,
        ),
        (
            "4. ego speed sufficient for the same target lane object",
            _build_sequence(
                track_id=4,
                initial_y=8.0,
                velocities_mps=[-12.0, -13.0, -14.0, -15.0],
                dt=0.2,
            ),
            14.0,
            0.2,
        ),
    ]


def main() -> None:
    for name, frames, ego_speed_mps, dt in build_demo_scenarios():
        run_scenario(name, frames, ego_speed_mps, dt)


if __name__ == "__main__":
    main()
