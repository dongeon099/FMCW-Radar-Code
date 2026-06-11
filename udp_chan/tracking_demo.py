# -*- coding: utf-8 -*-
"""Sensor-free demo for the tracking/lane/SPI pipeline.

실제 레이더가 없어도 다음 흐름을 확인할 수 있다.
Detection sequence -> EKF tracker -> Lane risk decision -> Mock SPI packet
"""

from __future__ import annotations

import logging
from typing import Dict, List

from lane_decision import LaneRiskDecision
from spi_sender import SPISender
from tracking import ObjectTracker


logger = logging.getLogger(__name__)


def build_demo_detection_sequence() -> List[List[Dict]]:
    """가상의 frame별 detection 목록을 만든다.

    - frame 0~11: 왼쪽 차선 물체가 접근한다.
    - frame 5~11: 오른쪽 차선 물체도 접근한다.
    - 이후 frame: detection이 사라져 missed/delete 동작을 확인한다.
    """
    frames: List[List[Dict]] = []

    for frame_index in range(12):
        frames.append(
            [
                {
                    "x": -0.55,
                    "y": 8.0 - frame_index * 0.45,
                    "z": 0.0,
                    "v": -2.0,
                    "cluster_id": 100,
                    "timestamp": frame_index * 0.1,
                }
            ]
        )

    for frame_index in range(5, 12):
        frames[frame_index].append(
            {
                "x": 0.55,
                "y": 7.0 - (frame_index - 5) * 0.4,
                "z": 0.0,
                "v": -1.6,
                "cluster_id": 200,
                "timestamp": frame_index * 0.1,
            }
        )

    for _ in range(7):
        frames.append([])

    return frames


def run_tracking_demo() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    tracker = ObjectTracker()
    lane_decision = LaneRiskDecision()
    spi_sender = SPISender(use_mock=True)
    dt = 0.1

    for frame_id, detections in enumerate(build_demo_detection_sequence()):
        tracks = tracker.update(detections, dt=dt, timestamp=frame_id * dt)
        lane_result = lane_decision.update(tracks)
        packet = spi_sender.build_packet(
            left_risk=lane_result.left_risk,
            right_risk=lane_result.right_risk,
            left_count=len(lane_result.left_objects),
            right_count=len(lane_result.right_objects),
        )
        spi_sender.send_packet(packet)

        logger.info(
            "frame=%02d detections=%d tracks=%s left_risk=%d right_risk=%d",
            frame_id,
            len(detections),
            [
                {
                    "id": track["track_id"],
                    "x": round(track["x"], 2),
                    "y": round(track["y"], 2),
                    "status": track["status"],
                    "missed": track["missed_count"],
                }
                for track in tracks
            ],
            lane_result.left_risk,
            lane_result.right_risk,
        )

    spi_sender.close()


if __name__ == "__main__":
    run_tracking_demo()
