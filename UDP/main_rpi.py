# -*- coding: utf-8 -*-
"""Raspberry Pi entrypoint: radar processing pipeline with UDP laptop output.

Flow:
    TLV parsing -> DBSCAN -> EKF tracking -> lane/TTC/risk -> SPI -> UDP send

The Raspberry Pi does not run the visualizer here. The laptop receives the UDP
JSON packets and performs visualization in laptop_visualizer.py.
"""
# 라즈베리파이에서는 시각화가 필요 없으므로 visualizer 관련 코드는 main_rpi.py에서 제외했습니다.
from __future__ import annotations

import logging
import time

from config import DT_DEFAULT, DT_MAX, DT_MIN, build_config
from lane_change_advisor import LaneChangeAdvisor
from lane_decision import LaneRiskDecision
from main import update_lane_change_advice, update_lane_risk_and_spi
from network_sender import RadarUDPSender, prepare_udp_objects
from parser import parse_tlv_points, read_packet_buffer
from processing import assign_track_ids, dbscan_scattering, extract_clusters, velocity_filter
from serial_io import open_serial_ports, send_cfg
from spi_sender import LANE_ID_NONE, SPISender
from tracking import ObjectTracker


logger = logging.getLogger(__name__)


def selected_lane_from_spi_packet(spi_packet) -> int:
    """Backward-compatible parser for the old 33-byte SPI packet format."""
    if spi_packet and len(spi_packet) > 7:
        try:
            return int(spi_packet[7])
        except (TypeError, ValueError):
            return LANE_ID_NONE
    return LANE_ID_NONE


def send_udp_result(
    udp_sender,
    frame_id,
    timestamp,
    tracks,
    lane_result,
    turn_signal,
    selected_lane,
    recommended_speed_mps=None,
):
    udp_objects = prepare_udp_objects(tracks, lane_result)
    udp_sender.send_radar_result(
        frame_id=frame_id,
        timestamp=timestamp,
        objects=udp_objects,
        turn_signal=turn_signal,
        selected_lane=selected_lane,
        recommended_speed_mps=recommended_speed_mps,
    )


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    cfg = build_config()

    data, cli = open_serial_ports(cfg)
    send_cfg(cli, cfg["cfg_file"])

    buffer = bytearray()
    prev_frame_ts = None
    prev_tracks = []
    next_track_id = 1
    tracker = ObjectTracker()
    lane_decision = LaneRiskDecision()
    advisor = LaneChangeAdvisor()
    spi_sender = SPISender()
    udp_sender = RadarUDPSender()
    frame_id = 0

    try:
        while True:
            try:
                packet = read_packet_buffer(data, buffer)
                if packet is None:
                    continue

                now_ts = time.monotonic()
                if prev_frame_ts is None:
                    dt = DT_DEFAULT
                else:
                    dt = max(DT_MIN, min(now_ts - prev_frame_ts, DT_MAX))
                prev_frame_ts = now_ts

                points, _num_detected_obj = parse_tlv_points(packet)

                if not points:
                    tracks = tracker.update([], dt=dt, timestamp=now_ts)
                    lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                    advice = update_lane_change_advice(
                        advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                    )
                    send_udp_result(
                        udp_sender,
                        frame_id,
                        now_ts,
                        tracks,
                        lane_result,
                        spi_sender.last_turn_signal,
                        spi_sender.last_selected_lane,
                        advice.acc_recommended_speed_mps,
                    )
                    frame_id = (frame_id + 1) & 0xFFFFFFFF
                    continue

                df, labels, _x, _y, filtered_points = dbscan_scattering(points)

                if df is None:
                    tracks = tracker.update([], dt=dt, timestamp=now_ts)
                    lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                    advice = update_lane_change_advice(
                        advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                    )
                    send_udp_result(
                        udp_sender,
                        frame_id,
                        now_ts,
                        tracks,
                        lane_result,
                        spi_sender.last_turn_signal,
                        spi_sender.last_selected_lane,
                        advice.acc_recommended_speed_mps,
                    )
                    frame_id = (frame_id + 1) & 0xFFFFFFFF
                    continue

                cluster_centroid_objects = extract_clusters(filtered_points, labels)

                tracked_objects, prev_tracks, next_track_id = assign_track_ids(
                    cluster_centroid_objects,
                    prev_tracks,
                    next_track_id,
                    dt=dt,
                )

                tracks = tracker.update(cluster_centroid_objects, dt=dt, timestamp=now_ts)
                lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                advice = update_lane_change_advice(
                    advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                )

                velocity_filter(tracked_objects)

                send_udp_result(
                    udp_sender,
                    frame_id,
                    now_ts,
                    tracks,
                    lane_result,
                    spi_sender.last_turn_signal,
                    spi_sender.last_selected_lane,
                    advice.acc_recommended_speed_mps,
                )
                frame_id = (frame_id + 1) & 0xFFFFFFFF

            except Exception as exc:
                logger.exception("frame processing error: %s", exc)
                print(f"프레임 처리 중 오류 발생: {exc}")

    except KeyboardInterrupt:
        print("\n사용자 중지")

    finally:
        udp_sender.close()
        spi_sender.close()
        data.close()
        cli.close()
        print("Serial/UDP 포트 닫힘")


if __name__ == "__main__":
    main()
