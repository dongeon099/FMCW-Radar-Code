import logging
import math
import time
import config as app_config
from config import build_config, DT_DEFAULT, DT_MIN, DT_MAX
from serial_io import open_cli_port, open_data_port, send_cfg
from parser import read_packet_buffer, parse_tlv_points
from processing import assign_track_ids, dbscan_scattering, extract_clusters,velocity_filter
from tracking import ObjectTracker
from lane_decision import LaneRiskDecision
from lane_change_advisor import LaneChangeAdvisor
from spi_sender import SPISender
from advanced_visualizer import create_visualizer

# 기존 로컬 디버깅에서 사용하면 되는 코드입니다. 라즈베리파이에서 실행할 때는 이 파일을 사용하세요.
logger = logging.getLogger(__name__)


def update_lane_risk_and_spi(lane_decision, spi_sender, tracks):
    """트래킹 결과를 좌/우 위험도로 바꾸고 STM32용 SPI 패킷으로 송신한다."""
    lane_result = lane_decision.update(tracks)
    packet = spi_sender.transfer_lane_result(lane_result, tracks=tracks)
    return lane_result, packet


def get_ego_current_speed_mps(cfg, spi_sender=None):
    """정상 SPI 수신값을 우선 사용하고, 없으면 config 기본값을 사용한다."""
    if spi_sender is not None and getattr(spi_sender, "last_miso_valid", False):
        speed = getattr(spi_sender, "last_ego_speed_mps", None)
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = None
        if speed is not None and math.isfinite(speed) and speed >= 0.0:
            return speed

    default_speed = getattr(app_config, "EGO_SPEED_DEFAULT_MPS", 0.0)
    value = cfg.get("ego_current_speed_mps", default_speed) if isinstance(cfg, dict) else default_speed
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return float(default_speed)
    if not math.isfinite(speed) or speed < 0.0:
        return float(default_speed)
    return speed


def get_current_steering_angle_deg(spi_sender=None):
    """정상 SPI 수신값이 있을 때 현재 조향각을 반환한다."""
    if spi_sender is None or not getattr(spi_sender, "last_miso_valid", False):
        return 0.0
    try:
        angle = float(getattr(spi_sender, "last_steering_angle_deg", 0.0))
    except (TypeError, ValueError):
        return 0.0
    return angle if math.isfinite(angle) else 0.0


def update_lane_change_advice(advisor, spi_sender, lane_result, tracks, cfg, dt, timestamp):
    """SPI turn signal과 lane_result를 이용해 차선 변경 advice를 별도 계산한다."""
    return advisor.update(
        turn_signal=spi_sender.last_turn_signal,
        lane_result=lane_result,
        tracks=tracks,
        ego_current_speed_mps=get_ego_current_speed_mps(cfg, spi_sender),
        current_steering_angle_deg=get_current_steering_angle_deg(spi_sender),
        dt=dt,
        timestamp=timestamp,
    )


def update_visualizer_safe(
    visualizer,
    detections,
    clusters,
    tracks,
    lane_result,
    spi_packet,
    frame_id,
    dt,
    processing_time_ms,
    advice=None,
):
    """Visualizer errors should never stop sensor processing."""
    if visualizer is None:
        return
    try:
        visualizer.update(
            detections=detections,
            clusters=clusters,
            tracks=tracks,
            lane_result=lane_result,
            spi_packet=spi_packet,
            advice=advice,
            frame_id=frame_id,
            dt=dt,
            processing_time_ms=processing_time_ms,
        )
    except Exception as e:
        logger.warning("Visualizer update failed: %s", e)

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    cfg = build_config()

    data = None
    cli = None
    spi_sender = None
    visualizer = None

    cli = open_cli_port(cfg)
    send_cfg(cli, cfg["cfg_file"])
    data = open_data_port(cfg)

    buffer = bytearray()
    prev_frame_ts = None
    prev_tracks = []
    next_track_id = 1
    tracker = ObjectTracker()
    lane_decision = LaneRiskDecision()
    advisor = LaneChangeAdvisor()
    spi_sender = SPISender()
    visualizer = create_visualizer(app_config)
    frame_id = 0

    try:
        while True:
            try:
                packet = read_packet_buffer(data, buffer)

                if packet is None:
                    continue

                frame_start_time = time.perf_counter()

                """  
                ## EKF 적용 전 준비: 프레임 시간 간격(dt) 계산
                - AWR6843은 일정한 주기로 레이더 프레임을 출력한다.
                - 한 프레임과 다음 프레임 사이의 시간 간격을 `dt`라고 한다.
                - 일반적으로 프레임 간격은 약 0.05~0.1초 정도로 볼 수 있다.
                - EKF의 예측 단계는 `dt`를 기준으로 물체의 다음 위치를 예측한다.
                - 따라서 `dt`가 너무 작거나 너무 크면 EKF 예측값이 불안정해질 수 있다.

                ### dt 제한 기준

                - 실제 프레임 간격이 `0.001초`보다 작으면 `dt = 0.001`로 고정한다.
                - 실제 프레임 간격이 `0.2초`보다 크면 `dt = 0.2`로 고정한다.

                ### 목적

                이렇게 하면 일시적인 프레임 지연이나 시간 측정 오류가 발생해도  
                EKF가 과도하게 빠르거나 느리게 예측하지 않도록 막을 수 있다.
                """
                now_ts = time.monotonic()
                if prev_frame_ts is None:
                    dt = DT_DEFAULT
                else: 
                    dt = now_ts - prev_frame_ts
                    dt = max(DT_MIN, min(dt, DT_MAX))
                prev_frame_ts = now_ts
                

                points, num_detected_obj = parse_tlv_points(packet)

                if not points:
                    # Detection이 없는 프레임도 tracker에 알려야 missed_count가 증가하고
                    # 오래 사라진 track이 정상적으로 삭제된다.
                    tracks = tracker.update([], dt=dt, timestamp=now_ts)
                    lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                    advice = update_lane_change_advice(
                        advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                    )
                    processing_time_ms = (time.perf_counter() - frame_start_time) * 1000.0
                    update_visualizer_safe(
                        visualizer,
                        detections=[],
                        clusters=[],
                        tracks=tracks,
                        lane_result=lane_result,
                        spi_packet=spi_packet,
                        advice=advice,
                        frame_id=frame_id,
                        dt=dt,
                        processing_time_ms=processing_time_ms,
                    )
                    frame_id = (frame_id + 1) & 0xFF
                    continue
###################### points 와 감지 객체는 정확하다고 보고 processing으로 넘어가는 구간 ########################
                ## df: 모든 점 label: dbscan후 같은 객체 묶은 것 filtered_points : 제한된 거리 안의 점
                df, labels, x, y, filtered_points = dbscan_scattering(points)

                if df is None:
                    tracks = tracker.update([], dt=dt, timestamp=now_ts)
                    lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                    advice = update_lane_change_advice(
                        advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                    )
                    processing_time_ms = (time.perf_counter() - frame_start_time) * 1000.0
                    update_visualizer_safe(
                        visualizer,
                        detections=points,
                        clusters=[],
                        tracks=tracks,
                        lane_result=lane_result,
                        spi_packet=spi_packet,
                        advice=advice,
                        frame_id=frame_id,
                        dt=dt,
                        processing_time_ms=processing_time_ms,
                    )
                    frame_id = (frame_id + 1) & 0xFF
                    continue
                # cluster_centroid_objects: centroid 된 좌표 및 속도 정보가 담긴 dict 리스트
                cluster_centroid_objects = extract_clusters(filtered_points, labels)
                
                tracked_objects, prev_tracks, next_track_id = assign_track_ids(
                    cluster_centroid_objects,
                    prev_tracks,
                    next_track_id,
                    dt=dt,
                )

                # 새 Tracking-by-Detection 파이프라인.
                # 기존 cluster_centroid_objects dict를 그대로 detection으로 사용한다.
                tracks = tracker.update(cluster_centroid_objects, dt=dt, timestamp=now_ts)
                lane_result, spi_packet = update_lane_risk_and_spi(lane_decision, spi_sender, tracks)
                advice = update_lane_change_advice(
                    advisor, spi_sender, lane_result, tracks, cfg, dt, now_ts
                )

                # Keep the existing velocity/lane filter alive for compatibility with older code paths.
                velocity_filter(tracked_objects)

                processing_time_ms = (time.perf_counter() - frame_start_time) * 1000.0
                update_visualizer_safe(
                    visualizer,
                    detections=points,
                    clusters=cluster_centroid_objects,
                    tracks=tracks,
                    lane_result=lane_result,
                    spi_packet=spi_packet,
                    advice=advice,
                    frame_id=frame_id,
                    dt=dt,
                    processing_time_ms=processing_time_ms,
                )
                frame_id = (frame_id + 1) & 0xFF

            except (OSError, IOError) as e:
                logger.exception("serial data port error: %s", e)
                print(f"DATA 포트 오류 발생, 프로그램을 중지합니다: {e}")
                break
            except Exception as e:
                logger.exception("frame processing error: %s", e)
                print(f"프레임 처리 중 오류 발생: {e}")

    except KeyboardInterrupt:
        print("\n사용자 중지")

    finally:
        if visualizer is not None:
            visualizer.close()
        if spi_sender is not None:
            spi_sender.close()
        if data is not None:
            data.close()
        if cli is not None:
            cli.close()
        print("Serial 포트 닫힘")


if __name__ == "__main__":
    main()
