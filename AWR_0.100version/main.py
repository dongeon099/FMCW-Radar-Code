import matplotlib.pyplot as plt
import time
from config import build_config, DT_DEFAULT, DT_MIN, DT_MAX
from serial_io import open_serial_ports, send_cfg
from parser import read_packet_buffer, parse_tlv_points
from processing import assign_track_ids, dbscan_scattering, extract_clusters,velocity_filter
from visualizer import visualize_points

def main():
    cfg = build_config()

    data, cli = open_serial_ports(cfg)
    send_cfg(cli, cfg["cfg_file"])

    plt.ion()
    fig, ax = plt.subplots()

    buffer = bytearray()
    prev_frame_ts = None
    prev_tracks = []
    next_track_id = 1

    try:
        while True:
            try:
                packet = read_packet_buffer(data, buffer)

                if packet is None:
                    continue

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
                    continue
###################### points 와 감지 객체는 정확하다고 보고 processing으로 넘어가는 구간 ########################
                ## df: 모든 점 label: dbscan후 같은 객체 묶은 것 filtered_points : 제한된 거리 안의 점
                df, labels, x, y, filtered_points = dbscan_scattering(points)

                if df is None:
                    continue
                # cluster_centroid_objects: centroid 된 좌표 , nearest_obj : centroid 중 제일 가까운 점 
                cluster_centroid_objects = extract_clusters(filtered_points, labels)
                
                tracked_objects, prev_tracks, next_track_id = assign_track_ids(
                    cluster_centroid_objects,
                    prev_tracks,
                    next_track_id,
                )

                velocity_obj = velocity_filter(tracked_objects) # y축 속도 필터링된 객체들 

                visualize_points(
                    fig,
                    ax,
                    df,
                    labels,
                    x,
                    y,
                    num_detected_obj,
                    tracked_objects,
                    velocity_obj
                )

            except Exception as e:
                print(f"프레임 처리 중 오류 발생: {e}")

    except KeyboardInterrupt:
        print("\n사용자 중지")

    finally:
        data.close()
        cli.close()
        print("Serial 포트 닫힘")


if __name__ == "__main__":
    main()
