import matplotlib.pyplot as plt

from config import build_config
from serial_io import open_serial_ports, send_cfg
from parser import read_packet_buffer, parse_tlv_points
from processing import dbscan_scattering, extract_clusters,velocity_filter
from visualizer import visualize_points
from moving_avg_filter import MovingAvergeFilter

def main():
    cfg = build_config()

    data, cli = open_serial_ports(cfg)
    send_cfg(cli, cfg["cfg_file"])

    plt.ion()
    fig, ax = plt.subplots()

    buffer = bytearray()
    


    try:
        while True:
            try:
                packet = read_packet_buffer(data, buffer)

                if packet is None:
                    continue

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
                velocity_obj = velocity_filter(cluster_centroid_objects) # y축 속도 필터링된 객체들 

                visualize_points(
                    fig,
                    ax,
                    df,
                    labels,
                    x,
                    y,
                    num_detected_obj,
                    cluster_centroid_objects,
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
