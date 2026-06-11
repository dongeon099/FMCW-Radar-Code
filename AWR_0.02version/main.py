import matplotlib.pyplot as plt

from config import build_config
from serial_io import open_serial_ports, send_cfg
from parser import read_packet_buffer, parse_tlv_points
from processing import dbscan_scattering, extract_clusters
from visualizer import visualize_points
from moving_avg_filter import MovingAvergeFilter

def main():
    cfg = build_config()

    data, cli = open_serial_ports(cfg)
    send_cfg(cli, cfg["cfg_file"])

    plt.ion()
    fig, ax = plt.subplots()

    buffer = bytearray()
    ### 이동평균 필터 실험
    nearest_filter = MovingAvergeFilter(window_size = 5)


    try:
        while True:
            try:
                packet = read_packet_buffer(data, buffer)

                if packet is None:
                    continue

                points, num_detected_obj = parse_tlv_points(packet)

                if not points:
                    continue

                df, labels, x, y, filtered_points = dbscan_scattering(points)

                if df is None:
                    continue

                cluster_objects, nearest_obj = extract_clusters(filtered_points, labels)
                nearest_obj = nearest_filter.update(nearest_obj) # 이동평균 필터 실험 
                visualize_points(
                    fig,
                    ax,
                    df,
                    labels,
                    x,
                    y,
                    num_detected_obj,
                    cluster_objects,
                    nearest_obj
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
