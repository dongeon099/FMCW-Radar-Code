"""AWR6843 serial parser and visualizer.

MATLAB awr6843_simul.m 을 Python으로 변환한 코드.
필요 패키지:
  pip install pyserial numpy matplotlib pandas scikit-learn
"""

import struct
import time

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
from sklearn.cluster import DBSCAN


# =========================
# 전역 상수 (설정값)
# =========================
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40
MAX_PACKET_LEN = 65535

# [DBSCAN] 클러스터링 파라미터
DBSCAN_EPS = 0.4
DBSCAN_MIN_SAMPLES = 3

# [그래프 고정] 화면 좌표 범위
VIEW_X_MIN, VIEW_X_MAX = -6.0, 6.0
VIEW_Y_MIN, VIEW_Y_MAX = 0.0, 12.0


def build_config():
    """실행 설정을 한 곳에서 관리."""
    return {
        "cli_port": "COM5",
        "data_port": "COM4",
        "baud_cli": 115200,
        "baud_data": 921600,
        "cfg_file": r"C:\ti\mmwave_sdk_03_06_02_00-LTS\packages\ti\demo\xwr68xx\mmw\profiles\profile_2d.cfg",
        "read_timeout": 0.01,
        "cli_timeout": 0.5,
    }


def open_serial_ports(cfg):
    """Serial 포트를 열고 반환."""
    data = serial.Serial(cfg["data_port"], cfg["baud_data"], timeout=cfg["read_timeout"])
    data.reset_input_buffer()
    print("Data 포트 열림")

    cli = serial.Serial(cfg["cli_port"], cfg["baud_cli"], timeout=cfg["cli_timeout"])
    time.sleep(1)
    return data, cli


def send_cfg(cli, cfg_file):
    """cfg 파일을 CLI 포트로 전송."""
    print("cfg 전송 시작...")
    with open(cfg_file, "r", encoding="utf-8", errors="ignore") as f:
        cfg_lines = f.readlines()

    for line in cfg_lines:
        line = line.strip()
        if line == "" or line.startswith("%"):
            continue

        cli.write((line + "\n").encode("utf-8"))
        print(f"보냄: {line}")
        time.sleep(0.05)

        while cli.in_waiting > 0:
            resp = cli.readline().decode("utf-8", errors="ignore").strip()
            if resp:
                print(f"응답: {resp}")

        time.sleep(0.05)

    print("cfg 전송 완료 / 레이더 시작")
    time.sleep(0.005)


def read_packet_buffer(data, buffer):
    """수신 버퍼에서 완전한 packet 1개를 반환. 없으면 None."""
    if data.in_waiting > 0:
        buffer.extend(data.read(data.in_waiting))

    print(f"현재 buffer 길이 : {len(buffer)}")

    if len(buffer) < HEADER_LEN:
        time.sleep(0.005)
        return None

    idx = buffer.find(MAGIC_WORD)
    if idx == -1:
        if len(buffer) > 5000:
            del buffer[:-1000]
        return None

    del buffer[:idx]
    if len(buffer) < HEADER_LEN:
        return None

    header = buffer[:HEADER_LEN]
    total_packet_len = struct.unpack_from("<I", header, 12)[0]

    if total_packet_len < HEADER_LEN or total_packet_len > MAX_PACKET_LEN:
        del buffer[:8]
        return None

    if len(buffer) < total_packet_len:
        time.sleep(0.005)
        return None

    packet = bytes(buffer[:total_packet_len])
    del buffer[:total_packet_len]
    return packet


def parse_tlv_points(packet):
    """패킷에서 points 와 헤더 정보 파싱."""
    num_detected_obj = struct.unpack_from("<I", packet, 28)[0]
    num_tlvs = struct.unpack_from("<I", packet, 32)[0]

    offset = HEADER_LEN
    points = []

    for _ in range(num_tlvs):
        if offset + 8 > len(packet):
            break

        tlv_type = struct.unpack_from("<I", packet, offset)[0]
        tlv_len = struct.unpack_from("<I", packet, offset + 4)[0]

        payload_start = offset + 8
        payload_end = offset + tlv_len

        if tlv_len < 8 or payload_end > len(packet):
            print("TLV 길이 이상 - 현재 패킷 스킵")
            break

        payload = packet[payload_start:payload_end]

        if tlv_type == 1:
            point_size = 16
            n_points = len(payload) // point_size
            for i in range(n_points):
                base = i * point_size
                if base + point_size > len(payload):
                    break
                x, y, z, v = struct.unpack_from("<ffff", payload, base)
                points.append([x, y, z, v])

        offset += tlv_len

    return points, num_detected_obj


def visualize_points(fig, ax, points, num_detected_obj):
    """점군 결과 출력 + 시각화."""
    points = np.array(points, dtype=float)
    x, y, z, v = points[:, 0], points[:, 1], points[:, 2], points[:, 3]
    distance = np.sqrt(x**2 + y**2 + z**2)

    xy = points[:, :2]
    labels = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit_predict(xy)

    df = pd.DataFrame(
        {
            "X_m": x,
            "Y_m": y,
            "Z_m": z,
            "Distance_m": distance,
            "Velocity_mps": v,
            "ClusterID": labels,
        }
    )

    print("\033c", end="")
    print("===== AWR6843 Detected Objects =====")
    print(f"Detected objects(header): {num_detected_obj}")
    print(df)

    unique_labels = sorted(set(labels))
    cluster_count = len([lb for lb in unique_labels if lb != -1])
    noise_count = int(np.sum(labels == -1))
    print(f"클러스터 수(DBSCAN): {cluster_count}, 노이즈 포인트: {noise_count}")

    ax.clear()
    sc = ax.scatter(x, y, s=60, c=labels, cmap="tab20")
    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_title("AWR6843 Position / DBSCAN Cluster")
    ax.grid(True)
    ax.set_xlim(VIEW_X_MIN, VIEW_X_MAX)
    ax.set_ylim(VIEW_Y_MIN, VIEW_Y_MAX)
    ax.set_aspect("equal", adjustable="box")

    if not hasattr(fig, "_awr_colorbar"):
        fig._awr_colorbar = plt.colorbar(sc, ax=ax)
        fig._awr_colorbar.set_label("Cluster ID (-1: noise)")
    else:
        fig._awr_colorbar.update_normal(sc)

    plt.pause(0.001)


def main():
    """프로그램 진입점."""
    cfg = build_config()
    data, cli = open_serial_ports(cfg)

    send_cfg(cli, cfg["cfg_file"])

    plt.ion()
    fig, ax = plt.subplots()

    buffer = bytearray()

    try:
        while True:
            packet = read_packet_buffer(data, buffer)
            if packet is None:
                continue

            points, num_detected_obj = parse_tlv_points(packet)
            if points:
                visualize_points(fig, ax, points, num_detected_obj)

    except KeyboardInterrupt:
        print("\n사용자 중지")
    finally:
        data.close()
        cli.close()
        print("Serial 포트 닫힘")


if __name__ == "__main__":
    main()