import time
import serial

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
        time.sleep(0.01)

        while cli.in_waiting > 0:
            resp = cli.readline().decode("utf-8", errors="ignore").strip()
            if resp:
                print(f"응답: {resp}")

        time.sleep(0.01)

    print("cfg 전송 완료 / 레이더 시작")
    time.sleep(0.001)