import time
import serial


def open_cli_port(cfg):
    """CLI 포트를 열고 반환."""
    cli = serial.Serial(cfg["cli_port"], cfg["baud_cli"], timeout=cfg["cli_timeout"])
    time.sleep(1)
    print(f"CLI 포트 열림: {cfg['cli_port']}", flush=True)
    return cli


def open_data_port(cfg, retry_timeout_sec=5.0):
    """Data 포트를 열고 반환."""
    deadline = time.monotonic() + retry_timeout_sec
    last_exc = None

    while time.monotonic() < deadline:
        try:
            data = serial.Serial(cfg["data_port"], cfg["baud_data"], timeout=cfg["read_timeout"])
            data.reset_input_buffer()
            print(f"Data 포트 열림: {cfg['data_port']}", flush=True)
            return data
        except serial.SerialException as exc:
            last_exc = exc
            time.sleep(0.1)

    raise last_exc


def open_serial_ports(cfg):
    """Serial 포트를 열고 반환."""
    cli = open_cli_port(cfg)
    data = open_data_port(cfg)
    return data, cli


def read_cli_response(cli, timeout_sec=2.0):
    deadline = time.monotonic() + timeout_sec
    responses = []

    while time.monotonic() < deadline:
        if cli.in_waiting > 0:
            resp = cli.readline().decode("utf-8", errors="ignore").strip()
            if not resp:
                continue
            responses.append(resp)
            print(f"응답: {resp}", flush=True)
            if resp == "Done" or resp.startswith("Error"):
                break
        else:
            time.sleep(0.01)

    return responses


def send_cfg(cli, cfg_file):
    """cfg 파일을 CLI 포트로 전송."""
    print("cfg 전송 시작...", flush=True)
    with open(cfg_file, "r", encoding="utf-8", errors="ignore") as f:
        cfg_lines = f.readlines()

    for line in cfg_lines:
        line = line.strip()
        if line == "" or line.startswith("%"):
            continue

        cli.reset_input_buffer()
        cli.write((line + "\r\n").encode("utf-8"))
        cli.flush()
        print(f"보냄: {line}", flush=True)

        responses = read_cli_response(cli, timeout_sec=4.0 if line == "sensorStart" else 2.0)
        if any(resp.startswith("Error") for resp in responses):
            raise RuntimeError(f"cfg command failed: {line}")

        time.sleep(0.03)

    print("cfg 전송 완료 / 레이더 시작", flush=True)
    time.sleep(0.2)
