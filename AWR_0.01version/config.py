# =========================
# 전역 상수 (설정값)
# =========================
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40
MAX_PACKET_LEN = 65535

# [DBSCAN] 클러스터링 파라미터
DBSCAN_EPS = 0.3
DBSCAN_MIN_SAMPLES = 10
MIN_RANGE = 0.5
MAX_RANGE = 10.0

# 속도 필터링 임계값 (m/s)
VELOCITY_THRESHOLD = 0.2
Y_DISTANCE_THRESHOLD = 0.2
X_RANGE = 0.5

# [그래프 고정] 화면 좌표 범위
VIEW_X_MIN, VIEW_X_MAX = -5.0, 5.0
VIEW_Y_MIN, VIEW_Y_MAX = 0.0, 10.0


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