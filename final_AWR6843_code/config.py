# =========================
# 전역 상수 (설정값)
# =========================
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40
MAX_PACKET_LEN = 65535

# [DBSCAN] 클러스터링 파라미터
DBSCAN_EPS = 0.8
DBSCAN_MIN_SAMPLES = 8
MIN_RANGE = 0.5
MAX_RANGE = 10.0

# EKF 예측 단계용 프레임 시간 간격 
DT_DEFAULT = 0.05
DT_MIN = 0.001
DT_MAX = 0.2

#프레임간 최근접 연관 거리 게이트(m)
TRACK_ASSOCIATION_MAX_DISTANCE = 0.8

# =========================
# Tracking-by-Detection settings
# =========================
# 기존 assign_track_ids에서 쓰던 거리 기준을 새 EKF association의 기본값으로 재사용한다.
ASSOCIATION_DISTANCE_THRESHOLD = TRACK_ASSOCIATION_MAX_DISTANCE

# True로 바꾸면 Mahalanobis distance 기반 gating을 사용한다.
# 처음 튜닝할 때는 센서 좌표계에서 직관적인 Euclidean distance(False)가 이해하기 쉽다.
ASSOCIATION_USE_MAHALANOBIS = False

# Track 상태 관리 기준.
MIN_HITS_TO_CONFIRM = 3
MAX_MISSED_FRAMES = 5

# EKF covariance/noise 설정.
# 값이 클수록 "센서/모델이 틀릴 수 있다"고 보고 더 부드럽게 따라간다.
EKF_INITIAL_POSITION_VARIANCE = 1.0
EKF_INITIAL_VELOCITY_VARIANCE = 10.0
EKF_PROCESS_NOISE_POSITION = 0.05
EKF_PROCESS_NOISE_VELOCITY = 0.5
EKF_MEASUREMENT_NOISE_POSITION = 0.2
EKF_MEASUREMENT_NOISE_VELOCITY = 1.0

# 속도 필터링 임계값 (m/s)
VELOCITY_THRESHOLD = 1.2
Y_DISTANCE_THRESHOLD = 1
X_RANGE = 0.5

# =========================
# Lane risk settings
# =========================
# 기존 velocity_filter의 lane split과 맞춘다.
# lane_1: negative x side -> left, lane_3: positive x side -> right.
LEFT_LANE_X_RANGE = (-X_RANGE * 3.0, -X_RANGE)
RIGHT_LANE_X_RANGE = (X_RANGE, X_RANGE * 3.0)

# Confirmed track만 위험 판단에 쓰면 순간 노이즈가 경고로 튀는 일을 줄일 수 있다.
LANE_USE_CONFIRMED_TRACKS_ONLY = True

# TTC(Time To Collision)와 거리 기반 위험도 기준.
TTC_CAUTION_THRESHOLD = 2.0
TTC_WARNING_THRESHOLD = 0.5
DISTANCE_CAUTION_THRESHOLD = 6.0
DISTANCE_WARNING_THRESHOLD = 3.0

# AWR/mmWave radial velocity 부호는 설정에 따라 달라질 수 있다.
# 현재는 radial velocity가 음수이면 접근 중으로 해석한다.
RADIAL_VELOCITY_NEGATIVE_IS_CLOSING = True

# =========================
# Lane change advisor settings
# =========================
# 1차 구현은 발표/보고서에서 설명하기 쉬운 단순 longitudinal gap 모델을 사용한다.
LANE_CHANGE_TIME_SEC = 3.0
LANE_CHANGE_SAFE_GAP_M = 8.0
LANE_CHANGE_MIN_REQUIRED_GAP_M = 5.0
LANE_CHANGE_ACCEL_ALPHA = 0.3
EGO_SPEED_DEFAULT_MPS = 0.0
EGO_MAX_REASONABLE_ACCEL_MPS2 = 3.0
EGO_MAX_REASONABLE_SPEED_MPS = 40.0

# =========================
# Adaptive cruise control settings
# =========================
ACC_CRUISE_SPEED_MPS = 5.0
ACC_STANDSTILL_GAP_M = 2.0
ACC_TIME_HEADWAY_SEC = 1.2
ACC_DISTANCE_KP = 0.35
ACC_RELATIVE_SPEED_KD = 0.8
ACC_CRUISE_SPEED_KP = 0.6
ACC_MAX_ACCEL_MPS2 = 2.0
ACC_MAX_DECEL_MPS2 = 4.0
ACC_RECOMMENDATION_HORIZON_SEC = 1.0
ACC_TTC_CAUTION_SEC = 3.0
ACC_TTC_EMERGENCY_SEC = 1.5

# Current steering angle is treated as a road-wheel angle for the bicycle model.
ACC_WHEELBASE_M = 0.30
ACC_PATH_HALF_WIDTH_M = 0.45
ACC_MAX_LOOKAHEAD_M = 10.0
ACC_MAX_PATH_OFFSET_M = 0.90
ACC_STEERING_SIGN = 1.0
ACC_USE_CONFIRMED_TRACKS_ONLY = True

# =========================
# STM32 SPI settings
# =========================
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 100000
SPI_MODE = 0
SPI_AUTO_DETECT_MODE = True
SPI_BITS_PER_WORD = 8
# SPI 상태 교환 주기. 0.02초 = 50Hz.
SPI_PACKET_PERIOD_SEC = 0.02
# 별도 50Hz worker가 계속 폴링하므로 같은 주기 안에서 연속 재시도하지 않는다.
SPI_FRAME_RETRY_COUNT = 1
SPI_FRAME_RETRY_DELAY_SEC = 0.0

# STM32 -> Raspberry Pi legacy turn signal status byte.
TURN_SIGNAL_NONE = 0
TURN_SIGNAL_RIGHT = 1
TURN_SIGNAL_LEFT = 2
TURN_SIGNAL_HAZARD = 3
TURN_SIGNAL_INVALID = 0xFF

# SPI_USE_MOCK이 True이면 실제 SPI 하드웨어 대신 Mock 데이터를 사용한다.
# New name used by spi_sender. USE_MOCK_SPI is kept for backward compatibility.
SPI_USE_MOCK = False

# 개발 PC/하드웨어 미연결 상태에서도 실행되도록 기본은 Mock으로 둔다.
USE_MOCK_SPI = SPI_USE_MOCK

# Legacy mock value kept for compatibility with older callers.
SPI_MOCK_TURN_SIGNAL = TURN_SIGNAL_NONE

# 33-byte MISO packet:
# AA 55 01 + ego speed(int16, x100) + steering angle(int16, x10)
# + turn signal + reserved bytes + checksum.
SPI_EGO_SPEED_SCALE_MPS = 0.01
SPI_STEERING_ANGLE_SCALE_DEG = 0.1
SPI_MOCK_EGO_SPEED_MPS = 0.0
SPI_MOCK_STEERING_ANGLE_DEG = 0.0

# Radar packet object limit. Packet length is fixed for simple STM32 C parsing.
SPI_MAX_OBJECTS = 3
SPI_SAFE_FALLBACK_TO_MOCK = False

# =========================
# Visualizer settings
# =========================
ENABLE_VISUALIZER = True

# Available values:
#   "off"       : no visualizer
#   "pyqtgraph" : advanced real-time visualizer
#   "legacy"    : old matplotlib visualizer kept for compatibility
VISUALIZER_BACKEND = "pyqtgraph"

VISUALIZER_UPDATE_HZ = 20
VISUALIZER_X_RANGE = (-5.0, 5.0)
VISUALIZER_Y_RANGE = (0.0, 10.0)
SHOW_RAW_DETECTIONS = True
SHOW_CLUSTERS = True
SHOW_TRACKS = True
SHOW_TRACK_HISTORY = True
SHOW_VELOCITY_VECTOR = True
SHOW_SPI_PACKET = True
SHOW_DEBUG_PANEL = True
TRACK_HISTORY_LENGTH = 30

# [그래프 고정] 화면 좌표 범위
VIEW_X_MIN, VIEW_X_MAX = -5.0, 5.0
VIEW_Y_MIN, VIEW_Y_MAX = 0.0, 10.0

# =========================
# UDP network settings
# =========================
UDP_ENABLED = True
LAPTOP_IP = "192.168.168.64"
UDP_PORT = 5005
UDP_RECV_BUFFER = 8192


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
