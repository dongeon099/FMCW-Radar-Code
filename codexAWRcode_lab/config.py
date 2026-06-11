# =========================
# Global constants
# =========================
MAGIC_WORD = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER_LEN = 40
MAX_PACKET_LEN = 65535

# [DBSCAN] Clustering parameters
DBSCAN_EPS = 0.3
DBSCAN_MIN_SAMPLES = 3
MIN_RANGE = 0.5
MAX_RANGE = 10.0

# EKF prediction frame interval limits
DT_DEFAULT = 0.05
DT_MIN = 0.001
DT_MAX = 0.2

# Existing velocity filter thresholds
VELOCITY_THRESHOLD = 0.2
Y_DISTANCE_THRESHOLD = 0.2
X_RANGE = 0.3

# [EKF added] Lane and multi-vehicle EKF tracker settings.
# x is the left/right direction, and y+ is the rear direction away from the ego car.
LANE_WIDTH = 3.5
EGO_LANE_CENTER_X = 0.0
VEHICLE_WIDTH = 1.8
LANE_ASSIGN_MARGIN = 0.2

# [EKF added] Simple nearest-neighbor multi-track settings.
EKF_VERBOSE = False
EKF_MIN_DISTANCE = MIN_RANGE
EKF_MAX_DISTANCE = MAX_RANGE
EKF_REAR_Y_MIN = Y_DISTANCE_THRESHOLD
EKF_MIN_POINT_COUNT = DBSCAN_MIN_SAMPLES
EKF_MAX_MISSED_FRAMES = 5
EKF_MAX_TRACKS = 10
EKF_ASSOCIATION_DISTANCE_THRESHOLD = 2.0
EKF_POSITION_NOISE = 0.35
EKF_VELOCITY_NOISE = 0.45
EKF_PROCESS_NOISE = 0.8

# [Visualizer] Fixed screen range
VIEW_X_MIN, VIEW_X_MAX = -5.0, 5.0
VIEW_Y_MIN, VIEW_Y_MAX = 0.0, 10.0


def build_config():
    """Runtime serial/config settings."""
    return {
        "cli_port": "COM5",
        "data_port": "COM4",
        "baud_cli": 115200,
        "baud_data": 921600,
        "cfg_file": r"C:\ti\mmwave_sdk_03_06_02_00-LTS\packages\ti\demo\xwr68xx\mmw\profiles\profile_2d.cfg",
        "read_timeout": 0.01,
        "cli_timeout": 0.5,
    }
