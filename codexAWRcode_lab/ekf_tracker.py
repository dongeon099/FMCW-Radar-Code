import math

import numpy as np

from config import (
    DT_DEFAULT,
    DT_MAX,
    DT_MIN,
    EGO_LANE_CENTER_X,
    EKF_ASSOCIATION_DISTANCE_THRESHOLD,
    EKF_MAX_DISTANCE,
    EKF_MAX_MISSED_FRAMES,
    EKF_MAX_TRACKS,
    EKF_MIN_DISTANCE,
    EKF_MIN_POINT_COUNT,
    EKF_POSITION_NOISE,
    EKF_PROCESS_NOISE,
    EKF_REAR_Y_MIN,
    EKF_VELOCITY_NOISE,
    EKF_VERBOSE,
    LANE_ASSIGN_MARGIN,
    LANE_WIDTH,
)


LANE_NAMES = {
    1: "left",
    2: "center",
    3: "right",
}


def safe_dt(dt):
    """EKF added: keep dt inside a stable range for prediction."""
    try:
        dt = float(dt)
    except (TypeError, ValueError):
        dt = DT_DEFAULT

    if not math.isfinite(dt) or dt <= 0.0:
        return DT_DEFAULT
    return max(DT_MIN, min(dt, DT_MAX))


def _lookup(obj, names, default=None):
    if not isinstance(obj, dict):
        return default

    lowered = {str(key).lower(): value for key, value in obj.items()}
    for name in names:
        if name in obj:
            return obj[name]
        value = lowered.get(name.lower())
        if value is not None:
            return value
    return default


def _finite_float(value, default=None):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return None
    return value


def assign_lane_id(x):
    """EKF added: classify a track into left, center, or right lane.

    The basic split is based on the ego lane center and LANE_WIDTH:
    left lane  -> x < center - LANE_WIDTH / 2
    center     -> inside the ego lane
    right lane -> x > center + LANE_WIDTH / 2

    LANE_ASSIGN_MARGIN adds a small dead-band around boundaries so a noisy
    track does not jump lanes every frame.
    """
    x = _finite_float(x, EGO_LANE_CENTER_X)
    if x is None:
        return 2

    half_lane = LANE_WIDTH / 2.0
    left_boundary = EGO_LANE_CENTER_X - half_lane - LANE_ASSIGN_MARGIN
    right_boundary = EGO_LANE_CENTER_X + half_lane + LANE_ASSIGN_MARGIN

    if x < left_boundary:
        return 1
    if x > right_boundary:
        return 3
    return 2


def normalize_cluster_object(obj):
    """EKF added: accept cluster_centroid_objects with several field aliases."""
    if isinstance(obj, dict):
        x = _finite_float(_lookup(obj, ["x", "X_m"], 0.0), 0.0)
        y = _finite_float(_lookup(obj, ["y", "Y_m"], 0.0), 0.0)
        z = _finite_float(_lookup(obj, ["z", "Z_m"], 0.0), 0.0)

        distance_value = _lookup(obj, ["distance", "range", "Distance_m"], None)
        velocity_value = _lookup(
            obj,
            ["velocity", "v", "radial_velocity", "doppler", "Velocity_mps"],
            0.0,
        )
        point_count_value = _lookup(obj, ["point_count", "num_points"], EKF_MIN_POINT_COUNT)
        cluster_id_value = _lookup(obj, ["cluster_id", "ClusterID"], _lookup(obj, ["track_id"], -1))
    else:
        try:
            arr = np.asarray(obj, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return None

        if arr.size >= 6:
            x, y, z = arr[1], arr[2], arr[3]
            velocity_value = arr[4]
            distance_value = arr[5]
            point_count_value = EKF_MIN_POINT_COUNT
            cluster_id_value = int(arr[0])
        elif arr.size >= 4:
            x, y, z = arr[0], arr[1], arr[2]
            velocity_value = arr[3]
            distance_value = None
            point_count_value = EKF_MIN_POINT_COUNT
            cluster_id_value = -1
        else:
            return None

        x = _finite_float(x, None)
        y = _finite_float(y, None)
        z = _finite_float(z, None)

    if x is None or y is None or z is None:
        return None

    distance = _finite_float(distance_value, None)
    if distance is None and distance_value is not None:
        return None
    if distance is None:
        distance = math.sqrt(x * x + y * y + z * z)

    velocity = _finite_float(velocity_value, None)
    if velocity is None:
        return None

    point_count = _finite_float(point_count_value, EKF_MIN_POINT_COUNT)
    if point_count is None:
        return None

    cluster_id = _finite_float(cluster_id_value, -1)
    if cluster_id is None:
        cluster_id = -1

    normalized = {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "distance": float(distance),
        # Radar velocity is treated as y-direction velocity (vy) in this simple capstone-level model.
        "velocity": float(velocity),
        "v": float(velocity),
        "point_count": int(point_count),
        "cluster_id": int(cluster_id),
        "lane_id": assign_lane_id(x),
    }

    if not all(math.isfinite(normalized[key]) for key in ("x", "y", "z", "distance", "velocity")):
        return None
    return normalized


def filter_detections(cluster_centroid_objects):
    """EKF added: remove noise before association."""
    detections = []
    for obj in cluster_centroid_objects or []:
        detection = normalize_cluster_object(obj)
        if detection is None:
            continue
        if detection["point_count"] < EKF_MIN_POINT_COUNT:
            continue
        if detection["distance"] < EKF_MIN_DISTANCE:
            continue
        if detection["distance"] > EKF_MAX_DISTANCE:
            continue
        if detection["y"] < EKF_REAR_Y_MIN:
            continue
        detections.append(detection)
    return detections


class EKFTrack:
    """EKF added: one vehicle track with its own EKF state.

    State vector:
        x = [x_position, y_position, vx, vy, ax, ay]^T

    The radar velocity is likely radial velocity. To keep the project explainable,
    this tracker uses the measured velocity as vy. Because this radar reports
    approaching vehicles with negative velocity, velocity < 0 means approaching.
    """

    def __init__(self, measurement, track_id):
        self.track_id = int(track_id)
        self.missed_count = 0
        self.age = 1
        self.hit_count = 1
        self.last_detection = dict(measurement)

        # Initial state uses measured x/y and measured velocity as vy.
        self.x = np.array(
            [
                [measurement["x"]],
                [measurement["y"]],
                [0.0],
                [measurement["velocity"]],
                [0.0],
                [0.0],
            ],
            dtype=float,
        )

        # P is state covariance. Larger values mean less confidence.
        self.P = np.diag([1.0, 1.0, 4.0, 4.0, 6.0, 6.0]).astype(float)

        # H maps state to measurement z = [measured_x, measured_y, measured_vy]^T.
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        # R is measurement noise for x, y, and radar velocity.
        self.R = np.diag(
            [
                EKF_POSITION_NOISE**2,
                EKF_POSITION_NOISE**2,
                EKF_VELOCITY_NOISE**2,
            ]
        ).astype(float)
        self.I = np.eye(6, dtype=float)

    def _process_noise(self, dt):
        """Q matrix: model uncertainty added during predict."""
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        return EKF_PROCESS_NOISE * np.array(
            [
                [0.25 * dt4, 0.0, 0.5 * dt3, 0.0, 0.5 * dt2, 0.0],
                [0.0, 0.25 * dt4, 0.0, 0.5 * dt3, 0.0, 0.5 * dt2],
                [0.5 * dt3, 0.0, dt2, 0.0, dt, 0.0],
                [0.0, 0.5 * dt3, 0.0, dt2, 0.0, dt],
                [0.5 * dt2, 0.0, dt, 0.0, 1.0, 0.0],
                [0.0, 0.5 * dt2, 0.0, dt, 0.0, 1.0],
            ],
            dtype=float,
        )

    def predict(self, dt):
        """Predict this track using a constant-acceleration model."""
        dt = safe_dt(dt)

        # F is the state transition matrix for:
        # x_k  = x_(k-1)  + vx_(k-1) * dt + 0.5 * ax_(k-1) * dt^2
        # y_k  = y_(k-1)  + vy_(k-1) * dt + 0.5 * ay_(k-1) * dt^2
        # vx_k = vx_(k-1) + ax_(k-1) * dt
        # vy_k = vy_(k-1) + ay_(k-1) * dt
        # ax_k = ax_(k-1), ay_k = ay_(k-1)
        self.F = np.array(
            [
                [1.0, 0.0, dt, 0.0, 0.5 * dt * dt, 0.0],
                [0.0, 1.0, 0.0, dt, 0.0, 0.5 * dt * dt],
                [0.0, 0.0, 1.0, 0.0, dt, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self._process_noise(dt)
        self.age += 1

    def update(self, measurement):
        """Update this track with z = [x, y, measured_velocity_as_vy]."""
        z = np.array(
            [
                [measurement["x"]],
                [measurement["y"]],
                [measurement["velocity"]],
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(z)):
            return False

        innovation = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ self.H.T @ np.linalg.pinv(S)

        self.x = self.x + K @ innovation
        self.P = (self.I - K @ self.H) @ self.P
        self.missed_count = 0
        self.hit_count += 1
        self.last_detection = dict(measurement)
        return True

    def mark_missed(self):
        """Nearest-neighbor association did not find a detection for this track."""
        self.missed_count += 1

    def position_cost(self, measurement):
        """Euclidean distance cost for simple nearest-neighbor association."""
        dx = float(self.x[0, 0]) - measurement["x"]
        dy = float(self.x[1, 0]) - measurement["y"]
        return math.hypot(dx, dy)

    def to_dict(self):
        """Convert an EKF track to the lane/TTC-friendly output shape."""
        x_pos = float(self.x[0, 0])
        y_pos = float(self.x[1, 0])
        vx = float(self.x[2, 0])
        vy = float(self.x[3, 0])
        ax = float(self.x[4, 0])
        ay = float(self.x[5, 0])
        distance = math.hypot(x_pos, y_pos)

        # EKF leaves collision scoring to a later module. That module can use
        # lane_id plus velocity/acceleration to select the needed lane.
        velocity = vy
        acceleration = ay
        lane_id = assign_lane_id(x_pos)

        return {
            "track_id": self.track_id,
            "x": x_pos,
            "y": y_pos,
            "z": float(self.last_detection.get("z", 0.0)),
            "distance": distance,
            "velocity": velocity,
            "v": velocity,
            "acceleration": acceleration,
            "vx": vx,
            "vy": vy,
            "ax": ax,
            "ay": ay,
            "lane_id": lane_id,
            "lane_name": LANE_NAMES[lane_id],
            # Radar convention in this project: negative velocity means the vehicle is approaching.
            "approaching": velocity < 0.0,
            "valid": self.missed_count == 0,
            "point_count": int(self.last_detection.get("point_count", 0)),
            "cluster_id": int(self.last_detection.get("cluster_id", -1)),
        }


class MultiLaneEKFTracker:
    """EKF added: manage several EKFTrack objects without Hungarian matching."""

    def __init__(self, verbose=EKF_VERBOSE):
        self.tracks = []
        self.next_track_id = 1
        self.verbose = bool(verbose)

    def update(self, cluster_centroid_objects=None, dt=DT_DEFAULT):
        """Update all tracks from DBSCAN cluster centroids and return lane outputs."""
        dt = safe_dt(dt)

        # 1. Predict every existing track before matching detections.
        for track in self.tracks:
            track.predict(dt)

        # 2. Normalize/filter detections from cluster_centroid_objects.
        detections = filter_detections(cluster_centroid_objects)

        # 3. Associate using greedy nearest-neighbor pairs.
        # This stays capstone-friendly: cost is just x/y distance, and each
        # detection can be used by only one track.
        matched_tracks = set()
        matched_detections = set()
        candidate_pairs = []

        for track_idx, track in enumerate(self.tracks):
            for detection_idx, detection in enumerate(detections):
                cost = track.position_cost(detection)
                if cost <= EKF_ASSOCIATION_DISTANCE_THRESHOLD:
                    candidate_pairs.append((cost, track_idx, detection_idx))

        candidate_pairs.sort(key=lambda item: item[0])
        for _, track_idx, detection_idx in candidate_pairs:
            if track_idx in matched_tracks or detection_idx in matched_detections:
                continue
            if self.tracks[track_idx].update(detections[detection_idx]):
                matched_tracks.add(track_idx)
                matched_detections.add(detection_idx)

        # 4. Tracks without a detection are kept briefly, then removed.
        for track_idx, track in enumerate(self.tracks):
            if track_idx not in matched_tracks:
                track.mark_missed()

        self.tracks = [
            track
            for track in self.tracks
            if track.missed_count <= EKF_MAX_MISSED_FRAMES
        ]

        # 5. Unmatched detections start new tracks, capped by EKF_MAX_TRACKS.
        unmatched_detections = [
            detection
            for detection_idx, detection in enumerate(detections)
            if detection_idx not in matched_detections
        ]
        unmatched_detections.sort(key=lambda item: item["distance"])

        for detection in unmatched_detections:
            if len(self.tracks) >= EKF_MAX_TRACKS:
                break
            self.tracks.append(EKFTrack(detection, self.next_track_id))
            self.next_track_id += 1

        lane_outputs = self._lane_outputs()

        if self.verbose:
            lane_counts = {
                lane_id: len(lane_data["objects"])
                for lane_id, lane_data in lane_outputs["lanes"].items()
            }
            print(
                "[EKF] tracks={} lane_counts={}".format(
                    len(lane_outputs["all_tracks"]),
                    lane_counts,
                )
            )

        return lane_outputs

    def _lane_outputs(self):
        lanes = {
            lane_id: {
                "lane_id": lane_id,
                "lane_name": LANE_NAMES[lane_id],
                "objects": [],
            }
            for lane_id in (1, 2, 3)
        }

        all_tracks = [track.to_dict() for track in self.tracks]
        all_tracks.sort(key=lambda item: (item["lane_id"], item["distance"], item["track_id"]))

        for item in all_tracks:
            lanes[item["lane_id"]]["objects"].append(item)

        return {
            "lanes": lanes,
            "all_tracks": all_tracks,
        }


def as_tracked_objects(lane_outputs):
    """EKF added: flatten lane outputs for the existing visualizer/SPI pipeline."""
    tracked = []
    if not lane_outputs:
        return tracked

    for lane_id, lane_data in lane_outputs.get("lanes", {}).items():
        for obj in lane_data.get("objects", []):
            item = dict(obj)
            item["lane_id"] = int(lane_id)
            item["v"] = item["velocity"]
            tracked.append(item)
    return tracked
