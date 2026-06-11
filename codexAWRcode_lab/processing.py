import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN

from config import (
    DBSCAN_EPS,
    DBSCAN_MIN_SAMPLES,
    MAX_RANGE,
    MIN_RANGE,
    VELOCITY_THRESHOLD,
    X_RANGE,
    Y_DISTANCE_THRESHOLD,
)


def dbscan_scattering(points):
    """Filter radar points by range, then cluster them with DBSCAN."""
    points = np.array(points, dtype=float)

    distance = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2 + points[:, 2] ** 2)
    valid = (distance >= MIN_RANGE) & (distance <= MAX_RANGE)
    points = points[valid]

    if len(points) == 0:
        return None, None, None, None, None

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    xy = points[:, :2]
    labels = DBSCAN(
        eps=DBSCAN_EPS,
        min_samples=DBSCAN_MIN_SAMPLES,
    ).fit_predict(xy)

    filtered_points = points[labels != -1]
    filtered_labels = labels[labels != -1]
    filtered_distance = np.sqrt(
        filtered_points[:, 0] ** 2
        + filtered_points[:, 1] ** 2
        + filtered_points[:, 2] ** 2
    )

    df = pd.DataFrame({
        "X_m": filtered_points[:, 0],
        "Y_m": filtered_points[:, 1],
        "Z_m": filtered_points[:, 2],
        "Distance_m": filtered_distance,
        "Velocity_mps": filtered_points[:, 3],
        "ClusterID": filtered_labels,
    })

    return df, filtered_labels, filtered_points[:, 0], filtered_points[:, 1], filtered_points


def extract_clusters(points, labels):
    """Build cluster_centroid_objects from DBSCAN output."""
    points = np.array(points, dtype=float)
    labels = np.array(labels)

    cluster_centroid_objects = []

    for cluster_id in set(labels):
        if cluster_id == -1:
            continue

        cluster_points = points[labels == cluster_id]

        centroid_x = np.mean(cluster_points[:, 0])
        centroid_y = np.mean(cluster_points[:, 1])
        centroid_z = np.mean(cluster_points[:, 2])
        centroid_v = np.mean(cluster_points[:, 3])

        centroid_distance = np.sqrt(
            centroid_x**2 + centroid_y**2 + centroid_z**2
        )

        cluster_centroid_objects.append({
            "track_id": cluster_id,
            "cluster_id": cluster_id,  # EKF added: DBSCAN id, not EKF track id.
            "x": centroid_x,
            "y": centroid_y,
            "z": centroid_z,
            "v": centroid_v,
            "velocity": centroid_v,  # EKF added: explicit velocity alias.
            "distance": centroid_distance,
            "range": centroid_distance,  # EKF added: range alias.
            "point_count": len(cluster_points),  # EKF added: noise filtering support.
        })

    return cluster_centroid_objects


def velocity_filter(obj):
    """Keep the old velocity_filter helper for compatibility with older code."""
    if not obj:
        velocity_filter.lane_1_obj = []
        velocity_filter.lane_2_obj = []
        velocity_filter.lane_3_obj = []
        return np.empty((0, 6), dtype=float)

    if isinstance(obj[0], dict):
        obj = np.array([
            [
                float(obj[i]["track_id"]),
                float(obj[i]["x"]),
                float(obj[i]["y"]),
                float(obj[i]["z"]),
                float(obj[i]["v"]),
                float(obj[i]["distance"]),
            ]
            for i in range(len(obj))
        ], dtype=float)
    else:
        obj = np.array(obj, dtype=float)

    velocity = obj[:, 4]
    y_distance = obj[:, 2]
    x_distance = obj[:, 1]

    base_valid = (
        (np.abs(velocity) > VELOCITY_THRESHOLD)
        & (y_distance > Y_DISTANCE_THRESHOLD)
        & (y_distance < MAX_RANGE)
    )
    lane_width = X_RANGE * 2.0
    lane_1_valid = base_valid & (x_distance > -X_RANGE - lane_width) & (x_distance <= -X_RANGE)
    lane_2_valid = base_valid & (x_distance < X_RANGE) & (x_distance > -X_RANGE)
    lane_3_valid = base_valid & (x_distance >= X_RANGE) & (x_distance < X_RANGE + lane_width)

    valid = (
        (np.abs(velocity) > VELOCITY_THRESHOLD)
        & (y_distance > Y_DISTANCE_THRESHOLD)
        & (y_distance < MAX_RANGE)
        & (x_distance < X_RANGE)
        & (x_distance > -X_RANGE)
    )

    velocity_obj = obj[valid]
    velocity_filter.lane_1_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_1_valid]
    ]
    velocity_filter.lane_2_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_2_valid]
    ]
    velocity_filter.lane_3_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_3_valid]
    ]

    return velocity_obj
