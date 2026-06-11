import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from config import DBSCAN_EPS, DBSCAN_MIN_SAMPLES, MIN_RANGE, MAX_RANGE


def dbscan_scattering(points):
    points = np.array(points, dtype=float)

    distance = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
    
    valid = (distance >= MIN_RANGE) & (distance <= MAX_RANGE)  # 거리 설정 
    points = points[valid]

    if len(points) == 0:
        return None, None, None, None, None

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    v = points[:, 3]

    distance = np.sqrt(x**2 + y**2 + z**2)

    xy = points[:, :2]
    labels = DBSCAN(
        eps=DBSCAN_EPS,
        min_samples=DBSCAN_MIN_SAMPLES
    ).fit_predict(xy)

    df = pd.DataFrame({
        "X_m": x,
        "Y_m": y,
        "Z_m": z,
        "Distance_m": distance,
        "Velocity_mps": v,
        "ClusterID": labels,
    })

    return df, labels, x, y, points


def extract_clusters(points, labels): #extract : 추출하다 
    points = np.array(points, dtype=float)
    labels = np.array(labels)

    cluster_objects = []

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

        cluster_objects.append({
            "id": cluster_id,
            "x": centroid_x,
            "y": centroid_y,
            "z": centroid_z,
            "v": centroid_v,
            "distance": centroid_distance,
        })

    if len(cluster_objects) == 0:
        nearest_obj = None
    else:
        nearest_obj = min(cluster_objects, key=lambda obj: obj["distance"])

    return cluster_objects, nearest_obj