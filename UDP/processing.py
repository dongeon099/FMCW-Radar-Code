import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from config import (DBSCAN_EPS, DBSCAN_MIN_SAMPLES, MIN_RANGE, MAX_RANGE, 
VELOCITY_THRESHOLD, Y_DISTANCE_THRESHOLD, X_RANGE, TRACK_ASSOCIATION_MAX_DISTANCE, DT_DEFAULT)


# DBSCAN을 사용하여 점들을 클러스터링하고, 유효한 점들만 필터링하는 함수입니다.
def dbscan_scattering(points):
    points = np.array(points, dtype=float)

    distance = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
    
    valid = (distance >= MIN_RANGE) & (distance <= MAX_RANGE)  # 거리 설정 #부울함수를 사용하면 true fail로 저장됨
    points = points[valid] # 여기서 거리에 벗어나는 점들은 여기서 다 걸러진다. 필터링된 포인트가 시작되는 시작점 

    if len(points) == 0:
        return None, None, None, None, None

    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    v = points[:, 3]

    distance = np.sqrt(x**2 + y**2 + z**2)
    #코드를 보면 알겠지만 DBSCAN을 하면 xy좌표에 따라 라벨링만 한다는 것을 알 수 있다. 
   ## 벽이나 물체의 너비를 보려면 eps, min_samples를 모두 작게해야 보이려나?..
   ## 목표1: 벽이 길게 point가 짝히는 eps, minsample 값을 오늘 찾아보자!
   
    xy = points[:, :2]
    labels = DBSCAN(
        eps=DBSCAN_EPS,
        min_samples=DBSCAN_MIN_SAMPLES
    ).fit_predict(xy)

    ## 노이즈를 제거하고 클러스터링된 포인트만 남긴다.
    filtered_points = points[labels != -1]
    filtered_labels = labels[labels != -1]
    filtered_distance = np.sqrt(filtered_points[:, 0]**2 + filtered_points[:, 1]**2 + filtered_points[:, 2]**2)
    ## df는 시각화와 분석을 위해 포인트 데이터를 구조화한 형태입니다.
    df = pd.DataFrame({
        "X_m": filtered_points[:, 0],
        "Y_m": filtered_points[:, 1],
        "Z_m": filtered_points[:, 2],
        "Distance_m": filtered_distance,
        "Velocity_mps": filtered_points[:, 3],
        "ClusterID": filtered_labels,
    })

    return df, filtered_labels, filtered_points[:,0], filtered_points[:,1], filtered_points

# extract_clusters에 들어가는 points는 dbscan_scattering에서 노이즈 제거, 제한된 거리로 필터링된 포인트들이다.
# 따라서 df도 노이즈제거, 제한된 거리로 필터링된 라이브러리다. 
def extract_clusters(points, labels): #extract : 추출하다 
    points = np.array(points, dtype=float)
    labels = np.array(labels)

    cluster_centroid_objects = [] #centroid 된 좌표 

    for cluster_id in set(labels):
        
        if cluster_id == -1:
            continue

        cluster_points = points[labels == cluster_id]   # 부울함수로서 true fail 로 출력된다는걸 명심 (마스킹) 


        centroid_x = np.mean(cluster_points[:, 0])
        centroid_y = np.mean(cluster_points[:, 1])
        centroid_z = np.mean(cluster_points[:, 2])
        centroid_v = np.mean(cluster_points[:, 3])

        centroid_distance = np.sqrt(
            centroid_x**2 + centroid_y**2 + centroid_z**2
        )
        #centroid 된 좌표 
        cluster_centroid_objects.append({
            "track_id": cluster_id,
            "x": centroid_x,
            "y": centroid_y,
            "z": centroid_z,
            "v": centroid_v,
            "distance": centroid_distance,
        })

    return cluster_centroid_objects

def assign_track_ids(
    cluster_objects,
    prev_tracks,
    next_track_id,
    max_association_distance=TRACK_ASSOCIATION_MAX_DISTANCE,
    dt=DT_DEFAULT,
    max_acceleration=5.0,
    velocity_weight=0.5,
):
    """위치 예측과 속도 일관성을 함께 사용해 안정적인 track_id를 할당."""
    if not cluster_objects:
        return [], [], next_track_id

    # dt가 0이거나 음수이면 vx, vy, acceleration 계산에서 나눗셈 문제가 생깁니다.
    # main.py에서 DT_DEFAULT를 기준으로 보정된 dt를 넘겨주지만,
    # 이 함수만 따로 호출해도 안전하도록 한 번 더 확인합니다.
    if dt is None:
        dt = DT_DEFAULT
    else:
        dt = float(dt)
    if dt <= 0.0:
        dt = DT_DEFAULT

    # 이전 프레임 트랙을 이번 프레임 객체와 비교하기 쉬운 형태로 복사합니다.
    # 예전 코드에서 저장된 prev_tracks에는 v, vx, vy가 없을 수 있으므로
    # get()으로 기본값을 넣어 에러 없이 동작하게 합니다.
    # 리스트 안의 딕셔너리 구조 
    remaining_prev = [
        {
            "track_id": int(track["track_id"]),
            "x": float(track["x"]),
            "y": float(track["y"]),
            "v": float(track.get("v", 0.0)), # 있으면 float으로, 없으면 0.0으로 처리합니다.
            "vx": float(track.get("vx", 0.0)),
            "vy": float(track.get("vy", 0.0)),
            # vx, vy가 실제로 있었는지 기록합니다.
            # 없으면 아직 이동 방향을 모르는 트랙이므로 기존 방식처럼 위치 거리로 비교합니다.
            "has_xy_velocity": "vx" in track and "vy" in track, # vx, vy가 있으면 True, 없으면 False
        }
        for track in (prev_tracks or [])
    ]

    tracked_objects = []
    for obj in cluster_objects:
        # 현재 객체의 위치와 속도를 float으로 맞춰 계산합니다.
        current_x = float(obj["x"])
        current_y = float(obj["y"])
        current_v = float(obj.get("v", 0.0))

        best_idx = -1
        best_score = float("inf")
        best_prev = None

        for idx, prev in enumerate(remaining_prev):
            # 이전 vx, vy가 있으면 등속운동 모델로 현재 위치를 예측합니다.
            # predicted_x = prev_x + prev_vx * dt
            # predicted_y = prev_y + prev_vy * dt
            #
            # 이전 vx, vy가 없다면 아직 방향 속도를 모르는 상태이므로
            # 기존 방식처럼 이전 위치 자체를 기준으로 거리 매칭을 합니다.
            if prev["has_xy_velocity"]:
                predicted_x = prev["x"] + prev["vx"] * dt
                predicted_y = prev["y"] + prev["vy"] * dt
            else:
                predicted_x = prev["x"]
                predicted_y = prev["y"]

            # 예측 위치와 현재 위치 사이의 오차입니다.
            # 이 값이 max_association_distance보다 크면 같은 객체 후보에서 제외합니다.
            position_error = np.hypot(current_x - predicted_x, current_y - predicted_y)

            # 속도 차이를 이용해 가속도를 계산합니다.
            # 등속운동을 가정하므로 가속도가 너무 크면 같은 객체로 보지 않습니다.
            velocity_error = abs(current_v - prev["v"])
            accel = velocity_error / dt

            if position_error > max_association_distance:
                continue
            if accel > max_acceleration:
                continue

            # 기본 점수는 위치 예측 오차 + 속도 오차 가중치입니다.
            # vx/vy가 없는 오래된 트랙은 첫 매칭만 기존 방식에 가깝게 위치를 더 믿습니다.
            if prev["has_xy_velocity"]:
                score = position_error + velocity_weight * velocity_error
            else:
                score = position_error

            if score < best_score:
                best_score = score
                best_idx = idx
                best_prev = prev

        if best_idx != -1:
            assigned_track_id = best_prev["track_id"]

            # 매칭된 트랙은 현재 위치와 이전 위치 차이로 vx, vy를 새로 추정합니다.
            # 다음 프레임에서는 이 vx, vy로 현재 위치를 예측할 수 있습니다.
            estimated_vx = (current_x - best_prev["x"]) / dt
            estimated_vy = (current_y - best_prev["y"]) / dt

            # 이미 사용한 이전 트랙은 다른 현재 객체에 중복 매칭되지 않도록 제거합니다.
            del remaining_prev[best_idx]
        else:
            assigned_track_id = next_track_id
            next_track_id += 1

            # 새 트랙은 비교할 이전 위치가 없어서 방향 속도를 아직 알 수 없습니다.
            # 우선 0으로 저장하고, 다음 프레임에서 매칭되면 위치 차이로 vx, vy를 추정합니다.
            estimated_vx = 0.0
            estimated_vy = 0.0

        tracked_obj = dict(obj)
        tracked_obj["track_id"] = assigned_track_id
        tracked_obj["x"] = current_x
        tracked_obj["y"] = current_y
        tracked_obj["v"] = current_v
        tracked_obj["vx"] = estimated_vx
        tracked_obj["vy"] = estimated_vy
        tracked_objects.append(tracked_obj)

    # 다음 프레임에서 사용할 트랙 상태입니다.
    # x, y, v뿐 아니라 추정한 vx, vy까지 저장해야 다음 호출에서 등속운동 예측을 할 수 있습니다.
    next_prev_tracks = [
        {
            "track_id": obj["track_id"],
            "x": obj["x"],
            "y": obj["y"],
            "v": obj.get("v", 0.0),
            "vx": obj.get("vx", 0.0),
            "vy": obj.get("vy", 0.0),
        }
        for obj in tracked_objects
    ]
    return tracked_objects, next_prev_tracks, next_track_id

def velocity_filter(obj):

    if not obj:
        velocity_filter.lane_1_obj = []  # Keep adjacent lane buffers empty when no objects exist.
        velocity_filter.lane_2_obj = []  # Keep existing lane data accessible without changing the return value.
        velocity_filter.lane_3_obj = []  # Store lane 3 separately while preserving the current pipeline.
        return np.empty((0, 6), dtype = float)  # 빈 배열 반환 (5는 객체의 속성 수)
    
    if isinstance(obj[0], dict): # 이 변수의 자료형이 맞는지 검사하는 코드
        obj = np.array([
            [
                float(obj[i]["track_id"]),
                float(obj[i]["x"]),      
                float(obj[i]["y"]),
                float(obj[i]["z"]),
                float(obj[i]["v"]),
                float(obj[i]["distance"])
            ]
            for i in range(len(obj))
        ], dtype=float)
    else:
        obj = np.array(obj, dtype=float)
    
    velocity = obj[:, 4] 
    Y_distance = obj[:, 2]
    X_distance = obj[:, 1]

    base_valid =(
    (np.abs(velocity) > VELOCITY_THRESHOLD) 
    &(Y_distance > Y_DISTANCE_THRESHOLD)  
    &(Y_distance < MAX_RANGE))
    lane_width = X_RANGE * 2.0  # Use the current lane half-range to split lane 1/2/3 without new config.
    lane_1_valid = base_valid &(X_distance > -X_RANGE - lane_width) &(X_distance <= -X_RANGE)
    lane_2_valid = base_valid &(X_distance < X_RANGE) &(X_distance > -X_RANGE)
    lane_3_valid = base_valid &(X_distance >= X_RANGE) &(X_distance < X_RANGE + lane_width)
    
    valid =(
    (np.abs(velocity) > VELOCITY_THRESHOLD) 
    &(Y_distance > Y_DISTANCE_THRESHOLD)  
    &(Y_distance < MAX_RANGE) 
    &(X_distance < X_RANGE) 
    &(X_distance > -X_RANGE))  

    velocity_obj = obj[valid]  
    velocity_filter.lane_1_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_1_valid]
    ]  # Save moving objects from lane 1 separately for later use.
    velocity_filter.lane_2_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_2_valid]
    ]  # Save current-lane moving objects while keeping the old numpy return.
    velocity_filter.lane_3_obj = [
        {"track_id": int(row[0]), "x": row[1], "y": row[2], "z": row[3], "v": row[4], "distance": row[5]}
        for row in obj[lane_3_valid]
    ]  # Save moving objects from lane 3 separately for later use.

    return velocity_obj
