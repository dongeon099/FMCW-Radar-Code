import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from config import (DBSCAN_EPS, DBSCAN_MIN_SAMPLES, MIN_RANGE, MAX_RANGE, 
VELOCITY_THRESHOLD, Y_DISTANCE_THRESHOLD, X_RANGE)


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
            "id": cluster_id,
            "x": centroid_x,
            "y": centroid_y,
            "z": centroid_z,
            "v": centroid_v,
            "distance": centroid_distance,
        })

    return cluster_centroid_objects

def velocity_filter(obj):

    if not obj:
        return np.empty((0, 6), dtype = float)  # 빈 배열 반환 (5는 객체의 속성 수)
    
    if isinstance(obj[0], dict): # 이 변수의 자료형이 맞는지 검사하는 코드
        obj = np.array([
            [
                float(obj[i]["id"]),
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
    
    valid =(
    (np.abs(velocity) > VELOCITY_THRESHOLD) 
    &(Y_distance > Y_DISTANCE_THRESHOLD)  
    &(Y_distance < MAX_RANGE) 
    &(X_distance < X_RANGE) 
    &(X_distance > -X_RANGE))  

    velocity_obj = obj[valid]  

    return velocity_obj