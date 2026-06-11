# Multi-Lane EKF Tracking

The radar pipeline stays close to the original flow:

```text
Radar Raw Data
-> point cloud parsing
-> range filtering
-> DBSCAN clustering
-> cluster_centroid_objects
-> MultiLaneEKFTracker.update(...)
-> lane-separated EKF objects
-> later TTC / SPI modules choose lane_id 1, 2, or 3
```

## Tracker

`ekf_tracker.py` contains:

- `EKFTrack`: one EKF per tracked vehicle
- `MultiLaneEKFTracker`: a small manager for multiple tracks
- `normalize_cluster_object()`: accepts field aliases such as `distance`, `range`, `v`, `velocity`, `Velocity_mps`
- `assign_lane_id()`: assigns lane 1, 2, or 3 from x position
- `as_tracked_objects()`: flattens lane outputs for older visualizer/SPI-style code

Each track uses:

```text
state x = [x_position, y_position, vx, vy, ax, ay]^T
measurement z = [measured_x, measured_y, measured_velocity]^T
```

The radar velocity is treated as `vy` for a simple capstone-level model. In this
project, `velocity < 0` means the vehicle is approaching.

## Lane Output

`update()` returns:

```python
{
    "lanes": {
        1: {"lane_id": 1, "lane_name": "left", "objects": [...]},
        2: {"lane_id": 2, "lane_name": "center", "objects": [...]},
        3: {"lane_id": 3, "lane_name": "right", "objects": [...]},
    },
    "all_tracks": [...]
}
```

Objects include `track_id`, `x`, `y`, `distance`, `velocity`, `acceleration`,
`vx`, `vy`, `ax`, `ay`, `approaching`, `valid`, `point_count`, and `cluster_id`.

## Association

The tracker does not use Hungarian Algorithm, JPDA, IMM, or advanced MOT logic.
It uses greedy nearest-neighbor association:

1. Predict all existing EKF tracks.
2. Normalize and filter DBSCAN detections.
3. Compute x/y distance cost between each track and detection.
4. Match the closest pairs under `EKF_ASSOCIATION_DISTANCE_THRESHOLD`.
5. Increase `missed_count` for unmatched tracks.
6. Delete tracks after `EKF_MAX_MISSED_FRAMES`.
7. Create new tracks for unmatched detections.

## TTC

TTC and collision scoring are intentionally not calculated inside the EKF. A later
TTC module can select a lane based on turn-signal state:

```python
tracker = MultiLaneEKFTracker()

lane_outputs = tracker.update(
    cluster_centroid_objects=cluster_centroid_objects,
    dt=dt,
)

left_lane_objects = lane_outputs["lanes"][1]["objects"]
center_lane_objects = lane_outputs["lanes"][2]["objects"]
right_lane_objects = lane_outputs["lanes"][3]["objects"]

# left turn signal ON  -> use lane_id 1
# right turn signal ON -> use lane_id 3
# no signal / center   -> use lane_id 2
```
