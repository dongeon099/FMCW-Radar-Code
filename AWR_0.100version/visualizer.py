import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

from config import VIEW_X_MIN, VIEW_X_MAX, VIEW_Y_MIN, VIEW_Y_MAX

# 기본값(필요 시 config.py에서 override)
# 레이더 시야각 설명 - AWR6843의 방위각 범위는 -60도에서 +60도 사이입니다. (총 120도)
AWR_AZIMUTH_MIN_DEG = -60.0
AWR_AZIMUTH_MAX_DEG = 60.0
AWR_BORESIGHT_DEG = 90.0


def _get_config_value(name, default):
    try:
        import config  # type: ignore

        return float(getattr(config, name, default))
    except Exception:
        return float(default)


def _resolve_fov_config():
   
    az_min = _get_config_value("AWR_AZIMUTH_MIN_DEG", AWR_AZIMUTH_MIN_DEG)
    az_max = _get_config_value("AWR_AZIMUTH_MAX_DEG", AWR_AZIMUTH_MAX_DEG)
    boresight = _get_config_value("AWR_BORESIGHT_DEG", AWR_BORESIGHT_DEG)
    range_max = _get_config_value("AWR_RANGE_MAX_M", max(0.0, VIEW_Y_MAX))
    return az_min, az_max, boresight, max(0.0, range_max)


def _draw_fov_overlay(ax, az_min_deg, az_max_deg, boresight_deg, range_max_m):
    """레이더 원점(0,0) 기준 허용 방위각+거리 영역 표시."""
    theta1 = boresight_deg + az_min_deg
    theta2 = boresight_deg + az_max_deg

    fov_patch = Wedge(
        center=(0.0, 0.0),
        r=range_max_m,
        theta1=theta1,
        theta2=theta2,
        facecolor="lightgreen",
        alpha=0.15,
        edgecolor="green",
        linewidth=1.2,
        linestyle="--",
        zorder=0,
        label="AWR6843 FOV",
    )
    ax.add_patch(fov_patch)


def visualize_points(fig, ax, df, labels, x, y, num_detected_obj, cluster_objects, velocity_obj):
    print("\033c", end="")
    print("===== AWR6843 Detected Objects =====")
    print(f"Detected objects(header): {num_detected_obj}")
    print(df)

    labels = np.asarray(labels)
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    velocity_obj = np.asarray(velocity_obj)

    unique_labels = sorted(set(labels.tolist())) if labels.size else []
    cluster_count = len([lb for lb in unique_labels if lb != -1])
    noise_count = int(np.sum(labels == -1)) if labels.size else 0

    print(f"클러스터 수(DBSCAN): {cluster_count}, 노이즈 포인트: {noise_count}")

    print(f"속도 필터 통과 객체: {len(velocity_obj)}개")

    ax.clear()

    az_min_deg, az_max_deg, boresight_deg, range_max_m = _resolve_fov_config()
    _draw_fov_overlay(ax, az_min_deg, az_max_deg, boresight_deg, range_max_m)

    # boresight(+Y) 기준 상대 방위각 계산: 0도가 정면
    abs_deg = np.degrees(np.arctan2(y, x))
    rel_deg = ((abs_deg - boresight_deg + 180.0) % 360.0) - 180.0
    distances = np.hypot(x, y)

    in_fov_mask = (
        (rel_deg >= az_min_deg)
        & (rel_deg <= az_max_deg)
        & (distances <= range_max_m)
        & (y >= 0.0)
    )

    
    if np.any(in_fov_mask):
        sc = ax.scatter(
            x[in_fov_mask],
            y[in_fov_mask],
            s=30,
            c=labels[in_fov_mask],
            cmap="tab20",
            zorder=2,
        )

        if not hasattr(fig, "_awr_colorbar"):
            fig._awr_colorbar = plt.colorbar(sc, ax=ax)
            fig._awr_colorbar.set_label("Cluster ID (-1: noise)")
        else:
            fig._awr_colorbar.update_normal(sc)

    if np.any(~in_fov_mask):
        ax.scatter(
            x[~in_fov_mask],
            y[~in_fov_mask],
            s=35,
            c="gray",
            marker="x",
            alpha=0.85,
            label="Out of FOV",
            zorder=3,
        )

    if cluster_objects:
        ax.scatter(
            [obj["x"] for obj in cluster_objects],
            [obj["y"] for obj in cluster_objects],
            s=80,
            c="red",
            marker="s",
            edgecolors="black",
            linewidths=1.0,
            label="Centroid",
            zorder=5,
        )

    if velocity_obj.size != 0:
        ax.scatter(
            velocity_obj[:, 1],
            velocity_obj[:, 2],
            s=140,
            c="deepskyblue",
            marker="*",
            edgecolors="black",
            linewidths=1.2,
            label="Velocity Filter Pass",
            zorder=8,
        )

    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_title("AWR6843 Position / DBSCAN Cluster")
    ax.grid(True)
    ax.set_xlim(VIEW_X_MIN, VIEW_X_MAX)
    ax.set_ylim(VIEW_Y_MIN, VIEW_Y_MAX)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")


    plt.pause(0.001)