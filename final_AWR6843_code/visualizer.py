import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

from config import VIEW_X_MIN, VIEW_X_MAX, VIEW_Y_MIN, VIEW_Y_MAX, X_RANGE

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
    theta1 = boresight_deg + az_min_deg
    theta2 = boresight_deg + az_max_deg

    ax.add_patch(
        Wedge(
            center=(0.0, 0.0),
            r=range_max_m,
            theta1=theta1,
            theta2=theta2,
            facecolor="lightgreen",
            alpha=0.12,
            edgecolor="green",
            linewidth=1.2,
            linestyle="--",
            zorder=0,
            label="Radar azimuth",
        )
    )


def _lane_bounds():
    lane_width = X_RANGE * 2.0  # Match velocity_filter lane width so the visual grid uses the same lane split.
    return [
        ("Lane 1", -X_RANGE - lane_width, -X_RANGE, "tab:orange", "^"),
        ("Lane 2", -X_RANGE, X_RANGE, "tab:blue", "o"),
        ("Lane 3", X_RANGE, X_RANGE + lane_width, "tab:green", "s"),
    ]


def _draw_lane_grid(ax):
    for lane_name, x_min, x_max, color, _ in _lane_bounds():
        ax.axvspan(x_min, x_max, color=color, alpha=0.08, zorder=1)
        ax.text(
            (x_min + x_max) / 2.0,
            VIEW_Y_MAX * 0.96,
            lane_name,
            ha="center",
            va="top",
            color=color,
            fontsize=10,
            fontweight="bold",
        )

    lane_edges = sorted({edge for _, x_min, x_max, _, _ in _lane_bounds() for edge in (x_min, x_max)})
    for edge in lane_edges:
        ax.axvline(edge, color="black", linewidth=0.8, linestyle=":", alpha=0.55, zorder=2)


def _get_lane_xy(lane_objects):
    if lane_objects is None:
        return np.empty((0, 2), dtype=float)

    if isinstance(lane_objects, np.ndarray):
        arr = np.asarray(lane_objects, dtype=float)
        if arr.size == 0:
            return np.empty((0, 2), dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        return arr[:, [1, 2]]

    if len(lane_objects) == 0:
        return np.empty((0, 2), dtype=float)

    return np.array([[float(obj["x"]), float(obj["y"])] for obj in lane_objects], dtype=float)


def _get_velocity_distance(obj):
    if isinstance(obj, dict):
        return float(obj["v"]), float(obj["distance"])

    row = np.asarray(obj, dtype=float)
    return float(row[4]), float(row[5])


def _print_lane_measurements(lane_name, lane_objects):
    print(f"{lane_name}:")
    if lane_objects is None or len(lane_objects) == 0:
        print("  -")
        return

    for obj in lane_objects:
        velocity, distance = _get_velocity_distance(obj)
        print(f"  v={velocity:.2f} m/s, distance={distance:.2f} m")


def _draw_lane_objects(ax, lane_name, lane_objects, color, marker):
    xy = _get_lane_xy(lane_objects)
    if xy.size == 0:
        return

    ax.scatter(
        xy[:, 0],
        xy[:, 1],
        s=120,
        c=color,
        marker=marker,
        edgecolors="black",
        linewidths=1.0,
        label=lane_name,
        zorder=5,
    )


def visualize_points(fig, ax, x, y, lane_1_velocity_obj, lane_2_velocity_obj, lane_3_velocity_obj):
    print("\033c", end="")
    _print_lane_measurements("Lane 1", lane_1_velocity_obj)
    _print_lane_measurements("Lane 2", lane_2_velocity_obj)
    _print_lane_measurements("Lane 3", lane_3_velocity_obj)

    ax.clear()

    if hasattr(fig, "_awr_colorbar"):
        fig._awr_colorbar.remove()  # Remove the old DBSCAN colorbar because lane view no longer uses cluster colors.
        delattr(fig, "_awr_colorbar")

    az_min_deg, az_max_deg, boresight_deg, range_max_m = _resolve_fov_config()
    _draw_fov_overlay(ax, az_min_deg, az_max_deg, boresight_deg, range_max_m)
    _draw_lane_grid(ax)

    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    if x.size != 0:
        abs_deg = np.degrees(np.arctan2(y, x))
        rel_deg = ((abs_deg - boresight_deg + 180.0) % 360.0) - 180.0
        distances = np.hypot(x, y)
        in_fov_mask = (
            (rel_deg >= az_min_deg)
            & (rel_deg <= az_max_deg)
            & (distances <= range_max_m)
            & (y >= 0.0)
        )

        ax.scatter(
            x[in_fov_mask],
            y[in_fov_mask],
            s=28,
            c="dimgray",
            alpha=0.8,
            label="Radar point",
            zorder=3,
        )

    for lane_name, _, _, color, marker in _lane_bounds():
        if lane_name == "Lane 1":
            _draw_lane_objects(ax, lane_name, lane_1_velocity_obj, color, marker)
        elif lane_name == "Lane 2":
            _draw_lane_objects(ax, lane_name, lane_2_velocity_obj, color, marker)
        else:
            _draw_lane_objects(ax, lane_name, lane_3_velocity_obj, color, marker)

    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")
    ax.set_title("AWR6843 Radar Points / Lane Grid")
    ax.set_xlim(VIEW_X_MIN, VIEW_X_MAX)
    ax.set_ylim(VIEW_Y_MIN, VIEW_Y_MAX)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.legend(loc="upper right")

    plt.pause(0.001)
