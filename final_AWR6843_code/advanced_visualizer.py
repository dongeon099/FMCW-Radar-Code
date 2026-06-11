# -*- coding: utf-8 -*-
"""Advanced real-time visualizer backend.

PyQtGraph install:
    pip install pyqtgraph PyQt5

PySide6 can also be used by pyqtgraph if it is installed instead of PyQt5.
This module keeps all GUI dependencies optional so the sensor/tracking/SPI
pipeline can continue to run when the GUI stack is not installed.
"""

from __future__ import annotations

import logging
import math
import time
from collections import defaultdict, deque
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import config as default_config


logger = logging.getLogger(__name__)


RISK_NAMES = {
    0: "CLEAR",
    1: "CAUTION",
    2: "WARNING",
}


def _get_config_value(config_obj, name: str, default):
    """Read a setting from a dict, config module, or fallback module."""
    if isinstance(config_obj, dict) and name in config_obj:
        return config_obj[name]
    if hasattr(config_obj, name):
        return getattr(config_obj, name)
    return getattr(default_config, name, default)


def _as_float(value, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _xy_from_objects(objects: Optional[Iterable]) -> Tuple[List[float], List[float]]:
    """Accept either dict detections/tracks or raw [x, y, z, v] point rows."""
    xs: List[float] = []
    ys: List[float] = []
    for obj in objects or []:
        if isinstance(obj, dict):
            xs.append(_as_float(obj.get("x")))
            ys.append(_as_float(obj.get("y")))
        else:
            try:
                xs.append(_as_float(obj[0]))
                ys.append(_as_float(obj[1]))
            except Exception:
                continue
    return xs, ys


def _front_distance(track: Dict) -> float:
    y_distance = _as_float(track.get("y"))
    if y_distance > 0.0:
        return y_distance
    return _as_float(track.get("distance"))


def _closing_speed(track: Dict) -> float:
    vy = _as_float(track.get("vy"))
    if vy < 0.0:
        return abs(vy)
    radial = _as_float(track.get("radial_velocity", track.get("v")))
    negative_is_closing = bool(
        _get_config_value(default_config, "RADIAL_VELOCITY_NEGATIVE_IS_CLOSING", True)
    )
    if negative_is_closing and radial < 0.0:
        return abs(radial)
    if not negative_is_closing and radial > 0.0:
        return radial
    return 0.0


def _track_ttc(track: Dict) -> Optional[float]:
    distance = _front_distance(track)
    closing_speed = _closing_speed(track)
    if distance <= 0.0 or closing_speed <= 0.0:
        return None
    return distance / closing_speed


def _format_ttc(value: Optional[float]) -> str:
    if value is None:
        return "-"
    return f"{value:.1f}s"


def _risk_name(value: int) -> str:
    return RISK_NAMES.get(int(value), str(value))


def _lane_code(label: str) -> str:
    if label == "left":
        return "L"
    if label == "right":
        return "R"
    return "U"


def _packet_hex(packet: Optional[Sequence[int]]) -> str:
    if not packet:
        return "-"
    return " ".join(f"{int(byte) & 0xFF:02X}" for byte in packet)


class MockVisualizer:
    """No-op fallback used when visualization is off or GUI dependencies are missing."""

    def update(self, *args, **kwargs) -> None:
        return

    def close(self) -> None:
        return


class LegacyVisualizer:
    """Lazy wrapper around the original matplotlib visualizer.

    The old visualizer remains available only when VISUALIZER_BACKEND="legacy".
    This keeps matplotlib out of the default PyQtGraph path.
    """

    def __init__(self, config_obj):
        self.enabled = False
        try:
            import matplotlib.pyplot as plt
            from visualizer import visualize_points

            self._plt = plt
            self._visualize_points = visualize_points
            self._plt.ion()
            self._fig, self._ax = self._plt.subplots()
            self.enabled = True
            logger.info("legacy matplotlib visualizer enabled")
        except Exception as exc:
            logger.warning("legacy visualizer unavailable: %s", exc)

    def update(
        self,
        detections,
        clusters,
        tracks,
        lane_result,
        spi_packet,
        frame_id,
        dt,
        processing_time_ms,
        advice=None,
    ) -> None:
        if not self.enabled:
            return
        try:
            x, y = _xy_from_objects(detections)
            left_objects = getattr(lane_result, "left_objects", [])
            right_objects = getattr(lane_result, "right_objects", [])
            self._visualize_points(self._fig, self._ax, x, y, left_objects, [], right_objects)
        except Exception as exc:
            logger.warning("legacy visualizer update failed: %s", exc)

    def close(self) -> None:
        if not self.enabled:
            return
        try:
            self._plt.close(self._fig)
        except Exception:
            pass


class AdvancedVisualizer:
    """PyQtGraph-based real-time top-view visualizer."""

    def __init__(self, config_obj):
        self.config = config_obj
        self.enabled = False
        self._last_draw_time = 0.0
        self._fps = 0.0
        self._history = defaultdict(
            lambda: deque(
                maxlen=int(_get_config_value(self.config, "TRACK_HISTORY_LENGTH", 30))
            )
        )
        self._history_missing_count: Dict[int, int] = defaultdict(int)
        self._history_curves: Dict[int, object] = {}
        self._track_text_items: List[object] = []

        self.update_hz = float(_get_config_value(config_obj, "VISUALIZER_UPDATE_HZ", 20))
        self.x_range = tuple(_get_config_value(config_obj, "VISUALIZER_X_RANGE", (-5.0, 5.0)))
        self.y_range = tuple(_get_config_value(config_obj, "VISUALIZER_Y_RANGE", (0.0, 30.0)))
        self.show_raw = bool(_get_config_value(config_obj, "SHOW_RAW_DETECTIONS", True))
        self.show_clusters = bool(_get_config_value(config_obj, "SHOW_CLUSTERS", True))
        self.show_tracks = bool(_get_config_value(config_obj, "SHOW_TRACKS", True))
        self.show_history = bool(_get_config_value(config_obj, "SHOW_TRACK_HISTORY", True))
        self.show_velocity = bool(_get_config_value(config_obj, "SHOW_VELOCITY_VECTOR", True))
        self.show_spi = bool(_get_config_value(config_obj, "SHOW_SPI_PACKET", True))
        self.show_debug = bool(_get_config_value(config_obj, "SHOW_DEBUG_PANEL", True))
        self.history_length = int(_get_config_value(config_obj, "TRACK_HISTORY_LENGTH", 30))

        try:
            import pyqtgraph as pg
            from pyqtgraph.Qt import QtWidgets

            self.pg = pg
            self.QtWidgets = QtWidgets
            self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            self._build_window()
            self.enabled = True
            logger.info("PyQtGraph advanced visualizer enabled")
        except Exception as exc:
            logger.warning(
                "PyQtGraph visualizer unavailable: %s. Install with: pip install pyqtgraph PyQt5",
                exc,
            )

    def _build_window(self) -> None:
        pg = self.pg
        QtWidgets = self.QtWidgets

        pg.setConfigOptions(antialias=True)

        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("AWR Tracking Advanced Visualizer")
        layout = QtWidgets.QGridLayout(self.window)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("k")
        self.plot.setLabel("bottom", "X left/right", units="m")
        self.plot.setLabel("left", "Y forward", units="m")
        self.plot.setXRange(float(self.x_range[0]), float(self.x_range[1]), padding=0.0)
        self.plot.setYRange(float(self.y_range[0]), float(self.y_range[1]), padding=0.0)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend(offset=(10, 10))
        layout.addWidget(self.plot, 0, 0, 3, 1)

        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        layout.addWidget(right_panel, 0, 1, 3, 1)

        self.risk_panel = self._make_text_panel("Risk Status")
        self.spi_panel = self._make_text_panel("SPI Packet")
        self.debug_panel = self._make_text_panel("Track Debug")
        right_layout.addWidget(self.risk_panel)
        right_layout.addWidget(self.spi_panel)
        right_layout.addWidget(self.debug_panel, stretch=1)

        self._draw_lane_regions()

        self.ego_item = pg.ScatterPlotItem(
            x=[0.0],
            y=[0.0],
            size=18,
            symbol="t",
            brush=pg.mkBrush(80, 200, 255, 230),
            pen=pg.mkPen("w", width=1),
            name="ego",
        )
        self.raw_item = pg.ScatterPlotItem(
            size=4,
            brush=pg.mkBrush(160, 160, 160, 150),
            pen=None,
            name="raw detections",
        )
        self.cluster_item = pg.ScatterPlotItem(
            size=9,
            brush=pg.mkBrush(255, 210, 70, 210),
            pen=pg.mkPen((255, 255, 255, 120)),
            name="cluster centers",
        )
        self.tentative_item = pg.ScatterPlotItem(
            size=10,
            brush=pg.mkBrush(90, 160, 255, 80),
            pen=pg.mkPen((90, 160, 255, 160), width=1),
            name="tentative tracks",
        )
        self.confirmed_item = pg.ScatterPlotItem(
            size=15,
            brush=pg.mkBrush(80, 220, 130, 170),
            pen=pg.mkPen((80, 255, 160, 240), width=2),
            name="confirmed tracks",
        )
        self.warning_item = pg.ScatterPlotItem(
            size=24,
            brush=pg.mkBrush(255, 70, 70, 90),
            pen=pg.mkPen((255, 80, 80, 255), width=3),
            name="warning tracks",
        )
        self.prediction_item = pg.ScatterPlotItem(
            size=7,
            symbol="x",
            brush=pg.mkBrush(180, 120, 255, 180),
            pen=pg.mkPen((180, 120, 255, 220)),
            name="EKF prediction",
        )
        self.updated_item = pg.ScatterPlotItem(
            size=7,
            symbol="+",
            brush=pg.mkBrush(80, 255, 220, 180),
            pen=pg.mkPen((80, 255, 220, 220)),
            name="EKF update",
        )
        self.velocity_item = pg.PlotDataItem(
            pen=pg.mkPen((120, 220, 255, 220), width=1.5),
            name="velocity vectors",
        )

        for item in (
            self.raw_item,
            self.cluster_item,
            self.tentative_item,
            self.confirmed_item,
            self.warning_item,
            self.prediction_item,
            self.updated_item,
            self.velocity_item,
            self.ego_item,
        ):
            self.plot.addItem(item)

        self.window.resize(1350, 850)
        self.window.show()

    def _make_text_panel(self, title: str):
        panel = self.QtWidgets.QPlainTextEdit()
        panel.setReadOnly(True)
        panel.setMinimumWidth(360)
        panel.setStyleSheet(
            "QPlainTextEdit { background: #111; color: #eee; "
            "font-family: Consolas, monospace; font-size: 11px; }"
        )
        panel.setPlainText(title)
        return panel

    def _draw_lane_regions(self) -> None:
        pg = self.pg
        left_range = tuple(_get_config_value(self.config, "LEFT_LANE_X_RANGE", (-3.5, -0.5)))
        right_range = tuple(_get_config_value(self.config, "RIGHT_LANE_X_RANGE", (0.5, 3.5)))

        lane_specs = [
            ("LEFT", left_range, pg.mkBrush(255, 160, 60, 35), pg.mkPen((255, 160, 60, 120))),
            ("RIGHT", right_range, pg.mkBrush(60, 180, 255, 35), pg.mkPen((60, 180, 255, 120))),
        ]
        for name, lane_range, brush, pen in lane_specs:
            region = pg.LinearRegionItem(
                values=(float(lane_range[0]), float(lane_range[1])),
                orientation="vertical",
                movable=False,
                brush=brush,
            )
            region.setZValue(-10)
            self.plot.addItem(region)
            for edge in lane_range:
                self.plot.addItem(pg.InfiniteLine(pos=float(edge), angle=90, pen=pen))
            label = pg.TextItem(name, color=(220, 220, 220), anchor=(0.5, 0.0))
            label.setPos((float(lane_range[0]) + float(lane_range[1])) / 2.0, self.y_range[1] * 0.96)
            self.plot.addItem(label)

    def update(
        self,
        detections,
        clusters,
        tracks,
        lane_result,
        spi_packet,
        frame_id,
        dt,
        processing_time_ms,
        advice=None,
    ) -> None:
        if not self.enabled:
            return
        try:
            merged_tracks = self._merge_lane_info(tracks or [], lane_result)
            self._update_track_history(merged_tracks)

            now = time.monotonic()
            min_interval = 1.0 / self.update_hz if self.update_hz > 0.0 else 0.0
            if min_interval > 0.0 and now - self._last_draw_time < min_interval:
                self._process_events()
                return

            if self._last_draw_time > 0.0:
                instant_fps = 1.0 / max(now - self._last_draw_time, 1e-6)
                self._fps = instant_fps if self._fps == 0.0 else (0.85 * self._fps + 0.15 * instant_fps)
            self._last_draw_time = now

            self._update_plot_items(detections, clusters, merged_tracks)
            self._update_text_items(merged_tracks)
            self._update_panels(
                lane_result,
                merged_tracks,
                spi_packet,
                frame_id,
                dt,
                processing_time_ms,
                advice=advice,
            )
            self._process_events()
        except Exception as exc:
            logger.warning("Visualizer update failed: %s", exc)

    def _merge_lane_info(self, tracks: Iterable[Dict], lane_result) -> List[Dict]:
        lane_objects = []
        lane_objects.extend(getattr(lane_result, "left_objects", []) or [])
        lane_objects.extend(getattr(lane_result, "right_objects", []) or [])
        lane_by_id = {obj.get("track_id"): obj for obj in lane_objects}

        merged: List[Dict] = []
        for track in tracks:
            copied = dict(track)
            lane_obj = lane_by_id.get(copied.get("track_id"))
            if lane_obj:
                copied["lane_label"] = lane_obj.get("lane_label", copied.get("lane_label", "unknown"))
                copied["risk_level"] = lane_obj.get("risk_level", copied.get("risk_level", 0))
                copied["ttc"] = lane_obj.get("ttc", _track_ttc(copied))
            else:
                copied.setdefault("lane_label", "unknown")
                copied.setdefault("risk_level", 0)
                copied.setdefault("ttc", _track_ttc(copied))
            merged.append(copied)
        return merged

    def _update_track_history(self, tracks: Iterable[Dict]) -> None:
        active_ids = set()
        for track in tracks:
            track_id = track.get("track_id")
            if track_id is None:
                continue
            track_id = int(track_id)
            active_ids.add(track_id)
            self._history[track_id].append((_as_float(track.get("x")), _as_float(track.get("y"))))
            self._history_missing_count[track_id] = 0

        for track_id in list(self._history.keys()):
            if track_id in active_ids:
                continue
            self._history_missing_count[track_id] += 1
            if self._history_missing_count[track_id] > self.history_length:
                self._remove_history(track_id)

    def _remove_history(self, track_id: int) -> None:
        self._history.pop(track_id, None)
        self._history_missing_count.pop(track_id, None)
        curve = self._history_curves.pop(track_id, None)
        if curve is not None:
            self.plot.removeItem(curve)

    def _update_plot_items(self, detections, clusters, tracks: List[Dict]) -> None:
        self._set_scatter_xy(self.raw_item, detections if self.show_raw else [])
        self._set_scatter_xy(self.cluster_item, clusters if self.show_clusters else [])

        tentative = [track for track in tracks if track.get("status") != "confirmed"]
        confirmed = [track for track in tracks if track.get("status") == "confirmed"]
        warning = [track for track in tracks if int(track.get("risk_level", 0)) >= 2]

        if self.show_tracks:
            self._set_scatter_xy(self.tentative_item, tentative)
            self._set_scatter_xy(self.confirmed_item, confirmed)
            self._set_scatter_xy(self.warning_item, warning)
        else:
            self._set_scatter_xy(self.tentative_item, [])
            self._set_scatter_xy(self.confirmed_item, [])
            self._set_scatter_xy(self.warning_item, [])

        self._update_velocity_vectors(tracks if self.show_velocity else [])
        self._update_history_curves() if self.show_history else self._clear_history_curves()
        self._update_ekf_debug_points(tracks)

    def _set_scatter_xy(self, item, objects) -> None:
        xs, ys = _xy_from_objects(objects)
        item.setData(x=xs, y=ys)

    def _update_velocity_vectors(self, tracks: Iterable[Dict]) -> None:
        xs: List[float] = []
        ys: List[float] = []
        scale = 0.4
        for track in tracks:
            x = _as_float(track.get("x"))
            y = _as_float(track.get("y"))
            vx = _as_float(track.get("vx"))
            vy = _as_float(track.get("vy"))
            if vx == 0.0 and vy == 0.0:
                continue
            xs.extend([x, x + vx * scale, math.nan])
            ys.extend([y, y + vy * scale, math.nan])
        self.velocity_item.setData(xs, ys)

    def _update_history_curves(self) -> None:
        pg = self.pg
        for track_id, points in self._history.items():
            if track_id not in self._history_curves:
                pen = pg.mkPen(
                    ((80 + track_id * 37) % 255, (160 + track_id * 53) % 255, 220, 150),
                    width=1,
                )
                curve = pg.PlotDataItem(pen=pen)
                self._history_curves[track_id] = curve
                self.plot.addItem(curve)
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            self._history_curves[track_id].setData(xs, ys)

    def _clear_history_curves(self) -> None:
        for curve in self._history_curves.values():
            curve.setData([], [])

    def _update_ekf_debug_points(self, tracks: Iterable[Dict]) -> None:
        prediction_points = []
        updated_points = []
        for track in tracks:
            if track.get("predicted_x") is not None and track.get("predicted_y") is not None:
                prediction_points.append({"x": track.get("predicted_x"), "y": track.get("predicted_y")})
            debug_info = track.get("debug_info") or {}
            if debug_info.get("predicted_x") is not None and debug_info.get("predicted_y") is not None:
                prediction_points.append(
                    {"x": debug_info.get("predicted_x"), "y": debug_info.get("predicted_y")}
                )
            if track.get("updated_x") is not None and track.get("updated_y") is not None:
                updated_points.append({"x": track.get("updated_x"), "y": track.get("updated_y")})
            if debug_info.get("updated_x") is not None and debug_info.get("updated_y") is not None:
                updated_points.append({"x": debug_info.get("updated_x"), "y": debug_info.get("updated_y")})
        self._set_scatter_xy(self.prediction_item, prediction_points)
        self._set_scatter_xy(self.updated_item, updated_points)

    def _update_text_items(self, tracks: List[Dict]) -> None:
        if not self.show_tracks:
            for item in self._track_text_items:
                item.setVisible(False)
            return

        self._ensure_text_item_count(len(tracks))
        for index, track in enumerate(tracks):
            item = self._track_text_items[index]
            track_id = track.get("track_id", "-")
            lane = _lane_code(track.get("lane_label", "unknown"))
            ttc = _format_ttc(track.get("ttc", _track_ttc(track)))
            item.setText(f"ID {track_id} | {lane} | TTC {ttc}")
            item.setPos(_as_float(track.get("x")) + 0.08, _as_float(track.get("y")) + 0.08)
            item.setVisible(True)

        for item in self._track_text_items[len(tracks) :]:
            item.setVisible(False)

    def _ensure_text_item_count(self, count: int) -> None:
        while len(self._track_text_items) < count:
            item = self.pg.TextItem(color=(235, 235, 235), anchor=(0.0, 1.0))
            self.plot.addItem(item)
            self._track_text_items.append(item)

    def _update_panels(
        self,
        lane_result,
        tracks: List[Dict],
        spi_packet,
        frame_id,
        dt,
        processing_time_ms,
        advice=None,
    ) -> None:
        left_objects = getattr(lane_result, "left_objects", []) or []
        right_objects = getattr(lane_result, "right_objects", []) or []
        left_risk = int(getattr(lane_result, "left_risk", 0))
        right_risk = int(getattr(lane_result, "right_risk", 0))
        left_nearest = self._nearest_distance(left_objects)
        right_nearest = self._nearest_distance(right_objects)
        min_ttc = self._minimum_ttc(left_objects + right_objects)

        risk_lines = [
            "Risk Status",
            f"LEFT RISK : {_risk_name(left_risk)}",
            f"RIGHT RISK: {_risk_name(right_risk)}",
            f"left object count : {len(left_objects)}",
            f"right object count: {len(right_objects)}",
            f"nearest left distance : {left_nearest}",
            f"nearest right distance: {right_nearest}",
            f"minimum TTC: {_format_ttc(min_ttc)}",
        ]
        risk_lines.extend(self._format_advice_lines(advice))
        risk_lines.extend(
            [
                f"frame id: {frame_id}",
                f"FPS: {self._fps:.1f}",
                f"dt: {float(dt):.4f}s",
                f"processing: {float(processing_time_ms):.2f} ms",
            ]
        )
        self.risk_panel.setPlainText("\n".join(risk_lines))

        self.spi_panel.setVisible(self.show_spi)
        if self.show_spi:
            self.spi_panel.setPlainText(self._format_spi_panel(spi_packet))

        self.debug_panel.setVisible(self.show_debug)
        if self.show_debug:
            self.debug_panel.setPlainText(self._format_debug_panel(tracks))

    def _advice_value(self, advice, name: str, default=None):
        if advice is None:
            return default
        if isinstance(advice, dict):
            return advice.get(name, default)
        return getattr(advice, name, default)

    def _format_advice_number(self, value, unit: str = "") -> str:
        if value is None:
            return "-"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return "-"
        if not math.isfinite(number):
            return "-"
        return f"{number:.2f}{unit}"

    def _format_advice_lines(self, advice) -> List[str]:
        if advice is None:
            return []

        return [
            "",
            "Vehicle / Lane Change",
            f"turn signal: {self._advice_value(advice, 'turn_signal', '-')}",
            f"ego speed: {self._format_advice_number(self._advice_value(advice, 'ego_speed_kmh'), ' km/h')}",
            f"possible: {self._advice_value(advice, 'lane_change_possible', '-')}",
            f"ego speed SI: {self._format_advice_number(self._advice_value(advice, 'ego_current_speed_mps'), ' m/s')}",
            f"recommended speed: {self._format_advice_number(self._advice_value(advice, 'ego_required_speed_mps'), ' m/s')}",
            f"steering angle: {self._format_advice_number(self._advice_value(advice, 'current_steering_angle_deg'), ' deg')}",
            f"SPI valid: {self._advice_value(advice, 'miso_valid', '-')}",
            f"SPI sequence: {self._advice_value(advice, 'spi_sequence', '-')}",
            f"SPI frames good/bad: {self._advice_value(advice, 'spi_valid_count', '-')} / {self._advice_value(advice, 'spi_invalid_count', '-')}",
            f"ACC reason: {self._advice_value(advice, 'acc_reason', '-')}",
            f"ACC lead distance: {self._format_advice_number(self._advice_value(advice, 'acc_lead_distance_m'), ' m')}",
            f"ACC TTC: {self._format_advice_number(self._advice_value(advice, 'acc_ttc_sec'), ' s')}",
            f"required accel: {self._format_advice_number(self._advice_value(advice, 'ego_required_accel_mps2'), ' m/s^2')}",
            f"target id: {self._advice_value(advice, 'target_object_id', '-')}",
            f"future gap: {self._format_advice_number(self._advice_value(advice, 'predicted_gap_after_lane_change_m'), ' m')}",
            f"safe gap: {self._format_advice_number(self._advice_value(advice, 'required_safe_gap_m'), ' m')}",
            f"reason: {self._advice_value(advice, 'reason', '-')}",
        ]

    def _nearest_distance(self, objects: Iterable[Dict]) -> str:
        distances = [_front_distance(obj) for obj in objects if _front_distance(obj) > 0.0]
        if not distances:
            return "-"
        return f"{min(distances):.2f} m"

    def _minimum_ttc(self, objects: Iterable[Dict]) -> Optional[float]:
        values = []
        for obj in objects:
            ttc = obj.get("ttc", _track_ttc(obj))
            if ttc is not None:
                values.append(float(ttc))
        return min(values) if values else None

    def _format_spi_panel(self, packet: Optional[Sequence[int]]) -> str:
        packet = list(packet or [])
        turn_request = packet[4] if len(packet) > 4 else "-"
        object_count = packet[5] if len(packet) > 5 else "-"
        risk_level = packet[6] if len(packet) > 6 else "-"
        lane_id = packet[7] if len(packet) > 7 else "-"
        recommended_speed = (
            ((packet[8] | (packet[9] << 8)) / 100.0) if len(packet) > 9 else None
        )
        safe_distance = (
            ((packet[10] | (packet[11] << 8)) / 100.0) if len(packet) > 11 else None
        )
        ttc_raw = (packet[12] | (packet[13] << 8)) if len(packet) > 13 else None
        ttc = None if ttc_raw in (None, 0xFFFF) else ttc_raw / 100.0
        checksum = f"{packet[-1] & 0xFF:02X}" if packet else "-"
        return "\n".join(
            [
                "SPI Packet",
                f"SPI TX: {_packet_hex(packet)}",
                f"turn_request: {turn_request}",
                f"object_count: {object_count}",
                f"risk_level: {risk_level}",
                f"lane_id: {lane_id}",
                f"ACC recommended speed: {recommended_speed if recommended_speed is not None else '-'} m/s",
                f"ACC safe distance: {safe_distance if safe_distance is not None else '-'} m",
                f"ACC TTC: {ttc if ttc is not None else '-'} s",
                f"checksum: {checksum}",
            ]
        )

    def _format_debug_panel(self, tracks: Iterable[Dict]) -> str:
        lines = [
            "Track Debug",
            "id | x | y | vx | vy | age | hits | miss | status | lane | risk | ttc",
        ]
        for track in tracks:
            ttc = track.get("ttc", _track_ttc(track))
            lines.append(
                " | ".join(
                    [
                        str(track.get("track_id", "-")),
                        f"{_as_float(track.get('x')):.2f}",
                        f"{_as_float(track.get('y')):.2f}",
                        f"{_as_float(track.get('vx')):.2f}",
                        f"{_as_float(track.get('vy')):.2f}",
                        str(track.get("age", "-")),
                        str(track.get("hits", "-")),
                        str(track.get("missed_count", "-")),
                        str(track.get("status", "-")),
                        str(track.get("lane_label", "unknown")),
                        str(track.get("risk_level", 0)),
                        _format_ttc(ttc),
                    ]
                )
            )
        return "\n".join(lines)

    def _process_events(self) -> None:
        try:
            self.app.processEvents()
        except Exception as exc:
            logger.warning("Visualizer event processing failed: %s", exc)

    def close(self) -> None:
        if not self.enabled:
            return
        try:
            self.window.close()
            self._process_events()
        except Exception as exc:
            logger.warning("Visualizer close failed: %s", exc)


def create_visualizer(config_obj):
    """Create the configured visualizer backend.

    VISUALIZER_BACKEND:
        "off"       -> no visualizer
        "pyqtgraph" -> AdvancedVisualizer
        "legacy"    -> original matplotlib visualizer wrapper
    """
    enabled = bool(_get_config_value(config_obj, "ENABLE_VISUALIZER", True))
    backend = str(_get_config_value(config_obj, "VISUALIZER_BACKEND", "pyqtgraph")).lower()

    if not enabled or backend == "off":
        logger.info("visualizer disabled")
        return None
    if backend == "legacy":
        visualizer = LegacyVisualizer(config_obj)
        return visualizer if visualizer.enabled else MockVisualizer()
    if backend == "pyqtgraph":
        visualizer = AdvancedVisualizer(config_obj)
        return visualizer if visualizer.enabled else MockVisualizer()

    logger.warning("unknown VISUALIZER_BACKEND=%s; visualizer disabled", backend)
    return MockVisualizer()
