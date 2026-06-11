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
import os
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
    front_offset = _as_float(
        _get_config_value(default_config, "EGO_FRONT_OFFSET_M", 0.0)
    )
    y_distance = _as_float(track.get("y"))
    if y_distance > 0.0:
        return max(0.0, y_distance - front_offset)
    distance = _as_float(track.get("distance"))
    if distance > 0.0:
        return max(0.0, distance - front_offset)
    x_distance = _as_float(track.get("x"))
    return max(0.0, math.hypot(x_distance, y_distance) - front_offset)


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


def _lane_display_name(label: str) -> str:
    text = str(label or "").lower()
    if text == "left":
        return "LEFT"
    if text == "center":
        return "CENTER"
    if text == "right":
        return "RIGHT"
    return "CENTER"


def _packet_hex(packet: Optional[Sequence[int]]) -> str:
    if not packet:
        return "-"
    return " ".join(f"{int(byte) & 0xFF:02X}" for byte in packet)


def _turn_signal_name(value) -> str:
    try:
        value = int(value)
    except (TypeError, ValueError):
        return "UNKNOWN"
    if value == getattr(default_config, "TURN_SIGNAL_LEFT", 1):
        return "LEFT"
    if value == getattr(default_config, "TURN_SIGNAL_RIGHT", 2):
        return "RIGHT"
    if value == getattr(default_config, "TURN_SIGNAL_HAZARD", 3):
        return "HAZARD"
    return "NONE"


def _selected_lane_name(value) -> str:
    try:
        value = int(value)
    except (TypeError, ValueError):
        return "-"
    if value == 1:
        return "LEFT"
    if value == 2:
        return "CENTER"
    if value == 3:
        return "RIGHT"
    return "-"


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


class DrivingDisplayCanvas:
    """Perspective driving display with fixed three-lane road."""

    def __init__(
        self,
        QtWidgets,
        QtCore,
        QtGui,
        view_range_m: float,
        lane_width_m: float,
        ego_vertical_position: float,
        ego_scale: float,
    ):
        self.QtCore = QtCore
        self.QtGui = QtGui
        self.widget = QtWidgets.QWidget()
        self.widget.setMinimumSize(760, 720)
        self.widget.setStyleSheet("background: #000000;")
        self.view_range_m = max(1.0, float(view_range_m))
        self.lane_width_m = max(0.001, float(lane_width_m))
        self.ego_vertical_position = max(0.35, min(0.86, float(ego_vertical_position)))
        self.ego_scale = max(0.4, min(1.2, float(ego_scale)))
        self.tracks: List[Dict] = []
        self.selected_lane = 0
        self.left_risk = 0
        self.right_risk = 0
        asset_path = os.path.join(os.path.dirname(__file__), "assets", "ego_car.png")
        self.ego_pixmap = QtGui.QPixmap(asset_path)
        self.widget.paintEvent = self._paint_event

    def update_scene(self, tracks: Iterable[Dict], selected_lane, left_risk: int, right_risk: int) -> None:
        self.tracks = [dict(track) for track in tracks or []]
        try:
            self.selected_lane = int(selected_lane)
        except (TypeError, ValueError):
            self.selected_lane = 0
        self.left_risk = int(left_risk)
        self.right_risk = int(right_risk)
        self.widget.update()

    def _paint_event(self, _event) -> None:
        painter = self.QtGui.QPainter(self.widget)
        painter.setRenderHint(self.QtGui.QPainter.Antialiasing, True)
        rect = self.widget.rect()
        painter.fillRect(rect, self.QtGui.QColor("#000000"))
        self._draw_scene_background(painter, rect)
        self._draw_road_surface(painter, rect)
        self._draw_lane_highlight(painter, rect)
        self._draw_lane_lines(painter, rect)
        self._draw_depth_fog(painter, rect)
        self._draw_detected_vehicles(painter, rect)
        self._draw_ego_vehicle(painter, rect)
        painter.end()

    def _draw_scene_background(self, painter, rect) -> None:
        gradient = self.QtGui.QLinearGradient(0, 0, 0, rect.height())
        gradient.setColorAt(0.00, self.QtGui.QColor(0, 0, 0))
        gradient.setColorAt(0.24, self.QtGui.QColor(2, 3, 5))
        gradient.setColorAt(0.55, self.QtGui.QColor(9, 12, 15))
        gradient.setColorAt(1.00, self.QtGui.QColor(0, 0, 0))
        painter.fillRect(rect, self.QtGui.QBrush(gradient))

        horizon = self.QtCore.QRectF(rect.width() * 0.18, rect.height() * 0.22, rect.width() * 0.64, 4)
        glow = self.QtGui.QLinearGradient(horizon.left(), 0, horizon.right(), 0)
        glow.setColorAt(0.0, self.QtGui.QColor(0, 0, 0, 0))
        glow.setColorAt(0.5, self.QtGui.QColor(170, 190, 210, 80))
        glow.setColorAt(1.0, self.QtGui.QColor(0, 0, 0, 0))
        painter.fillRect(horizon, self.QtGui.QBrush(glow))

    def _road_points(self, rect):
        w = rect.width()
        h = rect.height()
        return {
            "horizon_y": h * 0.16,
            "bottom_y": h * 1.14,
            "top_center": w * 0.50,
            "bottom_center": w * 0.50,
            "top_lane_width": w * 0.038,
            "bottom_lane_width": w * 0.235,
        }

    def _lane_boundary_x(self, rect, boundary_index: int, t: float) -> float:
        road = self._road_points(rect)
        top = road["top_center"] + boundary_index * road["top_lane_width"]
        bottom = road["bottom_center"] + boundary_index * road["bottom_lane_width"]
        curve = t ** 1.34
        side_bend = boundary_index * rect.width() * 0.030 * math.sin(t * math.pi * 0.78)
        return top * (1.0 - curve) + bottom * curve + side_bend

    def _screen_y_for_distance(self, rect, distance_m: float) -> float:
        road = self._road_points(rect)
        distance_m = max(0.0, min(self.view_range_m, distance_m))
        near = 1.0 - distance_m / self.view_range_m
        t = max(0.0, min(1.0, near ** 0.52))
        visible_bottom_y = rect.height() * 0.90
        return road["horizon_y"] * (1.0 - t) + visible_bottom_y * t

    def _t_for_y(self, rect, y: float) -> float:
        road = self._road_points(rect)
        return max(0.0, min(1.0, (y - road["horizon_y"]) / (road["bottom_y"] - road["horizon_y"])))

    def _draw_road_surface(self, painter, rect) -> None:
        road = self._road_points(rect)
        polygon = self.QtGui.QPolygonF(
            [
                self.QtCore.QPointF(self._lane_boundary_x(rect, -1.5, 0.0), road["horizon_y"]),
                self.QtCore.QPointF(self._lane_boundary_x(rect, 1.5, 0.0), road["horizon_y"]),
                self.QtCore.QPointF(self._lane_boundary_x(rect, 1.5, 1.0), road["bottom_y"]),
                self.QtCore.QPointF(self._lane_boundary_x(rect, -1.5, 1.0), road["bottom_y"]),
            ]
        )
        road_gradient = self.QtGui.QLinearGradient(0, road["horizon_y"], 0, road["bottom_y"])
        road_gradient.setColorAt(0.0, self.QtGui.QColor(16, 18, 21, 235))
        road_gradient.setColorAt(0.55, self.QtGui.QColor(29, 32, 35, 245))
        road_gradient.setColorAt(1.0, self.QtGui.QColor(8, 9, 10, 255))
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QBrush(road_gradient))
        painter.drawPolygon(polygon)

        self._draw_road_texture(painter, rect)
        self._draw_headlight_wash(painter, rect)

    def _draw_depth_fog(self, painter, rect) -> None:
        road = self._road_points(rect)
        fog = self.QtGui.QLinearGradient(0, road["horizon_y"], 0, rect.height() * 0.52)
        fog.setColorAt(0.0, self.QtGui.QColor(0, 0, 0, 170))
        fog.setColorAt(0.45, self.QtGui.QColor(20, 30, 40, 42))
        fog.setColorAt(1.0, self.QtGui.QColor(0, 0, 0, 0))
        painter.fillRect(
            self.QtCore.QRectF(0, road["horizon_y"] - 20, rect.width(), rect.height() * 0.42),
            self.QtGui.QBrush(fog),
        )

    def _draw_road_texture(self, painter, rect) -> None:
        road = self._road_points(rect)
        pen = self.QtGui.QPen(self.QtGui.QColor(255, 255, 255, 16), 1)
        painter.setPen(pen)
        for index in range(0, 130):
            t = (index % 65) / 65.0
            y = road["horizon_y"] + (road["bottom_y"] - road["horizon_y"]) * t
            left = self._lane_boundary_x(rect, -1.45, t)
            right = self._lane_boundary_x(rect, 1.45, t)
            span = right - left
            x = left + span * ((index * 37) % 100) / 100.0
            alpha = int(8 + 34 * t)
            painter.setPen(self.QtGui.QPen(self.QtGui.QColor(230, 235, 240, alpha), 1))
            painter.drawPoint(self.QtCore.QPointF(x, y))

    def _draw_headlight_wash(self, painter, rect) -> None:
        center = self.QtCore.QPointF(rect.width() * 0.50, rect.height() * 0.70)
        gradient = self.QtGui.QRadialGradient(center, rect.width() * 0.42)
        gradient.setColorAt(0.0, self.QtGui.QColor(235, 245, 255, 95))
        gradient.setColorAt(0.38, self.QtGui.QColor(180, 210, 245, 42))
        gradient.setColorAt(1.0, self.QtGui.QColor(0, 0, 0, 0))
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QBrush(gradient))
        painter.drawEllipse(center, rect.width() * 0.48, rect.height() * 0.16)

    def _draw_lane_highlight(self, painter, rect) -> None:
        lane_index = {1: -1, 2: 0, 3: 1}.get(self.selected_lane)
        if lane_index is None:
            return
        road = self._road_points(rect)
        left_boundary = lane_index - 0.5
        right_boundary = lane_index + 0.5
        polygon = self.QtGui.QPolygonF(
            [
                self.QtCore.QPointF(self._lane_boundary_x(rect, left_boundary, 0.03), road["horizon_y"] + 8),
                self.QtCore.QPointF(self._lane_boundary_x(rect, right_boundary, 0.03), road["horizon_y"] + 8),
                self.QtCore.QPointF(self._lane_boundary_x(rect, right_boundary, 0.98), road["bottom_y"]),
                self.QtCore.QPointF(self._lane_boundary_x(rect, left_boundary, 0.98), road["bottom_y"]),
            ]
        )
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QColor(33, 150, 255, 36))
        painter.drawPolygon(polygon)

    def _draw_lane_lines(self, painter, rect) -> None:
        for boundary in (-1.5, -0.5, 0.5, 1.5):
            risk = self._risk_for_boundary(boundary)
            if risk >= 2:
                color = self.QtGui.QColor(255, 55, 55, 235)
            elif risk == 1:
                color = self.QtGui.QColor(255, 190, 45, 230)
            else:
                color = self.QtGui.QColor(0, 145, 255, 230) if boundary in (-0.5, 0.5) else self.QtGui.QColor(16, 18, 20, 230)
            width = 3.4 if boundary in (-0.5, 0.5) else 2.2
            self._draw_curved_lane_line(painter, rect, boundary, color, width)

    def _risk_for_boundary(self, boundary: float) -> int:
        if boundary <= -0.5:
            return self.left_risk
        if boundary >= 0.5:
            return self.right_risk
        return 0

    def _draw_curved_lane_line(self, painter, rect, boundary: float, color, width: float) -> None:
        road = self._road_points(rect)
        path = self.QtGui.QPainterPath()
        path.moveTo(self._lane_boundary_x(rect, boundary, 0.0), road["horizon_y"])
        for step in range(1, 48):
            t = step / 47.0
            x = self._lane_boundary_x(rect, boundary, t)
            y = road["horizon_y"] + (road["bottom_y"] - road["horizon_y"]) * t
            path.lineTo(x, y)
        glow_pen = self.QtGui.QPen(self.QtGui.QColor(color.red(), color.green(), color.blue(), 70), width + 7)
        glow_pen.setCapStyle(self.QtCore.Qt.RoundCap)
        painter.setPen(glow_pen)
        painter.drawPath(path)
        pen = self.QtGui.QPen(color, width)
        pen.setCapStyle(self.QtCore.Qt.RoundCap)
        painter.setPen(pen)
        painter.drawPath(path)

    def _draw_detected_vehicles(self, painter, rect) -> None:
        lane_label_counts = {}
        for track in sorted(self.tracks, key=lambda item: _front_distance(item), reverse=True):
            x, y, scale = self._project_track(track, rect)
            risk = int(track.get("risk_level", track.get("risk", 0)))
            if risk >= 2:
                fill = self.QtGui.QColor(215, 46, 46, 245)
                outline = self.QtGui.QColor(255, 205, 205, 245)
            elif risk == 1:
                fill = self.QtGui.QColor(225, 155, 35, 245)
                outline = self.QtGui.QColor(255, 235, 150, 245)
            else:
                fill = self.QtGui.QColor(188, 194, 199, 245)
                outline = self.QtGui.QColor(245, 248, 250, 230)
            vehicle_w = 72 * scale
            vehicle_h = 96 * scale
            self._draw_vehicle(painter, x, y, vehicle_w, vehicle_h, fill, outline)
            lane = _lane_display_name(track.get("lane_label", "center"))
            offset_index = lane_label_counts.get(lane, 0)
            lane_label_counts[lane] = offset_index + 1
            side = -1 if x > rect.width() * 0.78 else 1
            label_x = x + side * vehicle_w * 0.78
            label_y = y - vehicle_h * 0.26 - offset_index * 58
            anchor_x = x + side * vehicle_w * 0.46
            anchor_y = y - vehicle_h * 0.05
            self._draw_track_label(painter, track, label_x, label_y, risk, anchor_x, anchor_y)

    def _draw_track_label(self, painter, track: Dict, x: float, y: float, risk: int, anchor_x: float, anchor_y: float) -> None:
        distance = _front_distance(track)
        velocity = _as_float(track.get("velocity", track.get("radial_velocity", track.get("v"))))
        lane = _lane_display_name(track.get("lane_label", "center"))
        lines = [f"{distance:.1f} m", f"{velocity:.1f} m/s", lane]
        font = painter.font()
        font.setPointSize(9)
        font.setBold(True)
        painter.setFont(font)
        metrics = painter.fontMetrics()
        width = max(metrics.horizontalAdvance(line) for line in lines) + 18
        height = 52
        canvas_rect = self.widget.rect()
        x = max(12.0, min(float(x), canvas_rect.width() - width - 12.0))
        y = max(10.0, min(float(y), canvas_rect.height() - height - 18.0))
        bg = self.QtGui.QColor(120, 20, 28, 185) if risk >= 2 else self.QtGui.QColor(20, 24, 31, 165)
        border = self.QtGui.QColor(255, 75, 80, 230) if risk >= 2 else self.QtGui.QColor(160, 180, 200, 110)
        rect = self.QtCore.QRectF(x, y, width, height)
        painter.setPen(self.QtGui.QPen(border, 1.1))
        connector_x = x if anchor_x < x else x + width
        painter.drawLine(
            self.QtCore.QPointF(anchor_x, anchor_y),
            self.QtCore.QPointF(connector_x, y + height * 0.50),
        )
        painter.setPen(self.QtGui.QPen(border, 1.2))
        painter.setBrush(bg)
        painter.drawRoundedRect(rect, 6, 6)
        painter.setPen(self.QtGui.QColor(242, 246, 250, 235))
        painter.drawText(rect.adjusted(8, 5, -8, -5), self.QtCore.Qt.AlignLeft | self.QtCore.Qt.AlignVCenter, "\n".join(lines))

    def _project_track(self, track: Dict, rect):
        lateral = _as_float(track.get("x"))
        distance = _front_distance(track)
        y = self._screen_y_for_distance(rect, distance)
        t = self._t_for_y(rect, y)
        lane_units = lateral / self.lane_width_m
        x = self._lane_boundary_x(rect, lane_units, t)
        scale = max(0.28, min(1.02, 1.10 - distance / (self.view_range_m * 1.05)))
        return x, y, scale

    def _draw_ego_vehicle(self, painter, rect) -> None:
        cx = rect.width() * 0.50
        cy = rect.height() * self.ego_vertical_position
        width = rect.width() * 0.25 * self.ego_scale
        height = rect.height() * 0.34 * self.ego_scale
        if not self.ego_pixmap.isNull():
            self._draw_ego_pixmap(painter, cx, cy, width, height)
            return
        self._draw_ego_rear_car(painter, cx, cy, width, height)

    def _draw_ego_pixmap(self, painter, cx, cy, width, height) -> None:
        shadow = self.QtCore.QRectF(cx - width * 0.48, cy + height * 0.28, width * 0.96, height * 0.16)
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QColor(0, 0, 0, 160))
        painter.drawEllipse(shadow)

        source = self._ego_source_rect()
        source_ratio = source.width() / max(1.0, source.height())
        target_ratio = width / max(1.0, height)
        if source_ratio > target_ratio:
            draw_w = width
            draw_h = width / source_ratio
        else:
            draw_h = height
            draw_w = height * source_ratio
        target = self.QtCore.QRectF(cx - draw_w / 2.0, cy - draw_h * 0.56, draw_w, draw_h)
        painter.drawPixmap(target, self.ego_pixmap, source)

    def _ego_source_rect(self):
        width = self.ego_pixmap.width()
        height = self.ego_pixmap.height()
        if width <= 0 or height <= 0:
            return self.QtCore.QRectF()
        return self.QtCore.QRectF(width * 0.08, height * 0.22, width * 0.84, height * 0.58)

    def _draw_ego_rear_car(self, painter, cx, cy, width, height) -> None:
        shadow = self.QtCore.QRectF(cx - width * 0.62, cy + height * 0.32, width * 1.24, height * 0.22)
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QColor(0, 0, 0, 150))
        painter.drawEllipse(shadow)

        body_gradient = self.QtGui.QLinearGradient(cx, cy - height * 0.55, cx, cy + height * 0.48)
        body_gradient.setColorAt(0.0, self.QtGui.QColor(72, 78, 78))
        body_gradient.setColorAt(0.34, self.QtGui.QColor(28, 32, 32))
        body_gradient.setColorAt(0.72, self.QtGui.QColor(18, 20, 20))
        body_gradient.setColorAt(1.0, self.QtGui.QColor(8, 9, 9))

        body = self.QtGui.QPainterPath()
        body.moveTo(cx - width * 0.38, cy - height * 0.34)
        body.cubicTo(cx - width * 0.52, cy - height * 0.20, cx - width * 0.55, cy + height * 0.18, cx - width * 0.45, cy + height * 0.40)
        body.cubicTo(cx - width * 0.27, cy + height * 0.52, cx + width * 0.27, cy + height * 0.52, cx + width * 0.45, cy + height * 0.40)
        body.cubicTo(cx + width * 0.55, cy + height * 0.18, cx + width * 0.52, cy - height * 0.20, cx + width * 0.38, cy - height * 0.34)
        body.cubicTo(cx + width * 0.20, cy - height * 0.48, cx - width * 0.20, cy - height * 0.48, cx - width * 0.38, cy - height * 0.34)

        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(185, 195, 198, 230), 2.2))
        painter.setBrush(self.QtGui.QBrush(body_gradient))
        painter.drawPath(body)

        rear_window = self.QtGui.QPainterPath()
        rear_window.moveTo(cx - width * 0.26, cy - height * 0.29)
        rear_window.cubicTo(cx - width * 0.19, cy - height * 0.42, cx + width * 0.19, cy - height * 0.42, cx + width * 0.26, cy - height * 0.29)
        rear_window.lineTo(cx + width * 0.20, cy - height * 0.02)
        rear_window.cubicTo(cx + width * 0.08, cy + height * 0.04, cx - width * 0.08, cy + height * 0.04, cx - width * 0.20, cy - height * 0.02)
        rear_window.closeSubpath()
        glass_gradient = self.QtGui.QLinearGradient(cx, cy - height * 0.42, cx, cy + height * 0.04)
        glass_gradient.setColorAt(0.0, self.QtGui.QColor(125, 145, 145, 150))
        glass_gradient.setColorAt(1.0, self.QtGui.QColor(15, 20, 20, 210))
        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(210, 220, 220, 105), 1.3))
        painter.setBrush(self.QtGui.QBrush(glass_gradient))
        painter.drawPath(rear_window)

        bumper = self.QtCore.QRectF(cx - width * 0.34, cy + height * 0.28, width * 0.68, height * 0.13)
        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(5, 6, 6, 210), 1.3))
        painter.setBrush(self.QtGui.QColor(26, 28, 28, 235))
        painter.drawRoundedRect(bumper, 6, 6)

        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QColor(235, 28, 36, 245))
        painter.drawRoundedRect(
            self.QtCore.QRectF(cx - width * 0.39, cy + height * 0.18, width * 0.19, height * 0.045),
            5,
            5,
        )
        painter.drawRoundedRect(
            self.QtCore.QRectF(cx + width * 0.20, cy + height * 0.18, width * 0.19, height * 0.045),
            5,
            5,
        )

        highlight = self.QtGui.QPainterPath()
        highlight.moveTo(cx - width * 0.31, cy - height * 0.18)
        highlight.cubicTo(cx - width * 0.16, cy - height * 0.27, cx + width * 0.16, cy - height * 0.27, cx + width * 0.31, cy - height * 0.18)
        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(235, 240, 240, 80), 2.0))
        painter.setBrush(self.QtCore.Qt.NoBrush)
        painter.drawPath(highlight)

        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(42, 46, 46, 230), 2.0))
        painter.drawLine(self.QtCore.QPointF(cx - width * 0.48, cy + height * 0.02), self.QtCore.QPointF(cx - width * 0.57, cy + height * 0.15))
        painter.drawLine(self.QtCore.QPointF(cx + width * 0.48, cy + height * 0.02), self.QtCore.QPointF(cx + width * 0.57, cy + height * 0.15))

    def _draw_vehicle(self, painter, cx, cy, width, height, fill, outline, label: str = "", ego: bool = False) -> None:
        panel = self.QtCore.QRectF(cx - width * 0.58, cy - height * 0.52, width * 1.16, height * 1.04)
        panel_fill = self.QtGui.QColor(fill.red(), fill.green(), fill.blue(), 92)
        painter.setPen(self.QtGui.QPen(outline, 1.4))
        painter.setBrush(panel_fill)
        painter.drawRoundedRect(panel, 9, 9)

        shadow = self.QtCore.QRectF(cx - width * 0.56, cy + height * 0.34, width * 1.12, height * 0.18)
        painter.setPen(self.QtCore.Qt.NoPen)
        painter.setBrush(self.QtGui.QColor(0, 0, 0, 105))
        painter.drawEllipse(shadow)

        body = self.QtGui.QPainterPath()
        body.moveTo(cx, cy - height * 0.52)
        body.cubicTo(cx + width * 0.45, cy - height * 0.43, cx + width * 0.55, cy - height * 0.16, cx + width * 0.47, cy + height * 0.40)
        body.cubicTo(cx + width * 0.30, cy + height * 0.53, cx - width * 0.30, cy + height * 0.53, cx - width * 0.47, cy + height * 0.40)
        body.cubicTo(cx - width * 0.55, cy - height * 0.16, cx - width * 0.45, cy - height * 0.43, cx, cy - height * 0.52)

        painter.setPen(self.QtGui.QPen(outline, 2.0 if ego else 1.4))
        painter.setBrush(fill)
        painter.drawPath(body)

        roof = self.QtGui.QPainterPath()
        roof.moveTo(cx, cy - height * 0.34)
        roof.cubicTo(cx + width * 0.24, cy - height * 0.26, cx + width * 0.27, cy + height * 0.05, cx + width * 0.18, cy + height * 0.20)
        roof.cubicTo(cx + width * 0.05, cy + height * 0.25, cx - width * 0.05, cy + height * 0.25, cx - width * 0.18, cy + height * 0.20)
        roof.cubicTo(cx - width * 0.27, cy + height * 0.05, cx - width * 0.24, cy - height * 0.26, cx, cy - height * 0.34)
        painter.setPen(self.QtGui.QPen(self.QtGui.QColor(230, 236, 240, 105), 1.2))
        painter.setBrush(self.QtGui.QColor(95, 110, 120, 150) if ego else self.QtGui.QColor(45, 55, 65, 150))
        painter.drawPath(roof)

        if ego:
            painter.setPen(self.QtGui.QPen(self.QtGui.QColor(255, 50, 50, 230), 3.0))
            painter.drawLine(self.QtCore.QPointF(cx - width * 0.34, cy + height * 0.34), self.QtCore.QPointF(cx - width * 0.12, cy + height * 0.38))
            painter.drawLine(self.QtCore.QPointF(cx + width * 0.12, cy + height * 0.38), self.QtCore.QPointF(cx + width * 0.34, cy + height * 0.34))

        if label:
            painter.setPen(self.QtGui.QColor(235, 240, 245, 230))
            font = painter.font()
            font.setPointSize(max(8, int(width * 0.16)))
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(
                self.QtCore.QRectF(cx - width, cy + height * 0.53, width * 2, 24),
                self.QtCore.Qt.AlignCenter,
                label,
            )


class CleanRearLaneVisualizer:
    """Driver-oriented laptop display for UDP radar results."""

    def __init__(self, config_obj):
        self.config = config_obj
        self.enabled = False
        self._last_draw_time = 0.0
        self._fps = 0.0
        self._track_text_items: List[object] = []
        self.update_hz = float(_get_config_value(config_obj, "VISUALIZER_UPDATE_HZ", 20))
        self.view_range_m = float(_get_config_value(config_obj, "CLEAN_REAR_VIEW_RANGE_M", 30.0))
        self.lane_width_m = float(_get_config_value(config_obj, "CLEAN_LANE_WIDTH_M", 0.6))
        self.ego_vertical_position = float(
            _get_config_value(config_obj, "CLEAN_EGO_VERTICAL_POSITION", 0.65)
        )
        self.ego_scale = float(_get_config_value(config_obj, "CLEAN_EGO_SCALE", 0.72))
        self.show_labels = bool(_get_config_value(config_obj, "CLEAN_SHOW_OBJECT_LABELS", True))
        self.speed_gauge_max_kmh = max(
            1.0,
            float(_get_config_value(config_obj, "CLEAN_SPEED_GAUGE_MAX_KMH", 10.0)),
        )

        try:
            import pyqtgraph as pg
            from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

            self.pg = pg
            self.QtCore = QtCore
            self.QtGui = QtGui
            self.QtWidgets = QtWidgets
            self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            self._build_window()
            self.enabled = True
            logger.info("clean rear lane visualizer enabled")
        except Exception as exc:
            logger.warning(
                "clean rear lane visualizer unavailable: %s. Install with: pip install pyqtgraph PyQt5",
                exc,
            )

    def _build_window(self) -> None:
        QtCore = self.QtCore
        QtWidgets = self.QtWidgets

        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("AWR6843 Driving Display")
        self.window.setStyleSheet(
            "QWidget { background: #000000; color: #f2f4f7; font-family: Arial; }"
            "QLabel { color: #eef3f8; }"
        )

        root = QtWidgets.QVBoxLayout(self.window)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QtWidgets.QWidget()
        header.setFixedHeight(118)
        header.setStyleSheet("QWidget { background-color: #070b10; border-radius: 12px; }")
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(28, 12, 28, 10)
        header_layout.setSpacing(18)
        root.addWidget(header)

        self.left_status = self._make_icon_status("L", "#1d9bf0", "CLEAR")
        self.title_label = self._make_title_label()
        self.udp_status_label = self._make_udp_status_label("Waiting for UDP data...")
        self.right_status = self._make_icon_status("R", "#1d9bf0", "CLEAR")
        header_layout.addWidget(self.left_status, stretch=1)
        title_stack = QtWidgets.QVBoxLayout()
        title_stack.setSpacing(6)
        title_stack.addWidget(self.title_label)
        title_stack.addWidget(self.udp_status_label)
        header_layout.addLayout(title_stack, stretch=3)
        header_layout.addWidget(self.right_status, stretch=1)

        self.road_canvas = DrivingDisplayCanvas(
            QtWidgets=self.QtWidgets,
            QtCore=self.QtCore,
            QtGui=self.QtGui,
            view_range_m=self.view_range_m,
            lane_width_m=self.lane_width_m,
            ego_vertical_position=self.ego_vertical_position,
            ego_scale=self.ego_scale,
        )
        road_row = QtWidgets.QWidget()
        road_layout = QtWidgets.QHBoxLayout(road_row)
        road_layout.setContentsMargins(0, 0, 0, 0)
        road_layout.setSpacing(10)
        recommendation_panel = QtWidgets.QWidget()
        recommendation_panel.setFixedWidth(205)
        recommendation_panel.setStyleSheet(
            "QWidget { background-color: #070b10; border-radius: 10px; }"
        )
        recommendation_layout = QtWidgets.QVBoxLayout(recommendation_panel)
        recommendation_layout.setContentsMargins(14, 18, 14, 18)
        recommendation_layout.setSpacing(12)
        recommendation_title = QtWidgets.QLabel("RECOMMENDED\nSPEED")
        recommendation_title.setAlignment(QtCore.Qt.AlignCenter)
        recommendation_title.setStyleSheet(
            "QLabel { color: #cbd5e1; font-size: 17px; font-weight: 800; "
            "background-color: transparent; }"
        )
        self.recommended_speed_label = QtWidgets.QLabel("- km/h")
        self.recommended_speed_label.setAlignment(QtCore.Qt.AlignCenter)
        self.recommended_speed_label.setStyleSheet(
            "QLabel { color: #38bdf8; font-size: 31px; font-weight: 900; "
            "background-color: transparent; }"
        )
        self.recommended_speed_bar = QtWidgets.QProgressBar()
        self.recommended_speed_bar.setOrientation(QtCore.Qt.Vertical)
        self.recommended_speed_bar.setRange(0, max(1, int(round(self.speed_gauge_max_kmh * 10.0))))
        self.recommended_speed_bar.setTextVisible(False)
        self._set_speed_bar(self.recommended_speed_bar, 0.0, "#38bdf8")
        recommendation_layout.addWidget(recommendation_title)
        recommendation_layout.addWidget(self.recommended_speed_label)
        recommendation_layout.addWidget(self.recommended_speed_bar, stretch=1)
        road_layout.addWidget(recommendation_panel)
        road_layout.addWidget(self.road_canvas.widget, stretch=1)
        root.addWidget(road_row, stretch=1)

        footer = QtWidgets.QWidget()
        footer.setFixedHeight(104)
        footer.setStyleSheet("QWidget { background-color: #070b10; border-radius: 12px; }")
        footer_layout = QtWidgets.QHBoxLayout(footer)
        footer_layout.setContentsMargins(20, 12, 20, 12)
        footer_layout.setSpacing(14)
        root.addWidget(footer)

        self.decision_label = self._make_decision_label("SAFE", "#16a34a")
        self.nearest_label = self._make_footer_label("Nearest -")
        self.ttc_label = self._make_footer_label("TTC -")
        self.turn_label = self._make_footer_label("Signal NONE")
        footer_layout.addWidget(self.decision_label, stretch=2)
        footer_layout.addWidget(self.nearest_label)
        footer_layout.addWidget(self.ttc_label)
        footer_layout.addWidget(self.turn_label)

        self.window.resize(980, 880)
        self.window.show()

    def _make_title_label(self):
        label = self.QtWidgets.QLabel("Rear Lane Change Assist")
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setStyleSheet(
            "QLabel { color: #f5f7fa; font-size: 34px; font-weight: 800; "
            "background-color: transparent; }"
        )
        return label

    def _make_udp_status_label(self, text: str):
        label = self.QtWidgets.QLabel(text)
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setStyleSheet(
            "QLabel { color: #94a3b8; font-size: 15px; font-weight: 700; "
            "background-color: transparent; }"
        )
        return label

    def _make_icon_status(self, side: str, color: str, text: str):
        label = self.QtWidgets.QLabel(f"{side}\n{text}")
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setStyleSheet(
            f"color: {color}; font-size: 26px; font-weight: 800; "
            "background-color: transparent;"
        )
        return label

    def _make_footer_label(self, text: str):
        label = self.QtWidgets.QLabel(text)
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setMinimumHeight(42)
        label.setStyleSheet(
            "QLabel { background-color: #14181e; color: #f2f4f7; "
            "border-radius: 8px; padding: 8px 12px; font-size: 15px; font-weight: 700; }"
        )
        return label

    def _make_speed_gauge(self, title: str, color: str):
        row = self.QtWidgets.QWidget()
        layout = self.QtWidgets.QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = self.QtWidgets.QLabel(title)
        title_label.setFixedWidth(72)
        title_label.setStyleSheet(
            "QLabel { color: #cbd5e1; font-size: 12px; font-weight: 800; "
            "background-color: transparent; }"
        )

        bar = self.QtWidgets.QProgressBar()
        bar.setRange(0, max(1, int(round(self.speed_gauge_max_kmh * 10.0))))
        bar.setTextVisible(False)
        bar.setFixedHeight(18)
        self._set_speed_bar(bar, 0.0, color)

        value_label = self.QtWidgets.QLabel("- km/h")
        value_label.setFixedWidth(76)
        value_label.setAlignment(self.QtCore.Qt.AlignRight | self.QtCore.Qt.AlignVCenter)
        value_label.setStyleSheet(
            "QLabel { color: #f8fafc; font-size: 13px; font-weight: 800; "
            "background-color: transparent; }"
        )

        layout.addWidget(title_label)
        layout.addWidget(bar, stretch=1)
        layout.addWidget(value_label)
        return row, bar, value_label

    def _set_speed_bar(self, bar, speed_kmh: float, color: str) -> None:
        value = max(0.0, min(self.speed_gauge_max_kmh, float(speed_kmh)))
        bar.setValue(int(round(value * 10.0)))
        bar.setStyleSheet(
            "QProgressBar { background-color: #202630; border: 1px solid #334155; "
            "border-radius: 5px; }"
            f"QProgressBar::chunk {{ background-color: {color}; border-radius: 4px; }}"
        )

    def _make_decision_label(self, text: str, color: str):
        label = self.QtWidgets.QLabel(text)
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setMinimumHeight(72)
        label.setStyleSheet(
            f"QLabel {{ background-color: {color}; color: white; border-radius: 10px; "
            "padding: 12px 18px; font-size: 34px; font-weight: bold; }"
        )
        return label

    def _make_label(self, text: str, size: int, bold: bool = False):
        label = self.QtWidgets.QLabel(text)
        weight = "700" if bold else "500"
        label.setStyleSheet(f"font-size: {size}px; font-weight: {weight};")
        return label

    def _make_pill(self, text: str, color: str):
        label = self._make_label(text, 14, bold=True)
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setStyleSheet(
            f"background: {color}; color: white; border-radius: 6px; "
            "padding: 9px 12px; font-size: 14px; font-weight: 700;"
        )
        return label

    def _make_status_label(self, text: str, color: str):
        label = self._make_label(text, 22, bold=True)
        label.setAlignment(self.QtCore.Qt.AlignCenter)
        label.setMinimumHeight(74)
        label.setStyleSheet(
            f"background: {color}; color: white; border-radius: 8px; "
            "padding: 14px; font-size: 22px; font-weight: 800;"
        )
        return label

    def _draw_clean_lanes(self) -> None:
        pg = self.pg
        lane_half = self.lane_width_m / 2.0
        left_range = tuple(
            _get_config_value(self.config, "LEFT_LANE_X_RANGE", (-self.lane_width_m - lane_half, -lane_half))
        )
        center_range = tuple(
            _get_config_value(self.config, "CENTER_LANE_X_RANGE", (-lane_half, lane_half))
        )
        right_range = tuple(
            _get_config_value(self.config, "RIGHT_LANE_X_RANGE", (lane_half, self.lane_width_m + lane_half))
        )

        lane_specs = [
            ("LEFT", left_range, (41, 121, 255, 38), (70, 145, 255, 150)),
            ("CENTER", center_range, (148, 163, 184, 24), (148, 163, 184, 120)),
            ("RIGHT", right_range, (14, 165, 233, 38), (45, 212, 255, 150)),
        ]
        for name, lane_range, brush_rgba, pen_rgba in lane_specs:
            region = pg.LinearRegionItem(
                values=(float(lane_range[0]), float(lane_range[1])),
                orientation="vertical",
                movable=False,
                brush=pg.mkBrush(*brush_rgba),
            )
            region.setZValue(-20)
            self.plot.addItem(region)
            for edge in lane_range:
                line = pg.InfiniteLine(pos=float(edge), angle=90, pen=pg.mkPen(pen_rgba, width=1.2))
                line.setZValue(-10)
                self.plot.addItem(line)
            label = pg.TextItem(name, color=(226, 232, 240), anchor=(0.5, 0.0))
            label.setPos((float(lane_range[0]) + float(lane_range[1])) / 2.0, self.view_range_m * 0.94)
            self.plot.addItem(label)

        for distance in (5, 10, 15, 20, 25, 30):
            if distance > self.view_range_m:
                continue
            marker = pg.InfiniteLine(
                pos=float(distance),
                angle=0,
                pen=pg.mkPen((148, 163, 184, 55), width=1),
            )
            marker.setZValue(-15)
            self.plot.addItem(marker)

    def _build_plot_items(self) -> None:
        pg = self.pg
        self.clear_item = pg.ScatterPlotItem(
            size=18,
            brush=pg.mkBrush(34, 197, 94, 210),
            pen=pg.mkPen((187, 247, 208), width=2),
        )
        self.caution_item = pg.ScatterPlotItem(
            size=23,
            brush=pg.mkBrush(245, 158, 11, 220),
            pen=pg.mkPen((254, 240, 138), width=2),
        )
        self.warning_item = pg.ScatterPlotItem(
            size=30,
            brush=pg.mkBrush(239, 68, 68, 230),
            pen=pg.mkPen((254, 202, 202), width=3),
        )
        self.ego_outline = pg.PlotDataItem(
            x=[-0.16, 0.16, 0.16, -0.16, -0.16],
            y=[0.2, 0.2, 1.0, 1.0, 0.2],
            pen=pg.mkPen((226, 232, 240), width=3),
            fillLevel=0,
            brush=pg.mkBrush(56, 189, 248, 120),
        )
        for item in (
            self.clear_item,
            self.caution_item,
            self.warning_item,
            self.ego_outline,
        ):
            self.plot.addItem(item)

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
            now = time.monotonic()
            min_interval = 1.0 / self.update_hz if self.update_hz > 0.0 else 0.0
            if min_interval > 0.0 and now - self._last_draw_time < min_interval:
                self._process_events()
                return
            if self._last_draw_time > 0.0:
                instant_fps = 1.0 / max(now - self._last_draw_time, 1e-6)
                self._fps = instant_fps if self._fps == 0.0 else (0.85 * self._fps + 0.15 * instant_fps)
            self._last_draw_time = now

            merged_tracks = self._merge_lane_info(tracks or [], lane_result)
            selected_lane = self._advice_value(advice, "selected_lane", 0)
            left_risk = int(getattr(lane_result, "left_risk", 0))
            right_risk = int(getattr(lane_result, "right_risk", 0))
            self.road_canvas.update_scene(merged_tracks, selected_lane, left_risk, right_risk)
            self._update_status(lane_result, merged_tracks, frame_id, advice)
            self._process_events()
        except Exception as exc:
            logger.warning("clean visualizer update failed: %s", exc)

    def _merge_lane_info(self, tracks: Iterable[Dict], lane_result) -> List[Dict]:
        lane_objects = []
        lane_objects.extend(getattr(lane_result, "left_objects", []) or [])
        lane_objects.extend(getattr(lane_result, "right_objects", []) or [])
        lane_by_id = {obj.get("track_id"): obj for obj in lane_objects}
        merged = []
        for track in tracks:
            copied = dict(track)
            lane_obj = lane_by_id.get(copied.get("track_id"))
            if lane_obj:
                copied["lane_label"] = lane_obj.get("lane_label", copied.get("lane_label", "unknown"))
                copied["risk_level"] = lane_obj.get("risk_level", copied.get("risk_level", 0))
                copied["ttc"] = lane_obj.get("ttc", copied.get("ttc", _track_ttc(copied)))
            if copied.get("lane_label", "unknown") == "unknown":
                copied["lane_label"] = self._lane_from_x(_as_float(copied.get("x")))
            copied.setdefault("ttc", _track_ttc(copied))
            copied.setdefault("risk_level", copied.get("risk", 0))
            merged.append(copied)
        return merged

    def _lane_from_x(self, x_value: float) -> str:
        left_low, left_high = _get_config_value(
            self.config,
            "LEFT_LANE_X_RANGE",
            (-1.5 * self.lane_width_m, -0.5 * self.lane_width_m),
        )
        center_low, center_high = _get_config_value(
            self.config,
            "CENTER_LANE_X_RANGE",
            (-0.5 * self.lane_width_m, 0.5 * self.lane_width_m),
        )
        right_low, right_high = _get_config_value(
            self.config,
            "RIGHT_LANE_X_RANGE",
            (0.5 * self.lane_width_m, 1.5 * self.lane_width_m),
        )
        if left_low <= x_value <= left_high:
            return "left"
        if right_low <= x_value <= right_high:
            return "right"
        if center_low <= x_value <= center_high:
            return "center"
        return "unknown"

    def _update_objects(self, tracks: List[Dict]) -> None:
        grouped = {0: [], 1: [], 2: []}
        for track in tracks:
            risk = int(track.get("risk_level", track.get("risk", 0)))
            risk = 2 if risk >= 2 else 1 if risk == 1 else 0
            grouped[risk].append(track)
        self._set_scatter_xy(self.clear_item, grouped[0])
        self._set_scatter_xy(self.caution_item, grouped[1])
        self._set_scatter_xy(self.warning_item, grouped[2])

    def _set_scatter_xy(self, item, objects) -> None:
        xs, ys = _xy_from_objects(objects)
        item.setData(x=xs, y=ys)

    def _update_labels(self, tracks: List[Dict]) -> None:
        if not self.show_labels:
            for item in self._track_text_items:
                item.setVisible(False)
            return
        self._ensure_text_item_count(len(tracks))
        for index, track in enumerate(tracks):
            item = self._track_text_items[index]
            track_id = track.get("track_id", "-")
            lane = _lane_code(track.get("lane_label", "unknown"))
            ttc = _format_ttc(track.get("ttc", _track_ttc(track)))
            distance = _front_distance(track)
            item.setText(f"#{track_id} {lane}  {distance:.1f}m  {ttc}")
            item.setPos(_as_float(track.get("x")) + 0.07, _as_float(track.get("y")) + 0.35)
            item.setVisible(True)
        for item in self._track_text_items[len(tracks) :]:
            item.setVisible(False)

    def _ensure_text_item_count(self, count: int) -> None:
        while len(self._track_text_items) < count:
            item = self.pg.TextItem(color=(241, 245, 249), anchor=(0.0, 1.0))
            self.plot.addItem(item)
            self._track_text_items.append(item)

    def _update_status(self, lane_result, tracks: List[Dict], frame_id, advice) -> None:
        left_objects = getattr(lane_result, "left_objects", []) or []
        right_objects = getattr(lane_result, "right_objects", []) or []
        left_risk = int(getattr(lane_result, "left_risk", 0))
        right_risk = int(getattr(lane_result, "right_risk", 0))
        min_ttc = self._minimum_ttc(tracks)
        nearest = self._nearest_track(tracks)
        turn_signal = self._advice_value(advice, "turn_signal", getattr(default_config, "TURN_SIGNAL_NONE", 0))
        selected_lane = self._advice_value(advice, "selected_lane", "-")
        recommended_speed_mps = self._advice_value(advice, "recommended_speed_mps", None)
        waiting_for_data = bool(self._advice_value(advice, "waiting_for_data", False))
        data_age = self._advice_value(advice, "last_data_age_sec", None)
        highest_risk = max([left_risk, right_risk] + [int(track.get("risk_level", track.get("risk", 0))) for track in tracks])

        if waiting_for_data:
            self.udp_status_label.setText("Waiting for UDP data...")
            self.udp_status_label.setStyleSheet(
                "QLabel { color: #f59e0b; font-size: 15px; font-weight: 800; background-color: transparent; }"
            )
        else:
            age_text = "" if data_age is None else f"  age {float(data_age):.1f}s"
            self.udp_status_label.setText(f"UDP LIVE  port {getattr(default_config, 'UDP_PORT', 5005)}{age_text}")
            self.udp_status_label.setStyleSheet(
                "QLabel { color: #22c55e; font-size: 15px; font-weight: 800; background-color: transparent; }"
            )

        self._set_lane_status(self.left_status, "LEFT", left_risk)
        self._set_lane_status(self.right_status, "RIGHT", right_risk)
        self.nearest_label.setText(f"Nearest {self._format_nearest(nearest)}")
        self.ttc_label.setText(f"TTC {_format_ttc(min_ttc)}")
        self.turn_label.setText(self._format_signal_text(turn_signal, selected_lane))
        self._update_recommended_speed(recommended_speed_mps, not waiting_for_data)
        self._set_decision_status(highest_risk, waiting_for_data)

    def _update_recommended_speed(self, recommended_speed_mps, valid: bool) -> None:
        if not valid or recommended_speed_mps is None:
            self.recommended_speed_label.setText("- km/h")
            self._set_speed_bar(self.recommended_speed_bar, 0.0, "#64748b")
            return

        recommended_kmh = max(0.0, _as_float(recommended_speed_mps) * 3.6)
        self._set_speed_bar(self.recommended_speed_bar, recommended_kmh, "#38bdf8")
        self.recommended_speed_label.setText(f"{recommended_kmh:.1f} km/h")

    def _format_signal_text(self, turn_signal, selected_lane) -> str:
        signal = _turn_signal_name(turn_signal)
        if signal == "LEFT":
            return "LEFT SIGNAL"
        if signal == "RIGHT":
            return "RIGHT SIGNAL"
        if signal == "HAZARD":
            return "HAZARD SIGNAL"
        return f"Signal NONE / Lane {_selected_lane_name(selected_lane)}"

    def _set_decision_status(self, risk: int, waiting_for_data: bool) -> None:
        if waiting_for_data:
            text, color = "WAITING", "#64748b"
        elif risk >= 2:
            text, color = "DANGER", "#dc2626"
        elif risk == 1:
            text, color = "CAUTION", "#d97706"
        else:
            text, color = "SAFE", "#16a34a"
        self.decision_label.setText(text)
        self.decision_label.setStyleSheet(
            f"QLabel {{ background-color: {color}; color: white; border-radius: 10px; "
            "padding: 12px 18px; font-size: 34px; font-weight: bold; }"
        )

    def _set_lane_status(self, label, lane_name: str, risk: int) -> None:
        if risk >= 2:
            text, color = "WARNING", "#ff3b30"
        elif risk == 1:
            text, color = "CAUTION", "#ffb020"
        else:
            text, color = "CLEAR", "#1d9bf0"
        side = "L" if lane_name == "LEFT" else "R"
        label.setText(f"{side}\n{text}")
        label.setStyleSheet(
            f"color: {color}; font-size: 26px; font-weight: 800; "
            "background-color: transparent;"
        )

    def _advice_value(self, advice, name: str, default=None):
        if advice is None:
            return default
        if isinstance(advice, dict):
            return advice.get(name, default)
        return getattr(advice, name, default)

    def _nearest_track(self, tracks: Iterable[Dict]) -> Optional[Dict]:
        candidates = [track for track in tracks if _front_distance(track) > 0.0]
        if not candidates:
            return None
        return min(candidates, key=_front_distance)

    def _format_nearest(self, track: Optional[Dict]) -> str:
        if track is None:
            return "-"
        lane = track.get("lane_label", "unknown")
        return f"{_front_distance(track):.2f} m ({lane})"

    def _minimum_ttc(self, tracks: Iterable[Dict]) -> Optional[float]:
        values = []
        for track in tracks:
            ttc = track.get("ttc", _track_ttc(track))
            if ttc is not None:
                values.append(float(ttc))
        return min(values) if values else None

    def _process_events(self) -> None:
        try:
            self.app.processEvents()
        except Exception as exc:
            logger.warning("clean visualizer event processing failed: %s", exc)

    def close(self) -> None:
        if not self.enabled:
            return
        try:
            self.window.close()
            self._process_events()
        except Exception as exc:
            logger.warning("clean visualizer close failed: %s", exc)


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
            "Lane Change Advice",
            f"possible: {self._advice_value(advice, 'lane_change_possible', '-')}",
            f"ego speed: {self._format_advice_number(self._advice_value(advice, 'ego_current_speed_mps'), ' m/s')}",
            f"required speed: {self._format_advice_number(self._advice_value(advice, 'ego_required_speed_mps'), ' m/s')}",
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
        checksum = f"{packet[-1] & 0xFF:02X}" if packet else "-"
        return "\n".join(
            [
                "SPI Packet",
                f"SPI TX: {_packet_hex(packet)}",
                f"turn_request: {turn_request}",
                f"object_count: {object_count}",
                f"risk_level: {risk_level}",
                f"lane_id: {lane_id}",
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
        "clean"     -> driver-oriented laptop UDP display
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
    if backend == "clean":
        visualizer = CleanRearLaneVisualizer(config_obj)
        return visualizer if visualizer.enabled else MockVisualizer()
    if backend == "pyqtgraph":
        visualizer = AdvancedVisualizer(config_obj)
        return visualizer if visualizer.enabled else MockVisualizer()

    logger.warning("unknown VISUALIZER_BACKEND=%s; visualizer disabled", backend)
    return MockVisualizer()
