# -*- coding: utf-8 -*-
"""Laptop UDP receiver for processed Raspberry Pi AWR6843 radar results."""

from __future__ import annotations

import json
import logging
import socket
from typing import Dict, Optional

import config as app_config


logger = logging.getLogger(__name__)


class RadarUDPReceiver:
    """Receive JSON UDP packets and discard stale frame numbers."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = app_config.UDP_PORT,
        buffer_size: int = app_config.UDP_RECV_BUFFER,
        timeout: float = 0.02,
    ):
        self.host = str(host)
        self.port = int(port)
        self.buffer_size = int(buffer_size)
        self.last_frame: Optional[int] = None
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((self.host, self.port))
        self._socket.settimeout(float(timeout))
        logger.info("UDP receiver listening on %s:%s", self.host, self.port)

    def receive(self) -> Optional[Dict]:
        """Return a decoded packet, or None when no fresh valid packet is available."""
        try:
            data, _addr = self._socket.recvfrom(self.buffer_size)
        except socket.timeout:
            return None
        except OSError as exc:
            logger.warning("UDP receive failed: %s", exc)
            return None

        try:
            packet = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            logger.warning("Ignoring invalid UDP JSON packet: %s", exc)
            return None

        frame = packet.get("frame")
        try:
            frame = int(frame)
        except (TypeError, ValueError):
            logger.warning("Ignoring UDP packet without valid frame: %s", packet)
            return None

        if self.last_frame is not None and frame <= self.last_frame:
            logger.debug("Ignoring stale UDP frame=%s last_frame=%s", frame, self.last_frame)
            return None

        objects = packet.get("objects")
        if not isinstance(objects, list):
            packet["objects"] = []

        self.last_frame = frame
        packet["frame"] = frame
        return packet

    def close(self) -> None:
        try:
            self._socket.close()
        except Exception:
            pass


def receive_radar_result(receiver: RadarUDPReceiver) -> Optional[Dict]:
    """Convenience wrapper for callers that prefer a function-style API."""
    return receiver.receive()
