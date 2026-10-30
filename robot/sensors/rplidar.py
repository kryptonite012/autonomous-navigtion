"""Minimal RPLIDAR A1/A2 driver (standard scan mode) over a serial port.

Implements the documented SCAN request (0xA5 0x20) and the 5 byte measurement
response. Good enough for 2D mapping; for SLAM Toolbox on ROS 2 use the
official ``rplidar_ros`` node instead (see ros2_ws/README.md).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

import numpy as np

from robot.hardware.base import HardwareUnavailable, LaserScan, Lidar

log = logging.getLogger(__name__)

SYNC = b"\xA5"
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_HEALTH = 0x52


class RPLidar(Lidar):
    def __init__(self, name: str, port: str, baud: int = 115200, bins: int = 360, max_range: float = 8.0):
        super().__init__(name)
        self.port = port
        self.baud = baud
        self.bins = bins
        self.max_range = max_range
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._scan: Optional[LaserScan] = None

    def _cmd(self, cmd: int) -> None:
        self._ser.write(SYNC + bytes([cmd]))

    def initialize(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise HardwareUnavailable("pyserial not installed") from exc
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as exc:
            raise HardwareUnavailable(f"cannot open {self.port}: {exc}") from exc
        self._ser.dtr = False   # spin motor on A1 USB adapters
        self._cmd(CMD_STOP)
        time.sleep(0.01)
        self._ser.reset_input_buffer()
        self._cmd(CMD_HEALTH)
        desc = self._ser.read(7)
        if len(desc) != 7 or desc[:2] != b"\xA5\x5A":
            self._ser.close()
            raise HardwareUnavailable("RPLIDAR did not answer health request")
        status = self._ser.read(3)
        if status and status[0] == 2:
            raise HardwareUnavailable("RPLIDAR reports hardware error")
        self._cmd(CMD_SCAN)
        if self._ser.read(7)[:2] != b"\xA5\x5A":
            raise HardwareUnavailable("RPLIDAR scan start failed")
        super().initialize()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="rplidar")
        self._thread.start()

    def _loop(self) -> None:
        ranges = np.full(self.bins, np.inf)
        while self._running:
            pkt = self._ser.read(5)
            if len(pkt) != 5:
                self._record_error("short read")
                continue
            start = pkt[0] & 0x1
            inv = (pkt[0] >> 1) & 0x1
            if start == inv or not (pkt[1] & 0x1):
                self._ser.read(1)   # resync
                continue
            if start and np.isfinite(ranges).any():
                with self._lock:
                    self._scan = LaserScan(-math.pi, 2 * math.pi / self.bins, ranges.copy(),
                                           self.max_range, time.time())
                ranges[:] = np.inf
            quality = pkt[0] >> 2
            angle = ((pkt[1] >> 1) | (pkt[2] << 7)) / 64.0
            dist = (pkt[3] | (pkt[4] << 8)) / 4000.0
            if quality == 0 or dist <= 0:
                continue
            # lidar angles are clockwise, ROS convention is counter-clockwise
            a = math.radians(-angle)
            idx = int(((a + math.pi) % (2 * math.pi)) / (2 * math.pi) * self.bins) % self.bins
            ranges[idx] = dist
            self._record_ok()

    def scan(self) -> Optional[LaserScan]:
        with self._lock:
            return self._scan

    def shutdown(self) -> None:
        self._running = False
        if self._ser:
            try:
                self._cmd(CMD_STOP)
                self._ser.dtr = True
                self._ser.close()
            except Exception:
                pass
        super().shutdown()
