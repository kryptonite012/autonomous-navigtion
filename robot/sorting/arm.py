"""Servo arm + sorter carousel sequences."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from robot.hardware.base import ServoController

log = logging.getLogger(__name__)


class ArmAborted(Exception):
    pass


class SortingArm:
    """3-servo arm (base, shoulder, gripper) plus a carousel servo selecting the bin.

    Servo angles are open-loop: hobby servos give no position feedback, so a
    move is assumed complete after a travel time proportional to the angle.
    """

    def __init__(self, servos: ServoController, sc: dict, speed_deg_s: float = 180.0,
                 abort: Optional[Callable[[], bool]] = None):
        self.servos = servos
        self.poses = sc["poses"]
        self.gripper_open = sc["gripper_open"]
        self.gripper_closed = sc["gripper_closed"]
        self.bin_angles = {int(k): float(v) for k, v in sc["bin_angles"].items()}
        self.speed = speed_deg_s
        self.abort = abort or (lambda: False)
        self._lock = threading.Lock()
        self.time_scale = 1.0     # tests shrink the waits

    def _wait(self, seconds: float) -> None:
        end = time.time() + seconds * self.time_scale
        while time.time() < end:
            if self.abort():
                raise ArmAborted()
            time.sleep(0.02)

    def move(self, channel: str, angle: float) -> None:
        prev = self.servos.get_angle(channel)
        self.servos.set_angle(channel, angle)
        travel = abs(angle - prev) / self.speed if prev is not None else 0.6
        self._wait(max(0.1, travel))

    def pose(self, name: str) -> None:
        for ch in ("base", "shoulder", "gripper"):
            if ch in self.poses[name]:
                self.move(ch, self.poses[name][ch])

    def home(self) -> None:
        with self._lock:
            self.pose("home")

    def pick(self) -> None:
        with self._lock:
            self.move("gripper", self.gripper_open)
            self.pose("pre_grip")
            self.move("shoulder", self.poses["grip"]["shoulder"])
            self.move("gripper", self.gripper_closed)
            self._wait(0.3)
            self.move("shoulder", self.poses["lift"]["shoulder"])

    def select_bin(self, bin_id: int) -> None:
        if bin_id not in self.bin_angles:
            raise ValueError(f"no carousel angle for bin {bin_id}")
        with self._lock:
            self.move("sorter", self.bin_angles[bin_id])

    def release(self) -> None:
        with self._lock:
            self.move("gripper", self.gripper_open)
            self._wait(0.3)

    def calibrate(self, channel: str, angle: float) -> None:
        """Direct servo positioning used by the calibration screen."""
        with self._lock:
            self.servos.set_angle(channel, angle)
