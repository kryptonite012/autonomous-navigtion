"""PCA9685 servo controller (16 channel PWM over I2C)."""

from __future__ import annotations

import logging
from typing import Optional

from robot.hardware.base import HardwareUnavailable, ServoController

log = logging.getLogger(__name__)


class PCA9685Servos(ServoController):
    def __init__(self, name: str, address: int, channels: dict[str, int],
                 limits: Optional[dict[str, tuple[float, float]]] = None):
        super().__init__(name)
        self.address = address
        self.channels = channels
        self.limits = limits or {}
        self._kit = None
        self._angles: dict[str, float] = {}

    def initialize(self) -> None:
        try:
            from adafruit_servokit import ServoKit
        except ImportError as exc:
            raise HardwareUnavailable("adafruit-circuitpython-servokit not installed") from exc
        try:
            self._kit = ServoKit(channels=16, address=self.address)
        except Exception as exc:
            self._record_error(exc)
            raise HardwareUnavailable(f"PCA9685 not found at 0x{self.address:02x}: {exc}") from exc
        super().initialize()

    def _check(self, channel: str, angle: float) -> float:
        if channel not in self.channels:
            raise ValueError(f"unknown servo channel {channel!r}")
        lo, hi = self.limits.get(channel, (0.0, 180.0))
        return max(lo, min(hi, float(angle)))

    def set_angle(self, channel: str, angle_deg: float) -> None:
        angle = self._check(channel, angle_deg)
        try:
            self._kit.servo[self.channels[channel]].angle = angle
            self._angles[channel] = angle
            self._record_ok()
        except Exception as exc:
            self._record_error(exc)
            raise

    def get_angle(self, channel: str) -> Optional[float]:
        # Hobby servos have no position feedback, this is the commanded angle.
        return self._angles.get(channel)

    def release(self) -> None:
        if not self._kit:
            return
        for ch in self.channels.values():
            try:
                self._kit.servo[ch].angle = None
            except Exception as exc:
                self._record_error(exc)

    def shutdown(self) -> None:
        self.release()
        super().shutdown()
