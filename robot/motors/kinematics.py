"""Differential drive kinematics and wheel odometry."""

from __future__ import annotations

import math
from dataclasses import dataclass


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)


class DiffDriveKinematics:
    def __init__(self, wheel_radius: float, wheel_base: float, max_wheel_speed: float):
        self.r = wheel_radius
        self.L = wheel_base
        # maximum wheel surface speed in m/s, used to map speed to duty cycle
        self.max_wheel_speed = max_wheel_speed

    def inverse(self, v: float, w: float) -> tuple[float, float]:
        """Body twist (m/s, rad/s) -> wheel surface speeds (m/s)."""
        return v - w * self.L / 2.0, v + w * self.L / 2.0

    def forward(self, vl: float, vr: float) -> tuple[float, float]:
        return (vl + vr) / 2.0, (vr - vl) / self.L

    def to_duty(self, v: float, w: float) -> tuple[float, float]:
        """Twist -> duty cycle, scaling both wheels down together if saturated."""
        vl, vr = self.inverse(v, w)
        peak = max(abs(vl), abs(vr), 1e-9)
        scale = 1.0 if peak <= self.max_wheel_speed else self.max_wheel_speed / peak
        return vl * scale / self.max_wheel_speed, vr * scale / self.max_wheel_speed


class WheelOdometry:
    """Integrates encoder ticks (and optionally an IMU yaw rate) into a pose."""

    def __init__(self, kin: DiffDriveKinematics, ticks_per_rev: int, gyro_weight: float = 0.0):
        self.kin = kin
        self.m_per_tick = 2 * math.pi * kin.r / ticks_per_rev
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self.gyro_weight = gyro_weight
        self._last_ticks: tuple[int, int] | None = None
        self._last_t: float | None = None

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.pose = Pose2D(x, y, theta)

    def update(self, ticks: tuple[int, int], t: float, gyro_z: float | None = None) -> Pose2D:
        if self._last_ticks is None or self._last_t is None:
            self._last_ticks, self._last_t = ticks, t
            return self.pose
        dt = t - self._last_t
        if dt <= 0:
            return self.pose
        dl = (ticks[0] - self._last_ticks[0]) * self.m_per_tick
        dr = (ticks[1] - self._last_ticks[1]) * self.m_per_tick
        self._last_ticks, self._last_t = ticks, t

        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.kin.L
        if gyro_z is not None and self.gyro_weight > 0:
            # complementary blend: wheel slip makes encoder yaw unreliable
            dth = (1 - self.gyro_weight) * dth + self.gyro_weight * gyro_z * dt
        mid = self.pose.theta + dth / 2.0
        self.pose.x += ds * math.cos(mid)
        self.pose.y += ds * math.sin(mid)
        self.pose.theta = normalize_angle(self.pose.theta + dth)
        self.v = ds / dt
        self.w = dth / dt
        return self.pose
