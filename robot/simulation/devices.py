"""Simulated implementations of the hardware interfaces.

Every class here reports ``simulated = True`` so the dashboard and logs can
always tell simulated data apart from real measurements.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import cv2
import numpy as np

from robot.hardware.base import (BatteryMonitor, BinSensor, Camera, DistanceSensor,
                                 EmergencyStopInput, EncoderPair, Imu, ImuReading,
                                 LaserScan, Lidar, MotorDriver, RangeReading,
                                 ServoController)
from robot.simulation.world import OBJECT_BGR, SimWorld


class SimMotorDriver(MotorDriver):
    simulated = True

    def __init__(self, name: str, world: SimWorld):
        super().__init__(name)
        self.world = world

    def set_duty(self, left: float, right: float) -> None:
        if self.world.motor_fault:
            self._record_error("motor driver not responding")
            raise IOError("motor driver not responding")
        self.world.set_duty(max(-1, min(1, left)), max(-1, min(1, right)))

    def stop(self) -> None:
        self.world.set_duty(0.0, 0.0)


class SimEncoders(EncoderPair):
    simulated = True

    def __init__(self, name: str, world: SimWorld, wheel_radius: float, ticks_per_rev: int):
        super().__init__(name)
        self.world = world
        self.m_per_tick = 2 * math.pi * wheel_radius / ticks_per_rev
        self._scale = (1.0 + world.rng.normal(0, world.odom_noise),
                       1.0 + world.rng.normal(0, world.odom_noise))

    def ticks(self) -> tuple[int, int]:
        if self.world.encoder_fault:
            self._record_error("encoder signal lost")
            raise IOError("encoder signal lost")
        s = self.world.state
        return (int(s.left_dist * self._scale[0] / self.m_per_tick),
                int(s.right_dist * self._scale[1] / self.m_per_tick))


class SimRangeSensor(DistanceSensor):
    simulated = True

    def __init__(self, name: str, world: SimWorld, angle_deg: float, max_range: float, kind: str,
                 beam_width_deg: float = 15.0):
        super().__init__(name)
        self.world = world
        self.angle_rad = math.radians(angle_deg)
        self.max_range_m = max_range
        self.kind = kind
        self.beam = math.radians(beam_width_deg)
        self.disconnected = False

    def read(self) -> RangeReading:
        now = time.time()
        if self.disconnected:
            self._record_error("no echo / bus error")
            return RangeReading(self.name, None, self.angle_rad, now, False, self.max_range_m)
        x, y, th = self.world.true_pose()
        r = self.world.robot_radius * 0.8
        ox, oy = x + r * math.cos(th + self.angle_rad), y + r * math.sin(th + self.angle_rad)
        best = None
        # ultrasonic sensors have a wide cone: return the nearest of 3 rays
        for off in (-self.beam / 2, 0.0, self.beam / 2):
            d = self.world.raycast(ox, oy, th + self.angle_rad + off, self.max_range_m)
            if d is not None and (best is None or d < best):
                best = d
        if best is not None:
            best = max(0.02, best + float(self.world.rng.normal(0, self.world.sensor_noise)))
        self._record_ok()
        return RangeReading(self.name, best, self.angle_rad, now, True, self.max_range_m)


class SimImu(Imu):
    simulated = True

    def __init__(self, name: str, world: SimWorld):
        super().__init__(name)
        self.world = world

    def read(self) -> ImuReading:
        s = self.world.state
        w = (s.vr - s.vl) / self.world.wheel_base
        n = self.world.rng.normal
        return ImuReading((n(0, 0.05), n(0, 0.05), 9.81 + n(0, 0.05)),
                          (n(0, 0.002), n(0, 0.002), w + n(0, 0.01)), 32.0, time.time())


class SimLidar(Lidar):
    simulated = True

    def __init__(self, name: str, world: SimWorld, beams: int = 360, max_range: float = 6.0):
        super().__init__(name)
        self.world = world
        self.beams = beams
        self.max_range = max_range

    def scan(self) -> Optional[LaserScan]:
        x, y, th = self.world.true_pose()
        inc = 2 * math.pi / self.beams
        ranges = np.full(self.beams, np.inf)
        for i in range(self.beams):
            d = self.world.raycast(x, y, th - math.pi + i * inc, self.max_range)
            if d is not None:
                ranges[i] = d + self.world.rng.normal(0, self.world.sensor_noise)
        return LaserScan(-math.pi, inc, ranges, self.max_range, time.time())


class SimServos(ServoController):
    simulated = True

    def __init__(self, name: str, world: SimWorld, channels: dict[str, int], gripper_closed: float,
                 bin_angles: dict):
        super().__init__(name)
        self.world = world
        self.channels = channels
        self.gripper_closed = gripper_closed
        self.bin_angles = {int(k): v for k, v in bin_angles.items()}
        self._angles: dict[str, float] = {}
        self.failed = False

    def set_angle(self, channel: str, angle_deg: float) -> None:
        if channel not in self.channels:
            raise ValueError(f"unknown servo channel {channel!r}")
        if self.failed:
            self._record_error("servo controller not responding")
            raise IOError("servo controller not responding")
        prev = self._angles.get("gripper")
        self._angles[channel] = float(angle_deg)
        if channel == "gripper":
            closing = angle_deg >= self.gripper_closed - 5
            if closing and (prev is None or prev < self.gripper_closed - 5):
                self.world.try_grip()
            elif not closing and self.world.held_object is not None:
                self.world.release(self._bin_under_gripper())

    def _bin_under_gripper(self) -> Optional[int]:
        # The object only falls into a bin when the arm is lifted over the
        # carousel and the carousel is aligned with one of the bin slots.
        if self._angles.get("shoulder", 0) < 120:
            return None
        sorter = self._angles.get("sorter")
        for bin_id, ang in self.bin_angles.items():
            if sorter is not None and abs(sorter - ang) < 5:
                return bin_id
        return None

    def get_angle(self, channel: str) -> Optional[float]:
        return self._angles.get(channel)

    def release(self) -> None:
        pass


class SimBattery(BatteryMonitor):
    simulated = True

    def __init__(self, name: str, world: SimWorld):
        super().__init__(name)
        self.world = world

    def voltage(self) -> Optional[float]:
        return self.world.battery_v + float(self.world.rng.normal(0, 0.01))


class SimEmergencyStop(EmergencyStopInput):
    simulated = True

    def __init__(self, name: str, world: SimWorld):
        super().__init__(name)
        self.world = world

    def is_pressed(self) -> bool:
        return self.world.estop_pressed


class SimBinSensor(BinSensor):
    simulated = True

    def __init__(self, name: str, world: SimWorld):
        super().__init__(name)
        self.world = world

    def event_count(self) -> int:
        return self.world.bin_events


class SimCamera(Camera):
    """Renders a synthetic pinhole camera view of the world objects."""

    simulated = True

    def __init__(self, name: str, world: SimWorld, width: int, height: int, focal_px: float,
                 mount_height: float, pitch_deg: float):
        super().__init__(name)
        self.world = world
        self.w, self.h = width, height
        self.f = focal_px
        self.cam_h = mount_height
        self.pitch = math.radians(pitch_deg)
        self.covered = False
        self.fps_measured = 0.0
        self._last_t = time.time()
        self._noise = np.random.default_rng(3)

    def _project(self, fwd: float, lat: float, z: float) -> Optional[tuple[float, float, float]]:
        """Robot frame point (x fwd, y left, z up) -> pixel (u, v, depth)."""
        zc = z - self.cam_h
        # rotate by pitch (negative pitch looks down)
        depth = fwd * math.cos(self.pitch) + zc * math.sin(self.pitch)
        up = -fwd * math.sin(self.pitch) + zc * math.cos(self.pitch)
        if depth <= 0.03:
            return None
        u = self.w / 2 - self.f * lat / depth
        v = self.h / 2 - self.f * up / depth
        return u, v, depth

    def read(self):
        now = time.time()
        dt = now - self._last_t
        self._last_t = now
        if dt > 0:
            self.fps_measured = 0.8 * self.fps_measured + 0.2 / dt
        img = np.empty((self.h, self.w, 3), np.uint8)
        if self.covered:
            img[:] = 3
            return False, img, now
        horizon = self._project(100.0, 0.0, self.cam_h)
        hv = int(np.clip(horizon[1] if horizon else 0, 0, self.h))
        img[:hv] = (200, 205, 210)      # wall
        img[hv:] = (120, 135, 150)      # floor
        x, y, th = self.world.true_pose()
        drawn = []
        for o in self.world.objects:
            if o.held or o.sorted_bin is not None:
                continue
            dx, dy = o.x - x, o.y - y
            fwd = dx * math.cos(th) + dy * math.sin(th)
            lat = -dx * math.sin(th) + dy * math.cos(th)
            if fwd <= 0.05 or fwd > 3.0:
                continue
            p = self._project(fwd, lat, o.radius)
            if p is None or not self.world.visible(o.x, o.y):
                continue
            drawn.append((p[2], p, o))
        # far to near so close objects occlude far ones
        for depth, (u, v, _), o in sorted(drawn, key=lambda t: -t[0]):
            r = max(2, int(self.f * o.radius / depth))
            if -r < u < self.w + r and -r < v < self.h + r:
                color = OBJECT_BGR.get(o.color, (128, 128, 128))
                cv2.circle(img, (int(u), int(v)), r, color, -1, cv2.LINE_AA)
                shade = tuple(int(c * 0.75) for c in color)
                cv2.ellipse(img, (int(u), int(v)), (r, r), 0, 20, 160, shade, max(1, r // 4))
        noise = self._noise.normal(0, 3, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        return True, img, now
