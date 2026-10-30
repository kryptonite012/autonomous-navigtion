"""Simple 2D physics world used by simulation mode.

It holds the *true* robot pose, static geometry and the sortable objects.
Simulated devices read from it with noise; the rest of the software never
sees the true pose, only what the sensors report.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import yaml

from robot.config import resolve_path

# BGR colours used to render objects in the synthetic camera image
OBJECT_BGR = {
    "red": (40, 40, 210),
    "green": (60, 170, 50),
    "blue": (200, 90, 30),
    "yellow": (40, 210, 230),
    "orange": (30, 130, 245),
    "white": (235, 235, 235),
    "black": (25, 25, 25),
}


@dataclass
class SimObject:
    id: int
    color: str
    x: float
    y: float
    radius: float = 0.025
    held: bool = False
    sorted_bin: Optional[int] = None


@dataclass
class SimState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vl: float = 0.0       # wheel surface speeds m/s
    vr: float = 0.0
    left_dist: float = 0.0   # accumulated wheel travel
    right_dist: float = 0.0
    collisions: int = 0      # steps where the robot drove into something
    contacts: int = 0        # steps where something (a moving obstacle) touched the robot


class SimWorld:
    def __init__(self, world_file: str, chassis: dict, seed: int = 7,
                 sensor_noise: float = 0.01, odom_noise: float = 0.02,
                 pickup_failure_rate: float = 0.0):
        with open(resolve_path(world_file), "r", encoding="utf-8") as fh:
            spec = yaml.safe_load(fh)
        self.name = spec.get("name", "world")
        self.bounds = tuple(spec["bounds"])
        self.rects = [tuple(o["rect"]) for o in spec.get("obstacles", [])]
        self.circles = [tuple(o["circle"]) for o in spec.get("pillars", [])]
        self.dynamic: list[list[float]] = []   # [x, y, r, vx, vy]
        self.objects = [SimObject(i + 1, o["color"], *o["position"])
                        for i, o in enumerate(spec.get("objects", []))]
        sx, sy, sth = spec.get("start_pose", [0, 0, 0])
        self.state = SimState(sx, sy, sth)
        self.start_pose = (sx, sy, sth)
        self.wheel_base = chassis["wheel_base_m"]
        self.robot_radius = chassis.get("robot_radius_m", 0.12)
        self.max_wheel = chassis["max_linear_mps"] * 1.6
        self.rng = np.random.default_rng(seed)
        self.sensor_noise = sensor_noise
        self.odom_noise = odom_noise
        self.pickup_failure_rate = pickup_failure_rate
        self.duty = (0.0, 0.0)
        self.motor_fault = False
        self.battery_v = 8.2
        self.estop_pressed = False
        self.bin_events = 0
        self.held_object: Optional[SimObject] = None
        self.encoder_fault = False
        self.lock = threading.RLock()
        self.sim_time = 0.0

    # ------------------------------------------------------------------ physics
    def set_duty(self, left: float, right: float) -> None:
        with self.lock:
            self.duty = (left, right)

    def _collides(self, x: float, y: float) -> bool:
        r = self.robot_radius
        x0, y0, x1, y1 = self.bounds
        if x - r < x0 or x + r > x1 or y - r < y0 or y + r > y1:
            return True
        for (ax, ay, bx, by) in self.rects:
            cx, cy = min(max(x, ax), bx), min(max(y, ay), by)
            if math.hypot(x - cx, y - cy) < r:
                return True
        for (cx, cy, cr) in self.circles:
            if math.hypot(x - cx, y - cy) < r + cr:
                return True
        for (cx, cy, cr, _, _) in self.dynamic:
            if math.hypot(x - cx, y - cy) < r + cr:
                return True
        return False

    def step(self, dt: float) -> None:
        with self.lock:
            s = self.state
            self.sim_time += dt
            dl, dr = (0.0, 0.0) if (self.motor_fault or self.estop_pressed) else self.duty
            # first order motor response, tau = 80 ms
            alpha = min(1.0, dt / 0.08)
            s.vl += alpha * (dl * self.max_wheel - s.vl)
            s.vr += alpha * (dr * self.max_wheel - s.vr)
            v = (s.vl + s.vr) / 2
            w = (s.vr - s.vl) / self.wheel_base
            nth = s.theta + w * dt
            nx = s.x + v * math.cos(s.theta + w * dt / 2) * dt
            ny = s.y + v * math.sin(s.theta + w * dt / 2) * dt
            if self._collides(nx, ny):
                if self._collides(s.x, s.y):
                    s.contacts += 1          # already touching: hit by a moving obstacle
                elif abs(v) > 1e-3:
                    s.collisions += 1        # the robot moved into an obstacle
                # wheels spin but the chassis is blocked: odometry drifts, as on a real robot
                s.left_dist += s.vl * dt
                s.right_dist += s.vr * dt
                # a round robot can still turn on the spot
                s.theta = math.atan2(math.sin(nth), math.cos(nth))
            else:
                s.x, s.y = nx, ny
                s.theta = math.atan2(math.sin(nth), math.cos(nth))
                s.left_dist += s.vl * dt
                s.right_dist += s.vr * dt
            for d in self.dynamic:
                px, py = d[0] + d[3] * dt, d[1] + d[4] * dt
                x0, y0, x1, y1 = self.bounds
                hit_robot = math.hypot(px - s.x, py - s.y) < d[2] + self.robot_radius
                hit_static = any(math.hypot(px - min(max(px, ax), bx), py - min(max(py, ay), by)) < d[2]
                                 for (ax, ay, bx, by) in self.rects) or \
                    any(math.hypot(px - cx, py - cy) < d[2] + cr for (cx, cy, cr) in self.circles)
                if hit_robot or hit_static:
                    # bounce back (like a ball) instead of passing through
                    d[3], d[4] = -d[3], -d[4]
                    continue
                d[0], d[1] = px, py
                if not (x0 + d[2] < d[0] < x1 - d[2]):
                    d[3] = -d[3]
                if not (y0 + d[2] < d[1] < y1 - d[2]):
                    d[4] = -d[4]
            draw = 0.05 + 0.4 * (abs(dl) + abs(dr))  # amps, rough
            self.battery_v = max(6.0, self.battery_v - draw * dt * 1e-4)
            if self.held_object:
                self.held_object.x, self.held_object.y = s.x, s.y

    def add_dynamic_obstacle(self, x: float, y: float, r: float = 0.1, vx: float = 0.0, vy: float = 0.0) -> None:
        with self.lock:
            self.dynamic.append([x, y, r, vx, vy])

    # ------------------------------------------------------------------ sensing
    def raycast(self, ox: float, oy: float, angle: float, max_range: float,
                include_objects: bool = False) -> Optional[float]:
        """Distance to the first surface along a ray, None if beyond max_range."""
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        x0, y0, x1, y1 = self.bounds
        # room walls (we are inside the box)
        for t in ((x0 - ox) / dx if dx < 0 else (x1 - ox) / dx if dx > 0 else math.inf,
                  (y0 - oy) / dy if dy < 0 else (y1 - oy) / dy if dy > 0 else math.inf):
            if 0 < t < best:
                best = t
        for (ax, ay, bx, by) in self.rects:
            t = _ray_aabb(ox, oy, dx, dy, ax, ay, bx, by)
            if t is not None and t < best:
                best = t
        circles = list(self.circles) + [(c[0], c[1], c[2]) for c in self.dynamic]
        if include_objects:
            circles += [(o.x, o.y, o.radius) for o in self.objects if not o.held and o.sorted_bin is None]
        for (cx, cy, cr) in circles:
            t = _ray_circle(ox, oy, dx, dy, cx, cy, cr)
            if t is not None and t < best:
                best = t
        return best if best <= max_range else None

    def true_pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.state.x, self.state.y, self.state.theta

    def visible(self, x: float, y: float) -> bool:
        sx, sy, _ = self.true_pose()
        ang = math.atan2(y - sy, x - sx)
        dist = math.hypot(x - sx, y - sy)
        hit = self.raycast(sx, sy, ang, dist)
        return hit is None or hit >= dist - 0.03

    # ------------------------------------------------------------------ objects
    def object_in_gripper_zone(self) -> Optional[SimObject]:
        """Object directly in front of the robot within arm reach."""
        sx, sy, th = self.true_pose()
        best = None
        for o in self.objects:
            if o.held or o.sorted_bin is not None:
                continue
            dx, dy = o.x - sx, o.y - sy
            fwd = dx * math.cos(th) + dy * math.sin(th)
            lat = -dx * math.sin(th) + dy * math.cos(th)
            if 0.10 <= fwd <= 0.22 and abs(lat) <= 0.05:
                if best is None or fwd < best[0]:
                    best = (fwd, o)
        return best[1] if best else None

    def try_grip(self) -> bool:
        with self.lock:
            obj = self.object_in_gripper_zone()
            if obj is None:
                return False
            if self.rng.random() < self.pickup_failure_rate:
                # knocked the object slightly away instead of gripping it
                th = self.state.theta
                obj.x += 0.04 * math.cos(th)
                obj.y += 0.04 * math.sin(th)
                return False
            obj.held = True
            self.held_object = obj
            return True

    def release(self, bin_id: Optional[int]) -> None:
        """Gripper opened. If above the sorter carousel the object falls into a bin."""
        with self.lock:
            obj = self.held_object
            if obj is None:
                return
            obj.held = False
            self.held_object = None
            if bin_id is not None:
                obj.sorted_bin = bin_id
                self.bin_events += 1
            else:
                th = self.state.theta
                obj.x = self.state.x + 0.15 * math.cos(th)
                obj.y = self.state.y + 0.15 * math.sin(th)

    def reset(self) -> None:
        with self.lock:
            x, y, th = self.start_pose
            self.state = SimState(x, y, th)
            self.duty = (0.0, 0.0)

    # ------------------------------------------------------------ scenarios
    def free_position(self, margin: float = 0.2, tries: int = 200) -> tuple[float, float]:
        x0, y0, x1, y1 = self.bounds
        for _ in range(tries):
            x = float(self.rng.uniform(x0 + margin, x1 - margin))
            y = float(self.rng.uniform(y0 + margin, y1 - margin))
            r = self.robot_radius
            self.robot_radius = margin
            blocked = self._collides(x, y)
            self.robot_radius = r
            if not blocked:
                return x, y
        raise RuntimeError("no free position found")

    def scatter_objects(self, count: int, colors: list[str]) -> list[SimObject]:
        """Replace all objects with ``count`` new ones at random free positions."""
        with self.lock:
            if self.held_object is not None:
                self.held_object = None
            objs = []
            for i in range(count):
                x, y = self.free_position(margin=0.15)
                objs.append(SimObject(i + 1, colors[i % len(colors)], x, y))
            self.objects = objs
            return objs

    def teleport(self, x: float, y: float, theta: float) -> None:
        """Move the robot without telling the odometry (kidnapped robot test)."""
        with self.lock:
            if self._collides(x, y):
                raise ValueError("position is inside an obstacle")
            self.state.x, self.state.y, self.state.theta = x, y, theta
            self.state.vl = self.state.vr = 0.0

    def clear_dynamic(self) -> None:
        with self.lock:
            self.dynamic.clear()

    def snapshot(self) -> dict:
        with self.lock:
            s = self.state
            return {
                "name": self.name,
                "bounds": list(self.bounds),
                "rects": [list(r) for r in self.rects],
                "circles": [list(c) for c in self.circles],
                "dynamic": [[round(v, 3) for v in d] for d in self.dynamic],
                "robot": {"x": round(s.x, 4), "y": round(s.y, 4), "theta": round(s.theta, 4)},
                "objects": [{"id": o.id, "color": o.color, "x": round(o.x, 3), "y": round(o.y, 3),
                             "held": o.held, "sorted_bin": o.sorted_bin} for o in self.objects],
                "collisions": s.collisions,
                "contacts": s.contacts,
                "battery_v": round(self.battery_v, 3),
                "faults": {"motor_fault": self.motor_fault, "encoder_fault": self.encoder_fault,
                           "estop_pressed": self.estop_pressed},
            }

    def reference_grid(self, grid, spacing: float = 0.35, beams: int = 90, max_range: float = 3.0):
        """Fill ``grid`` with a map of the static world by ray casting from many
        free positions. This is simulation ground truth, used for testing
        localisation/navigation without exploring first."""
        x0, y0, x1, y1 = self.bounds
        saved = self.dynamic
        self.dynamic = []
        try:
            x = x0 + spacing / 2
            while x < x1:
                y = y0 + spacing / 2
                while y < y1:
                    if not self._collides(x, y):
                        for k in range(beams):
                            a = 2 * math.pi * k / beams
                            d = self.raycast(x, y, a, max_range)
                            for _ in range(2):
                                grid.integrate_ray(x, y, a, d, max_range)
                    y += spacing
                x += spacing
        finally:
            self.dynamic = saved
        return grid


def _ray_aabb(ox, oy, dx, dy, ax, ay, bx, by) -> Optional[float]:
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, ax, bx), (oy, dy, ay, by)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0):
        return None
    return tmin if tmin > 0 else None


def _ray_circle(ox, oy, dx, dy, cx, cy, r) -> Optional[float]:
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 0 else None


class SimClock(threading.Thread):
    """Steps the world in real time on a background thread."""

    def __init__(self, world: SimWorld, rate_hz: float = 100.0):
        super().__init__(name="sim-clock", daemon=True)
        self.world = world
        self.dt = 1.0 / rate_hz
        self._stop_evt = threading.Event()

    def run(self) -> None:
        last = time.monotonic()
        while not self._stop_evt.is_set():
            now = time.monotonic()
            self.world.step(min(now - last, 0.05))
            last = now
            self._stop_evt.wait(self.dt)

    def stop(self) -> None:
        self._stop_evt.set()
