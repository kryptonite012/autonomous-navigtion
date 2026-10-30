"""Local motion control: pure pursuit path follower and potential field avoidance."""

from __future__ import annotations

import math
from typing import Optional

from robot.hardware.base import RangeReading
from robot.motors.kinematics import normalize_angle


class PurePursuit:
    def __init__(self, lookahead: float, max_v: float, max_w: float, goal_tolerance: float):
        self.lookahead = lookahead
        self.max_v = max_v
        self.max_w = max_w
        self.goal_tolerance = goal_tolerance

    def target_point(self, pose: tuple[float, float, float], path: list[tuple[float, float]],
                     start_idx: int = 0) -> tuple[tuple[float, float], int]:
        """Furthest path point within the lookahead circle (or the next one outside)."""
        x, y, _ = pose
        idx = start_idx
        # advance past points we are already close to
        while idx < len(path) - 1 and math.dist((x, y), path[idx]) < self.lookahead:
            idx += 1
        return path[idx], idx

    def compute(self, pose: tuple[float, float, float], target: tuple[float, float],
                final: bool) -> tuple[float, float]:
        x, y, th = pose
        dx, dy = target[0] - x, target[1] - y
        dist = math.hypot(dx, dy)
        heading_err = normalize_angle(math.atan2(dy, dx) - th)
        # rotate on the spot first when the target is well off to the side
        if abs(heading_err) > math.radians(50):
            return 0.0, math.copysign(min(self.max_w, 1.2 + abs(heading_err)), heading_err)
        curvature = 2.0 * math.sin(heading_err) / max(dist, 1e-3)
        v = self.max_v * max(0.25, math.cos(heading_err))
        if final:
            v = min(v, max(0.05, dist * 1.2))
        w = max(-self.max_w, min(self.max_w, v * curvature))
        return v, w


def potential_field(ranges: list[RangeReading], v: float, w: float, influence: float = 0.4,
                    gain: float = 0.6) -> tuple[float, float]:
    """Bend the commanded twist away from close obstacles.

    Each valid range reading inside ``influence`` produces a repulsive
    vector; its lateral component turns the robot and its forward
    component slows it. The safety controller still enforces hard stops.
    """
    if v <= 0.0:
        # turning on the spot or reversing: repulsion would only fight the turn
        return v, w
    fx = fy = 0.0
    for r in ranges:
        if not r.valid or r.distance_m is None or r.distance_m >= influence:
            continue
        mag = gain * (1.0 / max(r.distance_m, 0.05) - 1.0 / influence)
        fx -= mag * math.cos(r.angle_rad)
        fy -= mag * math.sin(r.angle_rad)
    if fx == 0.0 and fy == 0.0:
        return v, w
    v_new = max(0.0, v + 0.05 * fx) if v > 0 else v
    w_new = w + 0.8 * fy
    return v_new, w_new


def heading_to(pose: tuple[float, float, float], goal_theta: Optional[float]) -> float:
    if goal_theta is None:
        return 0.0
    return normalize_angle(goal_theta - pose[2])
