"""Dynamic Window Approach local planner (Fox, Burgard, Thrun 1997).

Each control cycle:

1. Build the *dynamic window*: the (v, w) pairs reachable from the current
   velocity within ``window_s`` under the acceleration limits, clipped to the
   speed limits.
2. Roll every sampled pair forward for ``horizon_s`` (constant twist arcs).
3. Discard trajectories that pass closer than the robot radius (+ margin) to
   any obstacle point (map walls near the robot + fresh range readings).
4. Score the rest: progress towards the path target, clearance, speed.

Compared with pure pursuit + potential field, DWA steers around obstacles
that were not in the map (people, moved chairs, a rolling ball) instead of
stopping and replanning. The safety controller still has the final word.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from robot.motors.kinematics import normalize_angle


@dataclass
class DWAConfig:
    max_v: float = 0.24
    max_w: float = 1.6
    acc_v: float = 0.6
    acc_w: float = 3.0
    window_s: float = 0.3
    horizon_s: float = 1.6
    sim_dt: float = 0.1
    v_samples: int = 7
    w_samples: int = 17
    robot_radius: float = 0.12
    safety_margin: float = 0.03
    w_goal: float = 1.0
    w_clear: float = 0.5
    w_speed: float = 0.25
    clearance_cap: float = 0.5        # clearance beyond this earns no extra score
    check_distance: float = 0.45      # collision-check each arc at least this far ahead


class DWAPlanner:
    def __init__(self, cfg: DWAConfig):
        self.cfg = cfg
        self.last_trajectory: list[tuple[float, float]] = []
        self.last_candidates = 0
        self.last_valid = 0

    def window(self, v: float, w: float) -> tuple[np.ndarray, np.ndarray]:
        c = self.cfg
        v_lo = max(0.0, v - c.acc_v * c.window_s)
        v_hi = min(c.max_v, v + c.acc_v * c.window_s)
        w_lo = max(-c.max_w, w - c.acc_w * c.window_s)
        w_hi = min(c.max_w, w + c.acc_w * c.window_s)
        vs = np.unique(np.concatenate([np.linspace(v_lo, v_hi, c.v_samples), [0.0]]))
        ws = np.unique(np.concatenate([np.linspace(w_lo, w_hi, c.w_samples), [0.0]]))
        vv, ww = np.meshgrid(vs, ws, indexing="ij")
        return vv.ravel(), ww.ravel()

    def rollout(self, pose: tuple[float, float, float], vs: np.ndarray, ws: np.ndarray) -> np.ndarray:
        """Trajectories [candidates, steps, 2] of (x, y)."""
        c = self.cfg
        steps = int(round(c.horizon_s / c.sim_dt))
        t = np.arange(1, steps + 1) * c.sim_dt                  # [steps]
        x0, y0, th0 = pose
        th = th0 + ws[:, None] * t[None, :]
        straight = np.abs(ws) < 1e-6
        safe_w = np.where(straight, 1.0, ws)
        r = vs / safe_w
        xs = np.where(straight[:, None], x0 + vs[:, None] * np.cos(th0) * t,
                      x0 + r[:, None] * (np.sin(th) - np.sin(th0)))
        ys = np.where(straight[:, None], y0 + vs[:, None] * np.sin(th0) * t,
                      y0 - r[:, None] * (np.cos(th) - np.cos(th0)))
        return np.stack([xs, ys], axis=2)

    def collision_arcs(self, pose: tuple[float, float, float], vs: np.ndarray, ws: np.ndarray,
                       samples: int = 16, check_distance: Optional[float] = None) -> np.ndarray:
        """Arc points used for collision checking, parameterised by distance.

        A slow arc covers little ground within the horizon, so a time-based
        check would let the robot creep straight at an obstacle. Checking at
        least ``check_distance`` along every arc forces it to steer (or stop
        and turn) instead.
        """
        c = self.cfg
        x0, y0, th0 = pose
        check = c.check_distance if check_distance is None else check_distance
        s_max = np.where(vs > 0, np.maximum(vs * c.horizon_s, check), 0.0)
        s = s_max[:, None] * (np.arange(1, samples + 1) / samples)[None, :]
        k = np.where(vs > 0, ws / np.where(vs > 0, vs, 1.0), 0.0)        # curvature
        th = th0 + k[:, None] * s
        straight = np.abs(k) < 1e-6
        safe_k = np.where(straight, 1.0, k)
        xs = np.where(straight[:, None], x0 + s * np.cos(th0), x0 + (np.sin(th) - np.sin(th0)) / safe_k[:, None])
        ys = np.where(straight[:, None], y0 + s * np.sin(th0), y0 - (np.cos(th) - np.cos(th0)) / safe_k[:, None])
        return np.stack([xs, ys], axis=2)

    def compute(self, pose: tuple[float, float, float], current: tuple[float, float],
                target: tuple[float, float], obstacles: np.ndarray,
                final: bool = False) -> Optional[tuple[float, float]]:
        """Best (v, w), or None if every reachable trajectory collides."""
        c = self.cfg
        vs, ws = self.window(*current)
        traj = self.rollout(pose, vs, ws)
        self.last_candidates = len(vs)
        if len(obstacles):
            # distance from every trajectory point to every obstacle point
            obs = np.asarray(obstacles, dtype=np.float64)
            near = np.hypot(obs[:, 0] - pose[0], obs[:, 1] - pose[1]) < max(c.horizon_s * c.max_v, c.check_distance) + 0.6
            obs = obs[near]
        else:
            obs = np.zeros((0, 2))
        if len(obs):
            # near the goal only check up to the goal (it may be close to a wall)
            check = min(c.check_distance, math.dist(pose[:2], target) + 0.05) if final else None
            arcs = self.collision_arcs(pose, vs, ws, check_distance=check)
            d = np.sqrt(((arcs[:, :, None, :] - obs[None, None, :, :]) ** 2).sum(axis=3))   # [cand, steps, obs]
            clearance = d.min(axis=(1, 2)) - c.robot_radius
        else:
            clearance = np.full(len(vs), c.clearance_cap)
        valid = clearance > c.safety_margin
        # rotating on the spot never sweeps new area: always allowed
        valid |= (vs == 0.0)
        self.last_valid = int(valid.sum())
        if not valid.any():
            self.last_trajectory = []
            return None
        end = traj[:, -1, :]
        dist_goal = np.hypot(end[:, 0] - target[0], end[:, 1] - target[1])
        start_dist = math.dist(pose[:2], target)
        progress = start_dist - dist_goal                        # metres gained
        # heading at the end of the arc relative to the target direction
        th_end = pose[2] + ws * c.horizon_s
        bearing = np.arctan2(target[1] - end[:, 1], target[0] - end[:, 0])
        head = 1.0 - np.abs(np.arctan2(np.sin(bearing - th_end), np.cos(bearing - th_end))) / math.pi
        goal = progress / (c.max_v * c.horizon_s) + 0.3 * head
        clear = np.minimum(clearance, c.clearance_cap) / c.clearance_cap
        speed = vs / c.max_v
        if final:
            # near the goal: don't reward speed, avoid overshooting
            speed = -np.abs(vs - min(c.max_v, start_dist)) / c.max_v
        score = c.w_goal * goal + c.w_clear * clear + c.w_speed * speed
        score[~valid] = -np.inf
        i = int(np.argmax(score))
        self.last_trajectory = [(round(float(x), 3), round(float(y), 3)) for x, y in traj[i]]
        return float(vs[i]), float(ws[i])
