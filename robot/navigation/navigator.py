"""Navigation state machine: goals, replanning, recovery and frontier exploration.

This is the stand-alone (non-ROS) navigation engine. With ROS 2 installed the
same role is played by Nav2 (see ros2_ws/src/nexus_navigation); this module
remains as the fallback planner and the one used in simulation tests.
"""

from __future__ import annotations

import enum
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from robot.hardware.base import RangeReading
from robot.mapping.occupancy_grid import FREE, UNKNOWN, OccupancyGrid
from robot.motors.kinematics import normalize_angle
from robot.navigation.control import PurePursuit, potential_field
from robot.navigation.dwa import DWAConfig, DWAPlanner
from robot.navigation.planners import clearance_cost, line_free, nearest_free, plan, simplify

log = logging.getLogger(__name__)


class NavState(str, enum.Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    PAUSED = "paused"
    RECOVERING = "recovering"
    WAITING_LOCALIZATION = "waiting_localization"
    REACHED = "reached"
    FAILED = "failed"
    EXPLORING = "exploring"


@dataclass
class Goal:
    x: float
    y: float
    theta: Optional[float] = None
    name: Optional[str] = None


class Navigator:
    def __init__(self, cfg: dict, grid_provider: Callable[[], OccupancyGrid],
                 on_event: Optional[Callable[[str, str, str], None]] = None):
        nc, ch = cfg["navigation"], cfg["chassis"]
        self.nc = nc
        self.grid_provider = grid_provider
        self.on_event = on_event or (lambda *a: None)
        self.inflation = nc["inflation_radius_m"]
        self.goal_tol = nc["goal_tolerance_m"]
        self.heading_tol = nc["heading_tolerance_rad"]
        self.replan_interval = nc["replan_interval_s"]
        self.algorithm = nc.get("planner", "astar")
        self.allow_unknown = nc.get("allow_unknown", False)
        self.stuck_timeout = nc["stuck_timeout_s"]
        self.max_recoveries = nc["max_recoveries"]
        self.pose_timeout = cfg["safety"]["pose_timeout_s"]
        self.named_locations: dict[str, list[float]] = dict(nc.get("named_locations", {}))
        self.follower = PurePursuit(nc["lookahead_m"], ch["max_linear_mps"] * 0.8,
                                    ch["max_angular_rps"] * 0.8, self.goal_tol)
        # local controller: "dwa" (default) or "pure_pursuit" (pure pursuit + potential field)
        self.controller = nc.get("controller", "dwa")
        dc = nc.get("dwa", {})
        self.dwa = DWAPlanner(DWAConfig(max_v=ch["max_linear_mps"] * 0.8, max_w=ch["max_angular_rps"] * 0.8,
                                        acc_v=ch["max_linear_accel_mps2"], robot_radius=ch["robot_radius_m"],
                                        **{k: v for k, v in dc.items() if k in DWAConfig.__dataclass_fields__}))
        self.dwa_lookahead = nc.get("dwa_lookahead_m", 0.5)
        self._last_cmd = (0.0, 0.0)
        self._scan_points: list[tuple[float, float]] = []
        self.state = NavState.IDLE
        self.goal: Optional[Goal] = None
        self.path: list[tuple[float, float]] = []
        self.path_idx = 0
        self.message = ""
        self.recoveries = 0
        self.exploring = False
        self.explore_done = False
        self._prev_state = NavState.IDLE
        self._last_plan = 0.0
        self._progress_pose: Optional[tuple[float, float, float]] = None
        self._progress_t = 0.0
        self._recover_until = 0.0
        self._recover_phase = 0
        self._local_obstacles: list[tuple[float, float, float]] = []
        self._failed_frontiers: list[tuple[float, float]] = []
        self._spin_until = 0.0
        self._no_frontier_count = 0
        self._best_dist = math.inf
        self._lock = threading.RLock()
        self.distance_remaining: Optional[float] = None

    # --------------------------------------------------------------- commands
    def set_goal(self, x: float, y: float, theta: Optional[float] = None, name: Optional[str] = None) -> None:
        with self._lock:
            self.exploring = False
            self.goal = Goal(x, y, theta, name)
            self._start()
            self.on_event("navigation_goal", "info", f"goal set to ({x:.2f}, {y:.2f})" + (f" [{name}]" if name else ""))

    def go_to_named(self, name: str) -> None:
        if name not in self.named_locations:
            raise KeyError(f"unknown location {name!r}")
        x, y, *rest = self.named_locations[name]
        self.set_goal(x, y, rest[0] if rest else None, name)

    def start_exploration(self) -> None:
        with self._lock:
            self.exploring = True
            self.explore_done = False
            self._failed_frontiers.clear()
            self.goal = None
            self._start()
            self.state = NavState.EXPLORING
            # look around first so the sensors populate the map
            self._spin_until = time.time() + 2 * math.pi / 0.8
            self._no_frontier_count = 0
            self.on_event("exploration_started", "info", "frontier exploration started")

    def _start(self) -> None:
        self.state = NavState.EXPLORING if self.exploring else NavState.NAVIGATING
        self.path, self.path_idx = [], 0
        self.recoveries = 0
        self._last_plan = 0.0
        self._progress_pose = None
        self.message = ""

    def pause(self) -> bool:
        with self._lock:
            if self.state in (NavState.NAVIGATING, NavState.EXPLORING, NavState.RECOVERING,
                              NavState.WAITING_LOCALIZATION):
                self._prev_state = NavState.EXPLORING if self.exploring else NavState.NAVIGATING
                self.state = NavState.PAUSED
                return True
            return False

    def resume(self) -> bool:
        with self._lock:
            if self.state == NavState.PAUSED:
                self.state = self._prev_state
                self._last_plan = 0.0
                self._progress_pose = None
                return True
            return False

    def cancel(self, reason: str = "cancelled") -> None:
        with self._lock:
            active = self.state not in (NavState.IDLE, NavState.REACHED, NavState.FAILED)
            self.state = NavState.IDLE
            self.goal = None
            self.exploring = False
            self.path = []
            self.message = reason
        if active:
            self.on_event("navigation_cancelled", "info", reason)

    @property
    def active(self) -> bool:
        return self.state in (NavState.NAVIGATING, NavState.EXPLORING, NavState.RECOVERING,
                              NavState.WAITING_LOCALIZATION)

    # --------------------------------------------------------------- planning
    def add_range_obstacles(self, pose: tuple[float, float, float], ranges: list[RangeReading]) -> None:
        now = time.time()
        x, y, th = pose
        with self._lock:
            for r in ranges:
                if r.valid and r.distance_m is not None and r.distance_m < 1.0:
                    a = th + r.angle_rad
                    self._local_obstacles.append((x + (r.distance_m + 0.03) * math.cos(a),
                                                  y + (r.distance_m + 0.03) * math.sin(a), now))
            self._local_obstacles = [o for o in self._local_obstacles if now - o[2] < 4.0][-300:]

    def _blocked(self, grid: OccupancyGrid, allow_unknown: bool) -> np.ndarray:
        blocked = grid.inflated_obstacles(self.inflation, allow_unknown)
        if self._local_obstacles:
            r = int(math.ceil(self.inflation / grid.resolution))
            for ox, oy, _ in self._local_obstacles:
                cx, cy = grid.world_to_cell(ox, oy)
                blocked[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1] = True
        return blocked

    def _plan_to(self, pose, gx: float, gy: float, allow_unknown: bool) -> Optional[list[tuple[float, float]]]:
        grid = self.grid_provider()
        blocked = self._blocked(grid, allow_unknown)
        s = grid.world_to_cell(pose[0], pose[1])
        g = grid.world_to_cell(gx, gy)
        if not grid.in_bounds(*g):
            self.message = "goal outside the map"
            return None
        # the robot's own footprint may sit inside inflation after a close pass
        s2 = nearest_free(blocked, s, max_radius=int(0.3 / grid.resolution))
        if s2 is None:
            self.message = "robot position is not in free space"
            return None
        if s2 != s:
            blocked[s[1], s[0]] = False
        if blocked[g[1], g[0]]:
            g2 = nearest_free(blocked, g, max_radius=int(0.15 / grid.resolution))
            if g2 is None:
                self.message = "goal is inside an obstacle or unknown space"
                return None
            g = g2
        cells = plan(blocked, s2, g, self.algorithm, clearance_cost(blocked))
        if cells is None:
            self.message = "no path found"
            return None
        cells = simplify(blocked, cells)
        pts = [grid.cell_to_world(cx, cy) for cx, cy in cells]
        pts[-1] = (gx, gy) if grid.world_to_cell(gx, gy) == g else pts[-1]
        return pts

    def _path_blocked(self, pose) -> bool:
        if not self.path:
            return False
        grid = self.grid_provider()
        blocked = self._blocked(grid, self.allow_unknown or self.exploring)
        prev = grid.world_to_cell(pose[0], pose[1])
        blocked[prev[1], prev[0]] = False
        for p in self.path[self.path_idx:self.path_idx + 4]:
            c = grid.world_to_cell(*p)
            if not grid.in_bounds(*c):
                return True
            if not line_free(blocked, prev, c):
                return True
            prev = c
        return False

    # ------------------------------------------------------------ exploration
    def find_frontier(self, pose) -> Optional[tuple[float, float]]:
        """Nearest cluster of free cells bordering unknown space."""
        import cv2
        grid = self.grid_provider()
        t = grid.ternary()
        free = (t == FREE).astype(np.uint8)
        unknown = (t == UNKNOWN).astype(np.uint8)
        near_unknown = cv2.dilate(unknown, np.ones((3, 3), np.uint8))
        blocked = self._blocked(grid, allow_unknown=True)
        frontier = (free & near_unknown & (~blocked).astype(np.uint8))
        n, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier, connectivity=8)
        best = None
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < 3:
                continue
            ys, xs = np.nonzero(labels == i)
            # use a member cell, not the centroid (which may sit in a wall)
            k = len(xs) // 2
            wx, wy = grid.cell_to_world(int(xs[k]), int(ys[k]))
            if any(math.dist((wx, wy), f) < 0.3 for f in self._failed_frontiers):
                continue
            d = math.dist((pose[0], pose[1]), (wx, wy))
            if d < 0.25:
                continue
            score = d - 0.02 * stats[i, cv2.CC_STAT_AREA]
            if best is None or score < best[0]:
                best = (score, (wx, wy))
        return best[1] if best else None

    # ------------------------------------------------------------------- step
    def step(self, pose: Optional[tuple[float, float, float]], pose_time: float,
             ranges: list[RangeReading]) -> Optional[tuple[float, float]]:
        """Advance the state machine; returns a twist to send, or None for 'stop'."""
        with self._lock:
            if not self.active:
                return None
            now = time.time()
            if pose is None or now - pose_time > self.pose_timeout:
                if self.state != NavState.WAITING_LOCALIZATION:
                    self.on_event("localization_lost", "warning", "pose invalid or stale, holding position")
                    self._prev_state = self.state
                self.state = NavState.WAITING_LOCALIZATION
                self.message = "waiting for valid localization"
                return None
            if self.state == NavState.WAITING_LOCALIZATION:
                self.state = self._prev_state if self._prev_state != NavState.WAITING_LOCALIZATION else NavState.NAVIGATING
                self._last_plan = 0.0

            self.add_range_obstacles(pose, ranges)

            if self.state == NavState.RECOVERING:
                return self._recover(now, pose)

            if self.exploring and now < self._spin_until:
                return 0.0, 0.8
            if self.exploring and (self.goal is None):
                f = self.find_frontier(pose)
                if f is None and self._no_frontier_count < 1:
                    # spin once more before concluding: sparse sensors miss a lot
                    self._no_frontier_count += 1
                    self._spin_until = now + 2 * math.pi / 0.8
                    return 0.0, 0.8
                if f is None:
                    self.state = NavState.REACHED
                    self.exploring = False
                    self.explore_done = True
                    self.message = "exploration complete: no reachable frontiers left"
                    self.on_event("exploration_complete", "info", self.message)
                    return None
                self._no_frontier_count = 0
                self.goal = Goal(f[0], f[1], None, "frontier")
                self.path, self.path_idx = [], 0
                self._last_plan = 0.0
                self._progress_pose = None

            g = self.goal
            dist = math.dist((pose[0], pose[1]), (g.x, g.y))
            self.distance_remaining = dist
            if dist <= self.goal_tol or (self.exploring and dist < 0.2):
                if g.theta is not None and not self.exploring:
                    err = normalize_angle(g.theta - pose[2])
                    if abs(err) > self.heading_tol:
                        return 0.0, math.copysign(min(1.0, 0.6 + abs(err)), err)
                if self.exploring:
                    self.goal = None
                    self.path = []
                    return None
                self.state = NavState.REACHED
                self.message = "goal reached"
                self.path = []
                self.on_event("navigation_reached", "info", f"reached ({g.x:.2f}, {g.y:.2f})")
                return None

            need_plan = (not self.path or self._last_plan == 0.0
                         or (now - self._last_plan > self.replan_interval and self._path_blocked(pose)))
            if need_plan:
                path = self._plan_to(pose, g.x, g.y, self.allow_unknown or self.exploring)
                self._last_plan = now
                if path is None:
                    if self.exploring:
                        self._failed_frontiers.append((g.x, g.y))
                        self.goal = None
                        return None
                    return self._begin_recovery(now, f"planning failed: {self.message}")
                self.path, self.path_idx = path, 0

            # stuck detection: the distance to the goal must keep shrinking
            if self._progress_pose is None or dist < self._best_dist - 0.05:
                self._progress_pose, self._progress_t, self._best_dist = pose, now, dist
            elif now - self._progress_t > self.stuck_timeout:
                return self._begin_recovery(now, "no progress (stuck)")

            target, self.path_idx = self.follower.target_point(pose, self.path, self.path_idx)
            final = self.path_idx == len(self.path) - 1
            if self.controller == "dwa":
                cmd = self._dwa_step(pose, final)
            else:
                v, w = self.follower.compute(pose, target, final)
                cmd = potential_field(ranges, v, w)
            self._last_cmd = cmd if cmd is not None else (0.0, 0.0)
            return cmd

    def set_scan_points(self, points: list[tuple[float, float]]) -> None:
        """World-frame LiDAR points from the latest scan (used by DWA)."""
        with self._lock:
            self._scan_points = points

    def _dwa_target(self, pose) -> tuple[float, float]:
        for p in self.path[self.path_idx:]:
            if math.dist(pose[:2], p) >= self.dwa_lookahead:
                return p
        return self.path[-1]

    def _dwa_obstacles(self, pose) -> np.ndarray:
        grid = self.grid_provider()
        pts: list[tuple[float, float]] = []
        cx, cy = grid.world_to_cell(pose[0], pose[1])
        r = int(1.0 / grid.resolution)
        y0, y1 = max(0, cy - r), min(grid.height, cy + r + 1)
        x0, x1 = max(0, cx - r), min(grid.width, cx + r + 1)
        if y1 > y0 and x1 > x0:
            sub = grid.ternary()[y0:y1, x0:x1]
            ys, xs = np.nonzero(sub == 100)
            pts.extend(grid.cell_to_world(int(x) + x0, int(y) + y0) for x, y in zip(xs, ys))
        now = time.time()
        # only fresh range hits: older ones belong to objects that may have moved
        pts.extend((ox, oy) for ox, oy, t in self._local_obstacles if now - t < 1.0)
        pts.extend(self._scan_points)
        return np.array(pts, dtype=np.float64).reshape(-1, 2)

    def _dwa_step(self, pose, final: bool) -> Optional[tuple[float, float]]:
        target = self._dwa_target(pose)
        err = normalize_angle(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
        if abs(err) > math.radians(90):
            # facing away from the path: turn on the spot first
            return 0.0, math.copysign(min(self.dwa.cfg.max_w, 1.0 + abs(err)), err)
        cmd = self.dwa.compute(pose, self._last_cmd, target, self._dwa_obstacles(pose), final)
        if cmd is None:
            # boxed in: stop; the stuck detector starts a recovery if it persists
            self.message = "no collision-free trajectory, waiting"
            return 0.0, 0.0
        return cmd

    def _begin_recovery(self, now: float, reason: str) -> Optional[tuple[float, float]]:
        self.recoveries += 1
        if self.recoveries > self.max_recoveries:
            if self.exploring:
                self._failed_frontiers.append((self.goal.x, self.goal.y)) if self.goal else None
                self.goal = None
                self.recoveries = 0
                return None
            self.state = NavState.FAILED
            self.message = f"navigation failed after {self.max_recoveries} recoveries: {reason}"
            self.path = []
            self.on_event("navigation_failed", "error", self.message)
            return None
        self.on_event("navigation_recovery", "warning", f"recovery {self.recoveries}: {reason}")
        self._prev_state = NavState.EXPLORING if self.exploring else NavState.NAVIGATING
        self.state = NavState.RECOVERING
        self._recover_phase = 0
        self._recover_until = now + 1.0
        self.path = []
        return None

    def _recover(self, now: float, pose) -> Optional[tuple[float, float]]:
        # phase 0: back off slowly, phase 1: rotate to look around, then replan
        if now < self._recover_until:
            return (-0.05, 0.0) if self._recover_phase == 0 else (0.0, 1.0)
        if self._recover_phase == 0:
            self._recover_phase = 1
            self._recover_until = now + 1.5
            return 0.0, 1.0
        self.state = self._prev_state
        self._last_plan = 0.0
        self._progress_pose = None
        return None

    def status(self, pose: Optional[tuple[float, float, float]] = None) -> dict:
        with self._lock:
            g = self.goal
            return {
                "state": self.state.value,
                "goal": None if g is None else {"x": g.x, "y": g.y, "theta": g.theta, "name": g.name},
                "path": [[round(p[0], 3), round(p[1], 3)] for p in self.path],
                "distance_remaining": None if self.distance_remaining is None else round(self.distance_remaining, 3),
                "message": self.message,
                "recoveries": self.recoveries,
                "exploring": self.exploring,
                "planner": self.algorithm,
                "controller": self.controller,
                "local_trajectory": self.dwa.last_trajectory if self.controller == "dwa" else [],
                "named_locations": self.named_locations,
            }
