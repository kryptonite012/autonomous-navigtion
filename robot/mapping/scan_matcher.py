"""Scan-to-map matching for LiDAR mapping without ROS (correlative matching).

While the robot maps with a LiDAR, each new scan is aligned against the map
built so far before it is inserted. The search is brute force over a small
window around the odometry prediction (x, y, theta), scored with the same
likelihood field the particle filter uses, followed by a finer second pass
around the best candidate. This removes most odometry drift (Olson,
"Real-time correlative scan matching", ICRA 2009, without the multi-
resolution tables). There is no loop closure: long loops in large spaces
still accumulate some error, so the full SLAM Toolbox stack is the better
choice when ROS 2 is available.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from robot.mapping.occupancy_grid import OCCUPIED, OccupancyGrid


@dataclass
class MatchResult:
    pose: tuple[float, float, float]
    score: float              # mean per-beam likelihood in [0, 1]
    beams: int
    accepted: bool
    shift: float              # metres the pose was moved


class ScanMatcher:
    def __init__(self, window_xy: float = 0.12, window_th: float = 0.12, step_xy: float = 0.02,
                 step_th: float = 0.015, sigma: float = 0.05, min_score: float = 0.45,
                 min_beams: int = 30, max_beams: int = 90, min_map_cells: int = 150):
        self.window_xy = window_xy
        self.window_th = window_th
        self.step_xy = step_xy
        self.step_th = step_th
        self.sigma = sigma
        self.min_score = min_score
        self.min_beams = min_beams
        self.max_beams = max_beams
        self.min_map_cells = min_map_cells
        self.matches = 0
        self.accepted = 0
        self.last: Optional[MatchResult] = None
        self.last_ms = 0.0
        self._field: Optional[np.ndarray] = None
        self._field_updates = -1

    def _distance_field(self, grid: OccupancyGrid) -> Optional[np.ndarray]:
        if grid.updates == self._field_updates and self._field is not None:
            return self._field
        occ = grid.ternary() == OCCUPIED
        if occ.sum() < self.min_map_cells:
            return None
        self._field = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32) * grid.resolution
        self._field_updates = grid.updates
        return self._field

    @staticmethod
    def scan_points(angle_min: float, angle_inc: float, ranges: np.ndarray, range_max: float,
                    max_beams: int) -> np.ndarray:
        """Robot-frame (x, y) of valid returns, subsampled to ``max_beams``."""
        r = np.asarray(ranges, dtype=np.float64)
        ang = angle_min + angle_inc * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.1) & (r < range_max * 0.98)
        r, ang = r[ok], ang[ok]
        if len(r) > max_beams:
            idx = np.linspace(0, len(r) - 1, max_beams).astype(int)
            r, ang = r[idx], ang[idx]
        return np.stack([r * np.cos(ang), r * np.sin(ang)], axis=1)

    def _score(self, field: np.ndarray, grid: OccupancyGrid, pts: np.ndarray,
               xs: np.ndarray, ys: np.ndarray, ths: np.ndarray) -> np.ndarray:
        """Mean likelihood for every (x, y, th) combination -> array [th, x, y]."""
        out = np.empty((len(ths), len(xs), len(ys)))
        for i, th in enumerate(ths):
            c, s = math.cos(th), math.sin(th)
            rx = c * pts[:, 0] - s * pts[:, 1]
            ry = s * pts[:, 0] + c * pts[:, 1]
            wx = xs[:, None, None] + rx[None, None, :]          # [x, 1, beams]
            wy = ys[None, :, None] + ry[None, None, :]          # [1, y, beams]
            wx, wy = np.broadcast_arrays(wx, wy)
            d = grid.sample_field(field, wx, wy)
            out[i] = np.exp(-0.5 * (d / self.sigma) ** 2).mean(axis=2)
        return out

    def match(self, grid: OccupancyGrid, pts: np.ndarray, guess: tuple[float, float, float]) -> Optional[MatchResult]:
        if len(pts) < self.min_beams:
            return None
        field = self._distance_field(grid)
        if field is None:
            return None
        self.matches += 1
        t0 = time.perf_counter()
        gx, gy, gth = guess
        best = (gx, gy, gth)
        best_score = -1.0
        # coarse pass then a fine pass around the coarse optimum
        for wxy, sxy, wth, sth in ((self.window_xy, self.step_xy, self.window_th, self.step_th),
                                   (self.step_xy, self.step_xy / 4, self.step_th, self.step_th / 4)):
            xs = best[0] + np.arange(-wxy, wxy + 1e-9, sxy)
            ys = best[1] + np.arange(-wxy, wxy + 1e-9, sxy)
            ths = best[2] + np.arange(-wth, wth + 1e-9, sth)
            sc = self._score(field, grid, pts, xs, ys, ths)
            i, j, k = np.unravel_index(int(np.argmax(sc)), sc.shape)
            if sc[i, j, k] >= best_score:
                best_score = float(sc[i, j, k])
                best = (float(xs[j]), float(ys[k]), math.atan2(math.sin(ths[i]), math.cos(ths[i])))
        guess_score = float(self._score(field, grid, pts, np.array([gx]), np.array([gy]), np.array([gth]))[0, 0, 0])
        # accept only a clearly better, well-supported alignment
        accepted = best_score >= self.min_score and best_score > guess_score + 0.01
        res = MatchResult(best if accepted else guess, best_score, len(pts), accepted,
                          math.dist(best[:2], (gx, gy)) if accepted else 0.0)
        if accepted:
            self.accepted += 1
        self.last = res
        self.last_ms = (time.perf_counter() - t0) * 1000
        return res

    def status(self) -> dict:
        r = self.last
        return {"matches": self.matches, "accepted": self.accepted,
                "last_score": None if r is None else round(r.score, 3),
                "last_shift_m": None if r is None else round(r.shift, 3),
                "last_ms": round(self.last_ms, 1)}
