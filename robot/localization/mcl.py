"""Monte Carlo localisation (particle filter) against a saved occupancy grid.

* Motion model: sample-based odometry model (rot1, trans, rot2) with noise
  proportional to the motion (Thrun et al., Probabilistic Robotics, 5.4).
* Sensor model: likelihood field. A distance transform of the occupied
  cells gives, for any point, the distance to the nearest obstacle; a beam
  end point that falls near an obstacle is likely. Fully vectorised over
  particles x beams, so 400 particles x 36 beams costs well under 10 ms.
* Resampling: low-variance resampling when the effective sample size drops
  below half the particle count; a few random particles are injected when
  the average likelihood collapses (recovery from a kidnap).

With a LiDAR this localises reliably. With only 3-4 ultrasonic/ToF beams
it can only slow drift down when the initial pose is known (tracking); it
cannot localise globally. ``Localizer`` (localizer.py) labels that mode
experimental.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import cv2
import numpy as np

from robot.mapping.occupancy_grid import FREE, OCCUPIED, OccupancyGrid


@dataclass
class Beam:
    angle: float                  # robot frame, rad
    distance: Optional[float]     # metres, None = no return
    max_range: float
    origin: tuple[float, float] = (0.0, 0.0)   # sensor position in robot frame


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


class ParticleFilter:
    def __init__(self, n: int = 400, alphas: Sequence[float] = (0.08, 0.15, 0.08, 0.02),
                 sigma_hit: float = 0.08, z_hit: float = 0.85, z_rand: float = 0.15,
                 max_beams: int = 36, seed: int = 0, min_likelihood: float = 0.55,
                 global_factor: int = 10):
        self.n = n
        self.n_track = n                  # particle count while tracking
        self.global_factor = global_factor
        self.min_likelihood = min_likelihood
        self.a1, self.a2, self.a3, self.a4 = alphas
        self.sigma_hit = sigma_hit
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.max_beams = max_beams
        self.rng = np.random.default_rng(seed)
        self.particles = np.zeros((n, 3))
        self.weights = np.full(n, 1.0 / n)
        self.grid: Optional[OccupancyGrid] = None
        self.dist_field: Optional[np.ndarray] = None
        self.free_cells: Optional[np.ndarray] = None
        self.last_likelihood = 0.0
        self._pending_inject = 0
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.updates = 0

    # ------------------------------------------------------------------- map
    def set_map(self, grid: OccupancyGrid) -> None:
        t = grid.ternary()
        not_occ = (t != OCCUPIED).astype(np.uint8)
        # distance (cells) from every cell to the nearest occupied cell
        self.dist_field = cv2.distanceTransform(not_occ, cv2.DIST_L2, 5).astype(np.float32) * grid.resolution
        self.grid = grid
        ys, xs = np.nonzero(t == FREE)
        self.free_cells = np.stack([xs, ys], axis=1) if len(xs) else None

    # ------------------------------------------------------------ initialise
    def _resize(self, n: int) -> None:
        self.n = n
        self.particles = np.zeros((n, 3))
        self.weights = np.full(n, 1.0 / n)

    def init_pose(self, x: float, y: float, theta: float, std_xy: float = 0.1, std_th: float = 0.15) -> None:
        if self.n != self.n_track:
            self._resize(self.n_track)
        self.particles[:, 0] = self.rng.normal(x, std_xy, self.n)
        self.particles[:, 1] = self.rng.normal(y, std_xy, self.n)
        self.particles[:, 2] = _wrap(self.rng.normal(theta, std_th, self.n))
        self.weights[:] = 1.0 / self.n
        self.w_slow = self.w_fast = 0.0

    def init_global(self, resize: bool = True) -> None:
        """Spread particles uniformly over the known free space.

        Uses ``global_factor`` times more particles until the filter has
        converged (see ``shrink``); with too few particles the cloud
        collapses onto a look-alike place.
        """
        if self.free_cells is None or self.grid is None:
            raise RuntimeError("no free space in the map")
        if resize:
            self._resize(self.n_track * self.global_factor)
        idx = self.rng.integers(0, len(self.free_cells), self.n)
        cells = self.free_cells[idx]
        g = self.grid
        self.particles[:, 0] = g.origin[0] + (cells[:, 0] + self.rng.random(self.n)) * g.resolution
        self.particles[:, 1] = g.origin[1] + (cells[:, 1] + self.rng.random(self.n)) * g.resolution
        self.particles[:, 2] = self.rng.uniform(-np.pi, np.pi, self.n)
        self.weights[:] = 1.0 / self.n
        self.w_slow = self.w_fast = 0.0

    def init_global_search(self, beams: list["Beam"], top_k: int = 12, step_m: float = 0.1,
                           headings: int = 36) -> bool:
        """Global localisation by correlative search, then particles.

        Scores every free position (``step_m`` grid) x ``headings`` against
        one scan with the likelihood field and seeds the filter around the
        ``top_k`` best poses. Far more reliable than uniform random particles,
        which can collapse onto a look-alike place. Returns False if the scan
        has too few returns (then falls back to ``init_global``).
        """
        hits = [b for b in beams if b.distance is not None and b.distance < b.max_range * 0.98]
        if len(hits) < 8 or self.free_cells is None or self.grid is None or self.dist_field is None:
            self.init_global()
            return False
        if len(hits) > self.max_beams:
            step = len(hits) / self.max_beams
            hits = [hits[int(i * step)] for i in range(self.max_beams)]
        g = self.grid
        stride = max(1, int(round(step_m / g.resolution)))
        cells = self.free_cells[(self.free_cells[:, 0] % stride == 0) & (self.free_cells[:, 1] % stride == 0)]
        px = g.origin[0] + (cells[:, 0] + 0.5) * g.resolution
        py = g.origin[1] + (cells[:, 1] + 0.5) * g.resolution
        ang = np.array([b.angle for b in hits])
        dist = np.array([b.distance for b in hits])
        ox = np.array([b.origin[0] for b in hits])
        oy = np.array([b.origin[1] for b in hits])
        best_scores, best_poses = [], []
        for h in np.linspace(-np.pi, np.pi, headings, endpoint=False):
            c, s_ = math.cos(h), math.sin(h)
            dx = c * ox - s_ * oy + dist * np.cos(h + ang)
            dy = s_ * ox + c * oy + dist * np.sin(h + ang)
            d = g.sample_field(self.dist_field, px[:, None] + dx, py[:, None] + dy)
            score = np.log(self.z_hit * np.exp(-0.5 * (d / self.sigma_hit) ** 2) + self.z_rand).sum(axis=1)
            k = min(top_k, len(score))
            idx = np.argpartition(-score, k - 1)[:k]
            best_scores.extend(score[idx].tolist())
            best_poses.extend([(px[i], py[i], h) for i in idx])
        order = np.argsort(best_scores)[::-1][:top_k]
        seeds = [best_poses[i] for i in order]
        self._resize(self.n_track * self.global_factor)
        per = self.n // len(seeds)
        for j, (sx, sy, sth) in enumerate(seeds):
            sl = slice(j * per, self.n if j == len(seeds) - 1 else (j + 1) * per)
            m = sl.stop - sl.start
            self.particles[sl, 0] = self.rng.normal(sx, step_m / 2, m)
            self.particles[sl, 1] = self.rng.normal(sy, step_m / 2, m)
            self.particles[sl, 2] = _wrap(self.rng.normal(sth, np.pi / headings, m))
        self.weights[:] = 1.0 / self.n
        self.w_slow = self.w_fast = 0.0
        return True

    # ---------------------------------------------------------------- motion
    def predict(self, prev: tuple[float, float, float], cur: tuple[float, float, float]) -> None:
        """Apply the odometry motion between two odometry poses."""
        dx, dy = cur[0] - prev[0], cur[1] - prev[1]
        trans = math.hypot(dx, dy)
        rot1 = math.atan2(dy, dx) - prev[2] if trans > 1e-4 else 0.0
        rot1 = math.atan2(math.sin(rot1), math.cos(rot1))
        # driving backwards: treat as negative translation, not a 180 deg turn
        if abs(rot1) > math.pi / 2:
            rot1 = math.atan2(math.sin(rot1 + math.pi), math.cos(rot1 + math.pi))
            trans = -trans
        rot2 = math.atan2(math.sin(cur[2] - prev[2] - rot1), math.cos(cur[2] - prev[2] - rot1))
        n = self.n
        s_rot1 = self.a1 * abs(rot1) + self.a2 * abs(trans)
        s_trans = self.a3 * abs(trans) + self.a4 * (abs(rot1) + abs(rot2))
        s_rot2 = self.a1 * abs(rot2) + self.a2 * abs(trans)
        r1 = rot1 + self.rng.normal(0, s_rot1 + 1e-6, n)
        tr = trans + self.rng.normal(0, s_trans + 1e-6, n)
        r2 = rot2 + self.rng.normal(0, s_rot2 + 1e-6, n)
        p = self.particles
        p[:, 0] += tr * np.cos(p[:, 2] + r1)
        p[:, 1] += tr * np.sin(p[:, 2] + r1)
        p[:, 2] = _wrap(p[:, 2] + r1 + r2)

    # ---------------------------------------------------------------- sensor
    def update(self, beams: list[Beam]) -> bool:
        """Weight particles by the beams. Returns False if there was nothing to use."""
        if self.dist_field is None or self.grid is None:
            return False
        hits = [b for b in beams if b.distance is not None and b.distance < b.max_range * 0.98]
        if len(hits) < 2:
            return False
        if len(hits) > self.max_beams:
            step = len(hits) / self.max_beams
            hits = [hits[int(i * step)] for i in range(self.max_beams)]
        if self._pending_inject and self.free_cells is not None:
            # random particles are inserted *before* weighting so this scan
            # immediately rates them; inserted after resampling they would
            # count as equally likely and inflate the spread estimate
            k, self._pending_inject = self._pending_inject, 0
            keep = self.particles.copy()
            keep_w = self.weights.copy()
            self.init_global(resize=False)
            self.particles[k:] = keep[k:]
            self.weights = keep_w
            self.weights[:k] = 1.0 / self.n
            self.weights /= self.weights.sum()
        ang = np.array([b.angle for b in hits])
        dist = np.array([b.distance for b in hits])
        ox = np.array([b.origin[0] for b in hits])
        oy = np.array([b.origin[1] for b in hits])
        p = self.particles
        c, s = np.cos(p[:, 2:3]), np.sin(p[:, 2:3])
        # sensor origin in the world, then beam end point
        sx = p[:, 0:1] + c * ox - s * oy
        sy = p[:, 1:2] + s * ox + c * oy
        ex = sx + dist * np.cos(p[:, 2:3] + ang)
        ey = sy + dist * np.sin(p[:, 2:3] + ang)
        d = self.grid.sample_field(self.dist_field, ex, ey)
        prob = self.z_hit * np.exp(-0.5 * (d / self.sigma_hit) ** 2) + self.z_rand
        log_w = np.log(prob).sum(axis=1)
        per_particle = np.exp(log_w / len(hits))         # mean per-beam likelihood of each particle
        log_w += np.log(np.maximum(self.weights, 1e-300))
        log_w -= log_w.max()
        w = np.exp(log_w)
        self.weights = w / w.sum()
        # how well the filter's belief explains the scan (weighted, so a few
        # stray particles don't make a good fit look poor); detects kidnapping
        self.last_likelihood = float(np.sum(self.weights * per_particle))
        # long/short term averages (augmented MCL)
        self.w_slow += 0.02 * (self.last_likelihood - self.w_slow) if self.w_slow else self.last_likelihood
        self.w_fast += 0.3 * (self.last_likelihood - self.w_fast) if self.w_fast else self.last_likelihood
        poor = self.last_likelihood < self.min_likelihood
        if self.neff() < self.n / 2 or poor:
            self.resample(extra_random=0.05 if poor else 0.0)
        self.updates += 1
        return True

    def neff(self) -> float:
        return float(1.0 / np.sum(self.weights ** 2))

    def shrink(self) -> None:
        """Back to the tracking particle count once global localisation converged."""
        if self.n > self.n_track:
            positions = (self.rng.random() + np.arange(self.n_track)) / self.n_track
            idx = np.minimum(np.searchsorted(np.cumsum(self.weights), positions), self.n - 1)
            self.particles = self.particles[idx].copy()
            self.n = self.n_track
            self.weights = np.full(self.n, 1.0 / self.n)

    def resample(self, extra_random: float = 0.0) -> None:
        n = self.n
        positions = (self.rng.random() + np.arange(n)) / n
        idx = np.searchsorted(np.cumsum(self.weights), positions)
        idx = np.minimum(idx, n - 1)
        self.particles = self.particles[idx].copy()
        self.weights[:] = 1.0 / n
        # inject random particles if the fast average fell well below the slow one
        p_random = max(0.0, 1.0 - self.w_fast / self.w_slow) if self.w_slow > 0 else 0.0
        self._pending_inject = int(min(0.25, max(p_random, extra_random)) * n)

    # -------------------------------------------------------------- estimate
    def estimate(self) -> tuple[tuple[float, float, float], float, float]:
        """Weighted mean pose, position std (m), heading std (rad)."""
        w = self.weights
        p = self.particles
        mx = float(np.sum(w * p[:, 0]))
        my = float(np.sum(w * p[:, 1]))
        sx, sy = float(np.sum(w * np.sin(p[:, 2]))), float(np.sum(w * np.cos(p[:, 2])))
        th = math.atan2(sx, sy)
        std_xy = math.sqrt(float(np.sum(w * ((p[:, 0] - mx) ** 2 + (p[:, 1] - my) ** 2))))
        r = min(1.0, math.hypot(sx, sy))
        std_th = math.sqrt(-2.0 * math.log(r)) if r > 1e-9 else math.pi
        return (mx, my, th), std_xy, std_th

    def sample(self, k: int = 150) -> list[list[float]]:
        idx = self.rng.choice(self.n, size=min(k, self.n), replace=False)
        return [[round(float(a), 3), round(float(b), 3), round(float(c), 2)] for a, b, c in self.particles[idx]]
