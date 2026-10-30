"""Pose estimation: odometry EKF + optional map-based particle filter.

The runtime always integrates odometry (the EKF) in the ``odom`` frame. When
a map is loaded and localisation is enabled, the particle filter estimates
the robot pose in the ``map`` frame and the localiser keeps the correction
``map <- odom`` (like the map->odom TF in ROS). Between filter updates the
pose follows odometry smoothly; each update only changes the correction.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Callable, Optional

from robot.hardware.base import LaserScan, RangeReading
from robot.localization.mcl import Beam, ParticleFilter
from robot.mapping.occupancy_grid import OccupancyGrid

log = logging.getLogger(__name__)


def compose(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    th = a[2] + b[2]
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], math.atan2(math.sin(th), math.cos(th)))


def inverse(a: tuple[float, float, float]) -> tuple[float, float, float]:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Localizer:
    def __init__(self, cfg: dict, has_lidar: bool, sensor_offset: float,
                 on_event: Optional[Callable[[str, str, str], None]] = None):
        lc = cfg.get("localization", {})
        self.mode = lc.get("mode", "auto")           # auto | on | off
        self.has_lidar = has_lidar
        self.sensor_offset = sensor_offset           # ultrasonic/ToF distance from the centre
        self.update_dist = lc.get("update_min_distance_m", 0.05)
        self.update_angle = lc.get("update_min_angle_rad", 0.1)
        self.converged_xy = lc.get("converged_std_m", 0.15)
        self.converged_th = lc.get("converged_std_rad", 0.25)
        self.lost_xy = lc.get("lost_std_m", 0.5)
        self.converged_likelihood = lc.get("converged_likelihood", 0.7)
        self.pf = ParticleFilter(n=lc.get("particles", 400), sigma_hit=lc.get("sigma_hit_m", 0.08),
                                 max_beams=lc.get("max_beams", 36), seed=lc.get("seed", 0))
        self.on_event = on_event or (lambda *a: None)
        self.correction = (0.0, 0.0, 0.0)            # map <- odom
        self.active = False
        self.map_name: Optional[str] = None
        self.std_xy = 0.0
        self.std_th = 0.0
        self.converged = False
        self.last_update = 0.0
        self.update_ms = 0.0
        self._last_odom: Optional[tuple[float, float, float]] = None
        self._pending_global = False
        self._lock = threading.RLock()

    @property
    def enabled(self) -> bool:
        if self.mode == "off":
            return False
        if self.mode == "on":
            return True
        return self.has_lidar      # auto: sparse range sensors are too weak by default

    @property
    def experimental(self) -> bool:
        return not self.has_lidar

    # ------------------------------------------------------------- lifecycle
    def set_map(self, grid: OccupancyGrid, name: Optional[str], current_map_pose: tuple[float, float, float]) -> None:
        with self._lock:
            self.pf.set_map(grid)
            self.map_name = name
            self.active = self.enabled
            if self.active:
                self.pf.init_pose(*current_map_pose)
                self.converged = False
                self.on_event("localization_started", "info",
                              f"map-based localisation on '{name}' ({'LiDAR' if self.has_lidar else 'range sensors, experimental'})")

    def deactivate(self) -> None:
        with self._lock:
            self.active = False

    def set_initial_pose(self, map_pose: tuple[float, float, float], odom_pose: tuple[float, float, float],
                         std_xy: float = 0.15, std_th: float = 0.2) -> None:
        with self._lock:
            self.correction = compose(map_pose, inverse(odom_pose))
            if self.active:
                self.pf.init_pose(*map_pose, std_xy=std_xy, std_th=std_th)
                self.converged = False
            self._last_odom = odom_pose

    def set_correction(self, map_pose: tuple[float, float, float], odom_pose: tuple[float, float, float]) -> None:
        """Used by LiDAR scan matching while mapping (the filter is inactive then)."""
        with self._lock:
            self.correction = compose(map_pose, inverse(odom_pose))

    def global_localization(self) -> None:
        with self._lock:
            if not self.active:
                raise RuntimeError("load a map first (and enable localisation)")
            if not self.has_lidar:
                raise RuntimeError("global localisation needs a LiDAR; set the initial pose instead")
            # the search itself runs on the next scan (update())
            self._pending_global = True
            self.converged = False
            self.pf._resize(self.pf.n_track * self.pf.global_factor)   # marks the pose as 'searching'
            self.on_event("localization_global", "info", "global localisation requested")

    # ---------------------------------------------------------------- update
    def map_pose(self, odom_pose: tuple[float, float, float]) -> tuple[float, float, float]:
        with self._lock:
            return compose(self.correction, odom_pose)

    def update(self, odom_pose: tuple[float, float, float], ranges: list[RangeReading],
               scan: Optional[LaserScan]) -> None:
        with self._lock:
            if not self.active:
                self._last_odom = odom_pose
                return
            if self._last_odom is None:
                self._last_odom = odom_pose
                return
            moved = math.dist(odom_pose[:2], self._last_odom[:2])
            turned = abs(math.atan2(math.sin(odom_pose[2] - self._last_odom[2]),
                                    math.cos(odom_pose[2] - self._last_odom[2])))
            # when not converged keep updating while standing still, otherwise
            # a lost robot that stops (as it must) would never recover
            idle_refresh = not self.converged and time.time() - self.last_update > 0.5
            if moved < self.update_dist and turned < self.update_angle and not self._pending_global \
                    and not idle_refresh:
                return
            t0 = time.perf_counter()
            beams = self._beams(ranges, scan)
            if self._pending_global:
                if scan is None and self.has_lidar:
                    return                          # wait for the next full scan
                self._pending_global = False
                found = self.pf.init_global_search(beams)
                self._last_odom = odom_pose
                self.on_event("localization_global", "info",
                              "global search done, tracking best candidates" if found
                              else "scan too sparse for a global search: particles spread uniformly")
                self.update_ms = (time.perf_counter() - t0) * 1000
                return
            self.pf.predict(self._last_odom, odom_pose)
            self._last_odom = odom_pose
            self.pf.update(beams)
            est, self.std_xy, self.std_th = self.pf.estimate()
            was = self.converged
            # a tight cloud is not enough: the scan must also match the map well,
            # otherwise the filter may have collapsed onto a look-alike place
            self.converged = (self.std_xy < self.converged_xy and self.std_th < self.converged_th
                              and self.pf.last_likelihood >= self.converged_likelihood)
            if self.converged:
                self.pf.shrink()
                # only move the map frame when the filter is confident
                self.correction = compose(est, inverse(odom_pose))
            if was and not self.converged and self.std_xy > self.lost_xy:
                self.on_event("localization_lost", "warning",
                              f"particle filter diverged (std {self.std_xy:.2f} m)")
            elif not was and self.converged:
                self.on_event("localization_converged", "info", f"localised (std {self.std_xy:.2f} m)")
            self.last_update = time.time()
            self.update_ms = (time.perf_counter() - t0) * 1000

    def _beams(self, ranges: list[RangeReading], scan: Optional[LaserScan]) -> list[Beam]:
        beams: list[Beam] = []
        if scan is not None:
            for i in range(0, len(scan.ranges), max(1, len(scan.ranges) // 72)):
                r = float(scan.ranges[i])
                beams.append(Beam(scan.angle_min + i * scan.angle_increment,
                                  r if math.isfinite(r) else None, scan.range_max))
        for r in ranges:
            if r.valid:
                beams.append(Beam(r.angle_rad, r.distance_m, r.max_range_m,
                                  (self.sensor_offset * math.cos(r.angle_rad),
                                   self.sensor_offset * math.sin(r.angle_rad))))
        return beams

    def pose_ok(self) -> bool:
        """False when map localisation is running but has lost track, or is
        still searching after a global localisation request."""
        with self._lock:
            if not self.active:
                return True
            if self.pf.n > self.pf.n_track:        # global search not finished
                return False
            return self.std_xy < self.lost_xy

    def status(self, with_particles: bool = False) -> dict:
        with self._lock:
            d = {
                "enabled": self.enabled,
                "mode": self.mode,
                "active": self.active,
                "method": "mcl" if self.active else "odometry",
                "experimental": self.active and self.experimental,
                "map": self.map_name,
                "converged": self.converged if self.active else None,
                "std_xy": round(self.std_xy, 3) if self.active else None,
                "std_theta": round(self.std_th, 3) if self.active else None,
                "particles": self.pf.n,
                "searching": self.active and self.pf.n > self.pf.n_track,
                "neff": round(self.pf.neff(), 1),
                "likelihood": round(self.pf.last_likelihood, 3),
                "update_ms": round(self.update_ms, 2),
                "correction": [round(v, 3) for v in self.correction],
            }
            if with_particles and self.active:
                d["particle_sample"] = self.pf.sample()
            return d
