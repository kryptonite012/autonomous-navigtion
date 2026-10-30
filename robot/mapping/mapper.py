"""Mapping service: integrates range data into the occupancy grid and manages map files."""

from __future__ import annotations

import logging
import math
import re
import shutil
import threading
import time
from pathlib import Path
from typing import Optional

from robot.config import resolve_path
from robot.hardware.base import LaserScan, RangeReading
from robot.mapping.occupancy_grid import OccupancyGrid

log = logging.getLogger(__name__)

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_\-]{0,63}$")


class MapStore:
    """Map files on disk: ``<maps_dir>/<name>.yaml`` + ``.pgm``."""

    def __init__(self, maps_dir: str):
        self.dir = resolve_path(maps_dir)
        self.dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def check_name(name: str) -> str:
        if not NAME_RE.match(name or ""):
            raise ValueError("map name must be 1-64 chars: letters, digits, '_' or '-'")
        return name

    def path(self, name: str) -> Path:
        return self.dir / f"{self.check_name(name)}.yaml"

    def list(self) -> list[dict]:
        out = []
        for y in sorted(self.dir.glob("*.yaml")):
            st = y.stat()
            out.append({"name": y.stem, "file_path": str(y), "modified": st.st_mtime})
        return out

    def save(self, grid: OccupancyGrid, name: str, extra: Optional[dict] = None) -> Path:
        _, yml = grid.save(self.dir / self.check_name(name), extra)
        return yml

    def load(self, name: str) -> OccupancyGrid:
        p = self.path(name)
        if not p.exists():
            raise FileNotFoundError(f"map {name!r} not found")
        return OccupancyGrid.load(p)

    def rename(self, old: str, new: str) -> Path:
        src, dst = self.path(old), self.path(new)
        if not src.exists():
            raise FileNotFoundError(f"map {old!r} not found")
        if dst.exists():
            raise FileExistsError(f"map {new!r} already exists")
        import yaml
        meta = yaml.safe_load(src.read_text())
        pgm_src = self.dir / meta["image"]
        pgm_dst = self.dir / f"{new}.pgm"
        shutil.move(pgm_src, pgm_dst)
        meta["image"] = pgm_dst.name
        dst.write_text(yaml.safe_dump(meta, sort_keys=False))
        src.unlink()
        return dst

    def delete(self, name: str) -> None:
        p = self.path(name)
        if not p.exists():
            raise FileNotFoundError(f"map {name!r} not found")
        import yaml
        meta = yaml.safe_load(p.read_text())
        (self.dir / meta["image"]).unlink(missing_ok=True)
        p.unlink()


class Mapper:
    def __init__(self, cfg: dict, has_lidar: bool, has_ranges: bool, robot_radius: float):
        self.cfg = cfg["mapping"]
        self.grid = OccupancyGrid.from_config(self.cfg)
        self.store = MapStore(self.cfg["maps_dir"])
        self.has_lidar = has_lidar
        self.has_ranges = has_ranges
        self.robot_radius = robot_radius
        self.active = False
        self.loaded_name: Optional[str] = None
        self.started_at: Optional[float] = None
        self.trajectory: list[tuple[float, float]] = []
        self._lock = threading.Lock()

    @property
    def available(self) -> bool:
        return self.has_lidar or self.has_ranges

    @property
    def experimental(self) -> bool:
        """Without LiDAR, maps come from 3-5 sparse range sensors + odometry.

        That produces a rough outline at best: odometry drift is not
        corrected (no loop closure) and the ultrasonic beam is ~15 deg wide.
        """
        return not self.has_lidar

    def start(self, reset: bool = False) -> None:
        if not self.available:
            raise RuntimeError("no range sensors available for mapping")
        with self._lock:
            if reset:
                self.grid.reset()
                self.trajectory.clear()
                self.loaded_name = None
            self.active = True
            self.started_at = time.time()

    def stop(self) -> None:
        self.active = False

    def reset(self) -> None:
        with self._lock:
            self.grid.reset()
            self.trajectory.clear()
            self.loaded_name = None

    def update(self, pose: tuple[float, float, float], ranges: list[RangeReading],
               scan: Optional[LaserScan] = None) -> None:
        if not self.active:
            return
        x, y, th = pose
        with self._lock:
            if not self.trajectory or math.dist(self.trajectory[-1], (x, y)) > 0.05:
                self.trajectory.append((round(x, 3), round(y, 3)))
                if len(self.trajectory) > 5000:
                    self.trajectory = self.trajectory[-5000:]
            self.grid.clear_area(x, y, self.robot_radius * 0.8)
            if scan is not None:
                self.grid.integrate_scan(pose, scan.angle_min, scan.angle_increment, scan.ranges,
                                         scan.range_max, step=2)
            max_r = self.cfg.get("max_range_m", 2.0)
            for r in ranges:
                if not r.valid:
                    continue
                # trust only the near part of sparse sensor rays
                d = r.distance_m if r.distance_m is not None and r.distance_m <= max_r else None
                half = math.radians(7.5) if "us_" in r.sensor else 0.0
                self.grid.integrate_ray(x, y, th + r.angle_rad, d, min(max_r, r.max_range_m), half)

    def save(self, name: str) -> Path:
        with self._lock:
            path = self.store.save(self.grid, name, {
                "experimental": self.experimental,
                "source": "lidar" if self.has_lidar else "range_sensors",
                "saved_at": time.time(),
            })
        self.loaded_name = name
        return path

    def load(self, name: str) -> None:
        g = self.store.load(name)
        with self._lock:
            self.grid = g
            self.loaded_name = name
            self.trajectory.clear()

    def snapshot(self) -> dict:
        with self._lock:
            t = self.grid.ternary()
            return {
                "width": self.grid.width,
                "height": self.grid.height,
                "resolution": self.grid.resolution,
                "origin": list(self.grid.origin),
                # run-length encoding keeps the websocket payload small
                "data_rle": _rle(t.ravel().tolist()),
                "trajectory": list(self.trajectory[-500:]),
            }

    def status(self) -> dict:
        s = self.grid.stats()
        s.update({"active": self.active, "available": self.available, "experimental": self.experimental,
                  "loaded_map": self.loaded_name, "trajectory_points": len(self.trajectory),
                  "source": "lidar" if self.has_lidar else ("range_sensors" if self.has_ranges else None)})
        return s


def _rle(values: list[int]) -> list[list[int]]:
    out: list[list[int]] = []
    for v in values:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return out
