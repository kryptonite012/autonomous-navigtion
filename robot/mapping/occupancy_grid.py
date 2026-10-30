"""Log-odds occupancy grid with ROS map_server compatible import/export."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np
import yaml

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[tuple[int, int]]:
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class OccupancyGrid:
    """Grid indexed [row, col] = [y, x]; row 0 is the *lowest* y (ROS convention)."""

    def __init__(self, width_m: float, height_m: float, resolution: float,
                 origin: tuple[float, float] = (0.0, 0.0), l_hit: float = 0.85, l_miss: float = -0.4,
                 l_min: float = -4.0, l_max: float = 4.0, occ_thresh: float = 0.65, free_thresh: float = 0.25):
        self.resolution = resolution
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.origin = (float(origin[0]), float(origin[1]))
        self.l_hit, self.l_miss, self.l_min, self.l_max = l_hit, l_miss, l_min, l_max
        self.occ_thresh, self.free_thresh = occ_thresh, free_thresh
        self.log_odds = np.zeros((self.height, self.width), np.float32)
        self.observed = np.zeros((self.height, self.width), bool)
        self.updates = 0

    @classmethod
    def from_config(cls, mc: dict) -> "OccupancyGrid":
        return cls(mc["width_m"], mc["height_m"], mc["resolution_m"], tuple(mc["origin"]),
                   mc["log_odds_hit"], mc["log_odds_miss"], mc["log_odds_min"], mc["log_odds_max"],
                   mc["occupied_threshold"], mc["free_threshold"])

    # ------------------------------------------------------------- coordinates
    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return (self.origin[0] + (cx + 0.5) * self.resolution,
                self.origin[1] + (cy + 0.5) * self.resolution)

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    # ----------------------------------------------------------------- updates
    def integrate_ray(self, ox: float, oy: float, angle: float, distance: Optional[float],
                      max_range: float, beam_halfwidth: float = 0.0) -> None:
        """Mark cells along the ray free and the end cell occupied.

        ``distance`` None means no return within ``max_range``: the whole
        ray is free. For wide-beam sensors (ultrasonic) several rays across
        the cone are cleared but only the centre end point marked occupied.
        """
        reach = max_range if distance is None else distance
        offsets = [0.0] if beam_halfwidth <= 0 else [-beam_halfwidth, 0.0, beam_halfwidth]
        c0 = self.world_to_cell(ox, oy)
        for off in offsets:
            a = angle + off
            # push a hit a quarter cell past the surface: a surface lying on a
            # cell boundary would otherwise be marked behind the wall for rays
            # travelling +x/+y but in front of it for -x/-y, shifting the map
            # by half a cell
            ext = 0.25 * self.resolution if distance is not None else 0.0
            ex, ey = ox + (reach + ext) * math.cos(a), oy + (reach + ext) * math.sin(a)
            c1 = self.world_to_cell(ex, ey)
            cells = list(bresenham(c0[0], c0[1], c1[0], c1[1]))
            hit = distance is not None and off == 0.0
            free_cells = cells[:-1] if hit else cells
            for cx, cy in free_cells:
                if self.in_bounds(cx, cy):
                    self.log_odds[cy, cx] = max(self.l_min, self.log_odds[cy, cx] + self.l_miss)
                    self.observed[cy, cx] = True
            if hit:
                cx, cy = c1
                if self.in_bounds(cx, cy):
                    self.log_odds[cy, cx] = min(self.l_max, self.log_odds[cy, cx] + self.l_hit)
                    self.observed[cy, cx] = True
        self.updates += 1

    def integrate_scan(self, pose: tuple[float, float, float], angle_min: float, angle_inc: float,
                       ranges: np.ndarray, range_max: float, step: int = 1) -> None:
        x, y, th = pose
        for i in range(0, len(ranges), step):
            r = float(ranges[i])
            d = r if math.isfinite(r) and r < range_max else None
            self.integrate_ray(x, y, th + angle_min + i * angle_inc, d, min(range_max, 4.0))

    def mark_obstacle(self, x: float, y: float) -> None:
        cx, cy = self.world_to_cell(x, y)
        if self.in_bounds(cx, cy):
            self.log_odds[cy, cx] = self.l_max
            self.observed[cy, cx] = True

    def clear_area(self, x: float, y: float, radius: float) -> None:
        """The robot's own footprint is free space."""
        cx, cy = self.world_to_cell(x, y)
        r = int(math.ceil(radius / self.resolution))
        y0, y1 = max(0, cy - r), min(self.height, cy + r + 1)
        x0, x1 = max(0, cx - r), min(self.width, cx + r + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        disk = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
        sub = self.log_odds[y0:y1, x0:x1]
        sub[disk] = np.minimum(sub[disk], self.l_min / 2)
        self.observed[y0:y1, x0:x1][disk] = True

    def reset(self) -> None:
        self.log_odds[:] = 0
        self.observed[:] = False
        self.updates = 0

    # ----------------------------------------------------------------- queries
    def probability(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def to_ros(self) -> np.ndarray:
        """int8 array with -1 unknown, 0..100 occupancy (nav_msgs/OccupancyGrid)."""
        p = self.probability()
        out = np.full(p.shape, UNKNOWN, np.int8)
        out[self.observed] = np.clip(np.round(p[self.observed] * 100), 0, 100).astype(np.int8)
        return out

    def ternary(self) -> np.ndarray:
        p = self.probability()
        out = np.full(p.shape, UNKNOWN, np.int8)
        out[self.observed & (p <= self.free_thresh)] = FREE
        out[self.observed & (p >= self.occ_thresh)] = OCCUPIED
        return out

    def stats(self) -> dict:
        t = self.ternary()
        total = t.size
        return {
            "width": self.width, "height": self.height, "resolution": self.resolution,
            "origin": list(self.origin),
            "free_ratio": round(float((t == FREE).sum()) / total, 4),
            "occupied_ratio": round(float((t == OCCUPIED).sum()) / total, 4),
            "unknown_ratio": round(float((t == UNKNOWN).sum()) / total, 4),
            "updates": self.updates,
        }

    def sample_field(self, field: np.ndarray, wx: np.ndarray, wy: np.ndarray, outside: float = 1.0) -> np.ndarray:
        """Bilinear lookup of a per-cell field (e.g. a distance transform) at
        continuous world coordinates; cell values live at cell centres."""
        fx = (np.asarray(wx) - self.origin[0]) / self.resolution - 0.5
        fy = (np.asarray(wy) - self.origin[1]) / self.resolution - 0.5
        x0 = np.floor(fx).astype(np.int64)
        y0 = np.floor(fy).astype(np.int64)
        tx, ty = fx - x0, fy - y0
        inside = (x0 >= 0) & (x0 < self.width - 1) & (y0 >= 0) & (y0 < self.height - 1)
        out = np.full(np.shape(fx), outside, np.float64)
        if inside.any():
            xi, yi = x0[inside], y0[inside]
            a, b = tx[inside], ty[inside]
            out[inside] = ((1 - a) * (1 - b) * field[yi, xi] + a * (1 - b) * field[yi, xi + 1]
                           + (1 - a) * b * field[yi + 1, xi] + a * b * field[yi + 1, xi + 1])
        return out

    def inflated_obstacles(self, radius_m: float, allow_unknown: bool = False) -> np.ndarray:
        """Boolean cost mask: True where the robot centre must not go."""
        t = self.ternary()
        blocked = (t == OCCUPIED) if allow_unknown else (t != FREE)
        occ = (t == OCCUPIED).astype(np.uint8)
        r = max(1, int(math.ceil(radius_m / self.resolution)))
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        inflated = cv2.dilate(occ, k).astype(bool)
        return blocked | inflated

    # --------------------------------------------------------------------- I/O
    def save(self, path_stem: str | Path, extra: Optional[dict] = None) -> tuple[Path, Path]:
        """Write ``<stem>.pgm`` + ``<stem>.yaml`` in map_server format."""
        stem = Path(path_stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        p = self.probability()
        img = np.full(p.shape, 205, np.uint8)               # unknown grey
        img[self.observed] = (255 * (1.0 - p[self.observed])).astype(np.uint8)
        # PGM rows go top-down: flip so +y is up in image viewers
        pgm = stem.with_suffix(".pgm")
        cv2.imwrite(str(pgm), np.flipud(img))
        meta = {
            "image": pgm.name,
            "mode": "trinary",
            "resolution": self.resolution,
            "origin": [self.origin[0], self.origin[1], 0.0],
            "negate": 0,
            "occupied_thresh": self.occ_thresh,
            "free_thresh": self.free_thresh,
        }
        if extra:
            meta["nexus"] = extra
        yml = stem.with_suffix(".yaml")
        with open(yml, "w", encoding="utf-8") as fh:
            yaml.safe_dump(meta, fh, sort_keys=False)
        return pgm, yml

    @classmethod
    def load(cls, yaml_path: str | Path) -> "OccupancyGrid":
        yaml_path = Path(yaml_path)
        with open(yaml_path, "r", encoding="utf-8") as fh:
            meta = yaml.safe_load(fh)
        img = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(meta["image"])
        img = np.flipud(img)
        res = float(meta["resolution"])
        g = cls(img.shape[1] * res, img.shape[0] * res, res, tuple(meta["origin"][:2]),
                occ_thresh=meta.get("occupied_thresh", 0.65), free_thresh=meta.get("free_thresh", 0.25))
        occ = (255 - img.astype(np.float32)) / 255.0
        if meta.get("negate"):
            occ = 1.0 - occ
        unknown = img == 205
        p = np.clip(occ, 0.02, 0.98)
        g.log_odds = np.log(p / (1 - p)).astype(np.float32)
        g.log_odds[unknown] = 0
        g.observed = ~unknown
        return g
