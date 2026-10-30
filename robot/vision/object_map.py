"""Places detected objects on the map.

A detection gives range and bearing only when the camera is calibrated and
the object rests on the floor (see ``ColorDetector._estimate_range``).
Combined with the robot pose this gives a world position. Repeated sightings
of the same colour within ``assoc_radius`` are merged (running mean), so
each physical object becomes one map entry with an observation count.
"""

from __future__ import annotations

import itertools
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from robot.vision.detector import Detection


@dataclass
class MappedObject:
    id: int
    color: str
    x: float
    y: float
    observations: int
    first_seen: float
    last_seen: float
    sorted: bool = False

    def as_dict(self) -> dict:
        return {"id": self.id, "color": self.color, "x": round(self.x, 3), "y": round(self.y, 3),
                "observations": self.observations, "first_seen": self.first_seen,
                "last_seen": self.last_seen, "sorted": self.sorted}


class ObjectMap:
    def __init__(self, camera_forward_m: float = 0.0, assoc_radius: float = 0.2, max_range: float = 2.0,
                 min_observations: int = 3):
        self.cam_fwd = camera_forward_m
        self.assoc = assoc_radius
        self.max_range = max_range
        self.min_obs = min_observations
        self.objects: dict[int, MappedObject] = {}
        self._ids = itertools.count(1)
        self._lock = threading.Lock()

    def update(self, dets: list[Detection], pose: Optional[tuple[float, float, float]]) -> int:
        if pose is None:
            return 0
        x, y, th = pose
        cx, cy = x + self.cam_fwd * math.cos(th), y + self.cam_fwd * math.sin(th)
        added = 0
        now = time.time()
        with self._lock:
            for d in dets:
                if d.distance_m is None or d.bearing_rad is None or d.distance_m > self.max_range:
                    continue
                a = th + d.bearing_rad
                ox, oy = cx + d.distance_m * math.cos(a), cy + d.distance_m * math.sin(a)
                best = None
                for o in self.objects.values():
                    if o.color == d.color and not o.sorted:
                        dist = math.hypot(o.x - ox, o.y - oy)
                        if dist < self.assoc and (best is None or dist < best[0]):
                            best = (dist, o)
                if best:
                    o = best[1]
                    k = min(o.observations, 20)       # keep adapting a little
                    o.x = (o.x * k + ox) / (k + 1)
                    o.y = (o.y * k + oy) / (k + 1)
                    o.observations += 1
                    o.last_seen = now
                else:
                    oid = next(self._ids)
                    self.objects[oid] = MappedObject(oid, d.color, ox, oy, 1, now, now)
                    added += 1
        return added

    def mark_sorted_near(self, color: str, x: float, y: float, radius: float = 0.45) -> Optional[int]:
        """After a verified pick, retire the nearest mapped object of that colour."""
        with self._lock:
            best = None
            for o in self.objects.values():
                if o.color == color and not o.sorted:
                    d = math.hypot(o.x - x, o.y - y)
                    if d < radius and (best is None or d < best[0]):
                        best = (d, o)
            if best:
                best[1].sorted = True
                return best[1].id
        return None

    def candidates(self, exclude_colors: set[str] = frozenset()) -> list[MappedObject]:
        with self._lock:
            return [o for o in self.objects.values()
                    if not o.sorted and o.observations >= self.min_obs and o.color not in exclude_colors]

    def clear(self) -> None:
        with self._lock:
            self.objects.clear()

    def list(self) -> list[dict]:
        with self._lock:
            return [o.as_dict() for o in self.objects.values() if o.observations >= self.min_obs]
