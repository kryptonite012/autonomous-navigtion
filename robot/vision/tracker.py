"""Centroid tracker that assigns persistent IDs to detections across frames."""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from robot.vision.detector import Detection


@dataclass
class Track:
    id: int
    color: str
    centroid: tuple[float, float]
    hits: int = 1
    missed: int = 0
    last: Detection | None = None


class CentroidTracker:
    def __init__(self, max_distance_px: float = 60.0, max_missed: int = 8, min_hits: int = 2):
        self.max_distance = max_distance_px
        self.max_missed = max_missed
        self.min_hits = min_hits
        self.tracks: dict[int, Track] = {}
        self._ids = itertools.count(1)

    def reset(self) -> None:
        self.tracks.clear()

    def update(self, detections: list[Detection]) -> list[Detection]:
        """Match detections to tracks (greedy nearest neighbour, same colour).

        Returns the detections of confirmed tracks (seen ``min_hits`` times)
        with ``object_id`` set. Unconfirmed one-frame blips are dropped, which
        removes most flicker false positives.
        """
        pairs = []
        for ti, t in self.tracks.items():
            for di, d in enumerate(detections):
                if d.color != t.color:
                    continue
                dist = math.dist(t.centroid, d.centroid)
                if dist <= self.max_distance:
                    pairs.append((dist, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = self.tracks[ti]
            d = detections[di]
            t.centroid, t.hits, t.missed, t.last = d.centroid, t.hits + 1, 0, d
            d.object_id = t.id
        for ti, t in list(self.tracks.items()):
            if ti not in used_t:
                t.missed += 1
                if t.missed > self.max_missed:
                    del self.tracks[ti]
        for di, d in enumerate(detections):
            if di not in used_d:
                tid = next(self._ids)
                self.tracks[tid] = Track(tid, d.color, d.centroid, last=d)
                d.object_id = tid
        return [d for d in detections if self.tracks[d.object_id].hits >= self.min_hits]
