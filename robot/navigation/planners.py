"""Grid path planners: A* and Dijkstra (8-connected), plus path simplification."""

from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np

Cell = tuple[int, int]   # (cx, cy)

_NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def _octile(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def plan(blocked: np.ndarray, start: Cell, goal: Cell, algorithm: str = "astar",
         cost: Optional[np.ndarray] = None, max_expansions: int = 400_000) -> Optional[list[Cell]]:
    """Shortest path on a boolean obstacle grid ``blocked[cy, cx]``.

    ``cost`` is an optional extra per-cell traversal cost (e.g. distance to
    walls) so paths keep away from obstacles where there is room.
    Dijkstra is A* with a zero heuristic.
    Returns the list of cells from start to goal, or None.
    """
    h, w = blocked.shape
    for cx, cy in (start, goal):
        if not (0 <= cx < w and 0 <= cy < h):
            return None
    if blocked[goal[1], goal[0]]:
        return None
    use_h = algorithm == "astar"
    g = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    openq = [(_octile(start, goal) if use_h else 0.0, 0.0, start)]
    closed: set[Cell] = set()
    expansions = 0
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        expansions += 1
        if expansions > max_expansions:
            return None
        for dx, dy, step in _NEIGHBOURS:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            # no corner cutting between two blocked cells
            if dx and dy and (blocked[cur[1], nx] or blocked[ny, cur[0]]):
                continue
            nc = gc + step + (float(cost[ny, nx]) if cost is not None else 0.0)
            nb = (nx, ny)
            if nc < g.get(nb, math.inf):
                g[nb] = nc
                parent[nb] = cur
                heapq.heappush(openq, (nc + (_octile(nb, goal) if use_h else 0.0), nc, nb))
    return None


def line_free(blocked: np.ndarray, a: Cell, b: Cell) -> bool:
    from robot.mapping.occupancy_grid import bresenham
    for cx, cy in bresenham(a[0], a[1], b[0], b[1]):
        if blocked[cy, cx]:
            return False
    return True


def simplify(blocked: np.ndarray, path: list[Cell]) -> list[Cell]:
    """Drop intermediate cells that have direct line of sight (string pulling)."""
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def nearest_free(blocked: np.ndarray, cell: Cell, max_radius: int = 10) -> Optional[Cell]:
    """Closest unblocked cell, used when the robot's own cell is inside inflation."""
    h, w = blocked.shape
    if 0 <= cell[0] < w and 0 <= cell[1] < h and not blocked[cell[1], cell[0]]:
        return cell
    for r in range(1, max_radius + 1):
        best = None
        for dx in range(-r, r + 1):
            for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                nx, ny = cell[0] + dx, cell[1] + dy
                if 0 <= nx < w and 0 <= ny < h and not blocked[ny, nx]:
                    d = dx * dx + dy * dy
                    if best is None or d < best[0]:
                        best = (d, (nx, ny))
        if best:
            return best[1]
    return None


def clearance_cost(blocked: np.ndarray, weight: float = 2.0, falloff_cells: int = 6) -> np.ndarray:
    """Extra cost near obstacles, decaying with distance."""
    import cv2
    free = (~blocked).astype(np.uint8)
    dist = cv2.distanceTransform(free, cv2.DIST_L2, 3)
    return (weight * np.clip(1.0 - dist / falloff_cells, 0, 1)).astype(np.float32)
