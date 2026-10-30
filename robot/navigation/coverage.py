"""Coverage (boustrophedon / lawnmower) waypoints over the free space of a map.

Used to generate patrol or search missions that sweep a room systematically,
e.g. to find every object for sorting. Rows run along x, ``spacing`` apart;
each row is split into the free intervals between obstacles (after inflating
them by the robot radius) and visited in alternating direction.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from robot.mapping.occupancy_grid import OccupancyGrid


def coverage_waypoints(grid: OccupancyGrid, spacing: float, inflation: float, min_segment: float = 0.25,
                       inset: float = 0.05, bounds: Optional[tuple[float, float, float, float]] = None,
                       max_points: int = 100) -> list[tuple[float, float]]:
    if spacing < 0.1:
        raise ValueError("spacing must be at least 0.1 m")
    blocked = grid.inflated_obstacles(inflation, allow_unknown=False)
    free = ~blocked
    ys, xs = np.nonzero(free)
    if len(xs) == 0:
        raise ValueError("the map has no known free space")
    res = grid.resolution
    if bounds is None:
        x0, y0 = grid.cell_to_world(int(xs.min()), int(ys.min()))
        x1, y1 = grid.cell_to_world(int(xs.max()), int(ys.max()))
    else:
        x0, y0, x1, y1 = bounds
    pts: list[tuple[float, float]] = []
    row = 0
    y = y0 + spacing / 2
    while y <= y1:
        _, cy = grid.world_to_cell(x0, y)
        if 0 <= cy < grid.height:
            cx0 = max(0, grid.world_to_cell(x0, y)[0])
            cx1 = min(grid.width - 1, grid.world_to_cell(x1, y)[0])
            line = free[cy, cx0:cx1 + 1]
            segs = []
            start = None
            for i, f in enumerate(np.append(line, False)):
                if f and start is None:
                    start = i
                elif not f and start is not None:
                    a, b = cx0 + start, cx0 + i - 1
                    if (b - a + 1) * res >= min_segment:
                        segs.append((grid.cell_to_world(a, cy)[0] + inset, grid.cell_to_world(b, cy)[0] - inset))
                    start = None
            if row % 2 == 1:
                segs = [(b, a) for a, b in reversed(segs)]
            for a, b in segs:
                wy = grid.cell_to_world(0, cy)[1]
                pts.append((round(a, 3), round(wy, 3)))
                pts.append((round(b, 3), round(wy, 3)))
            if segs:
                row += 1
        y += spacing
    if not pts:
        raise ValueError("no free row segments long enough: reduce spacing or min_segment")
    if len(pts) > max_points:
        raise ValueError(f"{len(pts)} waypoints exceed the limit of {max_points}: increase spacing")
    return pts
