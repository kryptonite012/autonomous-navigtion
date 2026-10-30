import math

import numpy as np
import pytest

from robot.config import load_config
from robot.mapping.occupancy_grid import OccupancyGrid
from robot.mapping.scan_matcher import ScanMatcher
from robot.simulation.world import SimWorld


@pytest.fixture(scope="module")
def world_map():
    cfg = load_config()
    w = SimWorld(cfg["simulation"]["world"], cfg["chassis"], seed=1)
    g = OccupancyGrid.from_config(cfg["mapping"])
    w.reference_grid(g)
    return w, g


def scan(w, pose, n=360, max_range=6.0, noise=0.01, seed=0):
    rng = np.random.default_rng(seed)
    r = np.full(n, np.inf)
    for k in range(n):
        d = w.raycast(pose[0], pose[1], pose[2] - math.pi + 2 * math.pi * k / n, max_range)
        if d is not None:
            r[k] = d + rng.normal(0, noise)
    return ScanMatcher.scan_points(-math.pi, 2 * math.pi / n, r, max_range, 90)


@pytest.mark.parametrize("offset", [(0.06, -0.05, 0.05), (-0.08, 0.03, -0.08), (0.0, 0.0, 0.0)])
def test_recovers_pose_offset(world_map, offset):
    w, g = world_map
    true = (0.3, -0.4, 0.4)
    pts = scan(w, true)
    m = ScanMatcher()
    res = m.match(g, pts, (true[0] + offset[0], true[1] + offset[1], true[2] + offset[2]))
    assert res is not None
    assert math.dist(res.pose[:2], true[:2]) < 0.025
    assert abs(res.pose[2] - true[2]) < 0.02
    assert res.score > 0.8


def test_rejects_when_map_too_empty():
    g = OccupancyGrid(4, 4, 0.05, (-2, -2))
    pts = np.stack([np.linspace(0.5, 1.5, 50), np.zeros(50)], axis=1)
    assert ScanMatcher().match(g, pts, (0, 0, 0)) is None


def test_rejects_sparse_scan(world_map):
    _, g = world_map
    assert ScanMatcher().match(g, np.zeros((5, 2)), (0, 0, 0)) is None
