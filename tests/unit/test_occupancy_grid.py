import math

import numpy as np

from robot.mapping.occupancy_grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, bresenham


def grid():
    return OccupancyGrid(4.0, 4.0, 0.05, (-2.0, -2.0))


def test_bresenham_endpoints():
    cells = list(bresenham(0, 0, 5, 3))
    assert cells[0] == (0, 0) and cells[-1] == (5, 3)
    assert len(cells) == 6


def test_ray_marks_free_and_hit():
    g = grid()
    for _ in range(5):
        g.integrate_ray(0.0, 0.0, 0.0, 1.0, 2.0)
    t = g.ternary()
    hit = g.world_to_cell(1.0, 0.0)
    mid = g.world_to_cell(0.5, 0.0)
    assert t[hit[1], hit[0]] == OCCUPIED
    assert t[mid[1], mid[0]] == FREE
    far = g.world_to_cell(1.5, 0.0)
    assert t[far[1], far[0]] == UNKNOWN


def test_no_return_clears_full_range():
    g = grid()
    for _ in range(3):
        g.integrate_ray(0.0, 0.0, math.pi / 2, None, 1.0)
    c = g.world_to_cell(0.0, 0.9)
    assert g.ternary()[c[1], c[0]] == FREE


def test_save_load_roundtrip(tmp_path):
    g = grid()
    for a in np.linspace(0, 2 * math.pi, 90, endpoint=False):
        for _ in range(3):
            g.integrate_ray(0.0, 0.0, a, 1.2, 2.0)
    pgm, yml = g.save(tmp_path / "room")
    assert pgm.exists() and yml.exists()
    text = yml.read_text()
    assert "resolution: 0.05" in text and "room.pgm" in text
    g2 = OccupancyGrid.load(yml)
    assert g2.width == g.width and g2.height == g.height
    assert (g2.ternary() == g.ternary()).mean() > 0.99


def test_inflation_blocks_near_obstacles():
    g = grid()
    g.mark_obstacle(0.0, 0.0)
    g.clear_area(1.0, 1.0, 0.5)
    blocked = g.inflated_obstacles(0.2, allow_unknown=True)
    c = g.world_to_cell(0.1, 0.0)
    assert blocked[c[1], c[0]]
    c = g.world_to_cell(1.0, 1.0)
    assert not blocked[c[1], c[0]]
