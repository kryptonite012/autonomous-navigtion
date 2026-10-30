import math

import numpy as np

from robot.navigation.dwa import DWAConfig, DWAPlanner


def planner():
    return DWAPlanner(DWAConfig(max_v=0.24, max_w=1.6))


def test_free_space_goes_straight_and_fast():
    p = planner()
    v, w = p.compute((0, 0, 0), (0.2, 0.0), (1.0, 0.0), np.zeros((0, 2)))
    assert v > 0.18 and abs(w) < 0.2


def test_turns_towards_target_on_the_side():
    p = planner()
    v, w = p.compute((0, 0, 0), (0.1, 0.0), (0.5, 0.5), np.zeros((0, 2)))
    assert w > 0.3


def test_steers_around_obstacle_in_front():
    p = planner()
    wall = np.array([[0.45, y] for y in np.linspace(-0.12, 0.12, 7)])
    v, w = p.compute((0, 0, 0), (0.2, 0.0), (1.2, 0.0), wall)
    assert abs(w) > 0.3                     # swerves
    # the chosen trajectory keeps clear of the obstacle
    traj = np.array(p.last_trajectory)
    d = np.min(np.hypot(traj[:, None, 0] - wall[None, :, 0], traj[:, None, 1] - wall[None, :, 1]))
    assert d > 0.12 + 0.03


def test_boxed_in_only_rotation_allowed():
    p = planner()
    ring = np.array([[0.2 * math.cos(a), 0.2 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 36)])
    v, w = p.compute((0, 0, 0), (0.0, 0.0), (1.0, 0.0), ring)
    assert v == 0.0


def test_dynamic_window_respects_acceleration():
    p = planner()
    vs, ws = p.window(0.0, 0.0)
    assert vs.max() <= 0.6 * 0.3 + 1e-9
    assert np.abs(ws).max() <= 3.0 * 0.3 + 1e-9
