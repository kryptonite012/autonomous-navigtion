"""Particle filter tests using the simulated room as ground truth."""

import math

import numpy as np
import pytest

from robot.config import load_config
from robot.localization.localizer import Localizer, compose, inverse
from robot.localization.mcl import Beam, ParticleFilter
from robot.mapping.occupancy_grid import OccupancyGrid
from robot.simulation.world import SimWorld


@pytest.fixture(scope="module")
def world_and_map():
    cfg = load_config()
    w = SimWorld(cfg["simulation"]["world"], cfg["chassis"], seed=1)
    g = OccupancyGrid.from_config(cfg["mapping"])
    w.reference_grid(g)
    return w, g


def lidar_beams(w, pose, n=72, max_range=6.0, noise=0.01, rng=None):
    rng = rng or np.random.default_rng(0)
    out = []
    for k in range(n):
        a = -math.pi + 2 * math.pi * k / n
        d = w.raycast(pose[0], pose[1], pose[2] + a, max_range)
        out.append(Beam(a, None if d is None else d + rng.normal(0, noise), max_range))
    return out


def path():
    """A loop through the free part of the room (true poses)."""
    pts = []
    for i in range(30):
        pts.append((-0.5 + i * 0.05, -0.3, 0.0))
    for i in range(20):
        pts.append((1.0, -0.3 + i * 0.05, math.pi / 2))
    return pts


def biased(poses, scale=1.08, drift=0.01):
    """Odometry with a scale error and heading drift accumulating per step."""
    out = [poses[0]]
    x, y, th = poses[0]
    for i in range(1, len(poses)):
        a, b = poses[i - 1], poses[i]
        d = inverse(a)
        rel = compose(d, b)
        rel = (rel[0] * scale, rel[1] * scale, rel[2] + drift)
        x, y, th = compose((x, y, th), rel)
        out.append((x, y, th))
    return out


def test_reference_map_has_walls_and_free_space(world_and_map):
    _, g = world_and_map
    st = g.stats()
    assert st["occupied_ratio"] > 0.01 and st["free_ratio"] > 0.15


def test_tracking_corrects_odometry_drift(world_and_map):
    w, g = world_and_map
    pf = ParticleFilter(n=400, seed=2)
    pf.set_map(g)
    truth = path()
    odom = biased(truth)
    pf.init_pose(*truth[0], std_xy=0.05, std_th=0.05)
    rng = np.random.default_rng(3)
    for i in range(1, len(truth)):
        pf.predict(odom[i - 1], odom[i])
        pf.update(lidar_beams(w, truth[i], rng=rng))
    est, std_xy, _ = pf.estimate()
    err_pf = math.dist(est[:2], truth[-1][:2])
    err_odom = math.dist(odom[-1][:2], truth[-1][:2])
    assert err_odom > 0.15
    assert err_pf < 0.08, (err_pf, err_odom)
    assert std_xy < 0.15


def test_global_localization_with_lidar(world_and_map):
    w, g = world_and_map
    pf = ParticleFilter(n=300, seed=5)
    pf.set_map(g)
    pf.init_global()
    truth = path()
    rng = np.random.default_rng(4)
    for i in range(1, len(truth)):
        pf.predict(truth[i - 1], truth[i])
        pf.update(lidar_beams(w, truth[i], rng=rng))
    est, std_xy, _ = pf.estimate()
    assert math.dist(est[:2], truth[-1][:2]) < 0.2
    assert std_xy < 0.2


def test_update_needs_enough_hits(world_and_map):
    _, g = world_and_map
    pf = ParticleFilter(n=50)
    pf.set_map(g)
    pf.init_pose(0, 0, 0)
    assert not pf.update([Beam(0.0, None, 2.0), Beam(1.0, 1.0, 2.0)])


def test_localizer_correction_and_ultrasonic_mode_flags(world_and_map):
    _, g = world_and_map
    cfg = load_config()
    loc = Localizer(cfg, has_lidar=False, sensor_offset=0.1)
    assert not loc.enabled                       # auto mode needs a LiDAR
    loc.set_map(g, "truth", (0, 0, 0))
    assert not loc.active
    loc.set_initial_pose((1.0, 0.5, 0.3), (0.0, 0.0, 0.0))
    mp = loc.map_pose((0.0, 0.0, 0.0))
    assert mp == pytest.approx((1.0, 0.5, 0.3))
    with pytest.raises(RuntimeError):
        loc.global_localization()
    cfg["localization"] = {"mode": "on"}
    loc2 = Localizer(cfg, has_lidar=False, sensor_offset=0.1)
    loc2.set_map(g, "truth", (0, 0, 0))
    assert loc2.active and loc2.status()["experimental"]
