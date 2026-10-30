"""Localisation, missions, low-battery return and object map in the simulator."""

import math
import time

import pytest

from robot.config import load_config
from robot.mapping.occupancy_grid import OccupancyGrid
from robot.navigation.missions import Mission, Waypoint
from robot.runtime import CommandRejected, Mode, Robot


def wait(cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def make_robot(tmp_path, lidar=False, **sim):
    cfg = load_config(overrides={
        "robot": {"mode": "simulation"},
        "mapping": {"maps_dir": str(tmp_path / "maps")},
        "hardware": {"lidar": {"enabled": lidar}},
        "simulation": {"pickup_failure_rate": 0.0, **sim},
    })
    r = Robot(cfg)
    r.start()
    time.sleep(0.3)
    g = OccupancyGrid.from_config(cfg["mapping"])
    r.hw.world.reference_grid(g)
    r.mapper.store.save(g, "truth")
    return r


@pytest.fixture
def robot(tmp_path):
    r = make_robot(tmp_path)
    yield r
    r.shutdown()


@pytest.fixture
def lidar_robot(tmp_path):
    r = make_robot(tmp_path, lidar=True, odom_noise=0.05, seed=11)
    yield r
    r.shutdown()


def drive(r, v, w, seconds):
    end = time.time() + seconds
    while time.time() < end:
        r.teleop(v, w)
        time.sleep(0.1)


def err(r):
    gt = r.ground_truth()
    p = r.pose()
    return math.dist((p["x"], p["y"]), (gt["x"], gt["y"]))


@pytest.mark.slow
def test_mcl_beats_odometry(lidar_robot):
    r = lidar_robot
    r.load_map("truth")
    assert r.localizer.active and r.pose()["source"].startswith("mcl")
    # route through open floor: wheel slip against furniture is a different
    # (documented) failure mode and is not what this test measures
    for v, w, s in [(0.2, 0, 3), (0, -1.0, 1.6), (0.2, 0, 3), (0, 1.0, 1.6), (0.2, 0, 3), (0.15, 0.3, 4)]:
        drive(r, v, w, s)
    assert r.hw.world.state.collisions == 0
    od = r.odom.pose
    gt = r.ground_truth()
    odom_err = math.dist((od.x, od.y), (gt["x"], gt["y"]))
    assert err(r) < 0.1
    assert err(r) < odom_err


@pytest.mark.slow
def test_kidnapped_robot_global_localization(lidar_robot):
    r = lidar_robot
    r.load_map("truth")
    r.hw.world.teleport(-1.5, 0.5, 1.0)
    r.global_localization()
    for _ in range(4):
        drive(r, 0, 0.8, 2)                # spin on the spot: no slip, full view
    assert err(r) < 0.2
    assert r.localizer.converged


def test_ultrasonic_only_localization_is_off_by_default(robot):
    robot.load_map("truth")
    assert not robot.localizer.active
    assert robot.pose()["source"].startswith("ekf")
    with pytest.raises(CommandRejected):
        robot.global_localization()


def test_initial_pose_shifts_map_frame(robot):
    robot.reset_pose(1.0, -0.5, math.pi / 2)
    p = robot.pose()
    assert (p["x"], p["y"], p["theta"]) == pytest.approx((1.0, -0.5, math.pi / 2), abs=0.02)


@pytest.mark.slow
def test_mission_visits_waypoints_and_returns_home(robot):
    robot.load_map("truth")
    robot.mission_library["tour"] = Mission("tour", [Waypoint(x=0.8, y=-0.3), Waypoint(x=0.0, y=-1.0)],
                                            return_home=True)
    robot.start_mission("tour")
    assert robot.mode == Mode.MISSION
    assert wait(lambda: robot.missions.p.state.value in ("completed", "failed", "aborted"), 90)
    assert robot.missions.p.state.value == "completed", robot.missions.p.message
    assert robot.missions.p.reached == 2
    gt = robot.ground_truth()
    assert math.dist((gt["x"], gt["y"]), (0.0, 0.0)) < 0.25
    assert robot.hw.world.state.collisions == 0


def test_mission_validation(robot):
    m = Mission("bad", [Waypoint(location="nowhere")])
    with pytest.raises(ValueError):
        m.validate(robot.navigator.named_locations)
    with pytest.raises(ValueError):
        Mission("x", []).validate({})
    with pytest.raises(KeyError):
        robot.start_mission("missing")


def test_low_battery_aborts_task_and_returns_to_dock(robot):
    robot.load_map("truth")
    robot.mission_library["far"] = Mission("far", [Waypoint(x=1.5, y=-1.2)])
    robot.start_mission("far")
    time.sleep(0.5)
    robot.hw.world.battery_v = 6.9            # below return_voltage (7.0), above the cut-off
    assert wait(lambda: robot.navigator.goal is not None and robot.navigator.goal.name == "dock", 3.0)
    assert robot.missions.p.state.value == "aborted"
    assert any(e.type == "low_battery_return" for e in robot.events.recent)


def test_object_map_places_visible_object(robot):
    # the red object at (0.6, -0.3) is in view from the start pose
    assert wait(lambda: any(o["color"] == "red" for o in robot.objects.list()), 5.0)
    red = [o for o in robot.objects.list() if o["color"] == "red"][0]
    assert math.dist((red["x"], red["y"]), (0.6, -0.3)) < 0.12
