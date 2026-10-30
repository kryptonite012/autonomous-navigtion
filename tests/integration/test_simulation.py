"""Closed-loop tests in the simulator (real time, a few seconds each)."""

import math
import time

import pytest

from robot.hardware.factory import build_hardware
from robot.runtime import CommandRejected, Robot


@pytest.fixture
def robot(cfg):
    r = Robot(cfg)
    r.start()
    time.sleep(0.3)
    yield r
    r.shutdown()


def wait(cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_missing_hardware_disables_features(cfg):
    cfg["robot"]["mode"] = "hardware"
    hw = build_hardware(cfg)          # no Pi libraries here: everything is missing
    caps = hw.capabilities()
    assert not any(caps.values())
    assert hw.motors is None
    r = Robot(cfg, hw)
    with pytest.raises(CommandRejected):
        r.teleop(0.1, 0.0)
    with pytest.raises(CommandRejected):
        r.start_sorting()
    assert r.safety.status.faults           # no motor driver -> motion not allowed


def test_manual_drive_moves_robot(robot):
    x0 = robot.hw.world.true_pose()[0]
    end = time.time() + 1.5
    while time.time() < end:
        robot.teleop(0.15, 0.0)
        time.sleep(0.1)
    assert robot.hw.world.true_pose()[0] - x0 > 0.1
    # odometry agrees with ground truth
    assert abs(robot.pose()["x"] - robot.hw.world.true_pose()[0]) < 0.03


def test_watchdog_stops_when_commands_stop(robot):
    robot.teleop(0.15, 0.0)
    time.sleep(1.2)
    p1 = robot.hw.world.true_pose()
    time.sleep(0.5)
    p2 = robot.hw.world.true_pose()
    assert math.dist(p1[:2], p2[:2]) < 0.01


def test_obstacle_triggers_stop(robot):
    w = robot.hw.world
    w.add_dynamic_obstacle(0.45, 0.0, 0.08)     # box placed in front of the robot
    for _ in range(40):
        try:
            robot.teleop(0.25, 0.0)
        except CommandRejected:
            break
        time.sleep(0.05)
    assert w.state.collisions == 0
    assert robot.safety.status.zone == "stop"


def test_sensor_disconnect_blocks_forward(robot):
    front = [s for s in robot.hw.ranges if s.name == "us_front"][0]
    front.disconnected = True
    time.sleep(0.8)
    assert "us_front" in robot.safety.status.stale_sensors
    robot.teleop(0.2, 0.0)
    time.sleep(0.1)
    assert robot.safety.status.applied[0] == 0.0


def test_hardware_estop_stops_everything(robot):
    robot.hw.world.estop_pressed = True
    assert wait(lambda: robot.safety.status.estop_hardware, 1.0)
    with pytest.raises(CommandRejected):
        robot.teleop(0.1, 0.0)
    robot.hw.world.estop_pressed = False
    time.sleep(0.2)
    with pytest.raises(CommandRejected):
        robot.teleop(0.1, 0.0)                  # latched until reset
    assert robot.reset_emergency_stop()[0]


def test_motor_failure_reported(robot):
    robot.hw.world.motor_fault = True
    try:
        robot.teleop(0.1, 0.0)
    except CommandRejected:
        pass
    assert wait(lambda: "motor_communication" in robot.safety.status.faults, 1.0)


def test_battery_undervoltage(robot):
    robot.hw.world.battery_v = 6.3
    assert wait(lambda: "battery_undervoltage" in robot.safety.status.faults, 2.5)


def test_localization_loss_holds_position(robot):
    robot.mapper.start()
    time.sleep(0.5)
    robot.navigator.set_goal(1.0, -0.5)
    robot.mode = robot.mode.NAVIGATION
    robot.pose_valid = False
    robot.hw.encoders.ticks = lambda: (_ for _ in ()).throw(IOError("encoder bus"))
    assert wait(lambda: robot.navigator.state.value == "waiting_localization", 2.0)
    p1 = robot.hw.world.true_pose()
    time.sleep(0.6)
    assert math.dist(p1[:2], robot.hw.world.true_pose()[:2]) < 0.02


@pytest.mark.slow
def test_navigate_to_goal_with_saved_map(robot, tmp_path):
    # build a ground-truth map of the world by ray casting from many poses
    g = robot.mapper.grid
    w = robot.hw.world
    for x in [i * 0.4 - 2.2 for i in range(12)]:
        for y in [j * 0.4 - 1.8 for j in range(10)]:
            if w._collides(x, y):
                continue
            for k in range(72):
                a = k * math.pi / 36
                d = w.raycast(x, y, a, 2.0)
                g.integrate_ray(x, y, a, d, 2.0)
    robot.mapper.save("truth")
    robot.mapper.load("truth")
    robot.navigate_to(1.4, -1.2)
    assert wait(lambda: robot.navigator.state.value in ("reached", "failed"), 60)
    assert robot.navigator.state.value == "reached", robot.navigator.message
    gx, gy, _ = w.true_pose()
    assert math.dist((gx, gy), (1.4, -1.2)) < 0.2
    assert w.state.collisions == 0


@pytest.mark.slow
def test_sorting_end_to_end(robot):
    robot.start_sorting()
    assert wait(lambda: len(robot.sorter.history) >= 1, 60)
    rec = robot.sorter.history[0]
    assert rec.status == "success"
    obj = [o for o in robot.hw.world.objects if o.sorted_bin is not None][0]
    assert obj.sorted_bin == robot.sorter.bin_for(obj.color)
    robot.stop_sorting()
