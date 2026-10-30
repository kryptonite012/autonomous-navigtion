import math

import pytest

from robot.motors.kinematics import DiffDriveKinematics, WheelOdometry, normalize_angle


def test_inverse_forward_roundtrip():
    k = DiffDriveKinematics(0.033, 0.16, 0.5)
    vl, vr = k.inverse(0.2, 1.0)
    v, w = k.forward(vl, vr)
    assert v == pytest.approx(0.2)
    assert w == pytest.approx(1.0)


def test_duty_saturation_keeps_curvature():
    k = DiffDriveKinematics(0.033, 0.16, 0.3)
    dl, dr = k.to_duty(1.0, 3.0)
    assert max(abs(dl), abs(dr)) == pytest.approx(1.0)
    vl, vr = k.inverse(1.0, 3.0)
    assert dl / dr == pytest.approx(vl / vr)


def test_odometry_straight_line():
    k = DiffDriveKinematics(0.033, 0.16, 0.5)
    odo = WheelOdometry(k, 1440)
    per_m = 1 / odo.m_per_tick
    odo.update((0, 0), 0.0)
    odo.update((int(per_m), int(per_m)), 1.0)
    assert odo.pose.x == pytest.approx(1.0, abs=1e-3)
    assert odo.pose.y == pytest.approx(0.0, abs=1e-6)
    assert odo.v == pytest.approx(1.0, abs=1e-3)


def test_odometry_rotation_in_place():
    k = DiffDriveKinematics(0.033, 0.16, 0.5)
    odo = WheelOdometry(k, 1440)
    quarter = (math.pi / 2) * 0.08 / odo.m_per_tick
    odo.update((0, 0), 0.0)
    odo.update((-int(quarter), int(quarter)), 1.0)
    assert odo.pose.theta == pytest.approx(math.pi / 2, abs=0.01)
    assert abs(odo.pose.x) < 1e-6


def test_normalize_angle():
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
