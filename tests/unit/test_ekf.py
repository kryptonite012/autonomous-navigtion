import math

import numpy as np
import pytest

from robot.localization.ekf import OdometryEKF
from robot.motors.kinematics import DiffDriveKinematics


def make():
    k = DiffDriveKinematics(0.033, 0.16, 0.5)
    return OdometryEKF(k, 1440), k


def drive(ekf, vl, vr, seconds, dt=0.05, gyro=None, slip=0.0, t0=0.0, ticks0=(0.0, 0.0)):
    """Feed ideal encoder ticks for wheel speeds vl, vr (m/s)."""
    tl, tr = ticks0
    t = t0
    ekf.update((int(tl), int(tr)), t)
    for _ in range(int(seconds / dt)):
        tl += vl * dt / ekf.m_per_tick
        tr += (vr + slip) * dt / ekf.m_per_tick
        t += dt
        g = None if gyro is None else gyro
        ekf.update((int(tl), int(tr)), t, g)
    return t, (tl, tr)


def test_straight_line():
    ekf, _ = make()
    drive(ekf, 0.2, 0.2, 5.0)
    assert ekf.pose.x == pytest.approx(1.0, abs=0.02)
    assert abs(ekf.pose.y) < 0.01
    assert ekf.v == pytest.approx(0.2, abs=0.01)


def test_rotation_in_place():
    ekf, k = make()
    w = 1.0
    vl, vr = k.inverse(0.0, w)
    drive(ekf, vl, vr, math.pi / 2 / w)
    assert ekf.pose.theta == pytest.approx(math.pi / 2, abs=0.05)


def test_gyro_corrects_slipping_wheel():
    """Right wheel slips (spins faster than ground speed): encoders say we
    turn, the gyro says we go straight. The EKF should follow the gyro."""
    ekf, _ = make()
    drive(ekf, 0.2, 0.2, 5.0, gyro=0.0, slip=0.05)
    enc_only, _ = make()
    drive(enc_only, 0.2, 0.2, 5.0, gyro=None, slip=0.05)
    assert abs(ekf.pose.theta) < abs(enc_only.pose.theta) / 3


def test_covariance_grows_and_stays_symmetric():
    ekf, _ = make()
    s0 = ekf.pose_std()[0]
    drive(ekf, 0.2, 0.25, 10.0)
    assert ekf.pose_std()[0] > s0
    assert np.allclose(ekf.P, ekf.P.T)
    assert np.all(np.linalg.eigvalsh(ekf.P) > -1e-12)


def test_reset():
    ekf, _ = make()
    drive(ekf, 0.2, 0.2, 2.0)
    ekf.reset(1.0, 2.0, 0.5)
    assert (ekf.pose.x, ekf.pose.y, ekf.pose.theta) == (1.0, 2.0, 0.5)
