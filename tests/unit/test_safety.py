import math
import time

import pytest

from robot.hardware.base import EmergencyStopInput, MotorDriver, RangeReading
from robot.motors.kinematics import DiffDriveKinematics
from robot.safety.controller import SafetyConfig, SafetyController


class FakeMotors(MotorDriver):
    def __init__(self):
        super().__init__("fake")
        self.duty = (0.0, 0.0)
        self.fail = False
        self.stops = 0

    def set_duty(self, left, right):
        if self.fail:
            raise IOError("i2c timeout")
        self.duty = (left, right)

    def stop(self):
        self.stops += 1
        self.duty = (0.0, 0.0)


class FakeEstop(EmergencyStopInput):
    def __init__(self):
        super().__init__("estop")
        self.pressed = False

    def is_pressed(self):
        return self.pressed


def make(**kw):
    m, e = FakeMotors(), FakeEstop()
    events = []
    cfg = SafetyConfig(max_linear_accel_mps2=100.0, **kw)
    sc = SafetyController(m, DiffDriveKinematics(0.033, 0.16, 0.48), cfg, e,
                          on_event=lambda *a: events.append(a), expected_sensors=["front"])
    return sc, m, e, events


def front(d, valid=True, t=None):
    return [RangeReading("front", d, 0.0, t or time.time(), valid)]


def test_forward_allowed_when_clear():
    sc, m, *_ = make()
    sc.update_ranges(front(None))
    sc.command(0.2, 0.0, "test")
    assert m.duty[0] > 0 and m.duty[1] > 0


def test_obstacle_in_stop_zone_blocks_forward_only():
    sc, m, _, events = make()
    sc.update_ranges(front(0.1))
    sc.command(0.2, 0.0, "test")
    assert m.duty == (0.0, 0.0)
    assert sc.status.zone == "stop"
    sc.command(0.0, 1.0, "test")          # turning away is allowed
    assert m.duty[0] < 0 < m.duty[1]
    sc.command(-0.2, 0.0, "test")         # slow blind reverse is allowed
    assert m.duty[0] < 0
    assert sc.status.applied[0] >= -0.061


def test_slow_zone_scales_speed():
    sc, m, *_ = make()
    sc.update_ranges(front(0.25))
    sc.command(0.2, 0.0, "test")
    assert sc.status.zone == "slow"
    assert sc.status.applied[0] == pytest.approx(0.2 * 0.4, abs=1e-3)


def test_software_estop_latches_and_resets():
    sc, m, _, events = make()
    sc.update_ranges(front(None))
    sc.command(0.2, 0, "t")
    sc.emergency_stop("test")
    assert m.duty == (0.0, 0.0)
    assert not sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0)
    assert any(e[0] == "emergency_stop" for e in events)
    ok, _ = sc.reset_emergency_stop()
    assert ok
    sc.update_ranges(front(None))
    assert sc.command(0.2, 0, "t")
    assert m.duty[0] > 0


def test_hardware_estop_overrides_and_requires_reset():
    sc, m, e, _ = make()
    sc.update_ranges(front(None))
    e.pressed = True
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0)
    ok, msg = sc.reset_emergency_stop()
    assert not ok and "still pressed" in msg
    e.pressed = False
    sc.update_ranges(front(None))
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0), "releasing the button alone must not restart motion"
    assert sc.reset_emergency_stop()[0]


def test_stale_front_sensor_blocks_forward():
    sc, m, *_ = make(sensor_timeout_s=0.2)
    sc.update_ranges(front(None, t=time.time() - 1.0))
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0)
    assert "front" in sc.status.stale_sensors


def test_invalid_sensor_reading_counts_as_stale():
    sc, m, *_ = make(sensor_timeout_s=0.2)
    sc.update_ranges(front(None, valid=False))
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0)


def test_command_watchdog():
    sc, m, *_ = make(command_timeout_s=0.1)
    sc.update_ranges(front(None))
    sc.command(0.2, 0, "t")
    assert m.duty[0] > 0
    time.sleep(0.15)
    sc.update_ranges(front(None))
    sc.tick()
    assert m.duty == (0.0, 0.0)


def test_battery_undervoltage_fault():
    sc, m, _, events = make()
    sc.update_ranges(front(None))
    sc.update_battery(6.2)
    assert "battery_undervoltage" in sc.status.faults
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0)
    assert not sc.clear_fault("battery_undervoltage")   # still low
    sc.update_battery(7.8)
    assert sc.clear_fault("battery_undervoltage")


def test_motor_communication_failure_faults():
    sc, m, _, events = make()
    sc.update_ranges(front(None))
    m.fail = True
    sc.command(0.2, 0, "t")
    assert "motor_communication" in sc.status.faults
    m.fail = False
    sc.command(0.2, 0, "t")
    assert m.duty == (0.0, 0.0), "fault stays latched until cleared"


def test_drive_stall_detected():
    sc, m, *_ = make()
    sc.update_ranges(front(None))
    for _ in range(5):
        time.sleep(0.02)
        sc.command(0.3, 0, "t")
    sc.update_encoders((100, 100))
    sc._stall_since = time.monotonic() - 2.0
    sc.update_encoders((100, 100))
    assert "drive_stall" in sc.status.faults


def test_velocity_limits_and_nonfinite():
    sc, m, *_ = make()
    sc.update_ranges(front(None))
    sc.command(5.0, 50.0, "t")
    assert sc.status.applied[0] <= 0.3 and abs(sc.status.applied[1]) <= 2.0
    with pytest.raises(ValueError):
        sc.command(math.nan, 0, "t")


def test_emergency_stop_latency_is_small():
    sc, m, *_ = make()
    sc.update_ranges(front(None))
    sc.command(0.2, 0, "t")
    t0 = time.perf_counter()
    sc.emergency_stop("latency")
    dt = time.perf_counter() - t0
    assert m.stops >= 1 and dt < 0.01
