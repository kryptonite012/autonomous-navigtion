"""Hardware-in-the-loop tests. Run ON THE ROBOT with:

    NEXUS_HIL=1 python3 -m pytest tests/hil -s

They need an operator: some tests ask you to press the e-stop or place an
object. Lift the robot (wheels off the ground) before running motor tests.
"""

import time

import pytest

from robot.config import load_config
from robot.hardware.factory import build_physical

pytestmark = pytest.mark.hardware


@pytest.fixture(scope="module")
def hw():
    cfg = load_config(overrides={"robot": {"mode": "hardware"}})
    h = build_physical(cfg)
    yield h
    h.shutdown()


def test_required_devices_present(hw):
    caps = hw.capabilities()
    missing = [k for k in ("drive", "odometry", "obstacle_detection", "vision") if not caps[k]]
    assert not missing, f"missing capabilities: {missing}; see hardware_check.py output"


def test_camera_delivers_frames(hw):
    time.sleep(1.0)
    ok, frame, ts = hw.camera.read()
    assert ok and frame is not None and frame.shape[0] >= 240
    assert time.time() - ts < 1.0


def test_front_sensor_sees_hand():
    input("\nHold a hand ~10 cm in front of the front ultrasonic sensor and press Enter")
    cfg = load_config(overrides={"robot": {"mode": "hardware"}})
    hw = build_physical(cfg)
    front = [s for s in hw.ranges if abs(s.angle_rad) < 0.1]
    assert front, "no front range sensor"
    readings = [front[0].read().distance_m for _ in range(5)]
    hw.shutdown()
    assert any(r is not None and r < 0.2 for r in readings), readings


def test_estop_input_reads_pressed(hw):
    input("\nPRESS the emergency stop and press Enter")
    assert hw.estop.is_pressed()
    input("RELEASE the emergency stop and press Enter")
    assert not hw.estop.is_pressed()


def test_motor_stop_latency_with_encoders(hw):
    """Wheels off the ground: drive, stop, measure how long the encoders keep counting."""
    input("\nLift the robot so the wheels spin freely, then press Enter")
    hw.motors.set_duty(0.4, 0.4)
    time.sleep(1.0)
    t0 = time.perf_counter()
    hw.motors.stop()
    last = hw.encoders.ticks()
    settled = None
    while time.perf_counter() - t0 < 2.0:
        time.sleep(0.01)
        now = hw.encoders.ticks()
        if now == last:
            settled = time.perf_counter() - t0
            break
        last = now
    print(f"\nwheels stopped {settled * 1000:.0f} ms after stop()" if settled else "\nwheels did not settle")
    assert settled is not None and settled < 0.5


def test_battery_reading_plausible(hw):
    v = hw.battery.voltage()
    assert v is not None and 5.0 < v < 9.0, v


def test_bin_sensor_counts_drop(hw):
    before = hw.bin_sensor.event_count()
    input("\nDrop a test object through the bin chute and press Enter")
    assert hw.bin_sensor.event_count() > before
