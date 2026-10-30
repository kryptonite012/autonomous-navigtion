"""Sorting state machine with fake arm/vision so it runs fast and deterministic."""

import time

import pytest

from robot.hardware.base import BinSensor, ServoController
from robot.sorting.arm import SortingArm
from robot.sorting.sorter import Sorter, SortState
from robot.vision.detector import Detection


class FakeServos(ServoController):
    def __init__(self, on_close=None, on_open=None):
        super().__init__("servos")
        self.angles = {}
        self.on_close = on_close or (lambda: None)
        self.on_open = on_open or (lambda: None)
        self.fail = False

    def set_angle(self, ch, a):
        if self.fail:
            raise IOError("servo controller not responding")
        prev = self.angles.get(ch)
        self.angles[ch] = a
        if ch == "gripper" and a >= 90 and (prev is None or prev < 90):
            self.on_close()
        if ch == "gripper" and a < 90 and prev is not None and prev >= 90:
            self.on_open()

    def get_angle(self, ch):
        return self.angles.get(ch)

    def release(self):
        pass


class FakeBin(BinSensor):
    def __init__(self):
        super().__init__("bin")
        self.count = 0

    def event_count(self):
        return self.count


class World:
    """Object sits in the pickup zone until the gripper picks it."""

    def __init__(self, color="red", pick_ok=True, drops_detected=True):
        self.color = color
        self.present = True
        self.held = False
        self.pick_ok = pick_ok
        self.drops_detected = drops_detected
        self.bin = FakeBin()

    def detections(self):
        if not self.present:
            return [], time.time()
        return [Detection(self.color, (290, 300, 60, 60), (320, 330), 3600 * 1.2, 0.9, time.time(),
                          object_id=7)], time.time()

    def close(self):
        if self.present and self.pick_ok:
            self.present, self.held = False, True

    def open(self):
        if self.held:
            self.held = False
            if self.drops_detected:
                self.bin.count += 1


def build(cfg, world, bin_sensor=True, retries=None):
    if retries is not None:
        cfg["sorting"]["max_retries"] = retries
    servos = FakeServos(world.close, world.open)
    arm = SortingArm(servos, cfg["sorting"])
    arm.time_scale = 0.01
    records, events = [], []
    s = Sorter(cfg, arm, world.bin if bin_sensor else None, world.detections,
               drive=lambda v, w: True, stop_drive=lambda: None, pose=lambda: (0.0, 0.0, 0.0),
               on_record=records.append, on_event=lambda *a: events.append(a))
    s.sc["verify_timeout_s"] = 0.3
    s.pick_confirm_s = 0.3
    s.retry_settle_s = 0.05
    return s, servos, records, events


def run_until(s, cond, timeout=5.0):
    s.start()
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(0.02)
    s.stop()


def test_successful_sort_verified_by_bin_sensor(cfg):
    w = World("green")
    s, servos, recs, _ = build(cfg, w)
    run_until(s, lambda: recs)
    assert recs[0].status == "success"
    assert recs[0].target_bin == 2
    assert servos.angles["sorter"] == cfg["sorting"]["bin_angles"][2]


def test_unknown_colour_goes_to_unclassified_bin(cfg):
    w = World("orange")
    s, _, recs, _ = build(cfg, w)
    run_until(s, lambda: recs)
    assert recs[0].target_bin == cfg["sorting"]["unclassified_bin"]


def test_without_bin_sensor_result_is_unverified_not_success(cfg):
    w = World("blue")
    s, _, recs, _ = build(cfg, w, bin_sensor=False)
    run_until(s, lambda: recs)
    assert recs[0].status == "unverified"
    assert s.stats["success"] == 0


def test_drop_not_detected_is_failure(cfg):
    w = World("red", drops_detected=False)
    s, _, recs, _ = build(cfg, w)
    run_until(s, lambda: recs)
    assert recs[0].status == "failed_verification"


def test_failed_pickup_retries_then_records(cfg):
    w = World("yellow", pick_ok=False)
    s, _, recs, events = build(cfg, w, retries=2)
    run_until(s, lambda: recs)
    assert recs[0].status == "failed_pickup"
    assert recs[0].attempts == 3
    assert sum(1 for e in events if e[0] == "pickup_failed") == 3


def test_full_bin_is_skipped(cfg):
    w = World("red")
    s, _, recs, _ = build(cfg, w)
    s.capacity = 1
    s.bin_counts[1] = 1
    s.max_search_cycles = 0
    run_until(s, lambda: not s.running, timeout=2.0)
    assert recs == []


def test_servo_failure_stops_with_error(cfg):
    w = World("red")
    s, servos, recs, events = build(cfg, w)
    servos.fail = True
    run_until(s, lambda: s.state == SortState.ERROR, timeout=2.0)
    assert s.state == SortState.ERROR
    assert any(e[0] == "sorting_error" for e in events)


def test_config_validation(cfg):
    w = World()
    s, *_ = build(cfg, w)
    with pytest.raises(ValueError):
        s.set_config({"red": 9}, 5)
    s.set_config({"red": 2, "blue": 1}, 5, 3)
    assert s.bin_for("red") == 2 and s.capacity == 3
