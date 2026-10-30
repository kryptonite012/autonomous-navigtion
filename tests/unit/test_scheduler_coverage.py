import datetime as dt

import pytest

from robot.config import load_config
from robot.mapping.occupancy_grid import OccupancyGrid
from robot.navigation.coverage import coverage_waypoints
from robot.navigation.scheduler import Schedule, Scheduler
from robot.simulation.world import SimWorld


def test_schedule_fires_once_per_day_within_grace():
    started = []
    sch = Scheduler(started.append, grace_min=10)
    sch.schedules["p"] = Schedule("p", "patrol", "21:00", ["mon", "tue"])
    mon = dt.datetime(2026, 11, 2, 20, 59)           # a Monday
    assert sch.check(mon) == []
    assert sch.check(mon.replace(hour=21, minute=3)) == ["p"]
    assert sch.check(mon.replace(hour=21, minute=4)) == []          # once per day
    assert sch.check(dt.datetime(2026, 11, 3, 21, 30)) == []        # Tuesday but past the grace window
    assert sch.check(dt.datetime(2026, 11, 4, 21, 0)) == []         # Wednesday not selected
    assert started == ["patrol"]


def test_schedule_skips_when_busy():
    events = []

    def busy(name):
        raise RuntimeError("navigation is running")
    sch = Scheduler(busy, on_event=lambda *a: events.append(a))
    sch.schedules["p"] = Schedule("p", "patrol", "08:00")
    sch.check(dt.datetime(2026, 11, 2, 8, 0))
    assert sch.schedules["p"].last_result.startswith("skipped")
    assert events[0][0] == "schedule_skipped"


def test_schedule_validation():
    with pytest.raises(ValueError):
        Schedule("x", "m", "25:00").validate({"m"})
    with pytest.raises(ValueError):
        Schedule("x", "m", "10:00", ["someday"]).validate({"m"})
    with pytest.raises(ValueError):
        Schedule("x", "missing", "10:00").validate({"m"})
    Schedule("x", "m", "10:00").validate({"m"})


@pytest.fixture(scope="module")
def room():
    cfg = load_config()
    w = SimWorld(cfg["simulation"]["world"], cfg["chassis"], seed=1)
    g = OccupancyGrid.from_config(cfg["mapping"])
    w.reference_grid(g)
    return w, g


def test_coverage_waypoints_are_in_free_space_and_alternate(room):
    w, g = room
    pts = coverage_waypoints(g, spacing=0.6, inflation=0.18)
    assert 6 <= len(pts) <= 100
    blocked = g.inflated_obstacles(0.18)
    for x, y in pts:
        cx, cy = g.world_to_cell(x, y)
        assert not blocked[cy, cx]
        assert not w._collides(x, y)
    # rows alternate direction (lawnmower)
    rows = sorted({y for _, y in pts})
    first = [x for x, y in pts if y == rows[0]]
    second = [x for x, y in pts if y == rows[1]]
    assert (first[0] < first[-1]) != (second[0] < second[-1])


def test_coverage_limits(room):
    _, g = room
    with pytest.raises(ValueError):
        coverage_waypoints(g, spacing=0.05, inflation=0.18)
    with pytest.raises(ValueError):
        coverage_waypoints(g, spacing=0.15, inflation=0.18, max_points=10)
