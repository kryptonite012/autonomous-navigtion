"""Waypoint missions on top of the navigator: sequences, patrol loops, dwell
times, and what to do when a waypoint cannot be reached."""

from __future__ import annotations

import enum
import re
import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Optional

from robot.navigation.navigator import NavState, Navigator

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _\-]{0,63}$")


@dataclass
class Waypoint:
    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    location: Optional[str] = None        # named location instead of x/y
    dwell_s: float = 0.0

    def label(self) -> str:
        return self.location or f"({self.x:.2f}, {self.y:.2f})"


@dataclass
class Mission:
    name: str
    waypoints: list[Waypoint]
    loop: bool = False
    max_loops: int = 0                    # 0 = until stopped (only with loop)
    return_home: bool = False             # go to the home location at the end
    on_failure: str = "skip"              # skip | abort

    def validate(self, named: dict) -> None:
        if not NAME_RE.match(self.name or ""):
            raise ValueError("mission name: 1-64 letters, digits, spaces, '_' or '-'")
        if not self.waypoints:
            raise ValueError("a mission needs at least one waypoint")
        if len(self.waypoints) > 100:
            raise ValueError("at most 100 waypoints")
        if self.on_failure not in ("skip", "abort"):
            raise ValueError("on_failure must be 'skip' or 'abort'")
        for wp in self.waypoints:
            if wp.location is not None:
                if wp.location not in named:
                    raise ValueError(f"unknown location {wp.location!r}")
            elif wp.x is None or wp.y is None:
                raise ValueError("each waypoint needs x and y, or a location")
            if not 0 <= wp.dwell_s <= 3600:
                raise ValueError("dwell must be 0..3600 s")

    def as_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Mission":
        wps = [Waypoint(**w) for w in d.get("waypoints", [])]
        return cls(name=d["name"], waypoints=wps, loop=bool(d.get("loop", False)),
                   max_loops=int(d.get("max_loops", 0)), return_home=bool(d.get("return_home", False)),
                   on_failure=d.get("on_failure", "skip"))


class MissionState(str, enum.Enum):
    IDLE = "idle"
    RUNNING = "running"
    DWELLING = "dwelling"
    RETURNING = "returning"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"


@dataclass
class MissionProgress:
    mission: Optional[Mission] = None
    state: MissionState = MissionState.IDLE
    index: int = 0
    loops_done: int = 0
    reached: int = 0
    skipped: list[str] = field(default_factory=list)
    started_at: Optional[float] = None
    message: str = ""


class MissionRunner:
    def __init__(self, navigator: Navigator, home_location: str = "home",
                 on_event: Optional[Callable[[str, str, str], None]] = None):
        self.nav = navigator
        self.home = home_location
        self.on_event = on_event or (lambda *a: None)
        self.p = MissionProgress()
        self._dwell_until = 0.0
        self._sent = False

    @property
    def running(self) -> bool:
        return self.p.state in (MissionState.RUNNING, MissionState.DWELLING, MissionState.RETURNING)

    def start(self, mission: Mission) -> None:
        mission.validate(self.nav.named_locations)
        self.p = MissionProgress(mission=mission, state=MissionState.RUNNING, started_at=time.time())
        self._send_current()
        self.on_event("mission_started", "info", f"mission '{mission.name}' started "
                      f"({len(mission.waypoints)} waypoints{', loop' if mission.loop else ''})")

    def stop(self, reason: str = "stopped by operator") -> None:
        if self.running:
            self.p.state = MissionState.ABORTED
            self.p.message = reason
            self.nav.cancel(f"mission aborted: {reason}")
            self.on_event("mission_aborted", "warning", f"mission '{self.p.mission.name}' aborted: {reason}")

    def _send_current(self) -> None:
        wp = self.p.mission.waypoints[self.p.index]
        if wp.location:
            self.nav.go_to_named(wp.location)
        else:
            self.nav.set_goal(wp.x, wp.y, wp.theta, f"wp{self.p.index + 1}")
        self.p.message = f"going to waypoint {self.p.index + 1}/{len(self.p.mission.waypoints)}: {wp.label()}"
        self._sent = True

    def _advance(self) -> None:
        m = self.p.mission
        self.p.index += 1
        if self.p.index >= len(m.waypoints):
            self.p.loops_done += 1
            if m.loop and (m.max_loops == 0 or self.p.loops_done < m.max_loops):
                self.p.index = 0
            elif m.return_home and self.home in self.nav.named_locations:
                self.p.state = MissionState.RETURNING
                self.p.message = f"returning to {self.home}"
                self.nav.go_to_named(self.home)
                return
            else:
                self._finish(MissionState.COMPLETED, "all waypoints visited")
                return
        self._send_current()

    def _finish(self, state: MissionState, msg: str) -> None:
        self.p.state = state
        self.p.message = msg
        sev = "info" if state == MissionState.COMPLETED else "error"
        self.on_event(f"mission_{state.value}", sev, f"mission '{self.p.mission.name}': {msg}")

    def step(self) -> None:
        """Call once per control cycle, after the navigator step."""
        if not self.running:
            return
        st = self.nav.state
        now = time.time()
        if self.p.state == MissionState.DWELLING:
            if now >= self._dwell_until:
                self.p.state = MissionState.RUNNING
                self._advance()
            return
        if self.p.state == MissionState.RETURNING:
            if st == NavState.REACHED:
                self._finish(MissionState.COMPLETED, "all waypoints visited, back home")
            elif st == NavState.FAILED:
                self._finish(MissionState.FAILED, f"could not return home: {self.nav.message}")
            elif st == NavState.IDLE:
                self._finish(MissionState.ABORTED, "navigation cancelled")
            return
        if st == NavState.REACHED:
            self.p.reached += 1
            wp = self.p.mission.waypoints[self.p.index]
            if wp.dwell_s > 0:
                self.p.state = MissionState.DWELLING
                self._dwell_until = now + wp.dwell_s
                self.p.message = f"waiting {wp.dwell_s:.0f} s at {wp.label()}"
            else:
                self._advance()
        elif st == NavState.FAILED:
            wp = self.p.mission.waypoints[self.p.index]
            if self.p.mission.on_failure == "abort":
                self._finish(MissionState.FAILED, f"waypoint {wp.label()} unreachable: {self.nav.message}")
            else:
                self.p.skipped.append(wp.label())
                self.on_event("mission_waypoint_skipped", "warning", f"skipping {wp.label()}: {self.nav.message}")
                self._advance()
        elif st == NavState.IDLE:
            self._finish(MissionState.ABORTED, "navigation cancelled")

    def status(self) -> dict:
        p = self.p
        m = p.mission
        return {
            "state": p.state.value,
            "mission": None if m is None else m.name,
            "waypoint_index": p.index,
            "waypoints": 0 if m is None else len(m.waypoints),
            "loops_done": p.loops_done,
            "reached": p.reached,
            "skipped": list(p.skipped),
            "message": p.message,
            "started_at": p.started_at,
        }
