"""Time-based mission schedules (e.g. "patrol every weekday at 21:00").

Times are local robot time. Each schedule fires at most once per day, at
the first check at or after its time (within ``grace_min`` minutes, so a
reboot at 21:20 does not start a 21:00 patrol hours late). If the robot is
busy or not ready, the run is skipped and an event explains why; it is not
queued.
"""

from __future__ import annotations

import datetime as dt
import re
from dataclasses import asdict, dataclass, field
from typing import Callable, Optional

TIME_RE = re.compile(r"^([01]\d|2[0-3]):([0-5]\d)$")
DAYS = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]


@dataclass
class Schedule:
    name: str
    mission: str
    time: str                                  # "HH:MM"
    days: list[str] = field(default_factory=lambda: list(DAYS))
    enabled: bool = True
    last_run: Optional[str] = None             # ISO date of the last attempt
    last_result: Optional[str] = None

    def validate(self, missions: set[str]) -> None:
        if not re.match(r"^[A-Za-z0-9][A-Za-z0-9 _\-]{0,63}$", self.name or ""):
            raise ValueError("schedule name: 1-64 letters, digits, spaces, '_' or '-'")
        if not TIME_RE.match(self.time or ""):
            raise ValueError("time must be HH:MM (24 h)")
        if not self.days or any(d not in DAYS for d in self.days):
            raise ValueError(f"days must be a non-empty subset of {DAYS}")
        if self.mission not in missions:
            raise ValueError(f"unknown mission {self.mission!r}")

    def due(self, now: dt.datetime, grace_min: int) -> bool:
        if not self.enabled or DAYS[now.weekday()] not in self.days:
            return False
        if self.last_run == now.date().isoformat():
            return False
        h, m = map(int, self.time.split(":"))
        start = now.replace(hour=h, minute=m, second=0, microsecond=0)
        return start <= now < start + dt.timedelta(minutes=grace_min)

    def as_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Schedule":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


class Scheduler:
    def __init__(self, start_mission: Callable[[str], None],
                 on_event: Optional[Callable[[str, str, str], None]] = None,
                 on_change: Optional[Callable[[Schedule], None]] = None, grace_min: int = 10):
        self.start_mission = start_mission
        self.on_event = on_event or (lambda *a: None)
        self.on_change = on_change or (lambda s: None)
        self.grace_min = grace_min
        self.schedules: dict[str, Schedule] = {}

    def check(self, now: Optional[dt.datetime] = None) -> list[str]:
        now = now or dt.datetime.now()
        fired = []
        for s in list(self.schedules.values()):
            if not s.due(now, self.grace_min):
                continue
            s.last_run = now.date().isoformat()
            try:
                self.start_mission(s.mission)
                s.last_result = "started"
                fired.append(s.name)
                self.on_event("schedule_started", "info", f"schedule '{s.name}' started mission '{s.mission}'")
            except Exception as exc:          # busy, e-stop, no map, ...
                s.last_result = f"skipped: {exc}"
                self.on_event("schedule_skipped", "warning", f"schedule '{s.name}' skipped: {exc}")
            self.on_change(s)
        return fired
