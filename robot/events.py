"""Small thread-safe publish/subscribe event bus used inside the robot process."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable

log = logging.getLogger(__name__)

SEVERITIES = ("debug", "info", "warning", "error", "critical")


@dataclass
class Event:
    type: str
    severity: str
    description: str
    timestamp: float = field(default_factory=time.time)
    data: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"type": self.type, "severity": self.severity, "description": self.description,
                "timestamp": self.timestamp, "data": self.data}


class EventBus:
    def __init__(self, history: int = 500):
        self._subs: list[Callable[[Event], None]] = []
        self._lock = threading.Lock()
        self.recent: deque[Event] = deque(maxlen=history)

    def subscribe(self, fn: Callable[[Event], None]) -> None:
        with self._lock:
            self._subs.append(fn)

    def publish(self, type_: str, severity: str = "info", description: str = "", **data) -> Event:
        if severity not in SEVERITIES:
            severity = "info"
        ev = Event(type_, severity, description, data=data)
        with self._lock:
            self.recent.append(ev)
            subs = list(self._subs)
        level = {"debug": 10, "info": 20, "warning": 30, "error": 40, "critical": 50}[severity]
        log.log(level, "[%s] %s", type_, description)
        for fn in subs:
            try:
                fn(ev)
            except Exception:
                log.exception("event subscriber failed")
        return ev
