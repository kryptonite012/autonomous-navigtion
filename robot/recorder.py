"""Flight recorder: raw sensor logs for offline replay and debugging.

Records, every control cycle, what the algorithms consumed (encoder ticks,
gyro, range readings, LiDAR scans), what they produced (pose estimates,
commanded and applied twist) and, in simulation, the ground truth. Lines are
JSON, gzip compressed: ~1-3 MB per minute with a LiDAR.

Typical use: record a run on the robot, download it from the dashboard, then
``python3 scripts/replay.py run.jsonl.gz`` on a laptop to re-run odometry,
mapping, scan matching and localisation with different parameters.
"""

from __future__ import annotations

import gzip
import json
import re
import threading
import time
from pathlib import Path
from typing import Iterator, Optional

from robot.config import resolve_path

NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_\-]{0,63}$")
FORMAT_VERSION = 1


class Recorder:
    def __init__(self, directory: str = "data/recordings", max_seconds: float = 1800.0):
        self.dir = resolve_path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.max_seconds = max_seconds
        self._fh: Optional[gzip.GzipFile] = None
        self._lock = threading.Lock()
        self.name: Optional[str] = None
        self.started: Optional[float] = None
        self.records = 0
        self.scans = 0

    @property
    def active(self) -> bool:
        return self._fh is not None

    def path(self, name: str) -> Path:
        if not NAME_RE.match(name or ""):
            raise ValueError("recording name: 1-64 letters, digits, '_' or '-'")
        return self.dir / f"{name}.jsonl.gz"

    def start(self, name: str, meta: dict) -> Path:
        p = self.path(name)
        with self._lock:
            if self._fh is not None:
                raise RuntimeError(f"already recording '{self.name}'")
            if p.exists():
                raise FileExistsError(f"recording '{name}' exists")
            self._fh = gzip.open(p, "wt", encoding="utf-8", compresslevel=5)
            self.name, self.started, self.records, self.scans = name, time.time(), 0, 0
            self._write({"type": "meta", "format": FORMAT_VERSION, "t": self.started, **meta})
        return p

    def _write(self, rec: dict) -> None:
        self._fh.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def log(self, rec: dict) -> None:
        with self._lock:
            if self._fh is None:
                return
            self._write(rec)
            self.records += 1
            if rec.get("scan") is not None:
                self.scans += 1
            if time.time() - self.started > self.max_seconds:
                self._close()

    def event(self, type_: str, severity: str, description: str) -> None:
        self.log({"type": "event", "t": time.time(), "event": type_, "severity": severity, "description": description})

    def stop(self) -> Optional[str]:
        with self._lock:
            return self._close()

    def _close(self) -> Optional[str]:
        if self._fh is None:
            return None
        self._write({"type": "end", "t": time.time(), "records": self.records})
        self._fh.close()
        self._fh = None
        name, self.name = self.name, None
        return name

    def list(self) -> list[dict]:
        out = []
        for p in sorted(self.dir.glob("*.jsonl.gz"), key=lambda f: -f.stat().st_mtime):
            out.append({"name": p.name[:-len(".jsonl.gz")], "size_bytes": p.stat().st_size,
                        "modified": p.stat().st_mtime, "recording": p.name[:-len(".jsonl.gz")] == self.name})
        return out

    def delete(self, name: str) -> None:
        p = self.path(name)
        if name == self.name:
            raise RuntimeError("stop the recording first")
        if not p.exists():
            raise FileNotFoundError(name)
        p.unlink()

    def status(self) -> dict:
        return {"active": self.active, "name": self.name, "records": self.records, "scans": self.scans,
                "duration_s": round(time.time() - self.started, 1) if self.active and self.started else None,
                "max_seconds": self.max_seconds}


def read(path: str | Path) -> Iterator[dict]:
    with gzip.open(path, "rt", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                yield json.loads(line)
