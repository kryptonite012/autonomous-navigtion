"""Colour sorting state machine.

A sort is only recorded as ``success`` when the bin sensor registers the
drop. Without a bin sensor the outcome is recorded as ``unverified``,
never as success.
"""

from __future__ import annotations

import enum
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from robot.hardware.base import BinSensor
from robot.motors.kinematics import normalize_angle
from robot.sorting.arm import ArmAborted, SortingArm
from robot.vision.detector import Detection

log = logging.getLogger(__name__)


class SortState(str, enum.Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    APPROACHING = "approaching"
    PICKING = "picking"
    VERIFYING_PICK = "verifying_pick"
    TRANSFERRING = "transferring"
    VERIFYING_SORT = "verifying_sort"
    PAUSED = "paused"
    ERROR = "error"


@dataclass
class SortRecord:
    object_id: int
    detected_color: str
    target_bin: int
    status: str            # success | failed_pickup | unverified | failed_verification | aborted | bin_full
    attempts: int
    timestamp: float = field(default_factory=time.time)
    message: str = ""

    def as_dict(self) -> dict:
        return dict(self.__dict__)


class Sorter:
    def __init__(self, cfg: dict, arm: Optional[SortingArm], bin_sensor: Optional[BinSensor],
                 detections: Callable[[], tuple[list[Detection], float]],
                 drive: Callable[[float, float], bool], stop_drive: Callable[[], None],
                 pose: Callable[[], Optional[tuple[float, float, float]]],
                 on_record: Optional[Callable[[SortRecord], None]] = None,
                 on_event: Optional[Callable[[str, str, str], None]] = None,
                 frame_size: tuple[int, int] = (640, 480),
                 front_distance: Optional[Callable[[], Optional[float]]] = None,
                 known_objects: Optional[Callable[[], list[tuple[str, float, float]]]] = None,
                 on_picked: Optional[Callable[[str], None]] = None):
        self.sc = cfg["sorting"]
        self.known_objects = known_objects or (lambda: [])
        self.on_picked = on_picked or (lambda color: None)
        self.arm = arm
        self.bin_sensor = bin_sensor
        self.detections = detections
        self.drive = drive
        self.stop_drive = stop_drive
        self.pose = pose
        self.front_distance = front_distance or (lambda: None)
        self._open_heading: Optional[tuple[float, float]] = None
        self.on_record = on_record or (lambda r: None)
        self.on_event = on_event or (lambda *a: None)
        self.frame_w, self.frame_h = frame_size
        self.bins: dict[str, int] = dict(self.sc["bins"])
        self.unclassified_bin = int(self.sc["unclassified_bin"])
        self.capacity = int(self.sc["bin_capacity"])
        self.bin_counts: dict[int, int] = {}
        self.state = SortState.IDLE
        self.message = ""
        self.target: Optional[Detection] = None
        self.history: list[SortRecord] = []
        self.stats = {"success": 0, "failed_pickup": 0, "unverified": 0, "failed_verification": 0,
                      "aborted": 0, "attempts": 0}
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._search_start_heading: Optional[float] = None
        self._search_turned = 0.0
        self._search_cycles = 0
        self._lost_frames = 0
        self.max_search_cycles = 6
        self.loop_hz = 10.0
        self.pick_confirm_s = 1.0      # how long the object must stay gone after lifting
        self.retry_settle_s = 0.5
        if arm is not None:
            arm.abort = lambda: self._stop.is_set()

    @property
    def available(self) -> bool:
        return self.arm is not None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---------------------------------------------------------- configuration
    def set_config(self, bins: dict[str, int], unclassified_bin: int, capacity: Optional[int] = None) -> None:
        valid = set(int(k) for k in self.sc["bin_angles"])
        for color, b in bins.items():
            if int(b) not in valid:
                raise ValueError(f"bin {b} for {color} has no carousel position")
        if int(unclassified_bin) not in valid:
            raise ValueError("unclassified bin has no carousel position")
        self.bins = {c: int(b) for c, b in bins.items()}
        self.unclassified_bin = int(unclassified_bin)
        if capacity is not None:
            if capacity < 1:
                raise ValueError("capacity must be >= 1")
            self.capacity = int(capacity)

    def bin_for(self, color: str) -> int:
        return self.bins.get(color, self.unclassified_bin)

    def empty_bins(self) -> None:
        self.bin_counts.clear()
        self.on_event("bins_emptied", "info", "bin counters reset by operator")

    # ---------------------------------------------------------------- control
    def start(self) -> None:
        if not self.available:
            raise RuntimeError("sorting mechanism unavailable (servo controller missing)")
        if self.running:
            if self._paused.is_set():
                self.resume()
            return
        self._stop.clear()
        self._paused.clear()
        self.state = SortState.SEARCHING
        self._reset_search()
        self._thread = threading.Thread(target=self._run, name="sorter", daemon=True)
        self._thread.start()
        self.on_event("sorting_started", "info", "sorting mode started")

    def pause(self) -> None:
        if self.running:
            self._paused.set()
            self.stop_drive()

    def resume(self) -> None:
        self._paused.clear()

    def stop(self) -> None:
        self._stop.set()
        self._paused.clear()
        self.stop_drive()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=5.0)
        self._thread = None
        if self.state != SortState.ERROR:
            self.state = SortState.IDLE
        self.target = None

    # -------------------------------------------------------------- main loop
    def _run(self) -> None:
        try:
            self.arm.home()
            while not self._stop.is_set():
                if self._paused.is_set():
                    if self.state != SortState.PAUSED:
                        self._resume_state = self.state
                        self.state = SortState.PAUSED
                    time.sleep(0.1)
                    continue
                if self.state == SortState.PAUSED:
                    self.state = getattr(self, "_resume_state", SortState.SEARCHING)
                if self.state == SortState.SEARCHING:
                    self._search_step()
                elif self.state == SortState.APPROACHING:
                    self._approach_step()
                elif self.state == SortState.PICKING:
                    self._pick_sequence()
                elif self.state in (SortState.IDLE, SortState.ERROR):
                    break
                time.sleep(1.0 / self.loop_hz)
        except ArmAborted:
            pass
        except Exception as exc:
            log.exception("sorter crashed")
            self.state = SortState.ERROR
            self.message = f"sorting error: {exc}"
            self.on_event("sorting_error", "error", self.message)
        finally:
            self.stop_drive()

    # ------------------------------------------------------------- searching
    def _candidates(self) -> list[Detection]:
        dets, stamp = self.detections()
        if time.time() - stamp > 1.0:
            return []
        out = []
        for d in dets:
            if self.bin_counts.get(self.bin_for(d.color), 0) >= self.capacity:
                continue
            out.append(d)
        return out

    def _reset_search(self) -> None:
        self._search_start_heading = None
        self._search_turned = 0.0

    def _search_step(self) -> None:
        cands = self._candidates()
        if cands:
            # largest blob first: usually the nearest object
            self.target = max(cands, key=lambda d: d.area)
            self.state = SortState.APPROACHING
            self._lost_frames = 0
            self.message = f"approaching {self.target.color} object #{self.target.object_id}"
            self.stop_drive()
            return
        full = [b for b, n in self.bin_counts.items() if n >= self.capacity]
        if full and len(full) >= len(set(self.bins.values()) | {self.unclassified_bin}):
            self._finish("all bins full")
            return
        pose = self.pose()
        if pose is not None:
            if self._search_start_heading is None:
                self._search_start_heading = pose[2]
                self._last_heading = pose[2]
            self._search_turned += abs(normalize_angle(pose[2] - self._last_heading))
            self._last_heading = pose[2]
            # remember the most open direction seen during the spin
            fd = self.front_distance()
            fd = 2.5 if fd is None else fd
            if self._open_heading is None or fd > self._open_heading[1]:
                self._open_heading = (pose[2], fd)
        if self._search_turned >= 2 * math.pi:
            self._search_cycles += 1
            self._reset_search()
            if self._search_cycles >= self.max_search_cycles:
                self._finish("no sortable objects found")
                return
            self._relocate()
            return
        self.message = "searching for objects"
        self.drive(0.0, 0.6)

    def _relocate(self) -> None:
        """Turn towards the most open direction seen and drive ~0.6 m.

        Small objects further than ~1.5 m are below the minimum blob size,
        so the robot has to move to find them. The safety controller still
        stops the drive if anything appears in front.
        """
        self.message = "moving to a new search position"
        target = self._open_heading[0] if self._open_heading else None
        self._open_heading = None
        max_dist = 0.6
        pose = self.pose()
        mapped = self._nearest_mapped(pose)
        if mapped is not None and pose is not None:
            color, mx, my = mapped
            # head for an object seen earlier (object map) instead of a blind move
            target = math.atan2(my - pose[1], mx - pose[0])
            max_dist = min(1.5, max(0.0, math.dist(pose[:2], (mx, my)) - 0.45))
            self.message = f"heading to mapped {color} object at ({mx:.2f}, {my:.2f})"
        t_end = time.time() + 4.0
        while target is not None and time.time() < t_end and not self._stop.is_set():
            pose = self.pose()
            if pose is None:
                break
            err = normalize_angle(target - pose[2])
            if abs(err) < 0.1:
                break
            self.drive(0.0, max(-0.8, min(0.8, 2.0 * err)))
            time.sleep(0.05)
        start = self.pose()
        t_end = time.time() + 6.0
        while time.time() < t_end and not self._stop.is_set():
            pose = self.pose()
            if pose is None or start is None or math.dist(pose[:2], start[:2]) > max_dist:
                break
            fd = self.front_distance()
            if fd is not None and fd < 0.35:
                break
            self.drive(0.15, 0.0)
            time.sleep(0.05)
        self.stop_drive()

    def _nearest_mapped(self, pose) -> Optional[tuple[str, float, float]]:
        if pose is None:
            return None
        best = None
        for color, x, y in self.known_objects():
            if self.bin_counts.get(self.bin_for(color), 0) >= self.capacity:
                continue
            d = math.dist(pose[:2], (x, y))
            if d > 0.3 and (best is None or d < best[0]):
                best = (d, (color, x, y))
        return best[1] if best else None

    def _finish(self, reason: str) -> None:
        self.stop_drive()
        self.message = reason
        self.state = SortState.IDLE
        self.on_event("sorting_finished", "info", reason)
        self._stop.set()

    # ------------------------------------------------------------ approaching
    def _in_pickup_zone(self, d: Detection) -> bool:
        z = self.sc["pickup_zone"]
        u = d.centroid[0] / self.frame_w
        v = (d.bbox[1] + d.bbox[3]) / self.frame_h     # bottom edge rests on the floor
        return z["x_min"] <= u <= z["x_max"] and z["y_min"] <= v <= z["y_max"] \
            and d.area >= self.sc["min_pickup_area_px"]

    def _approach_step(self) -> None:
        dets, stamp = self.detections()
        match = None
        if self.target is not None:
            same = [d for d in dets if d.object_id == self.target.object_id]
            if not same:
                same = [d for d in dets if d.color == self.target.color]
            if same:
                match = min(same, key=lambda d: math.dist(d.centroid, self.target.centroid))
        if match is None or time.time() - stamp > 1.0:
            self._lost_frames += 1
            self.stop_drive()
            if self._lost_frames > 15:
                self.message = "lost target, searching again"
                self.target = None
                self.state = SortState.SEARCHING
                self._reset_search()
            return
        self._lost_frames = 0
        self.target = match
        if self._in_pickup_zone(match):
            self.stop_drive()
            self.state = SortState.PICKING
            return
        # visual servoing: centre horizontally, then creep forward
        err_u = (match.centroid[0] - self.frame_w / 2) / (self.frame_w / 2)
        w = -1.2 * err_u
        bottom = (match.bbox[1] + match.bbox[3]) / self.frame_h
        v = 0.0 if abs(err_u) > 0.25 else (0.10 if bottom < 0.5 else 0.05)
        if not self.drive(v, w):
            self.message = "motion blocked by safety controller"

    # ---------------------------------------------------------------- picking
    def _pick_sequence(self) -> None:
        tgt = self.target
        color, oid = tgt.color, tgt.object_id or 0
        bin_id = self.bin_for(color)
        attempts = 0
        retries = int(self.sc["max_retries"])
        while attempts <= retries and not self._stop.is_set():
            attempts += 1
            self.stats["attempts"] += 1
            self.state = SortState.PICKING
            self.message = f"picking {color} object (attempt {attempts})"
            self.arm.pick()
            self.state = SortState.VERIFYING_PICK
            if self._pick_confirmed(tgt):
                self.on_picked(color)
                break
            self.arm.release()
            self.arm.home()
            self.on_event("pickup_failed", "warning", f"{color} object still on the floor after attempt {attempts}")
            if attempts > retries:
                self._record(SortRecord(oid, color, bin_id, "failed_pickup", attempts,
                                        message="object not lifted"))
                self.state = SortState.SEARCHING
                self._reset_search()
                return
            # re-acquire before retrying
            time.sleep(self.retry_settle_s)
            dets, _ = self.detections()
            same = [d for d in dets if d.color == color and self._in_pickup_zone(d)]
            if not same:
                self.target = None
                self.state = SortState.SEARCHING
                self._record(SortRecord(oid, color, bin_id, "failed_pickup", attempts,
                                        message="object moved out of reach"))
                return
            tgt = same[0]
        else:
            return

        self.state = SortState.TRANSFERRING
        self.message = f"moving {color} object to bin {bin_id}"
        before = self.bin_sensor.event_count() if self.bin_sensor else None
        self.arm.select_bin(bin_id)
        self.arm.release()
        self.state = SortState.VERIFYING_SORT
        status, msg = self._verify_drop(before)
        self.arm.home()
        if status == "success" or status == "unverified":
            self.bin_counts[bin_id] = self.bin_counts.get(bin_id, 0) + 1
            if self.bin_counts[bin_id] >= self.capacity:
                self.on_event("bin_full", "warning", f"bin {bin_id} is full")
        self._record(SortRecord(oid, color, bin_id, status, attempts, message=msg))
        self.target = None
        self.state = SortState.SEARCHING
        self._reset_search()
        self._search_cycles = 0

    def _pick_confirmed(self, tgt: Detection) -> bool:
        """After lifting, the object must have left the pickup zone in the image."""
        deadline = time.time() + self.pick_confirm_s
        clear_frames = 0
        while time.time() < deadline:
            dets, _ = self.detections()
            still = [d for d in dets if d.color == tgt.color and self._in_pickup_zone(d)]
            clear_frames = 0 if still else clear_frames + 1
            if clear_frames >= 3:
                return True
            time.sleep(0.08)
        return False

    def _verify_drop(self, before: Optional[int]) -> tuple[str, str]:
        if self.bin_sensor is None or before is None:
            return "unverified", "no bin sensor installed: drop not confirmed"
        deadline = time.time() + float(self.sc["verify_timeout_s"])
        while time.time() < deadline:
            if self.bin_sensor.event_count() > before:
                return "success", "bin sensor confirmed drop"
            time.sleep(0.05)
        return "failed_verification", "bin sensor did not register the object"

    def _record(self, rec: SortRecord) -> None:
        self.stats[rec.status] = self.stats.get(rec.status, 0) + 1
        self.history.append(rec)
        self.history = self.history[-500:]
        sev = "info" if rec.status == "success" else "warning"
        self.on_event("sort_" + rec.status, sev,
                      f"{rec.detected_color} -> bin {rec.target_bin}: {rec.status} ({rec.message})")
        self.on_record(rec)

    def status(self) -> dict:
        attempts_done = sum(self.stats[k] for k in ("success", "failed_pickup", "unverified", "failed_verification"))
        return {
            "available": self.available,
            "running": self.running,
            "state": self.state.value,
            "message": self.message,
            "target": None if self.target is None else self.target.as_dict(),
            "bins": self.bins,
            "unclassified_bin": self.unclassified_bin,
            "bin_capacity": self.capacity,
            "bin_counts": {str(k): v for k, v in self.bin_counts.items()},
            "statistics": dict(self.stats),
            "success_rate": round(self.stats["success"] / attempts_done, 3) if attempts_done else None,
            "verification": "bin_sensor" if self.bin_sensor else "none",
        }
