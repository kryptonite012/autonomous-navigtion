"""Connects the robot runtime to persistence and the WebSocket hub."""

from __future__ import annotations

import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Optional

from sqlalchemy import delete, select

from backend.database.db import Database
from backend.database.models import (MapRecord, MissionRecord, ObjectDetection, RobotSession,
                                     ScheduleRecord, SortingRecord, SystemEvent, Telemetry)
from backend.websocket.hub import Hub
from robot.events import Event
from robot.navigation.missions import Mission
from robot.navigation.scheduler import Schedule
from robot.runtime import Robot
from robot.sorting.sorter import SortRecord
from robot.vision.detector import Detection

log = logging.getLogger(__name__)

ALERT_SEVERITIES = ("warning", "error", "critical")


class RobotService:
    def __init__(self, robot: Robot, db: Database, hub: Hub, telemetry_db_interval_s: float = 5.0,
                 telemetry_retention_days: int = 14):
        self.robot = robot
        self.db = db
        self.hub = hub
        self.telemetry_db_interval = telemetry_db_interval_s
        self.retention_days = telemetry_retention_days
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._session_id: Optional[int] = None
        self._last_mode = None
        self._logged_ids: set[int] = set()
        robot.subscribe(self._on_event)
        robot.vision.add_listener(self._on_detections)
        robot.sorter.on_record = self._on_sort_record
        self._load_missions()
        self._load_schedules()
        robot.scheduler.on_change = self._store_schedule

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        self.robot.start()
        self._running = True
        self._thread = threading.Thread(target=self._publisher, name="ws-publisher", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close_session("stopped")
        self.robot.shutdown()

    # ------------------------------------------------------------ callbacks
    def _on_event(self, ev: Event) -> None:
        try:
            with self.db.session() as s:
                s.add(SystemEvent(event_type=ev.type, severity=ev.severity, description=ev.description[:2000],
                                  timestamp=datetime.fromtimestamp(ev.timestamp, timezone.utc)))
        except Exception:
            log.exception("failed to store event")
        self.hub.publish("events", ev.as_dict())
        if ev.severity in ALERT_SEVERITIES:
            self.hub.publish("alerts", ev.as_dict())

    def _on_detections(self, dets: list[Detection]) -> None:
        # only the first sighting of each tracked object goes to the database;
        # every frame would flood it for no benefit
        new = [d for d in dets if d.object_id not in self._logged_ids]
        if new:
            try:
                with self.db.session() as s:
                    for d in new:
                        self._logged_ids.add(d.object_id)
                        s.add(ObjectDetection(object_id=d.object_id, color=d.color, confidence=d.confidence,
                                              centroid_x=d.centroid[0], centroid_y=d.centroid[1], area=d.area,
                                              timestamp=datetime.fromtimestamp(d.timestamp, timezone.utc)))
            except Exception:
                log.exception("failed to store detections")
            if len(self._logged_ids) > 100_000:
                self._logged_ids.clear()

    def _on_sort_record(self, rec: SortRecord) -> None:
        try:
            with self.db.session() as s:
                s.add(SortingRecord(object_id=rec.object_id, detected_color=rec.detected_color,
                                    target_bin=rec.target_bin, sorting_status=rec.status, attempts=rec.attempts,
                                    message=rec.message,
                                    timestamp=datetime.fromtimestamp(rec.timestamp, timezone.utc)))
        except Exception:
            log.exception("failed to store sorting record")
        self.hub.publish("sorting", {"record": rec.as_dict(), "status": self.robot.sorter.status()})

    # -------------------------------------------------------------- sessions
    def _track_session(self) -> None:
        mode = self.robot.mode.value
        if mode == self._last_mode:
            return
        self._close_session("completed")
        self._last_mode = mode
        if mode == "idle":
            return
        with self.db.session() as s:
            rs = RobotSession(operating_mode=mode, hardware_mode=self.robot.hw.mode)
            s.add(rs)
            s.flush()
            self._session_id = rs.id

    def _close_session(self, status: str) -> None:
        if self._session_id is None:
            return
        try:
            with self.db.session() as s:
                rs = s.get(RobotSession, self._session_id)
                if rs:
                    rs.end_time = datetime.now(timezone.utc)
                    rs.session_status = "emergency_stopped" if self.robot.safety.status.estop_software else status
        except Exception:
            log.exception("failed to close robot session")
        self._session_id = None

    # ---------------------------------------------------------- publishing
    def _publisher(self) -> None:
        rate = self.robot.cfg["robot"].get("telemetry_rate_hz", 10)
        period = 1.0 / rate
        last_db = 0.0
        last_map = 0.0
        last_state = 0.0
        last_prune = 0.0
        while self._running:
            t0 = time.time()
            try:
                tel = self.robot.telemetry()
                tel["latency_ref"] = t0
                self.hub.publish("telemetry", tel)
                self.hub.publish("pose", self.robot.pose())
                dets, ts = self.robot.vision.latest()
                self.hub.publish("detections", {"timestamp": ts, "items": [d.as_dict() for d in dets],
                                                "stats": self.robot.vision.status()})
                self.hub.publish("navigation", self.robot.navigator.status())
                if t0 - last_state > 1.0:
                    last_state = t0
                    self.hub.publish("state", self.robot.state())
                    self.hub.publish("sorting", {"status": self.robot.sorter.status()})
                    self._track_session()
                if (self.robot.mapper.active or self.robot.navigator.active) and t0 - last_map > 1.0:
                    last_map = t0
                    self.hub.publish("map", self.robot.mapper.snapshot())
                if t0 - last_db > self.telemetry_db_interval:
                    last_db = t0
                    self._store_telemetry(tel)
                if t0 - last_prune > 3600:
                    last_prune = t0
                    self._prune()
            except Exception:
                log.exception("publisher step failed")
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _store_telemetry(self, tel: dict) -> None:
        pose = tel.get("pose") or {}
        with self.db.session() as s:
            s.add(Telemetry(battery_voltage=tel.get("battery_voltage"), linear_velocity=tel["linear_velocity"],
                            angular_velocity=tel["angular_velocity"], cpu_temperature=tel.get("cpu_temperature"),
                            pose_x=pose.get("x"), pose_y=pose.get("y"), pose_theta=pose.get("theta")))

    def _prune(self) -> None:
        cutoff = datetime.fromtimestamp(time.time() - self.retention_days * 86400, timezone.utc)
        with self.db.session() as s:
            s.execute(delete(Telemetry).where(Telemetry.timestamp < cutoff))
            s.execute(delete(ObjectDetection).where(ObjectDetection.timestamp < cutoff))

    # -------------------------------------------------------------- missions
    def _load_missions(self) -> None:
        with self.db.session() as s:
            for rec in s.execute(select(MissionRecord)).scalars():
                try:
                    self.robot.mission_library[rec.name] = Mission.from_dict(json.loads(rec.definition))
                except (ValueError, KeyError, TypeError) as exc:
                    log.warning("skipping invalid stored mission %s: %s", rec.name, exc)

    def save_mission(self, mission: Mission) -> None:
        mission.validate(self.robot.navigator.named_locations)
        data = json.dumps(mission.as_dict())
        with self.db.session() as s:
            rec = s.execute(select(MissionRecord).where(MissionRecord.name == mission.name)).scalar_one_or_none()
            if rec is None:
                s.add(MissionRecord(name=mission.name, definition=data))
            else:
                rec.definition = data
                rec.updated_at = datetime.now(timezone.utc)
        self.robot.mission_library[mission.name] = mission

    def delete_mission(self, name: str) -> bool:
        with self.db.session() as s:
            n = s.execute(delete(MissionRecord).where(MissionRecord.name == name)).rowcount
        return self.robot.mission_library.pop(name, None) is not None or n > 0

    # ------------------------------------------------------------- schedules
    def _load_schedules(self) -> None:
        with self.db.session() as s:
            for rec in s.execute(select(ScheduleRecord)).scalars():
                try:
                    self.robot.scheduler.schedules[rec.name] = Schedule.from_dict(json.loads(rec.definition))
                except (ValueError, KeyError, TypeError) as exc:
                    log.warning("skipping invalid stored schedule %s: %s", rec.name, exc)

    def _store_schedule(self, sched: Schedule) -> None:
        data = json.dumps(sched.as_dict())
        with self.db.session() as s:
            rec = s.execute(select(ScheduleRecord).where(ScheduleRecord.name == sched.name)).scalar_one_or_none()
            if rec is None:
                s.add(ScheduleRecord(name=sched.name, definition=data))
            else:
                rec.definition = data
                rec.updated_at = datetime.now(timezone.utc)

    def save_schedule(self, sched: Schedule) -> None:
        sched.validate(set(self.robot.mission_library))
        self.robot.scheduler.schedules[sched.name] = sched
        self._store_schedule(sched)

    def delete_schedule(self, name: str) -> bool:
        with self.db.session() as s:
            n = s.execute(delete(ScheduleRecord).where(ScheduleRecord.name == name)).rowcount
        return self.robot.scheduler.schedules.pop(name, None) is not None or n > 0

    # ----------------------------------------------------------------- maps
    def register_map(self, name: str, path: str, resolution: float, experimental: bool) -> None:
        with self.db.session() as s:
            rec = s.execute(select(MapRecord).where(MapRecord.map_name == name)).scalar_one_or_none()
            if rec is None:
                s.add(MapRecord(map_name=name, file_path=path, resolution=resolution, experimental=experimental))
            else:
                rec.file_path, rec.resolution, rec.experimental = path, resolution, experimental
                rec.created_at = datetime.now(timezone.utc)

    def rename_map(self, old: str, new: str, path: str) -> None:
        with self.db.session() as s:
            rec = s.execute(select(MapRecord).where(MapRecord.map_name == old)).scalar_one_or_none()
            if rec:
                rec.map_name, rec.file_path = new, path

    def delete_map(self, name: str) -> None:
        with self.db.session() as s:
            s.execute(delete(MapRecord).where(MapRecord.map_name == name))
