"""Robot runtime: owns the hardware, runs the control loop and exposes a
thread-safe API used by the web backend and the CLI tools."""

from __future__ import annotations

import enum
import logging
import math
import threading
import time
from collections import deque
from typing import Callable, Optional

from robot import system_metrics
from robot.events import Event, EventBus
from robot.hardware.base import HealthState, RangeReading
from robot.hardware.factory import HardwareSet, build_hardware
from robot.mapping.mapper import Mapper
from robot.motors.kinematics import DiffDriveKinematics, WheelOdometry
from robot.navigation.navigator import NavState, Navigator
from robot.safety.controller import SafetyConfig, SafetyController
from robot.sorting.arm import SortingArm
from robot.sorting.sorter import Sorter
from robot.vision.pipeline import VisionPipeline

log = logging.getLogger(__name__)


class Mode(str, enum.Enum):
    IDLE = "idle"
    MANUAL = "manual"
    MAPPING = "mapping"
    NAVIGATION = "navigation"
    SORTING = "sorting"


class CommandRejected(Exception):
    """A command was refused because the robot is not in a safe/compatible state."""


class Robot:
    def __init__(self, cfg: dict, hardware: Optional[HardwareSet] = None):
        self.cfg = cfg
        self.events = EventBus()
        self.hw = hardware or build_hardware(cfg)
        self.simulated = self.hw.mode == "simulation"
        self.caps = self.hw.capabilities()
        ch = cfg["chassis"]
        self.kin = DiffDriveKinematics(ch["wheel_radius_m"], ch["wheel_base_m"], ch["max_linear_mps"] * 1.6)
        self.odom = WheelOdometry(self.kin, ch["encoder_ticks_per_rev"],
                                  gyro_weight=0.85 if self.hw.imu else 0.0)
        self.safety = SafetyController(self.hw.motors, self.kin, SafetyConfig.from_config(cfg), self.hw.estop,
                                       on_event=self._event, expected_sensors=[r.name for r in self.hw.ranges])
        self.mapper = Mapper(cfg, self.hw.lidar is not None, bool(self.hw.ranges), ch["robot_radius_m"])
        self.navigator = Navigator(cfg, lambda: self.mapper.grid, on_event=self._event)
        self.vision = VisionPipeline(self.hw.camera, cfg)
        arm = SortingArm(self.hw.servos, cfg["sorting"]) if self.hw.servos else None
        self.sorter = Sorter(cfg, arm, self.hw.bin_sensor, self.vision.latest,
                             drive=lambda v, w: self.safety.command(v, w, "sorting"),
                             stop_drive=lambda: self.safety.stop("sorting"),
                             pose=self._pose_if_valid, on_event=self._event,
                             frame_size=(cfg["camera"]["width"], cfg["camera"]["height"]),
                             front_distance=lambda: self.safety.status.obstacle_front_m)
        self.mode = Mode.IDLE
        self.rate = cfg["robot"]["control_rate_hz"]
        self.ranges: list[RangeReading] = []
        self.imu_reading = None
        self.battery_v: Optional[float] = None
        self.pose_time = 0.0
        self.pose_valid = False
        self.loop_hz = 0.0
        self.loop_overruns = 0
        self.started_at = time.time()
        self.telemetry_history: deque[dict] = deque(maxlen=600)
        self._metrics = system_metrics.snapshot()
        self._metrics_t = 0.0
        self._lock = threading.RLock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._sim_clock = None
        self._teleop_time = 0.0
        self._last_scan = None

    # ----------------------------------------------------------------- events
    def _event(self, type_: str, severity: str, description: str) -> None:
        self.events.publish(type_, severity, description)

    def subscribe(self, fn: Callable[[Event], None]) -> None:
        self.events.subscribe(fn)

    # -------------------------------------------------------------- lifecycle
    def start(self) -> None:
        if self._running:
            return
        if self.simulated:
            from robot.simulation.world import SimClock
            self._sim_clock = SimClock(self.hw.world)
            self._sim_clock.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="control", daemon=True)
        self._thread.start()
        if self.vision.available:
            self.vision.start()
        missing = [h for h in self.hw.health() if h.state in (HealthState.MISSING, HealthState.FAULT)]
        self._event("robot_started", "info",
                    f"robot started in {self.hw.mode} mode; "
                    f"{len(self.hw.devices())} devices ready, {len(missing)} missing")
        for h in missing:
            self._event("hardware_missing", "warning", f"{h.name}: {h.message}")

    def shutdown(self) -> None:
        self.sorter.stop()
        self.navigator.cancel("shutdown")
        self.safety.stop("shutdown")
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.vision.stop()
        if self._sim_clock:
            self._sim_clock.stop()
        self.hw.shutdown()
        self._event("robot_stopped", "info", "robot runtime stopped")

    # ------------------------------------------------------------ control loop
    def _loop(self) -> None:
        period = 1.0 / self.rate
        next_t = time.monotonic()
        last = time.monotonic()
        while self._running:
            try:
                self._step()
            except Exception as exc:
                log.exception("control step failed")
                self.safety.stop("control-loop-error")
                self._event("control_error", "error", f"control loop error: {exc}")
            now = time.monotonic()
            self.loop_hz = 0.9 * self.loop_hz + 0.1 / max(1e-3, now - last)
            last = now
            next_t += period
            delay = next_t - now
            if delay < 0:
                self.loop_overruns += 1
                next_t = now
            else:
                time.sleep(delay)

    def _step(self) -> None:
        now = time.time()
        # --- sensing
        gyro_z = None
        if self.hw.imu:
            try:
                self.imu_reading = self.hw.imu.read()
                gyro_z = self.imu_reading.gyro[2]
            except Exception:
                self.imu_reading = None
        if self.hw.encoders:
            try:
                ticks = self.hw.encoders.ticks()
                with self._lock:
                    self.odom.update(ticks, now, gyro_z)
                    self.pose_time = now
                    self.pose_valid = True
                self.safety.update_encoders(ticks)
            except Exception as exc:
                self.pose_valid = False
                self._event("encoder_error", "error", f"encoder read failed: {exc}")
        ranges = []
        for s in self.hw.ranges:
            try:
                ranges.append(s.read())
            except Exception as exc:
                ranges.append(RangeReading(s.name, None, s.angle_rad, now, False))
                log.debug("range sensor %s failed: %s", s.name, exc)
        self.ranges = ranges
        self.safety.update_ranges(ranges)
        if self.hw.battery and (now - getattr(self, "_bat_t", 0)) > 1.0:
            self._bat_t = now
            self.battery_v = self.hw.battery.voltage()
            self.safety.update_battery(self.battery_v)
        scan = self.hw.lidar.scan() if self.hw.lidar else None
        if scan is not None and scan.timestamp != getattr(self._last_scan, "timestamp", None):
            self._last_scan = scan
        else:
            scan = None

        pose = self._pose_if_valid()
        # --- mapping
        if pose is not None and self.mapper.active:
            self.mapper.update(pose, ranges, scan)

        # --- navigation
        if self.navigator.active:
            twist = self.navigator.step(pose, self.pose_time, ranges)
            if twist is None:
                self.safety.stop("navigation")
            else:
                self.safety.command(twist[0], twist[1], "navigation")
            if not self.navigator.active and self.mode == Mode.NAVIGATION:
                self.mode = Mode.IDLE
            if not self.navigator.active and self.mode == Mode.MAPPING and self.navigator.explore_done:
                self._event("mapping_exploration_done", "info", "exploration finished, map can be saved")
        elif self.mode == Mode.SORTING:
            if not self.sorter.running:
                self.mode = Mode.IDLE
        else:
            # manual mode: commands come from teleop; the safety watchdog stops
            # the motors if the operator's stream of commands stops
            self.safety.tick()

        if now - self._metrics_t > 2.0:
            self._metrics_t = now
            self._metrics = system_metrics.snapshot()
            t = self._metrics.get("cpu_temperature")
            if t is not None and t > 80:
                self._event("cpu_hot", "warning", f"CPU temperature {t:.1f} C")
        if not self.telemetry_history or now - self.telemetry_history[-1]["timestamp"] >= 0.5:
            self.telemetry_history.append(self.telemetry())

    def _pose_if_valid(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            if not self.pose_valid or time.time() - self.pose_time > self.cfg["safety"]["pose_timeout_s"]:
                return None
            return self.odom.pose.as_tuple()

    # --------------------------------------------------------------- commands
    def _require(self, cap: str, what: str) -> None:
        if not self.caps.get(cap):
            raise CommandRejected(f"{what} unavailable: required hardware for '{cap}' is missing")

    def _require_motion_ok(self) -> None:
        st = self.safety.status
        if st.estop_hardware or st.estop_software:
            raise CommandRejected("emergency stop is active; reset it first")
        if st.faults:
            raise CommandRejected("safety fault active: " + ", ".join(st.faults))

    def teleop(self, v: float, w: float) -> dict:
        self._require("drive", "manual driving")
        if self.mode not in (Mode.IDLE, Mode.MANUAL, Mode.MAPPING) or self.navigator.active:
            raise CommandRejected(f"manual control not allowed while in {self.mode.value} mode")
        self._require_motion_ok()
        if self.mode == Mode.IDLE:
            self.mode = Mode.MANUAL
        self._teleop_time = time.time()
        self.safety.command(v, w, "manual")
        return self.safety.status.as_dict()

    def stop(self) -> None:
        self.navigator.cancel("stop requested")
        self.sorter.stop()
        self.safety.stop("operator")
        if self.mode != Mode.MAPPING:
            self.mode = Mode.IDLE

    def emergency_stop(self, reason: str = "operator emergency stop") -> None:
        # motors first, everything else after
        self.safety.emergency_stop(reason)
        self.navigator.cancel("emergency stop")
        threading.Thread(target=self.sorter.stop, daemon=True).start()
        self.mode = Mode.IDLE

    def reset_emergency_stop(self) -> tuple[bool, str]:
        return self.safety.reset_emergency_stop()

    def set_manual(self) -> None:
        if self.navigator.active or self.sorter.running:
            raise CommandRejected("stop the active task before switching to manual")
        self.mode = Mode.MANUAL

    # navigation
    def navigate_to(self, x: float, y: float, theta: Optional[float] = None) -> None:
        self._prepare_navigation()
        self.navigator.set_goal(x, y, theta)
        self.mode = Mode.NAVIGATION

    def navigate_to_named(self, name: str) -> None:
        self._prepare_navigation()
        self.navigator.go_to_named(name)
        self.mode = Mode.NAVIGATION

    def _prepare_navigation(self) -> None:
        self._require("navigation", "navigation")
        self._require_motion_ok()
        if self.sorter.running:
            raise CommandRejected("sorting is running")
        if self._pose_if_valid() is None:
            raise CommandRejected("no valid pose estimate")
        if self.mapper.grid.updates == 0 and self.mapper.loaded_name is None and not self.mapper.active:
            raise CommandRejected("no map: start mapping or load a saved map first")

    def pause_navigation(self) -> bool:
        ok = self.navigator.pause()
        self.safety.stop("navigation-pause")
        return ok

    def resume_navigation(self) -> bool:
        self._require_motion_ok()
        return self.navigator.resume()

    # mapping
    def start_mapping(self, explore: bool = True, reset: bool = False) -> None:
        if not self.mapper.available:
            raise CommandRejected("mapping unavailable: no range sensors")
        if not self.caps.get("odometry"):
            raise CommandRejected("mapping unavailable: wheel encoders required for pose estimation")
        if self.sorter.running:
            raise CommandRejected("sorting is running")
        self.mapper.start(reset=reset)
        self.mode = Mode.MAPPING
        if self.mapper.experimental:
            self._event("mapping_experimental", "warning",
                        "no LiDAR: mapping from sparse range sensors + odometry is EXPERIMENTAL "
                        "(no loop closure, drift accumulates)")
        if explore:
            self._require_motion_ok()
            self.navigator.start_exploration()

    def stop_mapping(self) -> None:
        self.mapper.stop()
        if self.navigator.exploring:
            self.navigator.cancel("mapping stopped")
        self.safety.stop("mapping")
        self.mode = Mode.IDLE

    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        if self.navigator.active:
            raise CommandRejected("cannot reset pose while navigating")
        with self._lock:
            self.odom.reset(x, y, theta)

    # sorting
    def start_sorting(self) -> None:
        self._require("sorting", "sorting")
        self._require_motion_ok()
        if self.navigator.active:
            raise CommandRejected("navigation is running")
        if not self.vision.running:
            self.vision.start()
        self.sorter.start()
        self.mode = Mode.SORTING

    def stop_sorting(self) -> None:
        self.sorter.stop()
        self.safety.stop("sorting")
        self.mode = Mode.IDLE

    # ------------------------------------------------------------------ state
    def pose(self) -> dict:
        with self._lock:
            p = self.odom.pose
            return {"x": round(p.x, 4), "y": round(p.y, 4), "theta": round(p.theta, 4),
                    "valid": self._pose_if_valid() is not None, "source": "wheel_odometry+imu"
                    if self.hw.imu else "wheel_odometry", "timestamp": self.pose_time}

    def telemetry(self) -> dict:
        imu = self.imu_reading
        st = self.safety.status
        return {
            "timestamp": time.time(),
            "linear_velocity": round(self.odom.v, 4),
            "angular_velocity": round(self.odom.w, 4),
            "commanded": list(st.applied),
            "battery_voltage": None if self.battery_v is None else round(self.battery_v, 3),
            "battery_percent": self.battery_percent(),
            "ranges": {r.sensor: (None if r.distance_m is None else round(r.distance_m, 3)) for r in self.ranges},
            "ranges_valid": {r.sensor: r.valid for r in self.ranges},
            "imu": None if imu is None else {"accel": [round(a, 3) for a in imu.accel],
                                             "gyro": [round(g, 4) for g in imu.gyro],
                                             "temperature": imu.temperature_c},
            "cpu_temperature": self._metrics.get("cpu_temperature"),
            "cpu_percent": self._metrics.get("cpu_percent"),
            "ram_percent": self._metrics.get("ram_percent"),
            "loop_hz": round(self.loop_hz, 1),
            "pose": self.pose(),
            "motor_status": "fault" if any(k in st.faults for k in ("motor_communication", "drive_stall"))
            else ("disabled" if not st.motion_allowed else "ok"),
        }

    def battery_percent(self) -> Optional[int]:
        if self.battery_v is None:
            return None
        cells = self.cfg["hardware"]["battery"].get("cells", 2)
        per = self.battery_v / cells
        # rough Li-ion curve: 3.3 V empty, 4.15 V full
        return int(max(0, min(100, (per - 3.3) / (4.15 - 3.3) * 100)))

    def state(self) -> dict:
        return {
            "name": self.cfg["robot"]["name"],
            "hardware_mode": self.hw.mode,
            "simulated": self.simulated,
            "mode": self.mode.value,
            "uptime_s": round(time.time() - self.started_at, 1),
            "pose": self.pose(),
            "safety": self.safety.status.as_dict(),
            "navigation": self.navigator.status(),
            "mapping": self.mapper.status(),
            "vision": self.vision.status(),
            "sorting": self.sorter.status(),
            "capabilities": self.caps,
        }

    def health(self) -> list[dict]:
        return [h.as_dict() for h in self.hw.health()]

    def ground_truth(self) -> Optional[dict]:
        """Simulation only: the true pose, for comparing against odometry."""
        if not self.simulated:
            return None
        x, y, th = self.hw.world.true_pose()
        return {"x": round(x, 4), "y": round(y, 4), "theta": round(th, 4)}
