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
from robot.localization.ekf import OdometryEKF
from robot.localization.localizer import Localizer
from robot.mapping.scan_matcher import ScanMatcher
from robot.mapping.mapper import Mapper
from robot.motors.kinematics import DiffDriveKinematics
from robot.navigation.missions import Mission, MissionRunner
from robot.navigation.navigator import NavState, Navigator
from robot.safety.controller import SafetyConfig, SafetyController
from robot.sorting.arm import SortingArm
from robot.sorting.sorter import Sorter
from robot.vision.object_map import ObjectMap
from robot.vision.pipeline import VisionPipeline

log = logging.getLogger(__name__)


class Mode(str, enum.Enum):
    IDLE = "idle"
    MANUAL = "manual"
    MAPPING = "mapping"
    NAVIGATION = "navigation"
    MISSION = "mission"
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
        # odometry EKF (encoders + gyro) in the odom frame; the localiser adds
        # the map correction when a map is loaded
        self.odom = OdometryEKF(self.kin, ch["encoder_ticks_per_rev"])
        lc = cfg.get("localization", {})
        self.localizer = Localizer(cfg, self.hw.lidar is not None,
                                   lc.get("range_sensor_offset_m", ch["robot_radius_m"] * 0.8), on_event=self._event)
        self.safety = SafetyController(self.hw.motors, self.kin, SafetyConfig.from_config(cfg), self.hw.estop,
                                       on_event=self._event, expected_sensors=[r.name for r in self.hw.ranges])
        self.mapper = Mapper(cfg, self.hw.lidar is not None, bool(self.hw.ranges), ch["robot_radius_m"])
        mc = cfg["mapping"]
        # LiDAR mapping corrects odometry drift by aligning each scan with the map so far
        self.scan_matcher = ScanMatcher() if (self.hw.lidar is not None and mc.get("scan_matching", True)) else None
        self._last_match_odom: Optional[tuple[float, float, float]] = None
        self.navigator = Navigator(cfg, lambda: self.mapper.grid, on_event=self._event)
        nc = cfg["navigation"]
        self.missions = MissionRunner(self.navigator, nc.get("home_location", "home"), on_event=self._event)
        self.mission_library: dict[str, Mission] = {}
        self.vision = VisionPipeline(self.hw.camera, cfg)
        if self.simulated and self.vision.available:
            # the synthetic camera is rendered with exactly these intrinsics
            # and mount values, so in simulation it is calibrated by construction
            self.vision.detector.calibrated = True
        self.objects = ObjectMap(camera_forward_m=cfg["camera"].get("mount_forward_m", 0.0))
        self.vision.add_listener(lambda dets: self.objects.update(dets, self._pose_if_valid())
                                 if self.vision.detector.calibrated else None)
        arm = SortingArm(self.hw.servos, cfg["sorting"]) if self.hw.servos else None
        self.sorter = Sorter(cfg, arm, self.hw.bin_sensor, self.vision.latest,
                             drive=lambda v, w: self.safety.command(v, w, "sorting"),
                             stop_drive=lambda: self.safety.stop("sorting"),
                             pose=self._pose_if_valid, on_event=self._event,
                             frame_size=(cfg["camera"]["width"], cfg["camera"]["height"]),
                             front_distance=lambda: self.safety.status.obstacle_front_m,
                             known_objects=lambda: [(o.color, o.x, o.y) for o in self.objects.candidates()],
                             on_picked=self._on_picked)
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
        self._returning_low_battery = False

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
            self._check_low_battery()
        scan = self.hw.lidar.scan() if self.hw.lidar else None
        if scan is not None and scan.timestamp != getattr(self._last_scan, "timestamp", None):
            self._last_scan = scan
        else:
            scan = None

        # --- localisation (only against a loaded map, never while mapping)
        if self.pose_valid and self.localizer.active and not self.mapper.active:
            self.localizer.update(self.odom.pose.as_tuple(), ranges, scan)

        pose = self._pose_if_valid()
        # --- mapping (with LiDAR scan matching to correct the pose first)
        if pose is not None and self.mapper.active:
            if scan is not None and self.scan_matcher is not None:
                pose = self._scan_match(pose, scan)
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
        self.missions.step()
        if self.mode == Mode.MISSION and not self.missions.running:
            self.mode = Mode.IDLE

        if now - self._metrics_t > 2.0:
            self._metrics_t = now
            self._metrics = system_metrics.snapshot()
            t = self._metrics.get("cpu_temperature")
            if t is not None and t > 80:
                self._event("cpu_hot", "warning", f"CPU temperature {t:.1f} C")
        if not self.telemetry_history or now - self.telemetry_history[-1]["timestamp"] >= 0.5:
            self.telemetry_history.append(self.telemetry())

    def _pose_if_valid(self) -> Optional[tuple[float, float, float]]:
        """Pose in the map frame, or None if it cannot be trusted."""
        with self._lock:
            if not self.pose_valid or time.time() - self.pose_time > self.cfg["safety"]["pose_timeout_s"]:
                return None
            if not self.localizer.pose_ok():
                return None
            return self.localizer.map_pose(self.odom.pose.as_tuple())

    def _scan_match(self, pose: tuple[float, float, float], scan) -> tuple[float, float, float]:
        odom = self.odom.pose.as_tuple()
        last = self._last_match_odom
        if last is not None and math.dist(odom[:2], last[:2]) < 0.03 and \
                abs(math.atan2(math.sin(odom[2] - last[2]), math.cos(odom[2] - last[2]))) < 0.03:
            return pose
        self._last_match_odom = odom
        pts = ScanMatcher.scan_points(scan.angle_min, scan.angle_increment, scan.ranges, scan.range_max,
                                      self.scan_matcher.max_beams)
        res = self.scan_matcher.match(self.mapper.grid, pts, pose)
        if res is not None and res.accepted:
            self.localizer.set_correction(res.pose, odom)
            return res.pose
        return pose

    def _on_picked(self, color: str) -> None:
        pose = self._pose_if_valid()
        if pose is not None:
            # the object was ~15 cm in front of the robot when it was lifted
            x = pose[0] + 0.15 * math.cos(pose[2])
            y = pose[1] + 0.15 * math.sin(pose[2])
            self.objects.mark_sorted_near(color, x, y)

    def _check_low_battery(self) -> None:
        nc = self.cfg["navigation"]
        if not nc.get("return_home_on_low_battery", False) or self.battery_v is None:
            return
        threshold = nc.get("return_voltage", 7.0)
        if self._returning_low_battery:
            if self.battery_v > threshold + 0.3:
                self._returning_low_battery = False
            return
        if self.battery_v >= threshold or self.mode not in (Mode.MISSION, Mode.SORTING, Mode.MAPPING,
                                                            Mode.NAVIGATION):
            return
        self._returning_low_battery = True
        loc = nc.get("return_location", "home")
        self._event("low_battery_return", "warning",
                    f"battery {self.battery_v:.2f} V below {threshold} V: stopping task, returning to '{loc}'")
        self.missions.stop("low battery")
        if self.sorter.running:
            # sorter.stop() joins its thread; never block the control loop on it
            threading.Thread(target=self.sorter.stop, daemon=True).start()
        if self.mapper.active:
            self.mapper.stop()
        self.navigator.cancel("low battery")
        try:
            if loc not in self.navigator.named_locations:
                raise CommandRejected(f"location {loc!r} is not defined")
            if self.mapper.grid.updates == 0 and self.mapper.loaded_name is None:
                raise CommandRejected("no map")
            self.navigator.go_to_named(loc)
            self.mode = Mode.NAVIGATION
        except (CommandRejected, KeyError) as exc:
            self.safety.stop("low-battery")
            self.mode = Mode.IDLE
            self._event("low_battery_return_failed", "error", f"cannot return to '{loc}': {exc}")

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
        self.missions.stop("stop requested")
        self.navigator.cancel("stop requested")
        self.sorter.stop()
        self.safety.stop("operator")
        if self.mode != Mode.MAPPING:
            self.mode = Mode.IDLE

    def emergency_stop(self, reason: str = "operator emergency stop") -> None:
        # motors first, everything else after
        self.safety.emergency_stop(reason)
        self.missions.stop("emergency stop")
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
        if self.missions.running:
            raise CommandRejected("a mission is running; stop it first")
        if self._pose_if_valid() is None:
            raise CommandRejected("no valid pose estimate" + (" (localisation lost: set the initial pose)"
                                                             if not self.localizer.pose_ok() else ""))
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
        if self.missions.running:
            raise CommandRejected("a mission is running")
        self.localizer.deactivate()
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
        """Tell the robot where it is in the map frame (initial pose)."""
        if self.navigator.active:
            raise CommandRejected("cannot reset pose while navigating")
        with self._lock:
            self.localizer.set_initial_pose((x, y, theta), self.odom.pose.as_tuple())

    def global_localization(self) -> None:
        if self.navigator.active:
            raise CommandRejected("stop navigation first")
        try:
            self.localizer.global_localization()
        except RuntimeError as exc:
            raise CommandRejected(str(exc)) from exc

    def load_map(self, name: str) -> None:
        if self.navigator.active or self.mapper.active:
            raise CommandRejected("stop mapping/navigation before loading a map")
        self.mapper.load(name)
        current = self._pose_if_valid() or self.localizer.map_pose(self.odom.pose.as_tuple())
        self.localizer.set_map(self.mapper.grid, name, current)

    # missions
    def start_mission(self, name: str) -> None:
        if name not in self.mission_library:
            raise KeyError(f"unknown mission {name!r}")
        self._prepare_navigation()
        self.missions.start(self.mission_library[name])
        self.mode = Mode.MISSION

    def stop_mission(self) -> None:
        self.missions.stop("stopped by operator")
        self.safety.stop("mission")
        self.mode = Mode.IDLE

    # sorting
    def start_sorting(self) -> None:
        self._require("sorting", "sorting")
        self._require_motion_ok()
        if self.navigator.active or self.missions.running:
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
            x, y, th = self.localizer.map_pose(self.odom.pose.as_tuple())
            if self.localizer.active:
                source = "mcl+ekf" if not self.localizer.experimental else "mcl+ekf (experimental)"
                std_xy, std_th = self.localizer.std_xy, self.localizer.std_th
            else:
                source = "ekf (encoders+gyro)" if self.hw.imu else "ekf (encoders)"
                std_xy, std_th = self.odom.pose_std()
            return {"x": round(x, 4), "y": round(y, 4), "theta": round(th, 4),
                    "valid": self._pose_if_valid() is not None, "source": source,
                    "std_xy": round(std_xy, 4), "std_theta": round(std_th, 4), "timestamp": self.pose_time}

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
            "mapping": {**self.mapper.status(),
                        "scan_matching": None if self.scan_matcher is None else self.scan_matcher.status()},
            "vision": self.vision.status(),
            "sorting": self.sorter.status(),
            "localization": self.localizer.status(),
            "mission": self.missions.status(),
            "objects_mapped": len(self.objects.list()),
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
