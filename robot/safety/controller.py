"""Centralised safety controller.

Every motion command from every source (manual teleop, navigation, sorting
alignment) goes through ``SafetyController.command``. Only this class writes
to the motor driver. It enforces:

* emergency stop (hardware button and software latch) with top priority
* battery undervoltage cut-off
* stale sensor data and command watchdog
* obstacle stop / slow-down zones
* velocity and acceleration limits
* motor / encoder plausibility (commanded motion but wheels not turning)

This is a software layer *in addition to* the physical e-stop, which cuts the
motor supply through a relay and does not depend on this code.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from robot.hardware.base import EmergencyStopInput, MotorDriver, RangeReading
from robot.motors.kinematics import DiffDriveKinematics

log = logging.getLogger(__name__)

FRONT_CONE = math.radians(35)


@dataclass
class SafetyConfig:
    emergency_stop_distance_m: float = 0.15
    slow_down_distance_m: float = 0.35
    slow_down_factor: float = 0.4
    side_clearance_m: float = 0.10
    sensor_timeout_s: float = 0.5
    command_timeout_s: float = 0.6
    battery_min_voltage: float = 6.6
    battery_warn_voltage: float = 7.0
    resume_clear_time_s: float = 1.0
    max_linear_mps: float = 0.3
    max_angular_rps: float = 2.0
    max_linear_accel_mps2: float = 0.6
    blind_reverse_mps: float = 0.06

    @classmethod
    def from_config(cls, cfg: dict) -> "SafetyConfig":
        s, c = cfg["safety"], cfg["chassis"]
        known = {k: v for k, v in s.items() if k in cls.__dataclass_fields__}
        return cls(**known, max_linear_mps=c["max_linear_mps"], max_angular_rps=c["max_angular_rps"],
                   max_linear_accel_mps2=c["max_linear_accel_mps2"])


@dataclass
class SafetyStatus:
    estop_hardware: bool = False
    estop_software: bool = False
    faults: dict[str, str] = field(default_factory=dict)
    warnings: dict[str, str] = field(default_factory=dict)
    obstacle_front_m: Optional[float] = None
    obstacle_left_m: Optional[float] = None
    obstacle_right_m: Optional[float] = None
    zone: str = "clear"          # clear | slow | stop
    stale_sensors: list[str] = field(default_factory=list)
    last_command_source: str = ""
    commanded: tuple[float, float] = (0.0, 0.0)
    applied: tuple[float, float] = (0.0, 0.0)
    limited_reason: str = ""

    @property
    def motion_allowed(self) -> bool:
        return not (self.estop_hardware or self.estop_software or self.faults)

    def as_dict(self) -> dict:
        return {
            "estop_hardware": self.estop_hardware,
            "estop_software": self.estop_software,
            "motion_allowed": self.motion_allowed,
            "faults": dict(self.faults),
            "warnings": dict(self.warnings),
            "obstacle_front_m": self.obstacle_front_m,
            "obstacle_left_m": self.obstacle_left_m,
            "obstacle_right_m": self.obstacle_right_m,
            "zone": self.zone,
            "stale_sensors": list(self.stale_sensors),
            "last_command_source": self.last_command_source,
            "commanded": list(self.commanded),
            "applied": list(self.applied),
            "limited_reason": self.limited_reason,
        }


class SafetyController:
    def __init__(self, motors: Optional[MotorDriver], kin: DiffDriveKinematics, cfg: SafetyConfig,
                 estop: Optional[EmergencyStopInput] = None,
                 on_event: Optional[Callable[[str, str, str], None]] = None,
                 expected_sensors: Optional[list[str]] = None):
        self.motors = motors
        self.kin = kin
        self.cfg = cfg
        self.estop = estop
        self.on_event = on_event or (lambda *a: None)
        self.status = SafetyStatus()
        self.expected_sensors = set(expected_sensors or [])
        self._ranges: dict[str, RangeReading] = {}
        self._last_valid: dict[str, float] = {}
        self._cmd = (0.0, 0.0)
        self._cmd_time = 0.0
        self._applied = (0.0, 0.0)
        self._applied_twist = (0.0, 0.0)
        self._last_tick = time.monotonic()
        self._lock = threading.RLock()
        self._clear_since: Optional[float] = None
        self._stall_since: Optional[float] = None
        self._last_ticks: Optional[tuple[int, int]] = None
        self._battery_v: Optional[float] = None
        if self.motors is None:
            self.status.faults["no_motor_driver"] = "motor driver unavailable"

    # ------------------------------------------------------------------ inputs
    def update_ranges(self, readings: list[RangeReading]) -> None:
        with self._lock:
            for r in readings:
                self._ranges[r.sensor] = r
                if r.valid:
                    self._last_valid[r.sensor] = r.timestamp

    def update_battery(self, voltage: Optional[float]) -> None:
        with self._lock:
            self._battery_v = voltage
            if voltage is None:
                self.status.warnings["battery"] = "battery voltage unknown"
                return
            if voltage < self.cfg.battery_min_voltage:
                if "battery_undervoltage" not in self.status.faults:
                    self._fault("battery_undervoltage", f"battery {voltage:.2f} V below {self.cfg.battery_min_voltage} V")
            elif voltage < self.cfg.battery_warn_voltage:
                self.status.warnings["battery"] = f"battery low ({voltage:.2f} V)"
            else:
                self.status.warnings.pop("battery", None)

    def update_encoders(self, ticks: Optional[tuple[int, int]]) -> None:
        """Detects a stalled or disconnected drive: duty applied, wheels static."""
        now = time.monotonic()
        with self._lock:
            if ticks is None:
                return
            moving_cmd = max(abs(self._applied[0]), abs(self._applied[1])) > 0.25
            if self._last_ticks is not None and moving_cmd and ticks == self._last_ticks:
                self._stall_since = self._stall_since or now
                if now - self._stall_since > 1.5 and "drive_stall" not in self.status.faults:
                    self._fault("drive_stall", "wheels not turning while driven (stall, jam or encoder failure)")
            else:
                self._stall_since = None
            self._last_ticks = ticks

    # ---------------------------------------------------------------- commands
    def command(self, v: float, w: float, source: str) -> bool:
        """Request a body velocity. Returns False if motion is not allowed now."""
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError("non-finite velocity command")
        with self._lock:
            self._cmd = (float(v), float(w))
            self._cmd_time = time.monotonic()
            self.status.last_command_source = source
            self.status.commanded = self._cmd
            allowed = self.status.motion_allowed
        self.tick()
        return allowed

    def stop(self, source: str = "system") -> None:
        with self._lock:
            self._cmd = (0.0, 0.0)
            self._cmd_time = time.monotonic()
            self.status.last_command_source = source
        self._write(0.0, 0.0)

    def emergency_stop(self, reason: str = "software emergency stop") -> None:
        """Highest priority: stop motors first, then update state."""
        if self.motors is not None:
            try:
                self.motors.stop()
            except Exception:
                log.exception("motor stop failed during e-stop")
        with self._lock:
            already = self.status.estop_software
            self.status.estop_software = True
            self._cmd = (0.0, 0.0)
            self._applied = (0.0, 0.0)
            self._clear_since = None
        if not already:
            self.on_event("emergency_stop", "critical", reason)

    def reset_emergency_stop(self) -> tuple[bool, str]:
        with self._lock:
            if self.estop is not None and self.estop.is_pressed():
                return False, "hardware emergency stop is still pressed"
            self.status.estop_software = False
            self._clear_since = None
        self.on_event("emergency_stop_reset", "warning", "emergency stop reset by operator")
        return True, "ok"

    def clear_fault(self, name: str) -> bool:
        with self._lock:
            if name == "battery_undervoltage" and self._battery_v is not None \
                    and self._battery_v < self.cfg.battery_min_voltage:
                return False
            removed = self.status.faults.pop(name, None) is not None
            if name == "drive_stall":
                self._stall_since = None
        if removed:
            self.on_event("fault_cleared", "info", name)
        return removed

    # ------------------------------------------------------------------- loop
    def _fault(self, name: str, message: str) -> None:
        self.status.faults[name] = message
        if self.motors is not None:
            try:
                self.motors.stop()
            except Exception:
                pass
        self._applied = (0.0, 0.0)
        self.on_event(name, "error", message)

    def _sector_min(self, lo: float, hi: float) -> Optional[float]:
        best = None
        for r in self._ranges.values():
            if r.valid and r.distance_m is not None and lo <= r.angle_rad <= hi:
                best = r.distance_m if best is None else min(best, r.distance_m)
        return best

    def tick(self) -> tuple[float, float]:
        now = time.monotonic()
        wall = time.time()
        with self._lock:
            st = self.status
            dt = max(1e-3, min(0.2, now - self._last_tick))
            self._last_tick = now

            hw_pressed = self.estop.is_pressed() if self.estop is not None else False
            if hw_pressed and not st.estop_hardware:
                self.on_event("emergency_stop", "critical", "hardware emergency stop pressed")
                # latch in software too: releasing the button alone must not restart motion
                st.estop_software = True
            st.estop_hardware = hw_pressed

            st.stale_sensors = sorted(s for s in self.expected_sensors
                                      if wall - self._last_valid.get(s, 0.0) > self.cfg.sensor_timeout_s)
            st.obstacle_front_m = self._sector_min(-FRONT_CONE, FRONT_CONE)
            st.obstacle_left_m = self._sector_min(FRONT_CONE, math.pi)
            st.obstacle_right_m = self._sector_min(-math.pi, -FRONT_CONE)

            v, w = self._cmd
            reason = ""
            if now - self._cmd_time > self.cfg.command_timeout_s:
                v, w = 0.0, 0.0
                if self._cmd != (0.0, 0.0):
                    reason = "command timeout"

            # velocity limits
            v = max(-self.cfg.max_linear_mps, min(self.cfg.max_linear_mps, v))
            w = max(-self.cfg.max_angular_rps, min(self.cfg.max_angular_rps, w))

            front = st.obstacle_front_m
            front_stale = any(n for n in st.stale_sensors if self._is_front(n))
            if front_stale and v > 0:
                v, reason = 0.0, "front sensor data stale"
            if front is not None and front < self.cfg.emergency_stop_distance_m:
                st.zone = "stop"
                if v > 0:
                    if self._applied[0] + self._applied[1] > 0:
                        self.on_event("obstacle_stop", "warning", f"obstacle at {front:.2f} m, forward motion blocked")
                    v, reason = 0.0, "obstacle in stop zone"
            elif front is not None and front < self.cfg.slow_down_distance_m:
                st.zone = "slow"
                if v > 0:
                    v *= self.cfg.slow_down_factor
                    reason = reason or "obstacle in slow zone"
            else:
                st.zone = "clear"
            # don't sweep the body into something close at the side
            if w > 0 and st.obstacle_left_m is not None and st.obstacle_left_m < self.cfg.side_clearance_m and v > 0:
                w, reason = 0.0, "left side too close"
            if w < 0 and st.obstacle_right_m is not None and st.obstacle_right_m < self.cfg.side_clearance_m and v > 0:
                w, reason = 0.0, "right side too close"
            # no rear sensors: reversing is blind and kept slow
            if v < 0:
                v = max(v, -self.cfg.blind_reverse_mps)

            if not st.motion_allowed:
                v, w = 0.0, 0.0
                reason = "emergency stop" if (st.estop_hardware or st.estop_software) else "fault: " + ",".join(st.faults)

            # acceleration limit (deceleration to zero is always immediate)
            prev_v = self._applied_twist[0]
            if v != 0.0 and abs(v) > abs(prev_v):
                max_dv = self.cfg.max_linear_accel_mps2 * dt
                v = prev_v + max(-max_dv, min(max_dv, v - prev_v))
            self._applied_twist = (v, w)
            st.limited_reason = reason
            st.applied = (round(v, 4), round(w, 4))
            allowed = st.motion_allowed
        if allowed:
            self._write(*self.kin.to_duty(v, w))
        else:
            self._write(0.0, 0.0)
        return v, w

    def _is_front(self, name: str) -> bool:
        r = self._ranges.get(name)
        return r is None or abs(r.angle_rad) <= FRONT_CONE

    def _write(self, left: float, right: float) -> None:
        if self.motors is None:
            return
        try:
            if left == 0.0 and right == 0.0:
                self.motors.stop()
            else:
                self.motors.set_duty(left, right)
            self._applied = (left, right)
        except Exception as exc:
            with self._lock:
                if "motor_communication" not in self.status.faults:
                    self._fault("motor_communication", f"motor driver error: {exc}")
