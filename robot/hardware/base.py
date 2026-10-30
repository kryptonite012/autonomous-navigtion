"""Hardware abstraction interfaces.

Business logic (navigation, sorting, safety, ...) only talks to these
interfaces. Physical drivers live in ``robot.hardware.pi`` and the simulated
ones in ``robot.simulation``; the factory decides which one to build.
"""

from __future__ import annotations

import abc
import enum
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


class HardwareUnavailable(RuntimeError):
    """Raised when a device (or its driver library) is not present."""


class HealthState(str, enum.Enum):
    OK = "ok"
    DEGRADED = "degraded"
    FAULT = "fault"
    MISSING = "missing"
    DISABLED = "disabled"


@dataclass
class DeviceHealth:
    name: str
    kind: str
    state: HealthState
    message: str = ""
    simulated: bool = False
    last_update: float = field(default_factory=time.time)

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "kind": self.kind,
            "state": self.state.value,
            "message": self.message,
            "simulated": self.simulated,
            "last_update": self.last_update,
        }


class Device(abc.ABC):
    """Common lifecycle for every hardware device."""

    kind: str = "device"
    simulated: bool = False

    def __init__(self, name: str):
        self.name = name
        self._initialized = False
        self._last_error: str = ""
        self._error_count = 0

    def initialize(self) -> None:
        """Open the device. Raise HardwareUnavailable if it is not present."""
        self._initialized = True

    def validate(self) -> bool:
        """Check that the device actually answers. Default: initialized."""
        return self._initialized

    def health(self) -> DeviceHealth:
        if not self._initialized:
            return DeviceHealth(self.name, self.kind, HealthState.MISSING, self._last_error, self.simulated)
        if self._error_count > 5:
            return DeviceHealth(self.name, self.kind, HealthState.FAULT, self._last_error, self.simulated)
        if self._error_count > 0:
            return DeviceHealth(self.name, self.kind, HealthState.DEGRADED, self._last_error, self.simulated)
        return DeviceHealth(self.name, self.kind, HealthState.OK, "", self.simulated)

    def shutdown(self) -> None:
        self._initialized = False

    # helpers for subclasses
    def _record_error(self, err: Exception | str) -> None:
        self._last_error = str(err)
        self._error_count += 1

    def _record_ok(self) -> None:
        if self._error_count:
            self._error_count = max(0, self._error_count - 1)


@dataclass
class RangeReading:
    sensor: str
    distance_m: Optional[float]   # None when nothing in range / invalid
    angle_rad: float
    timestamp: float
    valid: bool = True
    max_range_m: float = 2.0


@dataclass
class ImuReading:
    accel: tuple[float, float, float]   # m/s^2
    gyro: tuple[float, float, float]    # rad/s
    temperature_c: Optional[float]
    timestamp: float


@dataclass
class LaserScan:
    angle_min: float
    angle_increment: float
    ranges: np.ndarray            # metres, np.inf for no return
    range_max: float
    timestamp: float


class MotorDriver(Device):
    kind = "motor_driver"

    @abc.abstractmethod
    def set_duty(self, left: float, right: float) -> None:
        """Set wheel duty cycles in [-1, 1]. Only the safety layer calls this."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Stop both wheels immediately (brake)."""


class EncoderPair(Device):
    kind = "encoders"

    @abc.abstractmethod
    def ticks(self) -> tuple[int, int]:
        """Cumulative (left, right) tick counts."""


class DistanceSensor(Device):
    kind = "distance"
    angle_rad: float = 0.0
    max_range_m: float = 2.0

    @abc.abstractmethod
    def read(self) -> RangeReading: ...


class Imu(Device):
    kind = "imu"

    @abc.abstractmethod
    def read(self) -> ImuReading: ...


class Lidar(Device):
    kind = "lidar"

    @abc.abstractmethod
    def scan(self) -> Optional[LaserScan]: ...


class ServoController(Device):
    kind = "servos"

    @abc.abstractmethod
    def set_angle(self, channel: str, angle_deg: float) -> None: ...

    @abc.abstractmethod
    def get_angle(self, channel: str) -> Optional[float]: ...

    @abc.abstractmethod
    def release(self) -> None:
        """Stop sending pulses so servos go limp."""


class Camera(Device):
    kind = "camera"

    @abc.abstractmethod
    def read(self) -> tuple[bool, Optional[np.ndarray], float]:
        """Return (ok, BGR frame, timestamp)."""


class BatteryMonitor(Device):
    kind = "battery"

    @abc.abstractmethod
    def voltage(self) -> Optional[float]: ...


class EmergencyStopInput(Device):
    kind = "emergency_stop"

    @abc.abstractmethod
    def is_pressed(self) -> bool: ...


class BinSensor(Device):
    """Detects an object falling into the bin chute (break-beam or load cell)."""

    kind = "bin_sensor"

    @abc.abstractmethod
    def event_count(self) -> int:
        """Monotonic counter of detected drop events."""
