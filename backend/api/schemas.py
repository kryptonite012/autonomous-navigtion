"""Pydantic request/response models."""

from __future__ import annotations

import math
from typing import Optional

from pydantic import BaseModel, Field, field_validator


class LoginRequest(BaseModel):
    username: str = Field(min_length=1, max_length=64)
    password: str = Field(min_length=1, max_length=128)


class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    expires_at: str
    username: str
    role: str


class UserCreate(BaseModel):
    username: str = Field(min_length=3, max_length=64, pattern=r"^[A-Za-z0-9_.\-]+$")
    password: str = Field(min_length=10, max_length=72)
    role: str = Field(pattern=r"^(viewer|operator|admin)$")


class UserUpdate(BaseModel):
    role: Optional[str] = Field(default=None, pattern=r"^(viewer|operator|admin)$")
    is_active: Optional[bool] = None
    password: Optional[str] = Field(default=None, min_length=10, max_length=72)


class PasswordChange(BaseModel):
    current_password: str = Field(max_length=128)
    new_password: str = Field(min_length=10, max_length=72)


class Teleop(BaseModel):
    linear: float = Field(ge=-1.0, le=1.0, description="m/s, clamped further by the safety controller")
    angular: float = Field(ge=-4.0, le=4.0, description="rad/s")


class GoalRequest(BaseModel):
    x: Optional[float] = Field(default=None, ge=-100, le=100)
    y: Optional[float] = Field(default=None, ge=-100, le=100)
    theta: Optional[float] = Field(default=None, ge=-math.pi * 2, le=math.pi * 2)
    location: Optional[str] = Field(default=None, max_length=64)

    @field_validator("location")
    @classmethod
    def _loc(cls, v):
        if v is not None and not v.replace("_", "").replace("-", "").isalnum():
            raise ValueError("invalid location name")
        return v


class NamedLocation(BaseModel):
    name: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_\-]+$")
    x: float = Field(ge=-100, le=100)
    y: float = Field(ge=-100, le=100)
    theta: float = Field(default=0.0, ge=-math.pi * 2, le=math.pi * 2)


class PoseReset(BaseModel):
    x: float = Field(default=0.0, ge=-100, le=100)
    y: float = Field(default=0.0, ge=-100, le=100)
    theta: float = Field(default=0.0, ge=-math.pi * 2, le=math.pi * 2)


class MappingStart(BaseModel):
    explore: bool = True
    reset: bool = False


class MapName(BaseModel):
    name: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9][A-Za-z0-9_\-]*$")


class MapRename(BaseModel):
    old_name: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9][A-Za-z0-9_\-]*$")
    new_name: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9][A-Za-z0-9_\-]*$")


class HSVBounds(BaseModel):
    lower: list[int] = Field(min_length=3, max_length=3)
    upper: list[int] = Field(min_length=3, max_length=3)


class CalibrationRequest(BaseModel):
    color: str = Field(pattern=r"^[a-z]{2,16}$")
    ranges: Optional[list[HSVBounds]] = Field(default=None, max_length=3)
    roi: Optional[list[int]] = Field(default=None, min_length=4, max_length=4,
                                     description="x, y, w, h region containing only the target colour")
    persist: bool = False


class VisionSettings(BaseModel):
    min_area_px: Optional[int] = Field(default=None, ge=20, le=100000)
    min_confidence: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    blur_kernel: Optional[int] = Field(default=None, ge=1, le=21)
    morph_kernel: Optional[int] = Field(default=None, ge=1, le=21)
    enabled_colors: Optional[list[str]] = None


class SortingConfig(BaseModel):
    bins: dict[str, int]
    unclassified_bin: int = Field(ge=1, le=16)
    bin_capacity: Optional[int] = Field(default=None, ge=1, le=1000)
    persist: bool = False

    @field_validator("bins")
    @classmethod
    def _bins(cls, v):
        for color, b in v.items():
            if not color.isalpha() or len(color) > 16:
                raise ValueError(f"invalid colour {color!r}")
            if not 1 <= int(b) <= 16:
                raise ValueError("bin numbers must be 1..16")
        return v


class ServoCalibration(BaseModel):
    channel: str = Field(pattern=r"^(base|shoulder|gripper|sorter)$")
    angle: float = Field(ge=0, le=180)


class SafetySettings(BaseModel):
    emergency_stop_distance_m: Optional[float] = Field(default=None, ge=0.05, le=1.0)
    slow_down_distance_m: Optional[float] = Field(default=None, ge=0.1, le=2.0)
    slow_down_factor: Optional[float] = Field(default=None, ge=0.1, le=1.0)
    max_linear_mps: Optional[float] = Field(default=None, ge=0.05, le=0.5)
    max_angular_rps: Optional[float] = Field(default=None, ge=0.2, le=3.0)


class FaultClear(BaseModel):
    fault: str = Field(min_length=1, max_length=64)


class WaypointIn(BaseModel):
    x: Optional[float] = Field(default=None, ge=-100, le=100)
    y: Optional[float] = Field(default=None, ge=-100, le=100)
    theta: Optional[float] = Field(default=None, ge=-math.pi * 2, le=math.pi * 2)
    location: Optional[str] = Field(default=None, max_length=64, pattern=r"^[A-Za-z0-9_\-]+$")
    dwell_s: float = Field(default=0.0, ge=0, le=3600)


class MissionIn(BaseModel):
    waypoints: list[WaypointIn] = Field(min_length=1, max_length=100)
    loop: bool = False
    max_loops: int = Field(default=0, ge=0, le=1000)
    return_home: bool = False
    on_failure: str = Field(default="skip", pattern=r"^(skip|abort)$")


class InitialPose(BaseModel):
    x: float = Field(ge=-100, le=100)
    y: float = Field(ge=-100, le=100)
    theta: float = Field(default=0.0, ge=-math.pi * 2, le=math.pi * 2)


class SimFaults(BaseModel):
    motor_fault: Optional[bool] = None
    encoder_fault: Optional[bool] = None
    servo_fault: Optional[bool] = None
    camera_covered: Optional[bool] = None
    estop_pressed: Optional[bool] = None
    battery_voltage: Optional[float] = Field(default=None, ge=5.0, le=8.5)
    disconnected_sensors: Optional[list[str]] = Field(default=None, max_length=16)


class SimObstacle(BaseModel):
    x: float = Field(ge=-50, le=50)
    y: float = Field(ge=-50, le=50)
    radius: float = Field(default=0.1, ge=0.03, le=0.5)
    vx: float = Field(default=0.0, ge=-0.5, le=0.5)
    vy: float = Field(default=0.0, ge=-0.5, le=0.5)


class SimScatter(BaseModel):
    count: int = Field(default=6, ge=1, le=30)
    colors: list[str] = Field(default_factory=lambda: ["red", "green", "blue", "yellow", "orange"],
                              min_length=1, max_length=7)


class SimTeleport(BaseModel):
    x: float = Field(ge=-50, le=50)
    y: float = Field(ge=-50, le=50)
    theta: float = Field(default=0.0, ge=-math.pi * 2, le=math.pi * 2)


class WhiteBalanceRequest(BaseModel):
    mode: str = Field(default="reference", pattern=r"^(off|gray_world|reference)$")
    roi: Optional[list[int]] = Field(default=None, min_length=4, max_length=4,
                                     description="x, y, w, h of a white/grey sheet (mode=reference)")
    persist: bool = False


class RecordingName(BaseModel):
    name: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9][A-Za-z0-9_\-]*$")
