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
