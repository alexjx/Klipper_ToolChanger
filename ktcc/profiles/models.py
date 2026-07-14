"""Immutable per-tool parameter profiles and merge rules."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, fields, is_dataclass, replace
from enum import Enum
from types import MappingProxyType
from typing import Any, Mapping, Optional


class ProfileValidationError(ValueError):
    """A profile value cannot be represented safely by the target APIs."""


def _finite(name: str, value: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ProfileValidationError(f"{name} must be a finite number")


def _at_least(name: str, value: float, minimum: float) -> None:
    _finite(name, value)
    if value < minimum:
        raise ProfileValidationError(f"{name} must be at least {minimum}")


@dataclass(frozen=True)
class FirmwareRetractionProfile:
    retract_length: float = 0.0
    retract_speed: float = 20.0
    unretract_extra_length: float = 0.0
    unretract_speed: float = 10.0
    zhop: float = 0.0

    def __post_init__(self) -> None:
        _at_least("retract_length", self.retract_length, 0.0)
        _at_least("retract_speed", self.retract_speed, 1.0)
        _at_least("unretract_extra_length", self.unretract_extra_length, 0.0)
        _at_least("unretract_speed", self.unretract_speed, 1.0)
        _at_least("zhop", self.zhop, 0.0)


@dataclass(frozen=True)
class PressureAdvanceProfile:
    pressure_advance: float = 0.0
    smooth_time: float = 0.04

    def __post_init__(self) -> None:
        _at_least("pressure_advance", self.pressure_advance, 0.0)
        _at_least("smooth_time", self.smooth_time, 0.0)
        if self.smooth_time > 0.2:
            raise ProfileValidationError("smooth_time must not exceed Klipper's 0.2s limit")


def _freeze_scalars(values: Mapping[str, Any]) -> Mapping[str, Any]:
    frozen = {}
    for key, value in values.items():
        if not isinstance(key, str) or not key:
            raise ProfileValidationError("filament variable names must be non-empty strings")
        if not isinstance(value, (str, int, float, bool)):
            raise ProfileValidationError(f"filament variable {key!r} must be a scalar")
        frozen[key] = value
    return MappingProxyType(frozen)


@dataclass(frozen=True)
class ToolchangeFilamentProfile:
    unload_length: float = 0.0
    prime_length: float = 0.0
    variables: Mapping[str, Any] = MappingProxyType({})

    def __post_init__(self) -> None:
        _at_least("unload_length", self.unload_length, 0.0)
        _at_least("prime_length", self.prime_length, 0.0)
        object.__setattr__(self, "variables", _freeze_scalars(self.variables))


class WaitMode(str, Enum):
    HEAT = "HEAT"
    RANGE = "RANGE"
    STABLE = "STABLE"


@dataclass(frozen=True)
class ThermalPolicy:
    default_active: float = 0.0
    default_standby: float = 0.0
    idle_to_standby_time: float = 30.0
    standby_to_off_time: float = 600.0
    wait_mode: WaitMode = WaitMode.HEAT
    tolerance: float = 1.0
    timeout: float = 900.0
    stable_time: float = 0.0

    def __post_init__(self) -> None:
        _at_least("default_active", self.default_active, 0.0)
        _at_least("default_standby", self.default_standby, 0.0)
        _at_least("idle_to_standby_time", self.idle_to_standby_time, 0.1)
        _at_least("standby_to_off_time", self.standby_to_off_time, 0.1)
        _at_least("tolerance", self.tolerance, 0.0)
        _at_least("timeout", self.timeout, 0.0)
        if self.timeout == 0:
            raise ProfileValidationError("timeout must be greater than zero")
        _at_least("stable_time", self.stable_time, 0.0)
        if not isinstance(self.wait_mode, WaitMode):
            try:
                object.__setattr__(self, "wait_mode", WaitMode(str(self.wait_mode).upper()))
            except ValueError as exc:
                raise ProfileValidationError(f"unsupported wait mode: {self.wait_mode}") from exc


@dataclass(frozen=True)
class MotionProfile:
    shaper_freq_x: float = 0.0
    shaper_freq_y: float = 0.0
    shaper_type_x: str = "mzv"
    shaper_type_y: str = "mzv"
    damping_ratio_x: float = 0.1
    damping_ratio_y: float = 0.1

    def __post_init__(self) -> None:
        _at_least("shaper_freq_x", self.shaper_freq_x, 0.0)
        _at_least("shaper_freq_y", self.shaper_freq_y, 0.0)
        for axis in ("x", "y"):
            shaper_type = getattr(self, f"shaper_type_{axis}")
            if not isinstance(shaper_type, str) or not shaper_type.strip():
                raise ProfileValidationError(f"shaper_type_{axis} must be non-empty")
            ratio = getattr(self, f"damping_ratio_{axis}")
            _at_least(f"damping_ratio_{axis}", ratio, 0.0)
            if ratio > 1.0:
                raise ProfileValidationError(f"damping_ratio_{axis} must not exceed 1")


@dataclass(frozen=True)
class ToolProfile:
    print_retraction: FirmwareRetractionProfile = FirmwareRetractionProfile()
    pressure_advance: PressureAdvanceProfile = PressureAdvanceProfile()
    toolchange_filament: ToolchangeFilamentProfile = ToolchangeFilamentProfile()
    thermal_policy: ThermalPolicy = ThermalPolicy()
    motion: Optional[MotionProfile] = None


@dataclass(frozen=True)
class FirmwareRetractionPatch:
    retract_length: Optional[float] = None
    retract_speed: Optional[float] = None
    unretract_extra_length: Optional[float] = None
    unretract_speed: Optional[float] = None
    zhop: Optional[float] = None


@dataclass(frozen=True)
class PressureAdvancePatch:
    pressure_advance: Optional[float] = None
    smooth_time: Optional[float] = None


@dataclass(frozen=True)
class ToolchangeFilamentPatch:
    unload_length: Optional[float] = None
    prime_length: Optional[float] = None
    variables: Optional[Mapping[str, Any]] = None


@dataclass(frozen=True)
class ThermalPolicyPatch:
    default_active: Optional[float] = None
    default_standby: Optional[float] = None
    idle_to_standby_time: Optional[float] = None
    standby_to_off_time: Optional[float] = None
    wait_mode: Optional[WaitMode] = None
    tolerance: Optional[float] = None
    timeout: Optional[float] = None
    stable_time: Optional[float] = None


@dataclass(frozen=True)
class MotionProfilePatch:
    shaper_freq_x: Optional[float] = None
    shaper_freq_y: Optional[float] = None
    shaper_type_x: Optional[str] = None
    shaper_type_y: Optional[str] = None
    damping_ratio_x: Optional[float] = None
    damping_ratio_y: Optional[float] = None


@dataclass(frozen=True)
class ToolProfilePatch:
    print_retraction: Optional[FirmwareRetractionPatch] = None
    pressure_advance: Optional[PressureAdvancePatch] = None
    toolchange_filament: Optional[ToolchangeFilamentPatch] = None
    thermal_policy: Optional[ThermalPolicyPatch] = None
    motion: Optional[MotionProfilePatch] = None


def _apply_patch(value: Any, patch: Any) -> Any:
    changes = {
        field.name: getattr(patch, field.name)
        for field in fields(patch)
        if getattr(patch, field.name) is not None
    }
    return replace(value, **changes)


def _merge_profile(profile: ToolProfile, patch: ToolProfilePatch) -> ToolProfile:
    changes = {}
    for name in ("print_retraction", "pressure_advance", "toolchange_filament", "thermal_policy"):
        section_patch = getattr(patch, name)
        if section_patch is not None:
            changes[name] = _apply_patch(getattr(profile, name), section_patch)
    if patch.motion is not None:
        changes["motion"] = _apply_patch(profile.motion or MotionProfile(), patch.motion)
    return replace(profile, **changes)


def _revision(profile: ToolProfile) -> str:
    def canonical(value: Any) -> Any:
        if is_dataclass(value):
            return {field.name: canonical(getattr(value, field.name)) for field in fields(value)}
        if isinstance(value, Mapping):
            return {key: canonical(item) for key, item in value.items()}
        if isinstance(value, Enum):
            return value.value
        return value

    encoded = json.dumps(canonical(profile), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()[:16]


@dataclass(frozen=True)
class EffectiveToolProfile:
    profile: ToolProfile
    revision: str


@dataclass(frozen=True)
class ProfileLayers:
    configured: ToolProfile
    persisted: ToolProfilePatch = ToolProfilePatch()
    session: ToolProfilePatch = ToolProfilePatch()

    def resolve(self) -> EffectiveToolProfile:
        profile = _merge_profile(self.configured, self.persisted)
        profile = _merge_profile(profile, self.session)
        return EffectiveToolProfile(profile=profile, revision=_revision(profile))


@dataclass(frozen=True)
class AppliedProfileSnapshot:
    tool_id: int
    resource: str
    desired_revision: str
    applied_revision: Optional[str]
    profile: Optional[ToolProfile]

    @property
    def is_current(self) -> bool:
        return self.applied_revision == self.desired_revision
