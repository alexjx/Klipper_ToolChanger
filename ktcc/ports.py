"""Contracts between toolchanger features and machine integrations.

Feature modules depend on these protocols. Klipper implementations live under
``ktcc.klipper``; no Klipper object or untyped status dictionary crosses this
boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Mapping, Optional, Protocol, TYPE_CHECKING, runtime_checkable

from ktcc.toolchange.state import ChangerState, MountedState, TransitionPhase
from ktcc.tools import ExtruderRef, FanRef, HeaterRef, ToolId, ToolSpec, Vector3
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    MotionProfile,
    PressureAdvanceProfile,
    ToolProfile,
    WaitMode,
)

if TYPE_CHECKING:
    from ktcc.statistics.models import MechanicalKind


def _finite(name: str, value: float, *, minimum: Optional[float] = None) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} must be at least {minimum}")


class ThermalMode(str, Enum):
    OFF = "OFF"
    STANDBY = "STANDBY"
    PREHEAT = "PREHEAT"
    ACTIVE = "ACTIVE"


@dataclass(frozen=True)
class ThermalSnapshot:
    active_target: float
    standby_target: float
    mode: ThermalMode

    def __post_init__(self) -> None:
        _finite("active_target", self.active_target, minimum=0.0)
        _finite("standby_target", self.standby_target, minimum=0.0)


@dataclass(frozen=True)
class ActionContext:
    """Versioned, immutable data exposed to one configured machine action."""

    schema_version: int
    tool: ToolSpec
    thermal: ThermalSnapshot
    changer_before: ChangerState
    expected_after: ChangerState
    transition_id: str
    phase: TransitionPhase
    last_checkpoint: Optional[str] = None
    checkpoint_capability: Optional[str] = None
    effective_profile: Optional[ToolProfile] = None

    def __post_init__(self) -> None:
        if self.schema_version < 1:
            raise ValueError("action context schema_version must be positive")
        if not self.transition_id or len(self.transition_id) > 96:
            raise ValueError("transition_id must contain 1..96 characters")
        if self.last_checkpoint is not None and (
            not self.last_checkpoint or len(self.last_checkpoint) > 80
        ):
            raise ValueError("last_checkpoint must contain 1..80 characters")
        if self.checkpoint_capability is not None and (
            len(self.checkpoint_capability) < 16
            or len(self.checkpoint_capability) > 128
        ):
            raise ValueError("checkpoint_capability must contain 16..128 characters")
        if self.effective_profile is not None and not isinstance(
            self.effective_profile, ToolProfile
        ):
            raise ValueError("effective_profile must be a ToolProfile")


@dataclass(frozen=True)
class HeaterObservation:
    temperature: float
    target: float
    can_extrude: bool

    def __post_init__(self) -> None:
        _finite("temperature", self.temperature)
        _finite("target", self.target, minimum=0.0)
        if not isinstance(self.can_extrude, bool):
            raise ValueError("can_extrude must be boolean")


@dataclass(frozen=True)
class ThermalWaitRequest:
    frozen_target: float
    mode: WaitMode
    tolerance: float
    timeout: float
    stable_time: float = 0.0

    def __post_init__(self) -> None:
        _finite("frozen_target", self.frozen_target, minimum=0.0)
        if self.mode is WaitMode.HEAT and self.frozen_target == 0:
            raise ValueError("a heat wait requires a non-zero frozen_target")
        if not isinstance(self.mode, WaitMode):
            raise ValueError("mode must be a WaitMode")
        _finite("tolerance", self.tolerance, minimum=0.0)
        _finite("timeout", self.timeout, minimum=0.0)
        if self.timeout == 0:
            raise ValueError("timeout must be greater than zero")
        _finite("stable_time", self.stable_time, minimum=0.0)
        if self.mode is not WaitMode.STABLE and self.stable_time != 0:
            raise ValueError("stable_time is supported only for STABLE waits")


class VerificationKind(str, Enum):
    MOUNTED = "MOUNTED"
    UNMOUNTED = "UNMOUNTED"
    LOCKED = "LOCKED"
    UNLOCKED = "UNLOCKED"


@dataclass(frozen=True)
class VerificationRequest:
    kind: VerificationKind
    tool_id: Optional[ToolId] = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, VerificationKind):
            raise ValueError("kind must be a VerificationKind")
        if self.kind is VerificationKind.MOUNTED and self.tool_id is None:
            raise ValueError("mounted verification requires a tool_id")
        if self.kind is not VerificationKind.MOUNTED and self.tool_id is not None:
            raise ValueError("only mounted verification accepts a tool_id")


@dataclass(frozen=True)
class VerificationResult:
    request: VerificationRequest
    matches: bool
    observed_mounted: MountedState
    detail: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.matches, bool):
            raise ValueError("matches must be boolean")
        if len(self.detail) > 256:
            raise ValueError("verification detail exceeds 256 characters")


class EventLevel(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class ToolchangerEvent:
    code: str
    level: EventLevel
    message: str
    tool_id: Optional[ToolId] = None
    transition_id: Optional[str] = None
    phase: Optional[TransitionPhase] = None

    def __post_init__(self) -> None:
        if not self.code or len(self.code) > 80:
            raise ValueError("event code must contain 1..80 characters")
        if not self.message or len(self.message) > 512:
            raise ValueError("event message must contain 1..512 characters")
        if self.transition_id is not None and (
            not self.transition_id or len(self.transition_id) > 96
        ):
            raise ValueError("transition_id must contain 1..96 characters")


@dataclass(frozen=True, order=True)
class TimerHandle:
    value: int

    def __post_init__(self) -> None:
        if isinstance(self.value, bool) or not isinstance(self.value, int) or self.value < 0:
            raise ValueError("timer handle must be a non-negative integer")


TimerCallback = Callable[[], None]


@runtime_checkable
class ActionPort(Protocol):
    def run_from_command(self, action: str, context: ActionContext) -> None: ...

    def run_from_callback(self, action: str, context: ActionContext) -> None: ...


@runtime_checkable
class ThermalPort(Protocol):
    def set_target(self, heater: HeaterRef, target: float) -> None: ...

    def wait(self, heater: HeaterRef, request: ThermalWaitRequest) -> HeaterObservation: ...

    def observe(self, heater: HeaterRef) -> HeaterObservation: ...


@runtime_checkable
class ExtrusionProfilePort(Protocol):
    def activate_extruder(self, extruder: ExtruderRef) -> None: ...

    def apply_pressure_advance(
        self, extruder: ExtruderRef, profile: PressureAdvanceProfile
    ) -> None: ...

    def apply_firmware_retraction(self, profile: FirmwareRetractionProfile) -> None: ...

    def observe_retracted(self) -> bool: ...

    def unretract(self) -> None: ...


@runtime_checkable
class MachinePort(Protocol):
    def apply_offset(self, tool_id: ToolId, offset: Vector3) -> None: ...

    def set_fan(self, fan: FanRef, speed: float) -> None: ...

    def observe_position(self) -> Vector3: ...

    def apply_motion_profile(self, profile: MotionProfile) -> None: ...


@runtime_checkable
class ToolchangeReadinessPort(Protocol):
    """Validate machine prerequisites without performing motion."""

    def require_ready(self) -> None: ...


@runtime_checkable
class SchedulerPort(Protocol):
    def schedule_at(
        self, callback_id: str, deadline: float, callback: TimerCallback
    ) -> TimerHandle: ...

    def cancel(self, handle: TimerHandle) -> None: ...


@runtime_checkable
class StatePersistencePort(Protocol):
    def load_state(self) -> Optional[ChangerState]: ...

    def save_state(self, state: ChangerState) -> None: ...

    def mirror_tool_current(self, value: int) -> None: ...


@runtime_checkable
class ProfilePersistencePort(Protocol):
    def save_retraction(
        self, tool_id: ToolId, profile: FirmwareRetractionProfile
    ) -> None: ...

    def save_pressure_advance(
        self, tool_id: ToolId, profile: PressureAdvanceProfile
    ) -> None: ...

    def save_tool_offset(self, tool_id: ToolId, offset: Vector3) -> None: ...

    def save_global_offset(self, offset: Vector3) -> None: ...


@runtime_checkable
class VariableStorePort(Protocol):
    """Raw boundary implemented by the Klipper save_variables adapter."""

    def read_variables(self) -> Mapping[str, Any]: ...

    def write_variable(self, name: str, value: Any) -> None: ...


@runtime_checkable
class VerificationPort(Protocol):
    def verify(self, request: VerificationRequest) -> VerificationResult: ...


@runtime_checkable
class EventSink(Protocol):
    def emit(self, event: ToolchangerEvent) -> None: ...


@runtime_checkable
class StatisticsPort(Protocol):
    """Functional metrics observer; implementations must not control motion."""

    def mechanical_started(
        self, kind: "MechanicalKind", tool_id: ToolId, transition_id: str
    ) -> None: ...

    def mechanical_completed(
        self, kind: "MechanicalKind", tool_id: ToolId, transition_id: str
    ) -> None: ...

    def selection_changed(
        self, previous: Optional[ToolId], current: Optional[ToolId]
    ) -> None: ...

    def lock_completed(self) -> None: ...

    def unlock_completed(self) -> None: ...

    def thermal_changed(
        self, tool_id: ToolId, previous: ThermalMode, current: ThermalMode
    ) -> None: ...


@runtime_checkable
class ShutdownPort(Protocol):
    def shutdown(self, reason: str) -> None: ...


@runtime_checkable
class Clock(Protocol):
    def monotonic(self) -> float: ...
