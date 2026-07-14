"""Validated state for a physical-tool changer.

This module intentionally knows nothing about Klipper, persistence, or G-code.
"""

from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional, Tuple


class ChangerMode(str, Enum):
    IDLE = "IDLE"
    CHANGING = "CHANGING"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"
    SYNCHRONIZING = "SYNCHRONIZING"


class MountedKind(str, Enum):
    NONE = "NONE"
    KNOWN = "KNOWN"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class MountedState:
    kind: MountedKind
    tool_id: Optional[int] = None

    def __post_init__(self) -> None:
        if self.kind is MountedKind.KNOWN:
            if isinstance(self.tool_id, bool) or not isinstance(self.tool_id, int):
                raise ValueError("a known mounted tool requires an integer tool_id")
            if self.tool_id < 0:
                raise ValueError("tool_id must be non-negative")
        elif self.tool_id is not None:
            raise ValueError("NONE and UNKNOWN mounted states cannot carry a tool_id")

    @classmethod
    def none(cls) -> "MountedState":
        return cls(MountedKind.NONE)

    @classmethod
    def known(cls, tool_id: int) -> "MountedState":
        return cls(MountedKind.KNOWN, tool_id)

    @classmethod
    def unknown(cls) -> "MountedState":
        return cls(MountedKind.UNKNOWN)


class LockState(str, Enum):
    UNLOCKED = "UNLOCKED"
    LOCKED = "LOCKED"
    UNKNOWN = "UNKNOWN"


class TransitionOperation(str, Enum):
    SELECT = "SELECT"
    DROP = "DROP"
    LOCK = "LOCK"
    UNLOCK = "UNLOCK"
    RECOVER = "RECOVER"


class TransitionPhase(str, Enum):
    VALIDATING = "VALIDATING"
    PREHEATING = "PREHEATING"
    PERSISTING_INTENT = "PERSISTING_INTENT"
    DROPPING_CURRENT = "DROPPING_CURRENT"
    ACTIVATING_TARGET = "ACTIVATING_TARGET"
    APPLYING_TARGET_PROFILE = "APPLYING_TARGET_PROFILE"
    PICKING_TARGET = "PICKING_TARGET"
    FINALIZING = "FINALIZING"
    PERSISTING_COMMIT = "PERSISTING_COMMIT"


@dataclass(frozen=True)
class TransitionCheckpoint:
    name: str
    completed: bool

    def __post_init__(self) -> None:
        if not self.name or len(self.name) > 80:
            raise ValueError("checkpoint name must contain 1..80 characters")


@dataclass(frozen=True)
class TransitionRecord:
    transition_id: str
    operation: TransitionOperation
    source: MountedState
    requested_target: Optional[int]
    phase: TransitionPhase = TransitionPhase.VALIDATING
    checkpoints: Tuple[TransitionCheckpoint, ...] = ()
    risk_boundary_crossed: bool = False

    def __post_init__(self) -> None:
        if not self.transition_id or len(self.transition_id) > 96:
            raise ValueError("transition_id must contain 1..96 characters")
        if self.requested_target is not None:
            if isinstance(self.requested_target, bool) or not isinstance(
                self.requested_target, int
            ):
                raise ValueError("requested_target must be an integer")
            if self.requested_target < 0:
                raise ValueError("requested_target must be non-negative")
        names = [checkpoint.name for checkpoint in self.checkpoints]
        if len(names) != len(set(names)):
            raise ValueError("checkpoint names must be unique")

    @property
    def last_completed_checkpoint(self) -> Optional[str]:
        completed = [item.name for item in self.checkpoints if item.completed]
        return completed[-1] if completed else None

    def start_checkpoint(self, name: str, *, crosses_risk: bool = False) -> "TransitionRecord":
        if any(item.name == name for item in self.checkpoints):
            raise ValueError("checkpoint already exists: %s" % (name,))
        checkpoint = TransitionCheckpoint(name=name, completed=False)
        return replace(
            self,
            checkpoints=self.checkpoints + (checkpoint,),
            risk_boundary_crossed=self.risk_boundary_crossed or crosses_risk,
        )

    def complete_checkpoint(self, name: str) -> "TransitionRecord":
        found = False
        updated = []
        for item in self.checkpoints:
            if item.name == name:
                found = True
                if item.completed:
                    raise ValueError("checkpoint already completed: %s" % (name,))
                item = replace(item, completed=True)
            updated.append(item)
        if not found:
            raise ValueError("checkpoint was not started: %s" % (name,))
        return replace(self, checkpoints=tuple(updated))

    def move_to(self, phase: TransitionPhase) -> "TransitionRecord":
        order = tuple(TransitionPhase)
        if order.index(phase) < order.index(self.phase):
            raise ValueError("transition phase cannot move backwards")
        return replace(self, phase=phase)


@dataclass(frozen=True)
class ChangerState:
    schema_version: int
    revision: int
    mode: ChangerMode
    mounted: MountedState
    lock: LockState
    selected: Optional[int]
    transition: Optional[TransitionRecord] = None
    failure: Optional["FailureRecord"] = None

    def __post_init__(self) -> None:
        if self.schema_version < 1:
            raise ValueError("schema_version must be positive")
        if self.revision < 0:
            raise ValueError("revision cannot be negative")
        if self.selected is not None:
            if isinstance(self.selected, bool) or not isinstance(self.selected, int):
                raise ValueError("selected must be an integer")
            if self.selected < 0:
                raise ValueError("selected must be non-negative")

        if self.mode is ChangerMode.IDLE:
            if self.transition is not None or self.failure is not None:
                raise ValueError("IDLE cannot contain transition or failure records")
            if self.mounted.kind is MountedKind.NONE:
                valid = self.lock is LockState.UNLOCKED and self.selected is None
            elif self.mounted.kind is MountedKind.KNOWN:
                valid = (
                    self.lock is LockState.LOCKED
                    and self.selected == self.mounted.tool_id
                )
            else:
                valid = False
            if not valid:
                raise ValueError("IDLE state violates a normal physical-tool invariant")
        elif self.mode is ChangerMode.CHANGING:
            if self.transition is None or self.failure is not None:
                raise ValueError("CHANGING requires only a transition record")
        elif self.mode in (
            ChangerMode.RECOVERY_REQUIRED,
            ChangerMode.SYNCHRONIZING,
        ):
            if self.failure is None or self.transition is not None:
                raise ValueError("recovery modes require only a failure record")

    @classmethod
    def idle_without_tool(cls, revision: int = 0) -> "ChangerState":
        return cls(2, revision, ChangerMode.IDLE, MountedState.none(), LockState.UNLOCKED, None)

    @classmethod
    def idle_with_tool(cls, tool_id: int, revision: int = 0) -> "ChangerState":
        return cls(
            2,
            revision,
            ChangerMode.IDLE,
            MountedState.known(tool_id),
            LockState.LOCKED,
            tool_id,
        )

    def legacy_tool_current(self) -> int:
        """Project state to the v1 status value; never use this to drive state."""
        if self.mode is not ChangerMode.IDLE:
            return -2
        if self.mounted.kind is MountedKind.NONE:
            return -1
        return int(self.mounted.tool_id)


# Avoid a runtime import cycle while retaining useful type information.
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ktcc.toolchange.recovery import FailureRecord
