"""Failure recording, restart normalization, and recovery reconciliation."""

from dataclasses import dataclass, replace
from enum import Enum
import math
from typing import Optional, Sequence, Tuple

from ktcc.toolchange.state import (
    ChangerMode,
    ChangerState,
    LockState,
    MountedKind,
    MountedState,
    TransitionOperation,
)


MAX_MESSAGE_LENGTH = 512
MAX_HISTORY_ITEMS = 32
MAX_SNAPSHOT_ITEMS = 32


class FailureCode(str, Enum):
    INTERRUPTED = "INTERRUPTED"
    MALFORMED_PERSISTENCE = "MALFORMED_PERSISTENCE"
    PERSISTED_RECOVERY = "PERSISTED_RECOVERY"
    UNKNOWN_PERSISTED_TOOL = "UNKNOWN_PERSISTED_TOOL"
    TRANSITION_FAILED = "TRANSITION_FAILED"
    SYNCHRONIZATION_FAILED = "SYNCHRONIZATION_FAILED"


class CarriageEstimate(str, Enum):
    SOURCE_DOCK = "SOURCE_DOCK"
    TARGET_DOCK = "TARGET_DOCK"
    FREE = "FREE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class FailureRecord:
    failure_id: str
    transition_id: Optional[str]
    operation: TransitionOperation
    source_mounted: MountedState
    source_lock: LockState
    requested_target: Optional[int]
    phase: str
    action_name: Optional[str]
    last_completed_checkpoint: Optional[str]
    completed_checkpoints: Tuple[str, ...]
    risk_boundary_crossed: bool
    error_code: str
    exception_type: Optional[str]
    message: str
    mounted_estimate: MountedState
    lock_estimate: LockState
    carriage_estimate: CarriageEstimate
    heater_snapshot: Tuple[Tuple[str, float], ...] = ()
    state_revision: int = 0
    persisted_revision: Optional[int] = None
    occurred_monotonic: Optional[float] = None
    recovery_action_history: Tuple[str, ...] = ()
    operator_observation: Optional[str] = None

    def __post_init__(self) -> None:
        for label, value, maximum in (
            ("failure_id", self.failure_id, 96),
            ("phase", self.phase, 80),
            ("error_code", self.error_code, 80),
        ):
            if not value or len(value) > maximum:
                raise ValueError("%s must contain 1..%d characters" % (label, maximum))
        optional_fields = (
            ("transition_id", self.transition_id, 96),
            ("action_name", self.action_name, 80),
            ("last_completed_checkpoint", self.last_completed_checkpoint, 80),
            ("exception_type", self.exception_type, 120),
            ("operator_observation", self.operator_observation, MAX_MESSAGE_LENGTH),
        )
        for label, value, maximum in optional_fields:
            if value is not None and len(value) > maximum:
                raise ValueError("%s exceeds persisted size limit" % (label,))
        if len(self.message) > MAX_MESSAGE_LENGTH:
            raise ValueError("message exceeds persisted size limit")
        if len(self.completed_checkpoints) > MAX_HISTORY_ITEMS:
            raise ValueError("too many persisted checkpoints")
        if len(self.recovery_action_history) > MAX_HISTORY_ITEMS:
            raise ValueError("too many persisted recovery history items")
        if len(self.heater_snapshot) > MAX_SNAPSHOT_ITEMS:
            raise ValueError("too many persisted heater snapshot items")
        if self.state_revision < 0 or (
            self.persisted_revision is not None and self.persisted_revision < 0
        ):
            raise ValueError("revisions cannot be negative")
        if self.requested_target is not None and (
            isinstance(self.requested_target, bool)
            or not isinstance(self.requested_target, int)
        ):
            raise ValueError("requested_target must be an integer")
        if self.requested_target is not None and self.requested_target < 0:
            raise ValueError("requested_target must be non-negative")
        if len(set(self.completed_checkpoints)) != len(self.completed_checkpoints):
            raise ValueError("completed checkpoints must be unique")
        for value in self.completed_checkpoints + self.recovery_action_history:
            if not value or len(value) > 120:
                raise ValueError("persisted history entries must contain 1..120 characters")
        for name, target in self.heater_snapshot:
            if not name or len(name) > 120:
                raise ValueError("invalid heater name")
            if (
                isinstance(target, bool)
                or not isinstance(target, (int, float))
                or not math.isfinite(target)
            ):
                raise ValueError("heater targets must be finite numbers")
        if self.occurred_monotonic is not None and (
            isinstance(self.occurred_monotonic, bool)
            or not isinstance(self.occurred_monotonic, (int, float))
            or not math.isfinite(self.occurred_monotonic)
            or self.occurred_monotonic < 0
        ):
            raise ValueError("occurred_monotonic must be a non-negative finite number")


@dataclass(frozen=True)
class RecoveryDraft:
    failure_id: str
    mounted: MountedState
    lock: LockState
    carriage: CarriageEstimate = CarriageEstimate.UNKNOWN
    note: Optional[str] = None

    def validate(self, known_tool_ids: Sequence[int]) -> None:
        if not self.failure_id:
            raise ValueError("failure_id is required")
        known = frozenset(known_tool_ids)
        if self.mounted.kind is MountedKind.NONE:
            valid = self.lock is LockState.UNLOCKED
        elif self.mounted.kind is MountedKind.KNOWN:
            valid = (
                self.lock is LockState.LOCKED
                and self.mounted.tool_id in known
            )
        else:
            valid = False
        if not valid:
            raise ValueError(
                "recovery may confirm only NONE+UNLOCKED or KNOWN(existing tool)+LOCKED"
            )
        if self.note is not None and len(self.note) > MAX_MESSAGE_LENGTH:
            raise ValueError("recovery note exceeds persisted size limit")


def begin_synchronization(
    state: ChangerState,
    draft: RecoveryDraft,
    known_tool_ids: Sequence[int],
) -> ChangerState:
    if state.mode is not ChangerMode.RECOVERY_REQUIRED:
        raise ValueError("state is not awaiting recovery")
    if state.failure.failure_id != draft.failure_id:
        raise ValueError("failure_id does not match active recovery")
    draft.validate(known_tool_ids)
    failure = replace(state.failure, operator_observation=draft.note)
    selected = draft.mounted.tool_id if draft.mounted.kind is MountedKind.KNOWN else None
    return ChangerState(
        schema_version=state.schema_version,
        revision=state.revision + 1,
        mode=ChangerMode.SYNCHRONIZING,
        mounted=draft.mounted,
        lock=draft.lock,
        selected=selected,
        failure=failure,
    )


def complete_synchronization(state: ChangerState) -> ChangerState:
    if state.mode is not ChangerMode.SYNCHRONIZING:
        raise ValueError("state is not synchronizing")
    if state.mounted.kind is MountedKind.NONE:
        return ChangerState.idle_without_tool(state.revision + 1)
    if state.mounted.kind is MountedKind.KNOWN:
        return ChangerState.idle_with_tool(state.mounted.tool_id, state.revision + 1)
    raise ValueError("cannot complete synchronization with unknown mounted state")


def interrupted_restart(state: ChangerState) -> ChangerState:
    """Conservatively normalize an incomplete persisted transition on startup."""
    if state.mode is not ChangerMode.CHANGING:
        return state
    transition = state.transition
    completed = tuple(
        item.name for item in transition.checkpoints if item.completed
    )
    failure = FailureRecord(
        failure_id="interrupted-%s" % (transition.transition_id,),
        transition_id=transition.transition_id,
        operation=transition.operation,
        source_mounted=transition.source,
        source_lock=state.lock,
        requested_target=transition.requested_target,
        phase=transition.phase.value,
        action_name=None,
        last_completed_checkpoint=transition.last_completed_checkpoint,
        completed_checkpoints=completed,
        risk_boundary_crossed=transition.risk_boundary_crossed,
        error_code=FailureCode.INTERRUPTED.value,
        exception_type=None,
        message="Klipper restarted before the tool-change transaction committed",
        mounted_estimate=MountedState.unknown(),
        lock_estimate=LockState.UNKNOWN,
        carriage_estimate=CarriageEstimate.UNKNOWN,
        state_revision=state.revision,
        persisted_revision=state.revision,
    )
    return ChangerState(
        schema_version=state.schema_version,
        revision=state.revision + 1,
        mode=ChangerMode.RECOVERY_REQUIRED,
        mounted=MountedState.unknown(),
        lock=LockState.UNKNOWN,
        selected=None,
        failure=failure,
    )


def synthetic_failure(
    code: FailureCode,
    message: str,
    *,
    revision: int = 0,
    mounted: Optional[MountedState] = None,
    lock: LockState = LockState.UNKNOWN,
) -> FailureRecord:
    return FailureRecord(
        failure_id="startup-%s" % (code.value.lower(),),
        transition_id=None,
        operation=TransitionOperation.RECOVER,
        source_mounted=mounted or MountedState.unknown(),
        source_lock=lock,
        requested_target=None,
        phase="STARTUP",
        action_name=None,
        last_completed_checkpoint=None,
        completed_checkpoints=(),
        risk_boundary_crossed=False,
        error_code=code.value,
        exception_type=None,
        message=message[:MAX_MESSAGE_LENGTH],
        mounted_estimate=mounted or MountedState.unknown(),
        lock_estimate=lock,
        carriage_estimate=CarriageEstimate.UNKNOWN,
        state_revision=revision,
        persisted_revision=revision,
    )
