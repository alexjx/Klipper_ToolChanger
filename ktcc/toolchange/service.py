"""Transactional orchestration for physical tool changes and recovery.

The service is the sole in-memory owner of :class:`ChangerState`.  Machine
effects are deliberately expressed through typed ports; a configured action is
treated as an opaque mechanical risk boundary from the instant it is invoked.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from secrets import token_urlsafe
from typing import Callable, Mapping, Optional
from uuid import uuid4

from ktcc.profiles.service import ProfileService
from ktcc.statistics.service import MechanicalKind
from ktcc.thermal import ThermalService, WaitTarget
from ktcc.toolchange.state import (
    ChangerMode,
    ChangerState,
    LockState,
    MountedKind,
    MountedState,
    TransitionOperation,
    TransitionPhase,
    TransitionRecord,
)
from ktcc.tools import ToolId, ToolRegistry, ToolSpec
from ktcc.toolchange.recovery import (
    MAX_HISTORY_ITEMS,
    MAX_MESSAGE_LENGTH,
    CarriageEstimate,
    FailureCode,
    FailureRecord,
    RecoveryDraft,
    begin_synchronization,
    complete_synchronization,
    synthetic_failure,
)
from ktcc.ports import (
    ActionContext,
    ActionPort,
    Clock,
    ToolchangerEvent,
    EventLevel,
    EventSink,
    ExtrusionProfilePort,
    ShutdownPort,
    StatePersistencePort,
    StatisticsPort,
    ThermalMode,
    ThermalSnapshot,
    ToolchangeReadinessPort,
    VerificationKind,
    VerificationPort,
    VerificationRequest,
)


class ToolChangeStateError(RuntimeError):
    """A normal operation is forbidden by the authoritative changer state."""


class VerificationContradictionError(RuntimeError):
    """A configured verifier contradicted the requested physical invariant."""


class RecoveryAcknowledgementError(ValueError):
    """An operator acknowledgment is absent, stale, or malformed."""


@dataclass(frozen=True)
class RecoveryStatus:
    state: ChangerState
    draft: Optional[RecoveryDraft]
    acknowledgment_token: str
    permitted_commands: tuple[str, ...]
    available_actions: tuple[str, ...]


IdFactory = Callable[[], str]


@dataclass(frozen=True)
class _ThermalBefore:
    snapshot: ThermalSnapshot
    preheat_target: Optional[WaitTarget]


def _new_transition_id() -> str:
    return "tx-" + uuid4().hex


def _new_failure_id() -> str:
    return "failure-" + uuid4().hex


def _new_capability() -> str:
    return token_urlsafe(24)


class ToolChangeService:
    """Execute one durable physical-tool transaction at a time."""

    COMPLETED_EVENT = "toolchange.completed"
    RECOVERY_EVENT = "toolchange.recovery_required"

    def __init__(
        self,
        state: ChangerState,
        registry: ToolRegistry,
        readiness: ToolchangeReadinessPort,
        action: ActionPort,
        thermal: ThermalService,
        profiles: ProfileService,
        extrusion: ExtrusionProfilePort,
        persistence: StatePersistencePort,
        event_sink: EventSink,
        shutdown: ShutdownPort,
        clock: Clock,
        *,
        verification: Optional[VerificationPort] = None,
        transition_id_factory: IdFactory = _new_transition_id,
        failure_id_factory: IdFactory = _new_failure_id,
        capability_factory: IdFactory = _new_capability,
        statistics: Optional[StatisticsPort] = None,
    ) -> None:
        if not isinstance(state, ChangerState):
            raise TypeError("state must be a ChangerState")
        self._state = state
        self._registry = registry
        self._readiness = readiness
        self._action = action
        self._thermal = thermal
        self._profiles = profiles
        self._extrusion = extrusion
        self._persistence = persistence
        self._verification = verification
        self._event_sink = event_sink
        self._shutdown = shutdown
        self._clock = clock
        self._statistics = statistics
        self._transition_id_factory = transition_id_factory
        self._failure_id_factory = failure_id_factory
        self._capability_factory = capability_factory
        self._active_transition: Optional[TransitionRecord] = None
        self._active_capability: Optional[str] = None
        self._transaction_before: Optional[ChangerState] = None
        self._recovery_draft: Optional[RecoveryDraft] = None
        self._recovery_actions: dict[str, ToolSpec] = {}
        for tool in registry:
            name = tool.actions.recovery
            if name:
                if name in self._recovery_actions:
                    raise ValueError("recovery action names must be unique")
                self._recovery_actions[name] = tool
        self._restore_persisted_recovery_draft()

    @property
    def state(self) -> ChangerState:
        return self._state

    def require_ready(self) -> None:
        if self._state.mode is not ChangerMode.IDLE:
            raise ToolChangeStateError(
                "normal commands are blocked while changer state is %s"
                % self._state.mode.value
            )

    def select(self, target: ToolId | int) -> ChangerState:
        self.require_ready()
        target_spec = self._registry[target]
        if (
            self._state.mounted.kind is MountedKind.KNOWN
            and self._state.mounted.tool_id == target_spec.id.value
        ):
            return self._state
        return self._execute(target_spec)

    def drop_all(self) -> ChangerState:
        self.require_ready()
        if self._state.mounted.kind is MountedKind.NONE:
            return self._state
        if self._state.mounted.kind is not MountedKind.KNOWN:
            raise ToolChangeStateError("cannot drop an unknown mounted tool")
        return self._execute(None)

    def declare_current_tool(self, value: int) -> ChangerState:
        """Handle an explicit current-tool declaration from the facade.

        Normal selection never uses this method.  It exists for the preserved
        ``SAVE_CURRENT_TOOL``/manual lock commands and durably updates the
        existing current-tool value so the service remains the sole state owner.
        """

        self.require_ready()
        if isinstance(value, bool) or not isinstance(value, int) or value < -2:
            raise ValueError("current tool must be an integer at least -2")
        previous = self._selected_tool(self._state)
        revision = self._state.revision + 1
        if value == -2:
            self._thermal.off_all()
            state = ChangerState(
                schema_version=2,
                revision=revision,
                mode=ChangerMode.RECOVERY_REQUIRED,
                mounted=MountedState.unknown(),
                lock=LockState.UNKNOWN,
                selected=None,
                failure=synthetic_failure(
                    FailureCode.PERSISTED_RECOVERY,
                    "manual current-tool declaration requires reconciliation",
                    revision=revision,
                ),
            )
        elif value == -1:
            state = ChangerState.idle_without_tool(revision)
        else:
            self._registry[value]
            state = ChangerState.idle_with_tool(value, revision)
        self._persistence.save_state(state)
        self._state = state
        self._persistence.mirror_tool_current(value)
        self._statistics_call(
            "selection change",
            lambda: self._statistics.selection_changed(
                previous,
                None if value < 0 else ToolId(value),
            ),
        )
        return state

    def _execute(self, target: Optional[ToolSpec]) -> ChangerState:
        original = self._state
        source = self._source_spec(original)
        thermal_before: Optional[_ThermalBefore] = None
        transition: Optional[TransitionRecord] = None
        try:
            self._readiness.require_ready()
            self._validate_actions(source, target)
            if source is not None:
                # SET_RETRACTION would erase Klipper's G10 bookkeeping.  We
                # therefore require the source profile to already be current,
                # observe first, and only reactivate the named source extruder.
                if not self._profiles.applied(source.id).is_current:
                    raise ToolChangeStateError("source profile is not applied")
                if self._extrusion.observe_retracted():
                    self._extrusion.activate_extruder(source.extruder)
                    self._extrusion.unretract()
                    if self._extrusion.observe_retracted():
                        raise ToolChangeStateError("automatic G11 did not clear retraction")

            if target is not None:
                self._profiles.effective(target.id)
                before = self._thermal.snapshot(target.id)
                thermal_before = _ThermalBefore(
                    before, self._thermal.preheat_target(target.id)
                )
                self._thermal.preheat(target.id, target=WaitTarget.ACTIVE)

            transition = TransitionRecord(
                transition_id=self._transition_id_factory(),
                operation=(
                    TransitionOperation.SELECT if target is not None else TransitionOperation.DROP
                ),
                source=original.mounted,
                requested_target=None if target is None else target.id.value,
                phase=TransitionPhase.PERSISTING_INTENT,
            )
            intent = ChangerState(
                schema_version=2,
                revision=original.revision + 1,
                mode=ChangerMode.CHANGING,
                mounted=original.mounted,
                lock=original.lock,
                selected=original.selected,
                transition=transition,
            )
            capability = self._capability_factory()
            if not isinstance(capability, str) or not capability:
                raise ValueError("capability factory must return a non-empty string")
            self._persistence.save_state(intent)
            self._state = intent
            self._active_transition = transition
            self._active_capability = capability
            self._transaction_before = original
        except BaseException:
            self._state = original
            self._active_transition = None
            self._active_capability = None
            self._transaction_before = None
            if target is not None and thermal_before is not None:
                self._restore_thermal(target, thermal_before)
            raise

        try:
            if source is not None:
                self._run_mechanical_action(
                    source,
                    source.actions.dropoff,
                    TransitionPhase.DROPPING_CURRENT,
                    "dropoff_action",
                    target,
                )
                self._thermal.schedule_inactivity(source.id)
            if target is not None:
                self._move_phase(TransitionPhase.ACTIVATING_TARGET)
                self._extrusion.activate_extruder(target.extruder)
                self._move_phase(TransitionPhase.APPLYING_TARGET_PROFILE)
                self._profiles.reapply(target.id)
                self._run_mechanical_action(
                    target,
                    target.actions.pickup,
                    TransitionPhase.PICKING_TARGET,
                    "pickup_action",
                    target,
                )
                self._move_phase(TransitionPhase.FINALIZING)
                self._verify(
                    VerificationRequest(VerificationKind.MOUNTED, target.id),
                    tool=target,
                    action_name=target.actions.verify_mounted,
                    expected_target=target,
                )
                self._profiles.apply_offset(target.id)
                self._thermal.activate(target.id)
                final = ChangerState.idle_with_tool(
                    target.id.value, revision=self._state.revision + 1
                )
            else:
                self._move_phase(TransitionPhase.FINALIZING)
                self._verify(
                    VerificationRequest(VerificationKind.UNMOUNTED),
                    tool=source,
                    action_name=(
                        None if source is None else source.actions.verify_unmounted
                    ),
                    expected_target=None,
                )
                final = ChangerState.idle_without_tool(self._state.revision + 1)

            self._move_phase(TransitionPhase.PERSISTING_COMMIT)
            self._persistence.save_state(final)
            self._state = final
            self._persistence.mirror_tool_current(final.legacy_tool_current())
            self._statistics_call(
                "selection change",
                lambda: self._statistics.selection_changed(
                    self._selected_tool(original), self._selected_tool(final)
                ),
            )
            self._clear_transition_capability()
            # Observability is deliberately secondary: a logging/event sink
            # failure cannot invalidate an already committed physical state.
            try:
                self._event_sink.emit(
                    ToolchangerEvent(
                        self.COMPLETED_EVENT,
                        EventLevel.INFO,
                        "tool change transaction completed",
                        None if target is None else target.id,
                        transition.transition_id,
                        TransitionPhase.PERSISTING_COMMIT,
                    )
                )
            except BaseException:
                pass
            return final
        except BaseException as error:
            if (
                self._active_transition is not None
                and self._active_transition.risk_boundary_crossed
            ):
                self._enter_recovery(error)
            else:
                # An intent may already be durable, but no opaque mechanical
                # action has started.  Restore the original durable invariant;
                # a failed rollback remains an interrupted intent on restart,
                # while this live session still truthfully retains the original
                # physical estimate and does not invoke shutdown.
                try:
                    self._persistence.save_state(original)
                except BaseException:
                    pass
                self._state = original
                self._clear_transition_capability()
                if target is not None and thermal_before is not None:
                    self._restore_thermal(target, thermal_before)
            raise

    def _restore_thermal(self, target: ToolSpec, before: _ThermalBefore) -> None:
        try:
            snapshot = before.snapshot
            self._thermal.set_temperature(
                target.id,
                active_target=snapshot.active_target,
                standby_target=snapshot.standby_target,
                mode=None if snapshot.mode is ThermalMode.PREHEAT else snapshot.mode,
            )
            if snapshot.mode is ThermalMode.PREHEAT:
                preheat_target = before.preheat_target or WaitTarget.ACTIVE
                self._thermal.preheat(target.id, target=preheat_target)
        except BaseException:
            # Preserve the original command error.  Thermal restoration is a
            # best-effort compensating effect before mechanical risk.
            pass

    def _source_spec(self, state: ChangerState) -> Optional[ToolSpec]:
        if state.mounted.kind is MountedKind.NONE:
            return None
        if state.mounted.kind is not MountedKind.KNOWN:
            raise ToolChangeStateError("normal operation requires known mounted state")
        return self._registry[state.mounted.tool_id]

    @staticmethod
    def _validate_actions(source: Optional[ToolSpec], target: Optional[ToolSpec]) -> None:
        if source is not None and not source.actions.dropoff:
            raise ToolChangeStateError("source tool has no dropoff action")
        if target is not None and not target.actions.pickup:
            raise ToolChangeStateError("target tool has no pickup action")

    def _move_phase(self, phase: TransitionPhase) -> None:
        transition = self._active_transition
        if transition is None:
            raise ToolChangeStateError("no active transition")
        transition = transition.move_to(phase)
        self._active_transition = transition
        self._state = replace(self._state, transition=transition)

    def _run_mechanical_action(
        self,
        tool: ToolSpec,
        action_name: str,
        phase: TransitionPhase,
        checkpoint: str,
        target: Optional[ToolSpec],
    ) -> None:
        self._move_phase(phase)
        transition = self._active_transition.start_checkpoint(
            checkpoint, crosses_risk=True
        )
        self._active_transition = transition
        self._state = replace(self._state, transition=transition)
        context = self._action_context(tool, target)
        kind = (
            MechanicalKind.UNMOUNT
            if phase is TransitionPhase.DROPPING_CURRENT
            else MechanicalKind.MOUNT
        )
        self._statistics_call(
            "mechanical action start",
            lambda: self._statistics.mechanical_started(
                kind, tool.id, transition.transition_id
            ),
        )
        self._action.run_from_command(action_name, context)
        self._statistics_call(
            "mechanical action completion",
            lambda: self._statistics.mechanical_completed(
                kind, tool.id, transition.transition_id
            ),
        )
        transition = self._active_transition.complete_checkpoint(checkpoint)
        self._active_transition = transition
        self._state = replace(self._state, transition=transition)

    def _action_context(
        self, tool: ToolSpec, target: Optional[ToolSpec]
    ) -> ActionContext:
        transition = self._active_transition
        if transition is None:
            raise ToolChangeStateError("no active transition")
        expected = (
            ChangerState.idle_without_tool(self._state.revision + 1)
            if target is None
            else ChangerState.idle_with_tool(target.id.value, self._state.revision + 1)
        )
        return ActionContext(
            schema_version=2,
            tool=tool,
            thermal=self._thermal.snapshot(tool.id),
            changer_before=self._transaction_before or self._state,
            expected_after=expected,
            transition_id=transition.transition_id,
            phase=transition.phase,
            last_checkpoint=transition.last_completed_checkpoint,
            checkpoint_capability=self._active_capability,
            effective_profile=self._profiles.effective(tool.id).profile,
        )

    def checkpoint(
        self,
        transition_id: str,
        capability: str,
        name: str,
        *,
        completed: bool,
    ) -> ChangerState:
        transition = self._active_transition
        if (
            transition is None
            or transition.transition_id != transition_id
            or capability != self._active_capability
        ):
            raise ToolChangeStateError("checkpoint does not match the active transition")
        transition = (
            transition.complete_checkpoint(name)
            if completed
            else transition.start_checkpoint(name)
        )
        updated = replace(self._state, transition=transition)
        self._persistence.save_state(updated)
        self._active_transition = transition
        self._state = updated
        return updated

    def _verify(
        self,
        request: VerificationRequest,
        *,
        tool: Optional[ToolSpec] = None,
        action_name: Optional[str] = None,
        expected_target: Optional[ToolSpec] = None,
    ) -> None:
        if action_name:
            if tool is None:
                raise ToolChangeStateError(
                    "configured verification action requires a tool"
                )
            self._action.run_from_command(
                action_name, self._action_context(tool, expected_target)
            )
        if self._verification is None:
            return
        result = self._verification.verify(request)
        if not result.matches:
            raise VerificationContradictionError(result.detail or "verification contradicted")

    def _enter_recovery(self, error: BaseException) -> None:
        transition = self._active_transition
        if transition is None:
            # Defensive fallback: this path is entered only after intent.
            return
        completed = tuple(
            checkpoint.name for checkpoint in transition.checkpoints if checkpoint.completed
        )
        try:
            failure_id = self._failure_id_factory()
            if not isinstance(failure_id, str) or not failure_id:
                raise ValueError("invalid failure id")
        except BaseException:
            failure_id = ("failure-" + transition.transition_id)[:96]
        try:
            occurred = self._clock.monotonic()
        except BaseException:
            occurred = None
        action_name = self._current_action_name(transition)
        failure = FailureRecord(
            failure_id=failure_id,
            transition_id=transition.transition_id,
            operation=transition.operation,
            source_mounted=transition.source,
            source_lock=(
                self._transaction_before.lock
                if self._transaction_before is not None
                else self._state.lock
            ),
            requested_target=transition.requested_target,
            phase=transition.phase.value,
            action_name=action_name[:80] if action_name is not None else None,
            last_completed_checkpoint=transition.last_completed_checkpoint,
            completed_checkpoints=completed[-MAX_HISTORY_ITEMS:],
            risk_boundary_crossed=transition.risk_boundary_crossed,
            error_code=FailureCode.TRANSITION_FAILED.value,
            exception_type=type(error).__name__[:120],
            message=str(error)[:MAX_MESSAGE_LENGTH],
            mounted_estimate=MountedState.unknown(),
            lock_estimate=LockState.UNKNOWN,
            carriage_estimate=CarriageEstimate.UNKNOWN,
            heater_snapshot=self._safe_heater_snapshot(),
            state_revision=self._state.revision,
            persisted_revision=self._state.revision,
            occurred_monotonic=occurred,
        )
        # The observer is updated only after the durable commit and mirror both
        # succeed.  A commit-path failure must therefore close the selection
        # that was active before this transaction, not the unpublished final
        # state temporarily assigned before the mirror write.
        previous_selection = self._selected_tool(
            self._transaction_before or self._state
        )
        recovery = ChangerState(
            schema_version=2,
            revision=self._state.revision + 1,
            mode=ChangerMode.RECOVERY_REQUIRED,
            mounted=MountedState.unknown(),
            lock=LockState.UNKNOWN,
            selected=None,
            failure=failure,
        )
        self._state = recovery
        self._statistics_call(
            "recovery selection close",
            lambda: self._statistics.selection_changed(previous_selection, None),
        )
        try:
            try:
                self._thermal.off_all()
            except BaseException:
                pass
            try:
                self._persistence.save_state(recovery)
            except BaseException:
                pass
            try:
                self._persistence.mirror_tool_current(-2)
            except BaseException:
                pass
        finally:
            self._clear_transition_capability()
            try:
                self._shutdown.shutdown("tool change failed after mechanical risk")
            except BaseException:
                pass

    def _current_action_name(self, transition: TransitionRecord) -> Optional[str]:
        if transition.phase is TransitionPhase.DROPPING_CURRENT:
            if transition.source.kind is MountedKind.KNOWN:
                return self._registry[transition.source.tool_id].actions.dropoff
        if transition.phase is TransitionPhase.PICKING_TARGET:
            if transition.requested_target is not None:
                return self._registry[transition.requested_target].actions.pickup
        return None

    def _safe_heater_snapshot(self) -> tuple[tuple[str, float], ...]:
        try:
            return self._heater_snapshot()
        except BaseException:
            return ()

    def _heater_snapshot(self) -> tuple[tuple[str, float], ...]:
        values: dict[str, float] = {}
        for tool in self._registry:
            snapshot = self._thermal.snapshot(tool.id)
            if snapshot.mode is ThermalMode.OFF:
                target = 0.0
            elif snapshot.mode is ThermalMode.STANDBY:
                target = snapshot.standby_target
            else:
                target = snapshot.active_target
            values[str(tool.heater)] = target
        return tuple(values.items())

    # Recovery session -------------------------------------------------

    @staticmethod
    def acknowledgment_token(failure_id: str) -> str:
        if not failure_id:
            raise ValueError("failure_id is required")
        return "ACK:" + failure_id

    def recovery_status(self) -> RecoveryStatus:
        failure = self._require_recovery()
        return RecoveryStatus(
            self._state,
            self._recovery_draft,
            self.acknowledgment_token(failure.failure_id),
            ("STATUS", "ABORT", "RUN", "RECONCILE", "CONFIRM"),
            tuple(sorted(self._recovery_actions)),
        )

    def recovery_abort(self, failure_id: str) -> ChangerState:
        failure = self._require_failure_id(failure_id)
        self._thermal.off_all()
        updated = replace(
            failure,
            recovery_action_history=self._append_history(
                failure.recovery_action_history, "abort:heaters_off"
            ),
        )
        state = replace(self._state, revision=self._state.revision + 1, failure=updated)
        self._persistence.save_state(state)
        self._state = state
        self._recovery_draft = None
        return state

    def run_recovery_action(self, failure_id: str, action_name: str) -> ChangerState:
        failure = self._require_failure_id(failure_id)
        try:
            tool = self._recovery_actions[action_name]
        except KeyError as exc:
            raise KeyError("unknown registered recovery action: %s" % action_name) from exc
        started = replace(
            failure,
            recovery_action_history=self._append_history(
                failure.recovery_action_history, "start:" + action_name
            ),
        )
        state = replace(self._state, revision=self._state.revision + 1, failure=started)
        self._persistence.save_state(state)
        self._state = state
        # A mechanical recovery action invalidates an earlier observation.
        self._recovery_draft = None
        try:
            self._action.run_from_command(
                action_name, self._recovery_action_context(tool)
            )
        except BaseException as error:
            self._record_recovery_history(
                "failed:%s:%s" % (action_name, type(error).__name__)
            )
            raise
        return self._record_recovery_history("completed:" + action_name)

    def reconcile(
        self,
        failure_id: str,
        mounted: MountedState,
        lock: LockState,
        *,
        carriage: CarriageEstimate = CarriageEstimate.UNKNOWN,
        acknowledgment: str,
        note: Optional[str] = None,
    ) -> RecoveryDraft:
        self._require_failure_id(failure_id)
        if acknowledgment != self.acknowledgment_token(failure_id):
            raise RecoveryAcknowledgementError("acknowledgment token does not match failure")
        draft = RecoveryDraft(failure_id, mounted, lock, carriage, note)
        draft.validate([tool.id.value for tool in self._registry])
        failure = self._state.failure
        recorded = replace(
            failure,
            mounted_estimate=mounted,
            lock_estimate=lock,
            carriage_estimate=carriage,
            operator_observation=note,
            recovery_action_history=self._append_history(
                failure.recovery_action_history, "reconciled"
            ),
        )
        state = replace(
            self._state, revision=self._state.revision + 1, failure=recorded
        )
        self._persistence.save_state(state)
        self._state = state
        self._recovery_draft = draft
        return draft

    def confirm_recovery(self, failure_id: str) -> ChangerState:
        self._require_failure_id(failure_id)
        draft = self._recovery_draft
        if draft is None or draft.failure_id != failure_id:
            raise ToolChangeStateError("no acknowledged recovery draft is available")
        original = self._state
        try:
            syncing = begin_synchronization(
                original, draft, [tool.id.value for tool in self._registry]
            )
            self._persistence.save_state(syncing)
            self._state = syncing
            request = (
                VerificationRequest(VerificationKind.UNMOUNTED)
                if draft.mounted.kind is MountedKind.NONE
                else VerificationRequest(
                    VerificationKind.MOUNTED, ToolId(draft.mounted.tool_id)
                )
            )
            self._verify(request)
            if draft.mounted.kind is MountedKind.KNOWN:
                tool = self._registry[draft.mounted.tool_id]
                self._extrusion.activate_extruder(tool.extruder)
                self._profiles.reapply(tool.id)
                self._profiles.apply_offset(tool.id)
            idle = complete_synchronization(syncing)
            self._persistence.save_state(idle)
            self._state = idle
            self._persistence.mirror_tool_current(idle.legacy_tool_current())
            self._statistics_call(
                "recovery selection confirmation",
                lambda: self._statistics.selection_changed(
                    self._selected_tool(original), self._selected_tool(idle)
                ),
            )
            self._recovery_draft = None
            return idle
        except BaseException as error:
            self._synchronization_failed(original, draft, error)
            raise

    def _synchronization_failed(
        self, original: ChangerState, draft: RecoveryDraft, error: BaseException
    ) -> None:
        failure = replace(
            original.failure,
            error_code=FailureCode.SYNCHRONIZATION_FAILED.value,
            exception_type=type(error).__name__[:120],
            message=str(error)[:MAX_MESSAGE_LENGTH],
            mounted_estimate=draft.mounted,
            lock_estimate=draft.lock,
            carriage_estimate=draft.carriage,
            state_revision=self._state.revision,
        )
        recovery = ChangerState(
            schema_version=2,
            revision=self._state.revision + 1,
            mode=ChangerMode.RECOVERY_REQUIRED,
            mounted=MountedState.unknown(),
            lock=LockState.UNKNOWN,
            selected=None,
            failure=failure,
        )
        self._state = recovery
        try:
            self._persistence.save_state(recovery)
        except BaseException:
            pass
        try:
            self._persistence.mirror_tool_current(-2)
        except BaseException:
            pass

    def _require_recovery(self) -> FailureRecord:
        if self._state.mode is not ChangerMode.RECOVERY_REQUIRED:
            raise ToolChangeStateError("changer is not awaiting recovery")
        return self._state.failure

    def _require_failure_id(self, failure_id: str) -> FailureRecord:
        failure = self._require_recovery()
        if failure.failure_id != failure_id:
            raise ToolChangeStateError("failure_id does not match active recovery")
        return failure

    def _record_recovery_history(self, entry: str) -> ChangerState:
        failure = self._require_recovery()
        updated = replace(
            failure,
            recovery_action_history=self._append_history(
                failure.recovery_action_history, entry
            ),
        )
        state = replace(self._state, revision=self._state.revision + 1, failure=updated)
        try:
            self._persistence.save_state(state)
        finally:
            self._state = state
        return state

    @staticmethod
    def _append_history(history: tuple[str, ...], entry: str) -> tuple[str, ...]:
        return (history + (entry,))[-MAX_HISTORY_ITEMS:]

    def _recovery_action_context(self, tool: ToolSpec) -> ActionContext:
        failure = self._require_recovery()
        transition_id = failure.transition_id or failure.failure_id
        return ActionContext(
            2,
            tool,
            self._thermal.snapshot(tool.id),
            self._state,
            self._state,
            transition_id,
            TransitionPhase.FINALIZING,
            failure.last_completed_checkpoint,
            effective_profile=self._profiles.effective(tool.id).profile,
        )

    def _clear_transition_capability(self) -> None:
        self._active_transition = None
        self._active_capability = None
        self._transaction_before = None

    @staticmethod
    def _selected_tool(state: ChangerState) -> Optional[ToolId]:
        return None if state.selected is None else ToolId(state.selected)

    def _statistics_call(self, operation: str, callback: Callable[[], None]) -> None:
        """Notify the optional observer without changing machine semantics."""

        if self._statistics is None:
            return
        try:
            callback()
        except BaseException as error:
            try:
                transition = self._active_transition
                self._event_sink.emit(
                    ToolchangerEvent(
                        "statistics.observer_failed",
                        EventLevel.WARNING,
                        "%s failed: %s" % (operation, type(error).__name__),
                        transition_id=(
                            None if transition is None else transition.transition_id
                        ),
                        phase=None if transition is None else transition.phase,
                    )
                )
            except BaseException:
                pass

    def _restore_persisted_recovery_draft(self) -> None:
        if self._state.mode is not ChangerMode.RECOVERY_REQUIRED:
            return
        failure = self._state.failure
        if (
            not failure.recovery_action_history
            or failure.recovery_action_history[-1] != "reconciled"
        ):
            return
        draft = RecoveryDraft(
            failure.failure_id,
            failure.mounted_estimate,
            failure.lock_estimate,
            failure.carriage_estimate,
            failure.operator_observation,
        )
        try:
            draft.validate([tool.id.value for tool in self._registry])
        except ValueError:
            return
        self._recovery_draft = draft
