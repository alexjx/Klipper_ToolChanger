from __future__ import annotations

from dataclasses import replace

import pytest

from fakes import (
    FakeActionPort,
    FakeClock,
    FakeEventSink,
    FakeExtrusionProfilePort,
    FakeMachinePort,
    FakeProfilePersistencePort,
    FakeSchedulerPort,
    FakeShutdownPort,
    FakeStatePersistencePort,
    FakeThermalPort,
    FakeToolchangeReadinessPort,
    FakeVerificationPort,
    InjectedFailure,
)
from ktcc.profiles.service import ProfileService
from ktcc.statistics import MechanicalKind
from ktcc.thermal import ThermalService
from ktcc.toolchange.service import (
    RecoveryAcknowledgementError,
    ToolChangeService,
    ToolChangeStateError,
    VerificationContradictionError,
)
from ktcc.toolchange.state import (
    ChangerMode,
    ChangerState,
    LockState,
    MountedState,
    TransitionOperation,
    TransitionPhase,
    TransitionRecord,
)
from ktcc.tools import (
    DockSpec,
    ExtruderRef,
    HeaterRef,
    ToolActions,
    ToolId,
    ToolRegistry,
    ToolSpec,
    Vector3,
)
from ktcc.profiles.models import ThermalPolicy, ToolProfile
from ktcc.toolchange.recovery import CarriageEstimate, FailureCode, synthetic_failure
from ktcc.ports import HeaterObservation, VerificationKind, VerificationRequest, VerificationResult


class RecordingStatistics:
    def __init__(self):
        self.calls = []

    def mechanical_started(self, kind, tool_id, transition_id):
        self.calls.append(("started", kind, tool_id, transition_id))

    def mechanical_completed(self, kind, tool_id, transition_id):
        self.calls.append(("completed", kind, tool_id, transition_id))

    def selection_changed(self, previous, current):
        self.calls.append(("selection", previous, current))

    def lock_completed(self):
        pass

    def unlock_completed(self):
        pass

    def thermal_changed(self, tool_id, previous, current):
        pass


def _tool(number: int, *, configured_verification: bool = False) -> ToolSpec:
    name = "extruder" if number == 0 else f"extruder{number}"
    return ToolSpec(
        ToolId(number),
        ExtruderRef(name),
        HeaterRef(name),
        None,
        DockSpec(Vector3(number, 0, 0), Vector3(number, 1, 0)),
        Vector3(number / 10, 0, 0),
        ToolActions(
            pickup=f"pickup_{number}",
            dropoff=f"dropoff_{number}",
            verify_mounted=(
                f"verify_mounted_{number}" if configured_verification else None
            ),
            verify_unmounted=(
                f"verify_unmounted_{number}" if configured_verification else None
            ),
            recovery=f"recover_{number}",
        ),
        ToolProfile(
            thermal_policy=ThermalPolicy(
                default_active=200 + number,
                default_standby=140 + number,
                idle_to_standby_time=10,
                standby_to_off_time=20,
            )
        ),
    )


def _build(
    *,
    state: ChangerState | None = None,
    retracted: bool = False,
    verification=None,
    action=None,
    statistics=None,
    configured_verification=False,
):
    trace = []
    registry = ToolRegistry([
        _tool(0, configured_verification=configured_verification),
        _tool(1, configured_verification=configured_verification),
    ])
    extrusion = FakeExtrusionProfilePort(
        {tool.extruder for tool in registry}, trace, retracted=retracted
    )
    machine = FakeMachinePort(registry.ids, set(), trace)
    profile_persistence = FakeProfilePersistencePort(trace)
    profiles = ProfileService(registry, extrusion, machine, profile_persistence)
    thermal_port = FakeThermalPort(
        {
            tool.heater: HeaterObservation(20, 0, False)
            for tool in registry
        },
        trace,
    )
    scheduler = FakeSchedulerPort(
        {
            callback
            for tool in registry
            for callback in (
                ThermalService.standby_callback_id(tool.heater),
                ThermalService.off_callback_id(tool.heater),
            )
        },
        trace,
    )
    clock = FakeClock(12)
    thermal = ThermalService(registry, thermal_port, scheduler, clock)
    readiness = FakeToolchangeReadinessPort(trace)
    action = action or FakeActionPort(
        {
            f"pickup_{number}" for number in range(2)
        }
        | {f"dropoff_{number}" for number in range(2)}
        | {f"recover_{number}" for number in range(2)}
        | (
            {
                f"verify_mounted_{number}" for number in range(2)
            }
            | {f"verify_unmounted_{number}" for number in range(2)}
            if configured_verification else set()
        ),
        trace,
    )
    persistence = FakeStatePersistencePort(state, trace)
    events = FakeEventSink(
        {
            ToolChangeService.COMPLETED_EVENT,
            ToolChangeService.RECOVERY_EVENT,
            "statistics.observer_failed",
        },
        trace,
    )
    shutdown = FakeShutdownPort(trace)
    service = ToolChangeService(
        state or ChangerState.idle_with_tool(0),
        registry,
        readiness,
        action,
        thermal,
        profiles,
        extrusion,
        persistence,
        events,
        shutdown,
        clock,
        verification=verification,
        transition_id_factory=lambda: "tx-fixed",
        failure_id_factory=lambda: "failure-fixed",
        statistics=statistics,
    )
    return {
        "service": service,
        "trace": trace,
        "registry": registry,
        "profiles": profiles,
        "extrusion": extrusion,
        "thermal": thermal,
        "persistence": persistence,
        "action": action,
        "shutdown": shutdown,
        "statistics": statistics,
    }


def _prepare_source(env, tool=0):
    env["profiles"].activate(tool)
    env["trace"].clear()


def _effects(env):
    return [(item.kind, item.subject) for item in env["trace"]]


def test_select_has_one_exact_semantic_success_trace_and_fresh_contexts():
    env = _build()
    _prepare_source(env)

    final = env["service"].select(1)

    assert final == ChangerState.idle_with_tool(1, revision=2)
    assert _effects(env) == [
        ("readiness.require", "toolchange"),
        ("extrusion.observe_retracted", "firmware_retraction"),
        ("thermal.set_target", "extruder1"),
        ("persistence.save_state", "changer"),
        ("action.command", "dropoff_0"),
        ("extrusion.activate", "extruder1"),
        ("extrusion.apply_retraction", "firmware_retraction"),
        ("extrusion.apply_pa", "extruder1"),
        ("action.command", "pickup_1"),
        ("machine.apply_offset", "1"),
        ("thermal.set_target", "extruder1"),
        ("persistence.save_state", "changer"),
        ("persistence.mirror_current", "tool_current"),
        ("event.emit", ToolChangeService.COMPLETED_EVENT),
    ]
    action_effects = [e for e in env["trace"] if e.kind == "action.command"]
    assert action_effects[0].details[-1] == action_effects[1].details[-1] == "tx-fixed"


def test_action_context_uses_original_idle_snapshot_and_each_call_is_fresh():
    env = _build()
    _prepare_source(env)
    contexts = []
    original_run = env["action"].run_from_command

    def capture(action, context):
        contexts.append(context)
        original_run(action, context)

    env["action"].run_from_command = capture
    env["service"].select(1)

    assert len(contexts) == 2 and contexts[0] is not contexts[1]
    assert all(context.changer_before == ChangerState.idle_with_tool(0) for context in contexts)
    assert [context.phase for context in contexts] == [
        TransitionPhase.DROPPING_CURRENT,
        TransitionPhase.PICKING_TARGET,
    ]


def test_configured_verification_action_runs_at_finalizing_before_commit():
    env = _build(configured_verification=True)
    _prepare_source(env)
    contexts = []
    original_run = env["action"].run_from_command

    def capture(action, context):
        if action.startswith("verify_"):
            contexts.append(context)
        original_run(action, context)

    env["action"].run_from_command = capture
    env["service"].select(1)

    effects = _effects(env)
    verify_index = effects.index(("action.command", "verify_mounted_1"))
    assert effects[verify_index + 1] == ("machine.apply_offset", "1")
    assert contexts[0].phase is TransitionPhase.FINALIZING


def test_configured_verification_error_is_an_after_risk_recovery_failure():
    env = _build(configured_verification=True)
    _prepare_source(env)
    env["action"].inject_failure("action.command", "verify_mounted_1")

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert env["service"].state.mode is ChangerMode.RECOVERY_REQUIRED
    assert ("shutdown", "klipper") in _effects(env)


def test_retracted_source_is_unretracted_before_preheat_and_rechecked():
    env = _build(retracted=True)
    _prepare_source(env)

    env["service"].select(1)

    assert _effects(env)[:5] == [
        ("readiness.require", "toolchange"),
        ("extrusion.observe_retracted", "firmware_retraction"),
        ("extrusion.activate", "extruder"),
        ("extrusion.unretract", "extruder"),
        ("extrusion.observe_retracted", "firmware_retraction"),
    ]
    assert _effects(env)[5] == ("thermal.set_target", "extruder1")


def test_g11_failure_is_pre_risk_preserves_error_state_and_never_shutdowns():
    env = _build(retracted=True)
    _prepare_source(env)
    error = ValueError("cold extrusion")
    env["extrusion"].inject_failure("extrusion.unretract", "extruder", error)

    with pytest.raises(ValueError, match="cold extrusion") as caught:
        env["service"].select(1)

    assert caught.value is error
    assert env["service"].state == ChangerState.idle_with_tool(0)
    assert not any(kind == "shutdown" for kind, _ in _effects(env))
    assert not any(kind == "persistence.save_state" for kind, _ in _effects(env))


def test_intent_persistence_failure_never_calls_action_and_cancels_new_preheat():
    env = _build()
    _prepare_source(env)
    env["persistence"].inject_failure("persistence.save_state", "changer")

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert not any(kind.startswith("action.") for kind, _ in _effects(env))
    assert env["service"].state == ChangerState.idle_with_tool(0)
    assert _effects(env)[-1] == ("thermal.set_target", "extruder1")
    assert env["trace"][-1].details == (0.0,)


@pytest.mark.parametrize("prior", ["ACTIVE", "STANDBY", "PREHEAT"])
def test_pre_risk_failure_restores_prior_target_and_thermal_mode(prior):
    env = _build()
    _prepare_source(env)
    if prior == "ACTIVE":
        env["thermal"].activate(1)
    elif prior == "STANDBY":
        from ktcc.ports import ThermalMode

        env["thermal"].set_temperature(1, mode=ThermalMode.STANDBY)
    else:
        env["thermal"].preheat(1)
    before = env["thermal"].snapshot(1)
    env["trace"].clear()
    env["persistence"].inject_failure("persistence.save_state", "changer")

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert env["thermal"].snapshot(1) == before


@pytest.mark.parametrize(
    "kind,subject",
    [
        ("action.command", "dropoff_0"),
        ("extrusion.activate", "extruder1"),
        ("extrusion.apply_pa", "extruder1"),
        ("action.command", "pickup_1"),
        ("machine.apply_offset", "1"),
        ("persistence.save_state", "changer"),
    ],
)
def test_each_major_after_risk_failure_enters_recovery_off_then_persist_mirror_shutdown(
    kind, subject
):
    env = _build()
    _prepare_source(env)
    # The first state write is intent; arrange commit failure from inside the
    # drop action so the one-shot injection applies to the later write.
    if kind == "persistence.save_state":
        original_run = env["action"].run_from_command

        def run(action, context):
            original_run(action, context)
            if action == "dropoff_0":
                env["persistence"].inject_failure(kind, subject)

        env["action"].run_from_command = run
    else:
        target = env["action"] if kind.startswith("action.") else env["extrusion"]
        if kind == "machine.apply_offset":
            target = env["profiles"]._machine
        target.inject_failure(kind, subject)

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert env["service"].state.mode is ChangerMode.RECOVERY_REQUIRED
    assert env["service"].state.failure.risk_boundary_crossed
    kinds = _effects(env)
    shutdown_index = kinds.index(("shutdown", "klipper"))
    assert kinds[shutdown_index - 1] == (
        "persistence.mirror_current",
        "tool_current",
    )
    assert [(e.kind, e.subject, e.details) for e in env["trace"] if e.kind == "thermal.set_target"][-2:] == [
        ("thermal.set_target", "extruder", (0.0,)),
        ("thermal.set_target", "extruder1", (0.0,)),
    ]


def test_recovery_persistence_failure_still_mirrors_cleans_capability_and_shutdowns():
    env = _build()
    _prepare_source(env)
    original_run = env["action"].run_from_command

    def fail_and_arm(action, context):
        if action == "dropoff_0":
            env["persistence"].inject_failure("persistence.save_state", "changer")
            raise LookupError("coupler")
        original_run(action, context)

    env["action"].run_from_command = fail_and_arm

    with pytest.raises(LookupError, match="coupler"):
        env["service"].select(1)

    assert ("shutdown", "klipper") in _effects(env)
    assert env["persistence"].mirrored_tool_current == -2
    with pytest.raises(ToolChangeStateError, match="active transition"):
        env["service"].checkpoint("tx-fixed", "bad-capability", "forged", completed=False)


def test_authenticated_checkpoint_is_durable_and_wrong_capability_cannot_forge_it():
    env = _build()
    _prepare_source(env)
    original_run = env["action"].run_from_command

    def checkpointing(action, context):
        if action == "dropoff_0":
            with pytest.raises(ToolChangeStateError):
                env["service"].checkpoint(
                    context.transition_id, "publicly-guessed", "coupler", completed=False
                )
            capability = context.checkpoint_capability
            env["service"].checkpoint(
                context.transition_id, capability, "coupler", completed=False
            )
            env["service"].checkpoint(
                context.transition_id, capability, "coupler", completed=True
            )
        original_run(action, context)

    env["action"].run_from_command = checkpointing
    env["service"].select(1)

    changing_writes = [
        effect.details[0]
        for effect in env["trace"]
        if effect.kind == "persistence.save_state"
        and effect.details[0].mode is ChangerMode.CHANGING
    ]
    assert len(changing_writes) == 3
    assert changing_writes[-1].transition.last_completed_checkpoint == "coupler"


def test_final_mirror_failure_conservatively_persists_recovery_and_shutdowns():
    env = _build()
    _prepare_source(env)
    original_run = env["action"].run_from_command

    def arm_mirror(action, context):
        original_run(action, context)
        if action == "pickup_1":
            env["persistence"].inject_failure(
                "persistence.mirror_current", "tool_current"
            )

    env["action"].run_from_command = arm_mirror

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert env["service"].state.mode is ChangerMode.RECOVERY_REQUIRED
    assert env["persistence"].state.mode is ChangerMode.RECOVERY_REQUIRED
    assert env["persistence"].mirrored_tool_current == -2
    assert ("shutdown", "klipper") in _effects(env)


def test_completion_event_failure_is_secondary_to_committed_success():
    env = _build()
    _prepare_source(env)
    # The event fake records and then raises; the completed state must stand.
    event_sink = env["service"]._event_sink
    event_sink.inject_failure("event.emit", ToolChangeService.COMPLETED_EVENT)

    final = env["service"].select(1)

    assert final == ChangerState.idle_with_tool(1, 2)
    assert env["persistence"].state == final
    assert not any(kind == "shutdown" for kind, _ in _effects(env))


def test_none_to_target_profile_failure_before_pickup_rolls_back_without_shutdown():
    env = _build(state=ChangerState.idle_without_tool())
    env["extrusion"].inject_failure("extrusion.apply_pa", "extruder1")

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert env["service"].state == ChangerState.idle_without_tool()
    assert not any(kind.startswith("action.") for kind, _ in _effects(env))
    assert not any(kind == "shutdown" for kind, _ in _effects(env))


def test_drop_all_and_same_target_and_unknown_target_semantics():
    env = _build()
    _prepare_source(env)
    unchanged = env["service"].select(0)
    assert unchanged == ChangerState.idle_with_tool(0)
    assert env["trace"] == []
    with pytest.raises(KeyError):
        env["service"].select(99)
    final = env["service"].drop_all()
    assert final == ChangerState.idle_without_tool(2)
    assert env["persistence"].mirrored_tool_current == -1
    assert not any(subject.startswith("pickup") for _, subject in _effects(env))


def test_explicit_current_tool_declaration_updates_existing_key():
    env = _build(state=ChangerState.idle_without_tool())

    mounted = env["service"].declare_current_tool(1)

    assert mounted == ChangerState.idle_with_tool(1, revision=1)
    assert _effects(env) == [
        ("persistence.save_state", "changer"),
        ("persistence.mirror_current", "tool_current"),
    ]


def test_manual_unknown_legacy_declaration_enters_recovery_and_turns_heaters_off():
    env = _build(state=ChangerState.idle_without_tool())

    recovery = env["service"].declare_current_tool(-2)

    assert recovery.mode is ChangerMode.RECOVERY_REQUIRED
    assert recovery.failure.error_code == FailureCode.PERSISTED_RECOVERY.value
    assert [effect.details for effect in env["trace"] if effect.kind == "thermal.set_target"] == [
        (0.0,),
        (0.0,),
    ]
    assert _effects(env)[-2:] == [
        ("persistence.save_state", "changer"),
        ("persistence.mirror_current", "tool_current"),
    ]


def test_startup_changing_state_blocks_normal_commands():
    transition = TransitionRecord(
        "old", TransitionOperation.SELECT, MountedState.known(0), 1,
        TransitionPhase.PERSISTING_INTENT,
    )
    state = ChangerState(
        2, 1, ChangerMode.CHANGING, MountedState.known(0), LockState.LOCKED, 0,
        transition=transition,
    )
    env = _build(state=state)
    with pytest.raises(ToolChangeStateError, match="CHANGING"):
        env["service"].select(1)


def _recovery_state():
    failure = synthetic_failure(FailureCode.TRANSITION_FAILED, "failed", revision=3)
    return ChangerState(
        2, 4, ChangerMode.RECOVERY_REQUIRED, MountedState.unknown(),
        LockState.UNKNOWN, None, failure=failure,
    )


def test_reconcile_and_confirm_known_synchronizes_profile_offset_then_persists_idle():
    state = _recovery_state()
    env = _build(state=state)
    service = env["service"]
    token = service.recovery_status().acknowledgment_token
    service.reconcile(
        state.failure.failure_id,
        MountedState.known(1),
        LockState.LOCKED,
        carriage=CarriageEstimate.FREE,
        acknowledgment=token,
        note="inspected",
    )

    idle = service.confirm_recovery(state.failure.failure_id)

    assert idle.mode is ChangerMode.IDLE and idle.selected == 1
    assert _effects(env)[-5:] == [
        ("extrusion.apply_retraction", "firmware_retraction"),
        ("extrusion.apply_pa", "extruder1"),
        ("machine.apply_offset", "1"),
        ("persistence.save_state", "changer"),
        ("persistence.mirror_current", "tool_current"),
    ]


def test_reconcile_none_confirms_without_profile_effects_and_stale_or_unknown_rejected():
    state = _recovery_state()
    env = _build(state=state)
    service = env["service"]
    token = service.acknowledgment_token(state.failure.failure_id)
    with pytest.raises(RecoveryAcknowledgementError):
        service.reconcile(
            state.failure.failure_id, MountedState.none(), LockState.UNLOCKED,
            acknowledgment="wrong",
        )
    with pytest.raises(ValueError):
        service.reconcile(
            state.failure.failure_id, MountedState.unknown(), LockState.UNKNOWN,
            acknowledgment=token,
        )
    service.reconcile(
        state.failure.failure_id, MountedState.none(), LockState.UNLOCKED,
        acknowledgment=token,
    )
    idle = service.confirm_recovery(state.failure.failure_id)
    assert idle == ChangerState.idle_without_tool(7)
    assert not any(kind.startswith("extrusion.") for kind, _ in _effects(env))


def test_reconcile_persists_observation_before_publishing_draft():
    state = _recovery_state()
    env = _build(state=state)
    service = env["service"]
    env["persistence"].inject_failure("persistence.save_state", "changer")

    with pytest.raises(InjectedFailure):
        service.reconcile(
            state.failure.failure_id,
            MountedState.none(),
            LockState.UNLOCKED,
            acknowledgment=service.acknowledgment_token(state.failure.failure_id),
            note="empty and unlocked",
        )

    assert service.recovery_status().draft is None
    assert service.state == state

    draft = service.reconcile(
        state.failure.failure_id,
        MountedState.none(),
        LockState.UNLOCKED,
        acknowledgment=service.acknowledgment_token(state.failure.failure_id),
        note="empty and unlocked",
    )
    assert draft.note == "empty and unlocked"
    assert service.state.failure.mounted_estimate == MountedState.none()
    assert service.state.failure.operator_observation == "empty and unlocked"

    restarted = _build(state=service.state)["service"]
    assert restarted.recovery_status().draft == draft


def test_verifier_contradiction_returns_to_recovery_without_second_shutdown():
    state = _recovery_state()
    request = VerificationRequest(VerificationKind.MOUNTED, ToolId(1))
    result = VerificationResult(request, False, MountedState.none(), "sensor empty")
    verification = FakeVerificationPort({request: result}, [])
    env = _build(state=state, verification=verification)
    service = env["service"]
    service.reconcile(
        state.failure.failure_id, MountedState.known(1), LockState.LOCKED,
        acknowledgment=service.acknowledgment_token(state.failure.failure_id),
    )

    with pytest.raises(VerificationContradictionError, match="sensor empty"):
        service.confirm_recovery(state.failure.failure_id)

    assert service.state.mode is ChangerMode.RECOVERY_REQUIRED
    assert service.state.failure.error_code == FailureCode.SYNCHRONIZATION_FAILED.value
    assert not any(kind == "shutdown" for kind, _ in _effects(env))


def test_abort_and_registered_recovery_action_record_history_but_never_clear_recovery():
    state = _recovery_state()
    env = _build(state=state)
    service = env["service"]
    service.recovery_abort(state.failure.failure_id)
    service.run_recovery_action(state.failure.failure_id, "recover_0")

    assert service.state.mode is ChangerMode.RECOVERY_REQUIRED
    assert service.state.failure.recovery_action_history == (
        "abort:heaters_off",
        "start:recover_0",
        "completed:recover_0",
    )


def test_statistics_surround_mechanical_actions_and_selection_follows_commit():
    statistics = RecordingStatistics()
    env = _build(statistics=statistics)
    _prepare_source(env)

    env["service"].select(1)

    assert statistics.calls == [
        ("started", MechanicalKind.UNMOUNT, ToolId(0), "tx-fixed"),
        ("completed", MechanicalKind.UNMOUNT, ToolId(0), "tx-fixed"),
        ("started", MechanicalKind.MOUNT, ToolId(1), "tx-fixed"),
        ("completed", MechanicalKind.MOUNT, ToolId(1), "tx-fixed"),
        ("selection", ToolId(0), ToolId(1)),
    ]


def test_failed_mechanical_action_records_start_without_completion():
    statistics = RecordingStatistics()
    env = _build(statistics=statistics)
    _prepare_source(env)
    env["action"].inject_failure("action.command", "pickup_1")

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    assert statistics.calls == [
        ("started", MechanicalKind.UNMOUNT, ToolId(0), "tx-fixed"),
        ("completed", MechanicalKind.UNMOUNT, ToolId(0), "tx-fixed"),
        ("started", MechanicalKind.MOUNT, ToolId(1), "tx-fixed"),
        ("selection", ToolId(0), None),
    ]


def test_mirror_failure_never_publishes_target_selection_and_closes_for_recovery():
    statistics = RecordingStatistics()
    env = _build(statistics=statistics)
    _prepare_source(env)
    original_run = env["action"].run_from_command

    def arm_mirror(action, context):
        original_run(action, context)
        if action == "pickup_1":
            env["persistence"].inject_failure(
                "persistence.mirror_current", "tool_current"
            )

    env["action"].run_from_command = arm_mirror

    with pytest.raises(InjectedFailure):
        env["service"].select(1)

    selections = [call for call in statistics.calls if call[0] == "selection"]
    assert selections == [("selection", ToolId(0), None)]


def test_declaration_and_recovery_confirmation_publish_only_durable_selection():
    statistics = RecordingStatistics()
    env = _build(
        state=ChangerState.idle_without_tool(), statistics=statistics
    )
    env["service"].declare_current_tool(1)
    env["service"].declare_current_tool(-2)

    assert statistics.calls == [
        ("selection", None, ToolId(1)),
        ("selection", ToolId(1), None),
    ]

    recovery = env["service"].state
    token = env["service"].acknowledgment_token(recovery.failure.failure_id)
    env["service"].reconcile(
        recovery.failure.failure_id,
        MountedState.known(0),
        LockState.LOCKED,
        acknowledgment=token,
    )
    env["service"].confirm_recovery(recovery.failure.failure_id)
    assert statistics.calls[-1] == ("selection", None, ToolId(0))


def test_statistics_failure_is_warned_and_never_changes_toolchange_result():
    class FailingStatistics(RecordingStatistics):
        def mechanical_started(self, kind, tool_id, transition_id):
            raise RuntimeError("observer unavailable")

        def mechanical_completed(self, kind, tool_id, transition_id):
            raise RuntimeError("observer unavailable")

        def selection_changed(self, previous, current):
            raise RuntimeError("observer unavailable")

    env = _build(statistics=FailingStatistics())
    _prepare_source(env)

    final = env["service"].select(1)

    assert final == ChangerState.idle_with_tool(1, revision=2)
    warnings = [
        event
        for event in env["trace"]
        if event.kind == "event.emit"
        and event.subject == "statistics.observer_failed"
    ]
    assert len(warnings) == 5
