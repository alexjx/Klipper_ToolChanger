import pytest

from ktcc.persistence.codec import (
    CURRENT_TOOL_KEY,
    decode_variables,
    tool_current_for_state,
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
from ktcc.toolchange.recovery import (
    CarriageEstimate,
    FailureCode,
    RecoveryDraft,
    begin_synchronization,
    complete_synchronization,
    interrupted_restart,
    synthetic_failure,
)


def changing_state(*, crossed=False):
    transition = TransitionRecord(
        "tx-1",
        TransitionOperation.SELECT,
        MountedState.known(0),
        1,
        TransitionPhase.DROPPING_CURRENT,
    ).start_checkpoint("dropoff", crosses_risk=crossed)
    return ChangerState(
        2,
        4,
        ChangerMode.CHANGING,
        MountedState.known(0),
        LockState.LOCKED,
        0,
        transition=transition,
    )


def recovery_state():
    failure = synthetic_failure(FailureCode.TRANSITION_FAILED, "coupler failed", revision=4)
    return ChangerState(
        2,
        5,
        ChangerMode.RECOVERY_REQUIRED,
        MountedState.unknown(),
        LockState.UNKNOWN,
        None,
        failure=failure,
    )


@pytest.mark.parametrize(
    "state, expected",
    [
        (ChangerState.idle_without_tool(), -1),
        (ChangerState.idle_with_tool(3), 3),
        (changing_state(), -2),
        (recovery_state(), -2),
    ],
)
def test_legacy_tool_current_is_projection(state, expected):
    assert state.legacy_tool_current() == expected


@pytest.mark.parametrize(
    "mounted, lock, selected",
    [
        (MountedState.none(), LockState.LOCKED, None),
        (MountedState.none(), LockState.UNLOCKED, 0),
        (MountedState.known(0), LockState.UNLOCKED, 0),
        (MountedState.known(0), LockState.LOCKED, 1),
        (MountedState.unknown(), LockState.UNKNOWN, None),
    ],
)
def test_idle_rejects_invalid_physical_invariants(mounted, lock, selected):
    with pytest.raises(ValueError, match="physical-tool invariant"):
        ChangerState(2, 0, ChangerMode.IDLE, mounted, lock, selected)


def test_transition_checkpoints_are_explicit_and_risk_is_monotonic():
    record = changing_state().transition
    assert record.last_completed_checkpoint is None
    assert not record.risk_boundary_crossed
    record = record.complete_checkpoint("dropoff")
    record = record.start_checkpoint("pickup", crosses_risk=True)
    assert record.last_completed_checkpoint == "dropoff"
    assert record.risk_boundary_crossed
    with pytest.raises(ValueError, match="already exists"):
        record.start_checkpoint("pickup")
    with pytest.raises(ValueError, match="backwards"):
        record.move_to(TransitionPhase.VALIDATING)


@pytest.mark.parametrize("crossed", [False, True])
def test_restart_of_changing_is_always_interrupted_recovery(crossed):
    restarted = interrupted_restart(changing_state(crossed=crossed))
    assert restarted.mode is ChangerMode.RECOVERY_REQUIRED
    assert restarted.mounted == MountedState.unknown()
    assert restarted.lock is LockState.UNKNOWN
    assert restarted.failure.error_code == FailureCode.INTERRUPTED.value
    assert restarted.failure.risk_boundary_crossed is crossed
    assert restarted.revision == 5


@pytest.mark.parametrize(
    "draft",
    [
        RecoveryDraft("startup-transition_failed", MountedState.none(), LockState.LOCKED),
        RecoveryDraft("startup-transition_failed", MountedState.unknown(), LockState.UNKNOWN),
        RecoveryDraft("startup-transition_failed", MountedState.known(9), LockState.LOCKED),
        RecoveryDraft("startup-transition_failed", MountedState.known(0), LockState.UNLOCKED),
    ],
)
def test_reconciliation_rejects_invalid_or_unregistered_final_state(draft):
    with pytest.raises(ValueError):
        begin_synchronization(recovery_state(), draft, [0, 1, 2, 3])


@pytest.mark.parametrize(
    "mounted, lock, expected",
    [
        (MountedState.none(), LockState.UNLOCKED, -1),
        (MountedState.known(2), LockState.LOCKED, 2),
    ],
)
def test_reconciliation_requires_sync_before_returning_idle(mounted, lock, expected):
    draft = RecoveryDraft(
        "startup-transition_failed", mounted, lock, CarriageEstimate.FREE, "inspected"
    )
    syncing = begin_synchronization(recovery_state(), draft, [0, 1, 2, 3])
    assert syncing.mode is ChangerMode.SYNCHRONIZING
    assert syncing.legacy_tool_current() == -2
    idle = complete_synchronization(syncing)
    assert idle.mode is ChangerMode.IDLE
    assert idle.legacy_tool_current() == expected


def test_existing_persistence_keys_preserve_exact_profile_values():
    retract = {"retract_length": 4.5, "retract_speed": 60.0}
    values = {
        "tool_current": 2,
        "ktcc_global_offset": [0, 0, 0.1],
        "ktcc_tool_offset_2": [1, 2, 3],
        "ktcc_tool_retract_2": retract,
        "ktcc_tool_pa_info_2": {"pressure_advance": 0.1, "smooth_time": 0.04},
        "ktcc_statistics_tool_2": {"ignored": True},
    }
    decoded = decode_variables(values)
    assert decoded.state == ChangerState.idle_with_tool(2)
    assert decoded.profile_values["ktcc_tool_retract_2"] is retract
    assert "ktcc_statistics_tool_2" not in decoded.profile_values


@pytest.mark.parametrize(
    "state, expected",
    [
        (ChangerState.idle_without_tool(), -1),
        (ChangerState.idle_with_tool(2), 2),
        (changing_state(crossed=True), -2),
        (recovery_state(), -2),
    ],
)
def test_runtime_state_projects_to_existing_current_tool_key(state, expected):
    assert tool_current_for_state(state) == expected


@pytest.mark.parametrize(
    "overrides",
    [
        {"requested_target": True},
        {"occurred_monotonic": float("nan")},
        {"heater_snapshot": (("extruder", float("inf")),)},
        {"recovery_action_history": ("x" * 121,)},
    ],
)
def test_failure_record_rejects_unbounded_or_non_finite_values(overrides):
    failure = synthetic_failure(FailureCode.TRANSITION_FAILED, "failed")
    values = dict(failure.__dict__)
    values.update(overrides)
    with pytest.raises(ValueError):
        type(failure)(**values)


@pytest.mark.parametrize("legacy", [-2, -3, "not-an-int", True])
def test_recovery_or_malformed_current_values_require_reconciliation(legacy):
    decoded = decode_variables({CURRENT_TOOL_KEY: legacy})
    assert decoded.state.mode is ChangerMode.RECOVERY_REQUIRED
    assert decoded.state.legacy_tool_current() == -2
