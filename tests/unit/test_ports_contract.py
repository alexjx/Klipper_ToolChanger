from __future__ import annotations

import ast
from dataclasses import replace
from pathlib import Path

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
    FakeStatisticsPort,
    FakeStatePersistencePort,
    FakeThermalPort,
    FakeToolchangeReadinessPort,
    FakeVariableStorePort,
    FakeVerificationPort,
    InjectedFailure,
)
from ktcc.toolchange.state import ChangerState, MountedState, TransitionPhase
from ktcc.tools import (
    DockSpec,
    ExtruderRef,
    FanRef,
    HeaterRef,
    ToolActions,
    ToolId,
    ToolSpec,
    Vector3,
)
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    PressureAdvanceProfile,
    ToolProfile,
    WaitMode,
)
from ktcc.ports import (
    ActionContext,
    ActionPort,
    Clock,
    ToolchangerEvent,
    EventLevel,
    EventSink,
    ExtrusionProfilePort,
    HeaterObservation,
    MachinePort,
    ProfilePersistencePort,
    SchedulerPort,
    ShutdownPort,
    StatisticsPort,
    StatePersistencePort,
    ThermalMode,
    ThermalPort,
    ThermalSnapshot,
    ThermalWaitRequest,
    ToolchangeReadinessPort,
    VerificationKind,
    VerificationPort,
    VerificationRequest,
    VerificationResult,
    VariableStorePort,
)


def tool_spec(tool_id: int = 1) -> ToolSpec:
    return ToolSpec(
        id=ToolId(tool_id),
        extruder=ExtruderRef("extruder%d" % tool_id),
        heater=HeaterRef("extruder%d" % tool_id),
        fan=FanRef("part_fan_%d" % tool_id),
        dock=DockSpec(Vector3(0, 0, 0), Vector3(0, 0, 0)),
        offset=Vector3(0.1 * tool_id, 0.2 * tool_id, 0.3 * tool_id),
        actions=ToolActions(pickup="pickup", dropoff="dropoff"),
        configured_profile=ToolProfile(),
    )


def action_context() -> ActionContext:
    return ActionContext(
        schema_version=1,
        tool=tool_spec(1),
        thermal=ThermalSnapshot(205, 155, ThermalMode.ACTIVE),
        changer_before=ChangerState.idle_with_tool(0),
        expected_after=ChangerState.idle_with_tool(1, revision=1),
        transition_id="change-0-to-1",
        phase=TransitionPhase.PICKING_TARGET,
    )


def test_strict_fakes_implement_their_runtime_port_contracts():
    heater = HeaterRef("extruder1")
    extruder = ExtruderRef("extruder1")
    request = VerificationRequest(VerificationKind.MOUNTED, ToolId(1))
    result = VerificationResult(request, True, MountedState.known(1))

    assert isinstance(FakeActionPort({"pickup"}), ActionPort)
    assert isinstance(
        FakeThermalPort({heater: HeaterObservation(200, 205, True)}), ThermalPort
    )
    assert isinstance(FakeExtrusionProfilePort({extruder}), ExtrusionProfilePort)
    assert isinstance(FakeMachinePort({ToolId(1)}, {FanRef("part_fan_1")}), MachinePort)
    assert isinstance(FakeToolchangeReadinessPort(), ToolchangeReadinessPort)
    assert isinstance(FakeSchedulerPort({"standby"}), SchedulerPort)
    assert isinstance(FakeStatePersistencePort(), StatePersistencePort)
    assert isinstance(FakeProfilePersistencePort(), ProfilePersistencePort)
    assert isinstance(FakeVariableStorePort(), VariableStorePort)
    assert isinstance(FakeVerificationPort({request: result}), VerificationPort)
    assert isinstance(FakeEventSink({"change.completed"}), EventSink)
    assert isinstance(FakeShutdownPort(), ShutdownPort)
    assert isinstance(FakeStatisticsPort(), StatisticsPort)
    assert isinstance(FakeClock(), Clock)


def test_action_command_and_callback_paths_are_observably_distinct():
    port = FakeActionPort({"pickup"})
    context = action_context()

    port.run_from_command("pickup", context)
    port.run_from_callback("pickup", context)

    assert [(effect.kind, effect.subject) for effect in port.trace] == [
        ("action.command", "pickup"),
        ("action.callback", "pickup"),
    ]


def test_shared_trace_records_semantic_toolchange_effects_in_order():
    trace = []
    context = action_context()
    heater = context.tool.heater
    extruder_ref = context.tool.extruder
    actions = FakeActionPort({"dropoff", "pickup"}, trace)
    thermal = FakeThermalPort({heater: HeaterObservation(180, 0, True)}, trace)
    extrusion = FakeExtrusionProfilePort({extruder_ref}, trace)
    machine = FakeMachinePort({context.tool.id}, {context.tool.fan}, trace)
    persistence = FakeStatePersistencePort(context.changer_before, trace)
    events = FakeEventSink({"change.completed"}, trace)

    thermal.set_target(heater, 205)
    actions.run_from_command("dropoff", context)
    extrusion.activate_extruder(extruder_ref)
    extrusion.apply_firmware_retraction(FirmwareRetractionProfile(retract_length=0.5))
    extrusion.apply_pressure_advance(
        extruder_ref, PressureAdvanceProfile(pressure_advance=0.02)
    )
    actions.run_from_command("pickup", context)
    thermal.wait(heater, ThermalWaitRequest(205, WaitMode.HEAT, 2, 300))
    machine.apply_offset(context.tool.id, context.tool.offset)
    persistence.save_state(context.expected_after)
    persistence.mirror_tool_current(1)
    events.emit(
        ToolchangerEvent(
            "change.completed",
            EventLevel.INFO,
            "tool change committed",
            context.tool.id,
            context.transition_id,
            TransitionPhase.PERSISTING_COMMIT,
        )
    )

    assert [effect.kind for effect in trace] == [
        "thermal.set_target",
        "action.command",
        "extrusion.activate",
        "extrusion.apply_retraction",
        "extrusion.apply_pa",
        "action.command",
        "thermal.wait",
        "machine.apply_offset",
        "persistence.save_state",
        "persistence.mirror_current",
        "event.emit",
    ]
    assert [effect.subject for effect in trace[2:5]] == [
        "extruder1",
        "firmware_retraction",
        "extruder1",
    ]


def test_unknown_resources_and_actions_fail_before_recording():
    context = action_context()
    request = VerificationRequest(VerificationKind.MOUNTED, ToolId(1))
    result = VerificationResult(request, True, MountedState.known(1))

    cases = [
        (FakeActionPort({"pickup"}), lambda port: port.run_from_command("move", context)),
        (
            FakeThermalPort(
                {HeaterRef("extruder1"): HeaterObservation(20, 0, False)}
            ),
            lambda port: port.observe(HeaterRef("extruder9")),
        ),
        (
            FakeExtrusionProfilePort({ExtruderRef("extruder1")}),
            lambda port: port.activate_extruder(ExtruderRef("extruder9")),
        ),
        (
            FakeMachinePort({ToolId(1)}, {FanRef("part_fan_1")}),
            lambda port: port.apply_offset(ToolId(9), Vector3(0, 0, 0)),
        ),
        (
            FakeSchedulerPort({"standby"}),
            lambda port: port.schedule_at("powerdown", 1, lambda: None),
        ),
        (
            FakeVerificationPort({request: result}),
            lambda port: port.verify(VerificationRequest(VerificationKind.UNMOUNTED)),
        ),
        (
            FakeEventSink({"change.completed"}),
            lambda port: port.emit(ToolchangerEvent("unknown", EventLevel.INFO, "message")),
        ),
    ]

    for port, invoke in cases:
        with pytest.raises(KeyError):
            invoke(port)
        assert port.trace == []


def test_injected_failure_is_deterministic_one_shot_and_records_attempt():
    port = FakeActionPort({"pickup"})
    context = action_context()
    error = InjectedFailure("coupler did not lock")
    port.inject_failure("action.command", "pickup", error)

    with pytest.raises(InjectedFailure) as raised:
        port.run_from_command("pickup", context)
    assert raised.value is error
    assert [(effect.kind, effect.subject) for effect in port.trace] == [
        ("action.command", "pickup")
    ]

    port.run_from_command("pickup", context)
    assert len(port.trace) == 2


def test_scheduler_runs_due_callbacks_deterministically_and_honors_cancel():
    effects = []
    scheduler = FakeSchedulerPort({"standby", "powerdown"})
    powerdown = scheduler.schedule_at("powerdown", 20, lambda: effects.append("off"))
    scheduler.schedule_at("standby", 10, lambda: effects.append("standby"))
    scheduler.cancel(powerdown)

    scheduler.run_due(9)
    assert effects == []
    scheduler.run_due(10)
    assert effects == ["standby"]
    scheduler.run_due(30)
    assert effects == ["standby"]
    assert [effect.kind for effect in scheduler.trace] == [
        "scheduler.schedule",
        "scheduler.schedule",
        "scheduler.cancel",
        "scheduler.fire",
    ]


def test_clock_is_monotonic_and_rejects_backwards_time():
    clock = FakeClock(5)
    clock.advance(2.5)
    assert clock.monotonic() == 7.5
    with pytest.raises(ValueError, match="backwards"):
        clock.set(7)


def test_port_dtos_reject_ambiguous_or_unsupported_values():
    with pytest.raises(ValueError, match="non-zero"):
        ThermalWaitRequest(0, WaitMode.HEAT, 1, 30)
    assert ThermalWaitRequest(0, WaitMode.RANGE, 1, 30).frozen_target == 0
    with pytest.raises(ValueError, match="only for STABLE"):
        ThermalWaitRequest(200, WaitMode.HEAT, 1, 30, stable_time=2)
    with pytest.raises(ValueError, match="requires a tool_id"):
        VerificationRequest(VerificationKind.MOUNTED)
    with pytest.raises(ValueError, match="only mounted"):
        VerificationRequest(VerificationKind.LOCKED, ToolId(1))
    with pytest.raises(ValueError, match="checkpoint_capability"):
        replace(action_context(), checkpoint_capability="guessable")

    machine = FakeMachinePort({ToolId(1)}, {FanRef("part_fan_1")})
    with pytest.raises(ValueError, match="between 0 and 1"):
        machine.set_fan(FanRef("part_fan_1"), 1.1)
    assert machine.trace == []


def test_ports_and_fakes_have_no_klipper_imports():
    roots = (
        Path(__file__).resolve().parents[2] / "ktcc" / "ports.py",
        Path(__file__).resolve().parents[1] / "fakes.py",
    )
    imported_modules = []
    for path in roots:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.extend(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.append(node.module)

    assert not [name for name in imported_modules if name == "klippy" or name.startswith("klippy.")]
