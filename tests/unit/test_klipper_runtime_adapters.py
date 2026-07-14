from __future__ import annotations

from dataclasses import replace
from types import MappingProxyType

import pytest

from ktcc.klipper.runtime import (
    KlipperActionPort,
    KlipperExtrusionProfilePort,
    KlipperMachinePort,
    KlipperReadinessPort,
    KlipperSchedulerPort,
    KlipperShutdownPort,
    KlipperThermalPort,
    KlipperVerificationPort,
    ThermalTargetChanged,
    ThermalWaitShutdown,
    ThermalWaitTimeout,
    ToolchangeNotReadyError,
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
    MotionProfile,
    PressureAdvanceProfile,
    ToolProfile,
    WaitMode,
)
from ktcc.ports import (
    ActionContext,
    HeaterObservation,
    ThermalMode,
    ThermalSnapshot,
    ThermalWaitRequest,
    TimerHandle,
    VerificationKind,
    VerificationRequest,
    VerificationResult,
)


class CommandFailure(Exception):
    pass


class RecordingGCode:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.failure: BaseException | None = None

    def run_script_from_command(self, script: str) -> None:
        self.commands.append(script)
        if self.failure is not None:
            raise self.failure


class RecordingTemplate:
    def __init__(self) -> None:
        self.command_contexts = []
        self.command_failure: BaseException | None = None

    def run_gcode_from_command(self, context) -> None:
        self.command_contexts.append(context)
        if self.command_failure is not None:
            raise self.command_failure

class FakeReactor:
    NEVER = 1.0e30

    def __init__(self) -> None:
        self.now = 0.0
        self.pauses: list[float] = []
        self.registered: list[tuple[object, float, object]] = []
        self.unregistered: list[object] = []

    def monotonic(self) -> float:
        return self.now

    def pause(self, waketime: float) -> float:
        self.pauses.append(waketime)
        self.now = waketime
        return self.now

    def register_timer(self, callback, waketime: float):
        token = object()
        self.registered.append((callback, waketime, token))
        return token

    def unregister_timer(self, timer) -> None:
        self.unregistered.append(timer)


class FakeHeater:
    def __init__(self, temperature, target: float, *, can_extrude=True) -> None:
        self.temperature = temperature
        self.target = target
        self.can_extrude = can_extrude
        self.status_times: list[float] = []

    def get_status(self, eventtime: float):
        self.status_times.append(eventtime)
        temperature = (
            self.temperature(eventtime)
            if callable(self.temperature)
            else self.temperature
        )
        return {
            "temperature": temperature,
            "target": self.target,
            "can_extrude": self.can_extrude,
        }


class FakePrinterHeaters:
    def __init__(self) -> None:
        self.calls = []
        self.failure: BaseException | None = None

    def set_temperature(self, heater, target: float, wait: bool = False) -> None:
        self.calls.append((heater, target, wait))
        if self.failure is not None:
            raise self.failure
        heater.target = target


class FakeToolhead:
    def __init__(self, axes="xyz", position=(1.0, 2.0, 3.0, 4.0)) -> None:
        self.axes = axes
        self.position = position
        self.status_times = []

    def get_status(self, eventtime: float):
        self.status_times.append(eventtime)
        return {"homed_axes": self.axes}

    def get_position(self):
        return list(self.position)


def action_context() -> ActionContext:
    tool = ToolSpec(
        id=ToolId(2),
        extruder=ExtruderRef("extruder2"),
        heater=HeaterRef("extruder2"),
        fan=FanRef("part_fan_2"),
        dock=DockSpec(Vector3(10, 20, 30), Vector3(11, 21, 31)),
        offset=Vector3(0.1, -0.2, 0.3),
        actions=ToolActions("pickup", "dropoff"),
        configured_profile=ToolProfile(
            pressure_advance=PressureAdvanceProfile(0.025, 0.04)
        ),
    )
    return ActionContext(
        schema_version=1,
        tool=tool,
        thermal=ThermalSnapshot(220, 150, ThermalMode.PREHEAT),
        changer_before=ChangerState.idle_without_tool(),
        expected_after=ChangerState.idle_with_tool(2, revision=1),
        transition_id="select-2",
        phase=TransitionPhase.PICKING_TARGET,
        last_checkpoint="pickup.started",
        checkpoint_capability="0123456789abcdef",
    )


def test_action_port_selects_by_physical_tool_and_uses_distinct_runners():
    template = RecordingTemplate()
    factory_results = []
    callback_calls = []

    def factory():
        value = {"printer": object(), "schema_version": 999}
        factory_results.append(value)
        return value

    port = KlipperActionPort(
        {(2, "pickup"): template},
        lambda selected, context: callback_calls.append((selected, context)),
        factory,
    )
    context = action_context()

    port.run_from_command("pickup", context)
    port.run_from_callback("pickup", context)

    assert len(template.command_contexts) == len(callback_calls) == 1
    first = template.command_contexts[0]
    assert callback_calls[0][0] is template
    second = callback_calls[0][1]
    assert first is not second
    assert first["printer"] is factory_results[0]["printer"]
    assert first["schema_version"] == 1
    assert first["tool"]["id"] == 2
    assert first["tool"]["extruder"] == "extruder2"
    assert first["tool"]["profile"]["pressure_advance"]["pressure_advance"] == 0.025
    assert first["thermal"] == {"active": 220, "standby": 150, "mode": "PREHEAT"}
    assert first["transition"]["last_checkpoint"] == "pickup.started"
    assert first["changer"]["expected_after"]["mounted"]["tool_id"] == 2
    assert isinstance(first, MappingProxyType)
    with pytest.raises(TypeError):
        first["tool"] = None


def test_action_context_exposes_effective_profile_instead_of_configured_default():
    template = RecordingTemplate()
    port = KlipperActionPort({(2, "pickup"): template}, lambda *_: None)
    effective = ToolProfile(pressure_advance=PressureAdvanceProfile(0.123, 0.05))

    port.run_from_command(
        "pickup", replace(action_context(), effective_profile=effective)
    )

    profile = template.command_contexts[0]["tool"]["profile"]
    assert profile["pressure_advance"]["pressure_advance"] == 0.123


def test_action_port_preserves_template_exception_identity_and_rejects_unknown():
    template = RecordingTemplate()
    failure = CommandFailure("macro failed")
    def failed_callback(selected, context):
        del selected, context
        raise failure

    port = KlipperActionPort(
        {(ToolId(2), "pickup"): template}, failed_callback
    )

    with pytest.raises(CommandFailure) as caught:
        port.run_from_callback("pickup", action_context())
    assert caught.value is failure
    with pytest.raises(KeyError, match="tool=2 action=dropoff"):
        port.run_from_command("dropoff", action_context())


def thermal_port(heater: FakeHeater, reactor: FakeReactor | None = None):
    reactor = reactor or FakeReactor()
    manager = FakePrinterHeaters()
    shutdown = {"value": False}
    port = KlipperThermalPort(
        manager,
        {HeaterRef("extruder2"): heater},
        reactor,
        lambda: shutdown["value"],
        poll_interval=1.0,
    )
    return port, manager, reactor, shutdown


def test_thermal_set_target_uses_printer_heaters_and_observe_is_typed():
    heater = FakeHeater(21.5, 0, can_extrude=False)
    port, manager, reactor, _ = thermal_port(heater)

    port.set_target(HeaterRef("extruder2"), 205)
    observation = port.observe(HeaterRef("extruder2"))

    assert manager.calls == [(heater, 205.0, False)]
    assert observation == HeaterObservation(21.5, 205, False)
    assert heater.status_times == [reactor.now]


@pytest.mark.parametrize(
    ("mode", "temperature", "stable_time", "expected_time"),
    [
        (WaitMode.HEAT, lambda now: 190 + 5 * now, 0, 2),
        (WaitMode.RANGE, lambda now: 210 - 4 * now, 0, 2),
        (WaitMode.STABLE, lambda now: 201 if now >= 1 else 190, 2, 3),
    ],
)
def test_thermal_wait_modes_are_bounded_and_use_frozen_target(
    mode, temperature, stable_time, expected_time
):
    heater = FakeHeater(temperature, 200)
    port, _, reactor, _ = thermal_port(heater)
    request = ThermalWaitRequest(200, mode, 2, 10, stable_time=stable_time)

    result = port.wait(HeaterRef("extruder2"), request)

    assert reactor.now == expected_time
    assert result.target == 200
    assert heater.status_times[-1] == expected_time


def test_stable_wait_resets_continuous_in_range_window():
    samples = {0: 200, 1: 200, 2: 190, 3: 200, 4: 200, 5: 200}
    heater = FakeHeater(lambda now: samples[int(now)], 200)
    port, _, reactor, _ = thermal_port(heater)

    port.wait(
        HeaterRef("extruder2"),
        ThermalWaitRequest(200, WaitMode.STABLE, 1, 10, stable_time=2),
    )

    assert reactor.now == 5


def test_thermal_wait_timeout_target_change_and_shutdown_are_distinct():
    heater = FakeHeater(20, 200)
    port, _, reactor, shutdown = thermal_port(heater)
    request = ThermalWaitRequest(200, WaitMode.HEAT, 1, 2)

    with pytest.raises(ThermalWaitTimeout, match="within 2s"):
        port.wait(HeaterRef("extruder2"), request)
    assert reactor.now == 2

    reactor.now = 0
    heater.target = 199
    with pytest.raises(ThermalTargetChanged, match="200 to 199"):
        port.wait(HeaterRef("extruder2"), request)

    heater.target = 200
    shutdown["value"] = True
    with pytest.raises(ThermalWaitShutdown):
        port.wait(HeaterRef("extruder2"), request)


def test_thermal_adapter_preserves_heater_manager_exception_identity():
    heater = FakeHeater(20, 0)
    port, manager, _, _ = thermal_port(heater)
    failure = CommandFailure("heater rejected target")
    manager.failure = failure

    with pytest.raises(CommandFailure) as caught:
        port.set_target(HeaterRef("extruder2"), 250)
    assert caught.value is failure


def test_extrusion_profile_commands_are_explicit_and_retraction_state_is_narrow():
    gcode = RecordingGCode()
    firmware_retraction = type("FirmwareRetraction", (), {"is_retracted": True})()
    port = KlipperExtrusionProfilePort(gcode, firmware_retraction)

    port.activate_extruder(ExtruderRef("extruder2"))
    port.apply_pressure_advance(
        ExtruderRef("extruder2"), PressureAdvanceProfile(0.025, 0.04)
    )
    port.apply_firmware_retraction(
        FirmwareRetractionProfile(0.6, 24, 0.05, 25, zhop=0.5)
    )
    assert port.observe_retracted() is True
    port.unretract()

    assert gcode.commands == [
        "ACTIVATE_EXTRUDER EXTRUDER=extruder2",
        "SET_PRESSURE_ADVANCE EXTRUDER=extruder2 ADVANCE=0.025 SMOOTH_TIME=0.04",
        "SET_RETRACTION RETRACT_LENGTH=0.6 RETRACT_SPEED=24 "
        "UNRETRACT_EXTRA_LENGTH=0.05 UNRETRACT_SPEED=25",
        "G11",
    ]


def test_extrusion_profile_can_bypass_a_legacy_set_retraction_wrapper():
    gcode = RecordingGCode()
    firmware_retraction = type("FirmwareRetraction", (), {"is_retracted": False})()
    port = KlipperExtrusionProfilePort(
        gcode,
        firmware_retraction,
        set_retraction_command="SET_RETRACTION_ORIG",
    )

    port.apply_firmware_retraction(FirmwareRetractionProfile(0.8, 30, 0, 30))

    assert gcode.commands == [
        "SET_RETRACTION_ORIG RETRACT_LENGTH=0.8 RETRACT_SPEED=30 "
        "UNRETRACT_EXTRA_LENGTH=0 UNRETRACT_SPEED=30"
    ]


def test_gcode_resource_encoding_rejects_injection_and_preserves_failure():
    gcode = RecordingGCode()
    retraction = type("FirmwareRetraction", (), {"is_retracted": False})()
    extrusion = KlipperExtrusionProfilePort(gcode, retraction)
    machine = KlipperMachinePort(gcode, FakeToolhead())

    with pytest.raises(ValueError, match="encoded safely"):
        extrusion.activate_extruder(ExtruderRef("extruder2\nM112"))
    with pytest.raises(ValueError, match="encoded safely"):
        machine.set_fan(FanRef("fan 2"), 0.5)
    assert gcode.commands == []

    failure = CommandFailure("dispatch failed")
    gcode.failure = failure
    with pytest.raises(CommandFailure) as caught:
        extrusion.unretract()
    assert caught.value is failure


def test_scheduler_maps_one_shot_registration_completion_and_cancel():
    reactor = FakeReactor()
    port = KlipperSchedulerPort(reactor)
    calls = []

    first = port.schedule_at("heater:standby", 12.5, lambda: calls.append("first"))
    second = port.schedule_at("heater:off", 20, lambda: calls.append("second"))
    assert (first, second) == (TimerHandle(0), TimerHandle(1))
    assert [item[1] for item in reactor.registered] == [12.5, 20.0]

    result = reactor.registered[0][0](12.5)
    assert result == reactor.NEVER
    assert calls == ["first"]
    with pytest.raises(KeyError, match="completed timer"):
        port.cancel(first)

    port.cancel(second)
    assert reactor.unregistered == [reactor.registered[1][2]]


def test_scheduler_does_not_swallow_callback_exception():
    reactor = FakeReactor()
    port = KlipperSchedulerPort(reactor)
    failure = CommandFailure("application callback")
    port.schedule_at("callback", 1, lambda: (_ for _ in ()).throw(failure))

    with pytest.raises(CommandFailure) as caught:
        reactor.registered[0][0](1)
    assert caught.value is failure


def test_readiness_is_pure_xyz_validation_and_uses_configured_error_type():
    toolhead = FakeToolhead("zyx")
    port = KlipperReadinessPort(toolhead, lambda: 42.0)
    port.require_ready()
    assert toolhead.status_times == [42.0]

    toolhead.axes = "xz"
    with pytest.raises(ToolchangeNotReadyError, match="missing Y"):
        port.require_ready()

    original = CommandFailure("not homed")
    custom = KlipperReadinessPort(toolhead, lambda: 43, lambda message: original)
    with pytest.raises(CommandFailure) as caught:
        custom.require_ready()
    assert caught.value is original


def test_machine_port_emits_absolute_offset_fan_and_typed_position():
    gcode = RecordingGCode()
    machine = KlipperMachinePort(gcode, FakeToolhead(position=(3, 4, 5, 6)))

    machine.apply_offset(ToolId(2), Vector3(0.1, -0.2, 0.3))
    machine.set_fan(FanRef("part_fan_2"), 0.75)
    machine.apply_motion_profile(
        MotionProfile(42.0, 38.0, "mzv", "2hump_ei", 0.12, 0.15)
    )

    assert machine.observe_position() == Vector3(3, 4, 5)
    assert gcode.commands == [
        "SET_GCODE_OFFSET X=0.1 Y=-0.2 Z=0.3 MOVE=0",
        "SET_FAN_SPEED FAN=part_fan_2 SPEED=0.75",
        "SET_INPUT_SHAPER SHAPER_FREQ_X=42 SHAPER_FREQ_Y=38 "
        "DAMPING_RATIO_X=0.12 DAMPING_RATIO_Y=0.15 "
        "SHAPER_TYPE_X=mzv SHAPER_TYPE_Y=2hump_ei",
    ]
    with pytest.raises(ValueError, match="exceed"):
        machine.set_fan(FanRef("part_fan_2"), 1.01)


def test_shutdown_port_delegates_without_translating_exception():
    class Printer:
        def __init__(self):
            self.reasons = []
            self.failure = None

        def invoke_shutdown(self, reason):
            self.reasons.append(reason)
            if self.failure:
                raise self.failure

    printer = Printer()
    port = KlipperShutdownPort(printer)
    port.shutdown("mechanical state unknown")
    assert printer.reasons == ["mechanical state unknown"]

    failure = CommandFailure("already shut down")
    printer.failure = failure
    with pytest.raises(CommandFailure) as caught:
        port.shutdown("second failure")
    assert caught.value is failure


def test_verification_adapter_uses_only_configured_callback_and_validates_result():
    request = VerificationRequest(VerificationKind.MOUNTED, ToolId(2))
    expected = VerificationResult(request, True, MountedState.known(2), "switch high")
    seen = []
    port = KlipperVerificationPort(lambda item: seen.append(item) or expected)

    assert port.verify(request) is expected
    assert seen == [request]

    wrong_request = VerificationRequest(VerificationKind.UNMOUNTED)
    bad = KlipperVerificationPort(
        lambda item: replace(expected, request=wrong_request)
    )
    with pytest.raises(ValueError, match="does not match"):
        bad.verify(request)
