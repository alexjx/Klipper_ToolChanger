from __future__ import annotations

import pytest

from fakes import (
    FakeClock,
    FakeSchedulerPort,
    FakeThermalPort,
    InjectedFailure,
)
from ktcc.thermal import (
    ThermalConflictError,
    ThermalService,
    ThermalStateError,
    WaitTarget,
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
from ktcc.profiles.models import ThermalPolicy, ToolProfile, WaitMode
from ktcc.ports import HeaterObservation, ThermalMode, ThermalWaitRequest


def _tool(tool_id: int, heater: str | None = None) -> ToolSpec:
    return ToolSpec(
        id=ToolId(tool_id),
        extruder=ExtruderRef(f"extruder{tool_id}"),
        heater=HeaterRef(heater or f"extruder{tool_id}"),
        fan=None,
        dock=DockSpec(Vector3(0, 0, 0), Vector3(0, 0, 0)),
        offset=Vector3(0, 0, 0),
        actions=ToolActions(),
        configured_profile=ToolProfile(
            thermal_policy=ThermalPolicy(
                default_active=200 + tool_id,
                default_standby=140 + tool_id,
                idle_to_standby_time=10,
                standby_to_off_time=20,
                tolerance=2,
                timeout=300,
            )
        ),
    )


class _StatisticsObserver:
    def __init__(self, *, fail: bool = False):
        self.transitions = []
        self.fail = fail

    def thermal_changed(self, tool_id, previous, current):
        self.transitions.append((tool_id, previous, current))
        if self.fail:
            raise RuntimeError("statistics unavailable")


def _service(
    tools: list[ToolSpec], *, bed: bool = False, statistics=None
):
    trace = []
    observations = {
        tool.heater: HeaterObservation(20, 0, False) for tool in tools
    }
    auxiliary = {}
    if bed:
        auxiliary[0] = HeaterRef("heater_bed")
        observations[HeaterRef("heater_bed")] = HeaterObservation(55, 60, True)
    thermal = FakeThermalPort(observations, trace)
    callback_ids = {
        callback_id
        for tool in tools
        for callback_id in (
            ThermalService.standby_callback_id(tool.heater),
            ThermalService.off_callback_id(tool.heater),
        )
    }
    scheduler = FakeSchedulerPort(callback_ids, trace)
    clock = FakeClock()
    service = ThermalService(
        ToolRegistry(tools),
        thermal,
        scheduler,
        clock,
        auxiliary_heaters=auxiliary,
        statistics=statistics,
    )
    return service, thermal, scheduler, clock, trace


def test_statistics_observe_active_standby_off_timer_chain():
    observer = _StatisticsObserver()
    service, _, scheduler, clock, _ = _service([_tool(0)], statistics=observer)

    service.activate(0)
    service.schedule_idle(0)
    clock.advance(10)
    scheduler.run_due(clock.monotonic())
    clock.advance(20)
    scheduler.run_due(clock.monotonic())

    assert observer.transitions == [
        (ToolId(0), ThermalMode.OFF, ThermalMode.ACTIVE),
        (ToolId(0), ThermalMode.ACTIVE, ThermalMode.STANDBY),
        (ToolId(0), ThermalMode.STANDBY, ThermalMode.OFF),
    ]


def test_statistics_observe_zero_target_and_off_all_once_per_owner():
    observer = _StatisticsObserver()
    service, _, _, _, _ = _service(
        [_tool(0), _tool(1)], statistics=observer
    )
    service.activate(0)
    service.activate(1)

    service.m104(0, 0)
    service.off_all()

    assert observer.transitions == [
        (ToolId(0), ThermalMode.OFF, ThermalMode.ACTIVE),
        (ToolId(1), ThermalMode.OFF, ThermalMode.ACTIVE),
        (ToolId(0), ThermalMode.ACTIVE, ThermalMode.OFF),
        (ToolId(1), ThermalMode.ACTIVE, ThermalMode.OFF),
    ]


def test_zero_session_target_closes_current_thermal_mode():
    observer = _StatisticsObserver()
    service, _, _, _, _ = _service([_tool(0)], statistics=observer)
    service.activate(0)

    service.set_temperature(0, active_target=0)

    assert observer.transitions[-1] == (
        ToolId(0), ThermalMode.ACTIVE, ThermalMode.OFF
    )


def test_shared_heater_attributes_old_and_new_owner_once():
    observer = _StatisticsObserver()
    service, _, _, _, _ = _service(
        [_tool(0, "shared"), _tool(1, "shared")], statistics=observer
    )
    service.activate(0)
    service.m104(0, 0)
    service.activate(1)

    assert observer.transitions == [
        (ToolId(0), ThermalMode.OFF, ThermalMode.ACTIVE),
        (ToolId(0), ThermalMode.ACTIVE, ThermalMode.OFF),
        (ToolId(1), ThermalMode.OFF, ThermalMode.ACTIVE),
    ]


def test_target_only_change_does_not_emit_a_statistics_transition():
    observer = _StatisticsObserver()
    service, _, _, _, _ = _service([_tool(0)], statistics=observer)
    service.activate(0)
    observer.transitions.clear()

    service.set_temperature(0, active_target=230)

    assert observer.transitions == []


def test_physical_target_failure_is_not_reported_as_a_transition():
    observer = _StatisticsObserver()
    service, thermal, _, _, _ = _service([_tool(0)], statistics=observer)
    thermal.inject_failure(
        "thermal.set_target", "extruder0", InjectedFailure("heater")
    )

    with pytest.raises(InjectedFailure, match="heater"):
        service.activate(0)

    assert observer.transitions == []
    assert service.snapshot(0).mode is ThermalMode.OFF


def test_statistics_failure_never_changes_successful_heater_behavior():
    observer = _StatisticsObserver(fail=True)
    service, thermal, scheduler, clock, _ = _service(
        [_tool(0)], statistics=observer
    )

    service.activate(0)
    service.schedule_idle(0)
    clock.advance(10)
    scheduler.run_due(clock.monotonic())
    clock.advance(20)
    scheduler.run_due(clock.monotonic())

    assert thermal.observations[HeaterRef("extruder0")].target == 0
    assert service.snapshot(0).mode is ThermalMode.OFF
    assert len(service.controller_failures) == 0


def test_four_physical_heads_have_independent_sessions_and_controllers():
    tools = [_tool(index) for index in range(4)]
    service, thermal, _, _, trace = _service(tools)

    for index in range(4):
        service.m104(index, 210 + index)

    assert [thermal.observations[tool.heater].target for tool in tools] == [
        210,
        211,
        212,
        213,
    ]
    assert [service.snapshot(index).active_target for index in range(4)] == [
        210,
        211,
        212,
        213,
    ]
    assert [effect.subject for effect in trace] == [
        "extruder0",
        "extruder1",
        "extruder2",
        "extruder3",
    ]


def test_overlapping_preheats_on_distinct_heaters_do_not_interfere():
    tools = [_tool(0), _tool(1)]
    service, thermal, _, _, _ = _service(tools)

    service.preheat(0, temperature=215)
    service.preheat(1, target=WaitTarget.STANDBY, temperature=155)

    assert service.snapshot(0) == service.snapshot(0).__class__(
        215, 140, ThermalMode.PREHEAT
    )
    assert service.snapshot(1) == service.snapshot(1).__class__(
        201, 155, ThermalMode.STANDBY
    )
    assert service.preheat_target(0) is WaitTarget.ACTIVE
    assert service.preheat_target(1) is WaitTarget.STANDBY
    assert thermal.observations[tools[0].heater].target == 215
    assert thermal.observations[tools[1].heater].target == 155


def test_shared_heater_has_one_explicit_owner_and_rejects_a_competing_tool():
    shared = "shared_hotend"
    service, _, _, _, _ = _service([_tool(0, shared), _tool(1, shared)])

    service.preheat(0, temperature=205)

    assert service.owner(HeaterRef(shared)) == ToolId(0)
    with pytest.raises(ThermalConflictError, match="owned by T0"):
        service.preheat(1, temperature=205)

    service.cancel_preheat(0)
    service.preheat(1, temperature=205)
    assert service.owner(HeaterRef(shared)) == ToolId(1)


def test_m109_sets_active_target_then_waits_heat_only_in_one_operation():
    service, _, _, _, trace = _service([_tool(0)])

    service.m109(0, 225, tolerance=3, timeout=100)

    assert [effect.kind for effect in trace] == [
        "thermal.set_target",
        "thermal.wait",
    ]
    request = trace[1].details[0]
    assert request == ThermalWaitRequest(225, WaitMode.HEAT, 3, 100)
    assert service.snapshot(0).mode is ThermalMode.ACTIVE


def test_m109_zero_turns_heater_off_without_wait_but_explicit_heat_wait_rejects_zero():
    service, thermal, _, _, _ = _service([_tool(0)])

    service.m104(0, 0)
    assert service.snapshot(0).mode is ThermalMode.OFF
    assert thermal.observations[HeaterRef("extruder0")].target == 0

    assert service.m109(0, 0) is None
    with pytest.raises(ValueError, match="non-zero"):
        service.wait(0, target=WaitTarget.ACTIVE, mode=WaitMode.HEAT)


@pytest.mark.parametrize("mode", [WaitMode.HEAT, WaitMode.RANGE, WaitMode.STABLE])
def test_wait_freezes_requested_session_target_and_preserves_mode_parameters(mode):
    service, _, _, _, trace = _service([_tool(0)])
    service.m104(0, 220)

    service.wait(
        0,
        target=WaitTarget.ACTIVE,
        mode=mode,
        tolerance=4,
        timeout=90,
        stable_time=5 if mode is WaitMode.STABLE else None,
    )

    request = trace[-1].details[0]
    assert request.frozen_target == 220
    assert request.mode is mode
    assert request.tolerance == 4
    assert request.timeout == 90
    assert request.stable_time == (5 if mode is WaitMode.STABLE else 0)


def test_wait_target_changed_and_timeout_errors_from_port_are_not_hidden():
    service, thermal, _, _, _ = _service([_tool(0)])
    service.m104(0, 220)
    thermal.inject_failure(
        "thermal.wait", "extruder0", RuntimeError("TARGET_CHANGED")
    )
    with pytest.raises(RuntimeError, match="TARGET_CHANGED"):
        service.wait(0, target=WaitTarget.ACTIVE)

    thermal.inject_failure("thermal.wait", "extruder0", RuntimeError("TIMEOUT"))
    with pytest.raises(RuntimeError, match="TIMEOUT"):
        service.wait(0, target=WaitTarget.ACTIVE)
    assert thermal.observations[HeaterRef("extruder0")].target == 220


def test_idle_timer_transitions_to_standby_then_off_at_deterministic_deadlines():
    tool = _tool(0)
    service, thermal, scheduler, clock, trace = _service([tool])
    service.activate(0)
    service.schedule_idle(0)

    clock.advance(10)
    scheduler.run_due(clock.monotonic())
    assert service.snapshot(0).mode is ThermalMode.STANDBY
    assert service.preheat_target(0) is None
    assert thermal.observations[tool.heater].target == 140

    clock.advance(19)
    scheduler.run_due(clock.monotonic())
    assert service.snapshot(0).mode is ThermalMode.STANDBY
    clock.advance(1)
    scheduler.run_due(clock.monotonic())
    assert service.snapshot(0).mode is ThermalMode.OFF
    assert [effect.kind for effect in trace].count("scheduler.fire") == 2


def test_legacy_dropoff_standby_demand_repairs_remaining_powerdown_timer():
    tool = _tool(0)
    service, _, scheduler, clock, trace = _service([tool])
    service.activate(0)
    service.set_temperature(0, mode=ThermalMode.STANDBY)

    service.schedule_inactivity(0)

    scheduled = [effect for effect in trace if effect.kind == "scheduler.schedule"]
    assert scheduled[-1].subject == ThermalService.off_callback_id(tool.heater)
    assert scheduled[-1].details[0] == 20.0
    clock.advance(20)
    scheduler.run_due(clock.monotonic())
    assert service.snapshot(0).mode is ThermalMode.OFF


def test_zero_standby_target_turns_off_releases_owner_and_skips_off_timer():
    tool = _tool(0)
    service, _, scheduler, clock, trace = _service([tool])
    service.m568(0, standby_target=0)
    service.activate(0)
    service.schedule_idle(0)

    clock.advance(10)
    scheduler.run_due(clock.monotonic())

    assert service.snapshot(0).mode is ThermalMode.OFF
    assert service.owner(tool.heater) is None
    assert scheduler._scheduled == {}
    assert [effect.subject for effect in trace if effect.kind == "scheduler.schedule"] == [
        ThermalService.standby_callback_id(tool.heater)
    ]


def test_session_timer_overrides_are_per_tool_and_drive_each_timer_chain():
    tools = [_tool(0), _tool(1)]
    service, _, scheduler, clock, trace = _service(tools)
    service.set_tool_temperature(
        0, idle_to_standby_time=3, standby_to_off_time=11
    )
    service.m568(1, idle_to_standby_time=7, standby_to_off_time=13)
    service.activate(0)
    service.activate(1)

    service.schedule_idle(0)
    service.schedule_idle(1)
    scheduled = [effect for effect in trace if effect.kind == "scheduler.schedule"]
    assert [(effect.subject, effect.details[0]) for effect in scheduled] == [
        (ThermalService.standby_callback_id(tools[0].heater), 3.0),
        (ThermalService.standby_callback_id(tools[1].heater), 7.0),
    ]

    clock.advance(3)
    scheduler.run_due(clock.monotonic())
    off_zero = [
        effect
        for effect in trace
        if effect.kind == "scheduler.schedule"
        and effect.subject == ThermalService.off_callback_id(tools[0].heater)
    ]
    assert off_zero[0].details[0] == 14.0
    assert service.snapshot(1).mode is ThermalMode.ACTIVE


def test_session_timer_overrides_enforce_policy_minimums():
    service, _, _, _, _ = _service([_tool(0)])

    with pytest.raises(ValueError, match="at least 0.1"):
        service.set_tool_temperature(0, idle_to_standby_time=0)
    with pytest.raises(ValueError, match="at least 0.1"):
        service.m568(0, standby_to_off_time=0.09)


def test_mode_omission_updates_only_the_target_matching_current_demand():
    tool = _tool(0)
    service, thermal, scheduler, clock, trace = _service([tool])
    service.activate(0)
    service.schedule_idle(0)
    timer = next(iter(scheduler._scheduled))
    trace.clear()

    service.m568(0, active_target=230, standby_target=150)
    assert [(effect.kind, effect.details) for effect in trace] == [
        ("thermal.set_target", (230.0,))
    ]
    assert timer in scheduler._scheduled

    clock.advance(10)
    scheduler.run_due(clock.monotonic())
    trace.clear()
    off_timer = next(iter(scheduler._scheduled))
    service.set_tool_temperature(0, standby_target=155)
    assert [(effect.kind, effect.details) for effect in trace] == [
        ("thermal.set_target", (155.0,))
    ]
    assert off_timer in scheduler._scheduled
    assert thermal.observations[tool.heater].target == 155


@pytest.mark.parametrize("preheat_target", [WaitTarget.ACTIVE, WaitTarget.STANDBY])
def test_mode_omission_updates_the_selected_preheat_target(preheat_target):
    service, thermal, _, _, trace = _service([_tool(0)])
    service.preheat(0, target=preheat_target)
    trace.clear()

    if preheat_target is WaitTarget.ACTIVE:
        service.m568(0, active_target=235, standby_target=160)
        expected = 235
    else:
        service.m568(0, active_target=235, standby_target=160)
        expected = 160

    assert thermal.observations[HeaterRef("extruder0")].target == expected
    assert [effect.kind for effect in trace] == ["thermal.set_target"]
    assert service.preheat_target(0) is preheat_target


def test_timeout_override_reschedules_active_and_standby_deadlines_from_now():
    tool = _tool(0)
    service, _, scheduler, clock, trace = _service([tool])
    service.activate(0)
    service.schedule_idle(0)
    clock.advance(2)
    trace.clear()

    service.set_tool_temperature(0, idle_to_standby_time=5)
    assert [effect.kind for effect in trace] == [
        "scheduler.cancel",
        "scheduler.schedule",
    ]
    assert trace[1].details[0] == 7.0

    clock.advance(5)
    scheduler.run_due(clock.monotonic())
    assert service.snapshot(0).mode is ThermalMode.STANDBY
    trace.clear()
    service.m568(0, standby_to_off_time=4)
    assert [effect.kind for effect in trace] == [
        "scheduler.cancel",
        "scheduler.schedule",
    ]
    assert trace[1].details[0] == 11.0


def test_new_demand_cancels_timer_and_stale_callback_is_a_noop():
    tool = _tool(0)
    service, _, scheduler, _, trace = _service([tool])
    service.activate(0)
    service.schedule_idle(0)
    stale_callback = next(iter(scheduler._scheduled.values())).callback

    service.m104(0, 230)
    trace_length = len(trace)
    stale_callback()

    assert len(trace) == trace_length
    assert service.snapshot(0).mode is ThermalMode.ACTIVE


def test_timer_callback_records_failure_without_leaking_into_scheduler():
    tool = _tool(0)
    service, thermal, scheduler, clock, _ = _service([tool])
    service.activate(0)
    service.schedule_idle(0)
    thermal.inject_failure("thermal.set_target", "extruder0", InjectedFailure("heater"))

    clock.advance(10)
    scheduler.run_due(clock.monotonic())

    assert len(service.controller_failures) == 1
    assert service.controller_failures[0].operation == "standby"


def test_cancel_preheat_is_idempotent_when_other_tool_owns_shared_heater():
    service, _, _, _, _ = _service([_tool(0, "shared"), _tool(1, "shared")])
    service.preheat(0)

    service.cancel_preheat(1)
    assert service.owner(HeaterRef("shared")) == ToolId(0)

    service.activate(0)
    with pytest.raises(ThermalStateError, match="non-preheat"):
        service.cancel_preheat(0)


def test_standby_target_preheat_can_be_cancelled_and_is_not_an_idle_demand():
    service, thermal, _, _, _ = _service([_tool(0)])
    service.preheat(0, target=WaitTarget.STANDBY, temperature=150)

    with pytest.raises(ThermalStateError, match="ACTIVE"):
        service.schedule_idle(0)

    service.cancel_preheat(0)
    assert service.snapshot(0).mode is ThermalMode.OFF
    assert thermal.observations[HeaterRef("extruder0")].target == 0


def test_off_all_cancels_demands_once_per_distinct_physical_heater():
    service, thermal, _, _, trace = _service(
        [_tool(0, "shared"), _tool(1, "shared"), _tool(2)]
    )
    service.preheat(0)
    service.preheat(2)
    trace.clear()

    service.off_all()

    assert [effect.subject for effect in trace if effect.kind == "thermal.set_target"] == [
        "shared",
        "extruder2",
    ]
    assert all(observation.target == 0 for observation in thermal.observations.values())
    assert service.owner(HeaterRef("shared")) is None


def test_compatibility_wait_explicit_bed_does_not_create_a_fake_tool():
    service, _, _, _, trace = _service([_tool(0)], bed=True)

    result = service.temperature_wait_with_tolerance(heater_id=0)

    assert len(result) == 1
    assert [(effect.kind, effect.subject) for effect in trace] == [
        ("thermal.observe", "heater_bed"),
        ("thermal.wait", "heater_bed"),
    ]


def test_compatibility_wait_without_heater_waits_bed_then_current_tool():
    service, thermal, _, _, trace = _service([_tool(0)], bed=True)
    thermal.set_target(HeaterRef("extruder0"), 205)
    trace.clear()

    result = service.temperature_wait_with_tolerance(current_tool=0)

    assert len(result) == 2
    assert [(effect.kind, effect.subject) for effect in trace] == [
        ("thermal.observe", "heater_bed"),
        ("thermal.wait", "heater_bed"),
        ("thermal.observe", "extruder0"),
        ("thermal.wait", "extruder0"),
    ]


def test_compatibility_wait_without_current_tool_still_waits_bed_and_returns():
    service, _, _, _, trace = _service([_tool(0)], bed=True)

    result = service.temperature_wait_with_tolerance()

    assert len(result) == 1
    assert [(effect.kind, effect.subject) for effect in trace] == [
        ("thermal.observe", "heater_bed"),
        ("thermal.wait", "heater_bed"),
    ]


def test_compatibility_wait_uses_legacy_integer_40_threshold_but_explicit_wait_does_not():
    service, thermal, _, _, trace = _service([_tool(0)], bed=True)
    thermal.observations[HeaterRef("heater_bed")] = HeaterObservation(20, 40.9, False)
    thermal.set_target(HeaterRef("extruder0"), 40.9)
    trace.clear()

    service.temperature_wait_with_tolerance(current_tool=0)
    assert [(effect.kind, effect.subject) for effect in trace] == [
        ("thermal.observe", "heater_bed"),
        ("thermal.observe", "extruder0"),
    ]

    trace.clear()
    service.wait(0, target=WaitTarget.CURRENT, mode=WaitMode.RANGE)
    assert [effect.kind for effect in trace] == ["thermal.observe", "thermal.wait"]
