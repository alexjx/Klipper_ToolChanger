from __future__ import annotations

import pytest

from fakes import (
    FakeExtrusionProfilePort,
    FakeMachinePort,
    FakeProfilePersistencePort,
    InjectedFailure,
)
from ktcc.profiles.service import ProfileService
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
from ktcc.profiles.models import (
    FirmwareRetractionPatch,
    FirmwareRetractionProfile,
    MotionProfilePatch,
    PressureAdvancePatch,
    PressureAdvanceProfile,
    ToolProfile,
    ToolProfilePatch,
)


def make_service(*, trace=None, persisted_patches=None):
    tools = []
    for number in range(4):
        tools.append(
            ToolSpec(
                id=ToolId(number),
                extruder=ExtruderRef("extruder" if number == 0 else f"extruder{number}"),
                heater=HeaterRef("extruder" if number == 0 else f"extruder{number}"),
                fan=None,
                dock=DockSpec(Vector3(number, 0, 0), Vector3(number, 1, 0)),
                offset=Vector3(number, number + 0.1, number + 0.2),
                actions=ToolActions(),
                configured_profile=ToolProfile(
                    print_retraction=FirmwareRetractionProfile(
                        retract_length=number + 0.5,
                        retract_speed=20 + number,
                        unretract_speed=10 + number,
                    ),
                    pressure_advance=PressureAdvanceProfile(number / 100, 0.04),
                ),
            )
        )
    registry = ToolRegistry(tools)
    shared = [] if trace is None else trace
    extrusion = FakeExtrusionProfilePort({tool.extruder for tool in registry}, shared)
    machine = FakeMachinePort(registry.ids, set(), shared)
    persistence = FakeProfilePersistencePort(shared)
    service = ProfileService(
        registry,
        extrusion,
        machine,
        persistence,
        persisted_patches=persisted_patches,
    )
    return service, extrusion, machine, persistence


def effects(trace):
    return [(effect.kind, effect.subject) for effect in trace]


def test_four_tools_keep_independent_config_persisted_and_session_layers():
    patches = {
        ToolId(1): ToolProfilePatch(
            print_retraction=FirmwareRetractionPatch(retract_length=7.0)
        )
    }
    service, _, _, _ = make_service(persisted_patches=patches)
    service.update_session_patch(
        2,
        ToolProfilePatch(
            pressure_advance=PressureAdvancePatch(pressure_advance=0.222)
        ),
    )

    assert [service.desired(i).print_retraction.retract_length for i in range(4)] == [
        0.5,
        7.0,
        2.5,
        3.5,
    ]
    assert [service.desired(i).pressure_advance.pressure_advance for i in range(4)] == [
        0.0,
        0.01,
        0.222,
        0.03,
    ]
    assert service.layers(1).persisted == patches[ToolId(1)]
    assert service.layers(2).session.pressure_advance is not None


def test_retraction_persists_before_apply_and_only_applies_for_mounted_tool():
    trace = []
    service, _, _, _ = make_service(trace=trace)
    profile = FirmwareRetractionProfile(2.3, 35, 0.1, 25)

    state = service.update_retraction(1, profile, mounted_tool=ToolId(1))
    assert effects(trace) == [
        ("persistence.save_retraction", "1"),
        ("extrusion.apply_retraction", "firmware_retraction"),
    ]
    assert state.firmware_retraction_revision == state.desired.revision

    trace.clear()
    state = service.update_retraction(2, profile, mounted_tool=ToolId(1))
    assert effects(trace) == [("persistence.save_retraction", "2")]
    assert state.firmware_retraction_revision is None


def test_pa_always_names_target_extruder_and_persistence_is_explicit():
    trace = []
    service, _, _, _ = make_service(trace=trace)
    transient = PressureAdvanceProfile(0.31, 0.06)
    saved = PressureAdvanceProfile(0.32, 0.07)

    service.update_pressure_advance(3, transient)
    service.update_pressure_advance(3, saved, persist=True)

    assert effects(trace) == [
        ("extrusion.apply_pa", "extruder3"),
        ("persistence.save_pa", "3"),
        ("extrusion.apply_pa", "extruder3"),
    ]
    assert service.layers(3).persisted.pressure_advance == PressureAdvancePatch(0.32, 0.07)
    assert service.layers(3).session.pressure_advance is None


def test_activate_and_reapply_idempotently_replay_complete_target_profile():
    trace = []
    service, _, _, _ = make_service(trace=trace)

    first = service.activate(2)
    second = service.reapply(2)

    assert first.is_current and second.is_current
    assert effects(trace) == [
        ("extrusion.activate", "extruder2"),
        ("extrusion.apply_retraction", "firmware_retraction"),
        ("extrusion.apply_pa", "extruder2"),
        ("extrusion.apply_retraction", "firmware_retraction"),
        ("extrusion.apply_pa", "extruder2"),
    ]


def test_hardware_failure_keeps_desired_and_does_not_claim_changed_resource_applied():
    service, extrusion, _, _ = make_service()
    before = service.activate(0)
    candidate = FirmwareRetractionProfile(8, 40, 0, 30)
    extrusion.inject_failure("extrusion.apply_retraction", "firmware_retraction")

    with pytest.raises(InjectedFailure):
        service.update_retraction(0, candidate, mounted_tool=ToolId(0))

    after = service.applied(0)
    assert service.desired(0).print_retraction == candidate
    assert after.desired.revision != before.desired.revision
    assert after.applied_revision == before.applied_revision
    assert after.firmware_retraction_revision == before.firmware_retraction_revision
    # The unchanged PA component remains accurately current at the new revision.
    assert after.pressure_advance_revision == after.desired.revision
    assert not after.is_current


def test_retrying_only_the_failed_component_repairs_aggregate_applied_state():
    service, extrusion, _, _ = make_service()
    service.activate(0)
    candidate = FirmwareRetractionProfile(8, 40, 0, 30)
    extrusion.inject_failure("extrusion.apply_retraction", "firmware_retraction")

    with pytest.raises(InjectedFailure):
        service.update_retraction(0, candidate, mounted_tool=ToolId(0))

    repaired = service.update_retraction(0, candidate, mounted_tool=ToolId(0))
    assert repaired.is_current


def test_mid_reapply_failure_does_not_commit_partial_candidate_revision():
    service, extrusion, _, _ = make_service()
    current = service.activate(1)
    service.update_session_patch(
        1,
        ToolProfilePatch(
            pressure_advance=PressureAdvancePatch(pressure_advance=0.199)
        ),
    )
    extrusion.inject_failure("extrusion.apply_pa", "extruder1")

    with pytest.raises(InjectedFailure):
        service.reapply(1)

    partial = service.applied(1)
    assert partial.desired.revision != current.desired.revision
    assert partial.applied_revision == current.applied_revision
    assert partial.pressure_advance_revision == current.pressure_advance_revision
    assert partial.firmware_retraction_revision == partial.desired.revision
    assert not partial.is_current


def test_offsets_are_persisted_then_composed_once_and_applied_only_to_mounted_tool():
    trace = []
    service, _, _, _ = make_service(trace=trace)

    service.update_global_offset(Vector3(0.1, 0.2, 0.3), mounted_tool=None)
    unmounted = service.update_tool_offset(
        2, Vector3(10, 20, 30), mounted_tool=ToolId(1)
    )
    mounted = service.update_tool_offset(
        1, Vector3(1, 2, 3), mounted_tool=ToolId(1)
    )

    assert unmounted.effective == Vector3(10.1, 20.2, 30.3)
    assert not unmounted.is_current
    assert mounted.effective == Vector3(1.1, 2.2, 3.3)
    assert mounted.is_current
    assert effects(trace) == [
        ("persistence.save_global_offset", "global"),
        ("persistence.save_tool_offset", "2"),
        ("persistence.save_tool_offset", "1"),
        ("machine.apply_offset", "1"),
    ]


def test_global_offset_update_reapplies_composition_to_mounted_tool():
    trace = []
    service, _, _, _ = make_service(trace=trace)
    service.update_tool_offset(3, Vector3(3, 4, 5), mounted_tool=None)

    service.update_global_offset(Vector3(0.5, -0.5, 1), mounted_tool=ToolId(3))

    assert effects(trace)[-2:] == [
        ("persistence.save_global_offset", "global"),
        ("machine.apply_offset", "3"),
    ]
    assert trace[-1].details == (Vector3(3.5, 3.5, 6),)
    assert service.offset_state(3).is_current


def test_offset_apply_failure_preserves_last_applied_snapshot():
    service, _, machine, _ = make_service()
    assert service.apply_offset(1).is_current
    previous = service.offset_state(1).applied
    machine.inject_failure("machine.apply_offset", "1")

    with pytest.raises(InjectedFailure):
        service.update_tool_offset(
            1, Vector3(9, 8, 7), mounted_tool=ToolId(1)
        )

    state = service.offset_state(1)
    assert state.applied == previous
    assert not state.is_current


@pytest.mark.parametrize("operation", ["layers", "activate", "apply_offset"])
def test_unknown_tool_is_rejected(operation):
    service, _, _, _ = make_service()
    with pytest.raises(KeyError):
        getattr(service, operation)(99)


def test_persistence_failure_changes_neither_desired_nor_hardware():
    service, _, _, persistence = make_service()
    before = service.effective(2)
    persistence.inject_failure("persistence.save_retraction", "2")

    with pytest.raises(InjectedFailure):
        service.update_retraction(
            2, FirmwareRetractionProfile(9, 20, 0, 10), mounted_tool=ToolId(2)
        )

    assert service.effective(2) == before


def test_session_offset_is_visible_without_persistence_and_save_promotes_it():
    trace = []
    service, _, _, _ = make_service(trace=trace)
    candidate = Vector3(3.0, -2.0, 1.0)

    session = service.update_session_tool_offset(1, candidate)

    assert session.session == candidate
    assert session.persisted is None
    assert not any(effect.kind == "persistence.save_tool_offset" for effect in trace)

    saved = service.update_tool_offset(1, candidate, mounted_tool=None)
    assert saved.session is None
    assert saved.persisted == candidate


def test_optional_motion_profile_is_applied_as_part_of_one_profile_revision():
    trace = []
    service, _, _, _ = make_service(trace=trace)
    service.update_session_patch(
        2,
        ToolProfilePatch(
            motion=MotionProfilePatch(
                shaper_freq_x=44.0,
                shaper_freq_y=39.0,
                shaper_type_y="ei",
            )
        ),
    )

    applied = service.activate(2)

    assert applied.is_current
    assert applied.motion_revision == applied.desired.revision
    assert ("machine.apply_motion", "input_shaper") in effects(trace)
