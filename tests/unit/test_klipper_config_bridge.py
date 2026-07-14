from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from ktcc.klipper.config import (
    ToolGroupSnapshot,
    ToolSnapshot,
    build_config_bridge,
)
from ktcc.config.resolver import ResourceCatalog
from ktcc.config.schema import MissingReferenceError, RemovedFeatureError


class TemplateStub:
    def __init__(self, name: str) -> None:
        self.name = name


def catalog(count: int = 4) -> ResourceCatalog:
    extruders = frozenset(
        "extruder" if index == 0 else f"extruder{index}" for index in range(count)
    )
    return ResourceCatalog(
        extruders=extruders,
        heaters=extruders,
        fans=frozenset(f"part_fan_{index}" for index in range(count)),
    )


def tool_snapshot(tool_id: int, **overrides: object) -> dict[str, object]:
    values: dict[str, object] = {
        "id": tool_id,
        "tool_group": 0,
        "extruder": "extruder" if tool_id == 0 else f"extruder{tool_id}",
        "fan": f"part_fan_{tool_id}",
        "zone": [tool_id * 10, 200, 0],
        "park": [tool_id * 10, 240, 0],
        "offset": [0, -39, 10],
        "retract_length": 0.5 + tool_id,
        "pressure_advance": tool_id / 100,
    }
    values.update(overrides)
    return values


def test_bridge_is_include_order_independent_for_four_physical_tools() -> None:
    group = ToolGroupSnapshot(
        0,
        values={
            "meltzonelength": 22,
            "idle_to_standby_time": 120,
            "idle_to_powerdown_time": 900,
        },
    )
    shuffled = [tool_snapshot(index) for index in (3, 0, 2, 1)]

    first = build_config_bridge([group], shuffled, catalog())
    second = build_config_bridge([group], reversed(shuffled), catalog())

    assert first.registry.ids == second.registry.ids
    assert [int(spec.id) for spec in first.registry] == [0, 1, 2, 3]
    assert first.registry[3].configured_profile.print_retraction.retract_length == 3.5
    assert first.registry[2].configured_profile.pressure_advance.pressure_advance == 0.02
    assert first.registry[1].configured_profile.toolchange_filament.unload_length == 22
    assert first.registry[0].configured_profile.thermal_policy.idle_to_standby_time == 120


def test_group_defaults_are_overridden_by_tool_scalars_and_templates() -> None:
    group_pickup = TemplateStub("group pickup")
    group_dropoff = TemplateStub("group dropoff")
    tool_pickup = TemplateStub("tool pickup")
    group = {
        "id": 0,
        "values": {
            "meltzonelength": 18,
            "retract_speed": 30,
            "shaper_freq_x": 42,
        },
        "action_templates": {
            "pickup": group_pickup,
            "dropoff": group_dropoff,
        },
    }
    tool = ToolSnapshot(
        0,
        0,
        values={
            **tool_snapshot(0),
            "meltzonelength": 25,
            "retract_speed": 45,
            "heater_active_temp": 245,
            "heater_standby_temp": 175,
            "select_wait_mode": "STABLE",
            "select_wait_tolerance": 2,
            "select_wait_timeout": 300,
            "select_wait_stable_time": 4,
        },
        action_templates={"pickup": tool_pickup},
    )

    result = build_config_bridge([group], [tool], catalog())
    spec = result.registry[0]
    profile = spec.configured_profile

    assert profile.toolchange_filament.unload_length == 25
    assert profile.print_retraction.retract_speed == 45
    assert profile.thermal_policy.default_active == 245
    assert profile.thermal_policy.default_standby == 175
    assert profile.thermal_policy.wait_mode == "STABLE"
    assert profile.motion is not None
    assert profile.motion.shaper_freq_x == 42
    assert spec.actions.pickup == "tool:0:pickup"
    assert spec.actions.dropoff == "tool:0:dropoff"
    assert [(binding.action_id, binding.template) for binding in result.action_bindings] == [
        ("tool:0:pickup", tool_pickup),
        ("tool:0:dropoff", group_dropoff),
    ]
    assert all(binding.template is not group_pickup for binding in result.action_bindings)


@pytest.mark.parametrize(
    "values,match",
    [
        ({"is_virtual": True}, "virtual tools"),
        ({"physical_parent": 0}, "physical_parent"),
        ({"physical_parent_id": 0}, "physical_parent_id"),
        ({"remap": "0:1"}, "remap"),
        ({"tool_remap": {0: 1}}, "tool_remap"),
    ],
)
def test_removed_virtual_parent_and_remap_features_fail_explicitly(
    values: dict[str, object], match: str
) -> None:
    snapshot = tool_snapshot(0, **values)

    with pytest.raises(RemovedFeatureError, match=match):
        build_config_bridge([{"id": 0}], [snapshot], catalog())


def test_inert_removed_fields_are_accepted_but_not_retained() -> None:
    snapshot = tool_snapshot(
        0,
        is_virtual=False,
        physical_parent=-1,
        physical_parent_id=None,
        remap="",
    )

    result = build_config_bridge([{"id": 0}], [snapshot], catalog())

    assert len(result.registry) == 1


@pytest.mark.parametrize(
    "override,match",
    [
        ({"extruder": "missing_extruder"}, "unknown extruder"),
        ({"heater": "missing_heater"}, "unknown heater"),
        ({"fan": "missing_fan"}, "unknown fan"),
        ({"tool_group": 7}, "missing toolgroup 7"),
    ],
)
def test_resource_and_group_references_are_validated(
    override: dict[str, object], match: str
) -> None:
    with pytest.raises(MissingReferenceError, match=match):
        build_config_bridge(
            [{"id": 0}], [tool_snapshot(0, **override)], catalog()
        )


def test_all_supported_actions_have_stable_ids_and_opaque_bindings() -> None:
    templates = {
        "pickup": TemplateStub("pickup"),
        "dropoff": TemplateStub("dropoff"),
        "verify_mounted": TemplateStub("mounted"),
        "verify_unmounted": TemplateStub("unmounted"),
        "recovery": TemplateStub("recovery"),
    }
    tool = tool_snapshot(
        0,
        pickup_gcode_template=templates["pickup"],
        dropoff_gcode_template=templates["dropoff"],
        verify_mounted_gcode_template=templates["verify_mounted"],
        verify_unmounted_gcode_template=templates["verify_unmounted"],
        recovery_gcode_template=templates["recovery"],
        # Raw legacy G-code is intentionally not copied into the core model.
        pickup_gcode="G1 X999",
    )

    result = build_config_bridge([{"id": 0}], [tool], catalog())
    spec = result.registry[0]

    assert spec.actions.pickup == "tool:0:pickup"
    assert spec.actions.dropoff == "tool:0:dropoff"
    assert spec.actions.verify_mounted == "tool:0:verify_mounted"
    assert spec.actions.verify_unmounted == "tool:0:verify_unmounted"
    assert spec.actions.recovery == "tool:0:recovery"
    assert [binding.action_id for binding in result.action_bindings] == [
        "tool:0:pickup",
        "tool:0:dropoff",
        "tool:0:verify_mounted",
        "tool:0:verify_unmounted",
        "tool:0:recovery",
    ]
    assert [binding.template for binding in result.action_bindings] == list(
        templates.values()
    )
    with pytest.raises(FrozenInstanceError):
        result.action_bindings[0].action_id = "mutated"


def test_absent_or_explicitly_disabled_actions_create_no_binding() -> None:
    result = build_config_bridge(
        [{"id": 0, "action_templates": {"pickup": TemplateStub("group")}}],
        [
            {
                **tool_snapshot(0),
                "action_templates": {"pickup": None},
            }
        ],
        catalog(),
    )

    assert result.registry[0].actions.pickup == ""
    assert result.action_bindings == ()
