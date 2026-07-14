from dataclasses import FrozenInstanceError

import pytest

from ktcc.config.normalization import normalize_tool_group_section, normalize_tool_section
from ktcc.config.resolver import ConfigBuilder, ResourceCatalog, resolve_config
from ktcc.config.schema import (
    ConfigError,
    DuplicateSectionError,
    InvalidShapeError,
    MissingReferenceError,
    RemovedFeatureError,
)
from ktcc.profiles.models import (
    AppliedProfileSnapshot,
    FirmwareRetractionPatch,
    FirmwareRetractionProfile,
    PressureAdvancePatch,
    PressureAdvanceProfile,
    ProfileLayers,
    ProfileValidationError,
    ThermalPolicy,
    ThermalPolicyPatch,
    ToolProfile,
    ToolProfilePatch,
    ToolchangeFilamentProfile,
)


def group(**overrides):
    values = {
        "idle_to_standby_time": 300,
        "idle_to_powerdown_time": 600,
        "pickup_gcode": "PICK T={ myself.name }",
        "dropoff_gcode": "DROP T={ myself.name }",
    }
    values.update(overrides)
    return normalize_tool_group_section("toolgroup 0", values)


def tool(tool_id, **overrides):
    values = {
        "tool_group": 0,
        "extruder": "extruder" if tool_id == 0 else f"extruder{tool_id}",
        "fan": f"part_fan_{tool_id}",
        "meltzonelength": 25,
        "zone": f"{tool_id * 90 - 12},200,0",
        "park": f"{tool_id * 90 - 12},239.5,0",
        "offset": "0,-39,10",
        "pressure_advance": tool_id / 100,
        "retract_length": 0.5 + tool_id,
    }
    values.update(overrides)
    return normalize_tool_section(f"tool {tool_id}", values)


def catalog():
    return ResourceCatalog(
        extruders=frozenset({"extruder", "extruder1", "extruder2", "extruder3"}),
        heaters=frozenset({"extruder", "extruder1", "extruder2", "extruder3"}),
        fans=frozenset({f"part_fan_{index}" for index in range(4)}),
    )


def test_collect_then_resolve_is_order_independent_and_id_ordered():
    first = ConfigBuilder()
    for raw in (tool(3), tool(1), group(), tool(0), tool(2)):
        first.collect_group(raw) if hasattr(raw, "values") and not hasattr(raw, "group_id") else first.collect_tool(raw)

    second = ConfigBuilder()
    second.collect_group(group())
    for raw in (tool(0), tool(1), tool(2), tool(3)):
        second.collect_tool(raw)

    first_registry = first.resolve(catalog())
    second_registry = second.resolve(catalog())
    assert first_registry.ids == second_registry.ids
    assert [int(tool_spec.id) for tool_spec in first_registry] == [0, 1, 2, 3]
    assert [str(tool_spec.extruder) for tool_spec in first_registry] == [
        "extruder", "extruder1", "extruder2", "extruder3"
    ]


def test_four_physical_tools_keep_distinct_profiles_and_group_actions():
    registry = resolve_config([group()], [tool(index) for index in range(4)], catalog())
    assert len(registry) == 4
    assert {spec.configured_profile.pressure_advance.pressure_advance for spec in registry} == {
        0.0, 0.01, 0.02, 0.03
    }
    assert registry[3].configured_profile.print_retraction.retract_length == 3.5
    assert registry[2].configured_profile.toolchange_filament.unload_length == 25
    assert registry[1].configured_profile.thermal_policy.idle_to_standby_time == 300
    assert registry[0].actions.pickup == "PICK T={ myself.name }"


def test_tool_values_override_group_defaults_without_runtime_parent():
    registry = resolve_config(
        [group(meltzonelength=18, pickup_gcode="GROUP_PICK")],
        [tool(0, meltzonelength=23, pickup_gcode="TOOL_PICK")],
        catalog(),
    )
    assert registry[0].configured_profile.toolchange_filament.unload_length == 23
    assert registry[0].actions.pickup == "TOOL_PICK"
    assert not hasattr(registry[0], "toolgroup")


def test_duplicate_and_missing_references_have_typed_errors():
    with pytest.raises(DuplicateSectionError, match="duplicate tool id"):
        resolve_config([group()], [tool(0), tool(0)])
    with pytest.raises(MissingReferenceError, match="missing toolgroup 9"):
        resolve_config([group()], [normalize_tool_section("tool 0", {
            "tool_group": 9, "extruder": "extruder", "zone": "0,0,0",
            "park": "0,0,0", "offset": "0,0,0",
        })])
    with pytest.raises(MissingReferenceError, match="unknown fan"):
        resolve_config([group()], [tool(0, fan="missing_fan")], catalog())
    with pytest.raises(ConfigError, match="at least one physical tool"):
        resolve_config([group()], [], catalog())


@pytest.mark.parametrize("field,value", [("zone", "1,2"), ("park", [1, 2, 3, 4]), ("offset", "a,2,3")])
def test_coordinate_shapes_are_exact_and_numeric(field, value):
    with pytest.raises(InvalidShapeError, match=field):
        resolve_config([group()], [tool(0, **{field: value})], catalog())


def test_virtual_parent_and_remap_execution_are_explicitly_removed():
    with pytest.raises(RemovedFeatureError, match="virtual tools"):
        normalize_tool_section("tool 4", {"is_virtual": True})
    with pytest.raises(RemovedFeatureError, match="physical_parent"):
        normalize_tool_section("tool 4", {"physical_parent": 0})
    with pytest.raises(RemovedFeatureError, match="remap"):
        normalize_tool_group_section("toolgroup 0", {"tool_remap": "0:1"})
    # Inert legacy declarations are shapes, not runtime semantics.
    normalized = normalize_tool_section("tool 0", {"is_virtual": False, "physical_parent": -1})
    assert "physical_parent" not in normalized.values


def test_profile_layers_have_explicit_precedence_and_stable_revision():
    configured = ToolProfile(
        print_retraction=FirmwareRetractionProfile(retract_length=1.0),
        pressure_advance=PressureAdvanceProfile(pressure_advance=0.02),
        thermal_policy=ThermalPolicy(default_active=200, default_standby=150),
    )
    persisted = ToolProfilePatch(
        print_retraction=FirmwareRetractionPatch(retract_length=1.5),
        pressure_advance=PressureAdvancePatch(pressure_advance=0.04),
    )
    session = ToolProfilePatch(
        thermal_policy=ThermalPolicyPatch(default_active=245),
        pressure_advance=PressureAdvancePatch(pressure_advance=0.05),
    )
    effective = ProfileLayers(configured, persisted, session).resolve()
    repeated = ProfileLayers(configured, persisted, session).resolve()

    assert effective.profile.print_retraction.retract_length == 1.5
    assert effective.profile.pressure_advance.pressure_advance == 0.05
    assert effective.profile.thermal_policy.default_active == 245
    assert effective.profile.thermal_policy.default_standby == 150
    assert effective.revision == repeated.revision
    assert configured.pressure_advance.pressure_advance == 0.02


def test_profiles_and_extension_variables_are_deeply_immutable():
    source = {"stage": 0.7}
    filament = ToolchangeFilamentProfile(25, 23, source)
    source["stage"] = 0.2
    assert filament.variables["stage"] == 0.7
    with pytest.raises(TypeError):
        filament.variables["stage"] = 0.1
    with pytest.raises(FrozenInstanceError):
        filament.prime_length = 12


def test_raw_config_values_are_deeply_immutable():
    nested = {"offset": [1, 2, 3], "metadata": {"name": "synthetic"}}
    raw = normalize_tool_section("tool 0", nested)
    nested["offset"][0] = 99
    nested["metadata"]["name"] = "mutated"
    assert raw.values["offset"] == (1, 2, 3)
    assert raw.values["metadata"]["name"] == "synthetic"
    with pytest.raises(TypeError):
        raw.values["metadata"]["name"] = "forbidden"


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_coordinates_must_be_finite(value):
    with pytest.raises(ValueError, match="finite"):
        resolve_config([group()], [tool(0, offset=[0, 0, value])], catalog())


def test_applied_snapshot_exposes_partial_or_current_application():
    effective = ProfileLayers(ToolProfile()).resolve()
    pending = AppliedProfileSnapshot(0, "extruder", effective.revision, None, None)
    applied = AppliedProfileSnapshot(
        0, "extruder", effective.revision, effective.revision, effective.profile
    )
    assert not pending.is_current
    assert applied.is_current


@pytest.mark.parametrize(
    "factory",
    [
        lambda: FirmwareRetractionProfile(retract_speed=0.9),
        lambda: PressureAdvanceProfile(pressure_advance=-0.1),
        lambda: PressureAdvanceProfile(smooth_time=0.201),
        lambda: ThermalPolicy(idle_to_standby_time=0),
        lambda: ToolchangeFilamentProfile(unload_length=-1),
    ],
)
def test_profiles_enforce_klipper_and_model_limits(factory):
    with pytest.raises(ProfileValidationError):
        factory()
