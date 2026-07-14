from __future__ import annotations

from dataclasses import FrozenInstanceError
from typing import Any, Mapping

import pytest

from ktcc.persistence import StateRepository
from ktcc.persistence.codec import PersistenceSnapshot
from ktcc.profiles.storage import (
    ProfileStorageError,
    PersistedProfileData,
    decode_persisted_profiles,
)
from ktcc.toolchange.state import ChangerState
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
    PressureAdvancePatch,
    PressureAdvanceProfile,
    ToolProfile,
    ToolProfilePatch,
)


def _registry() -> ToolRegistry:
    return ToolRegistry(
        ToolSpec(
            id=ToolId(number),
            extruder=ExtruderRef("extruder" if number == 0 else f"extruder{number}"),
            heater=HeaterRef("extruder" if number == 0 else f"extruder{number}"),
            fan=None,
            dock=DockSpec(Vector3(number, 0, 0), Vector3(number, 1, 0)),
            offset=Vector3(0, 0, 0),
            actions=ToolActions(),
            configured_profile=ToolProfile(),
        )
        for number in range(4)
    )


def _decoded(profiles: Mapping[str, Any]) -> PersistenceSnapshot:
    return PersistenceSnapshot(
        state=ChangerState.idle_without_tool(),
        profile_values=profiles,
    )


class _Store:
    def __init__(self) -> None:
        self.variables: dict[str, Any] = {}

    def read_variables(self) -> Mapping[str, Any]:
        return dict(self.variables)

    def write_variable(self, name: str, value: Any) -> None:
        self.variables[name] = value


def test_four_tool_real_deployment_shapes_decode_without_cross_tool_leakage():
    raw: dict[str, Any] = {"ktcc_global_offset": [0.001, 0.002, 0.003]}
    for number in range(4):
        raw[f"ktcc_tool_retract_{number}"] = {
            "retract_length": number + 0.4,
            "retract_speed": number + 20,
            "unretract_extra_length": number + 0.04,
            "unretract_speed": number + 21,
        }
        raw[f"ktcc_tool_pa_info_{number}"] = {
            "pressure_advance": number / 100 + 0.01,
            "smooth_time": number / 100 + 0.03,
        }
        raw[f"ktcc_tool_offset_{number}"] = [number, number + 0.1, number + 0.2]

    result = decode_persisted_profiles(_registry(), _decoded(raw))

    assert result.global_offset == Vector3(0.001, 0.002, 0.003)
    assert tuple(result.patches) == tuple(ToolId(i) for i in range(4))
    assert tuple(result.tool_offsets) == tuple(ToolId(i) for i in range(4))
    for number in range(4):
        patch = result.patches[ToolId(number)]
        assert patch.print_retraction == FirmwareRetractionPatch(
            number + 0.4, number + 20, number + 0.04, number + 21, None
        )
        assert patch.pressure_advance == PressureAdvancePatch(
            number / 100 + 0.01, number / 100 + 0.03
        )
        assert result.tool_offsets[ToolId(number)] == Vector3(
            number, number + 0.1, number + 0.2
        )


def test_repository_write_shapes_round_trip_to_typed_profile_data():
    store = _Store()
    repository = StateRepository(store)
    repository.save_retraction(
        ToolId(2), FirmwareRetractionProfile(0.6, 24, 0.06, 25, zhop=0.9)
    )
    repository.save_pressure_advance(ToolId(2), PressureAdvanceProfile(0.03, 0.04))
    repository.save_tool_offset(ToolId(2), Vector3(0.2, 0.3, 0.4))
    repository.save_global_offset(Vector3(0.01, 0.02, 0.03))

    result = decode_persisted_profiles(_registry(), repository.load())

    patch = result.patches[ToolId(2)]
    assert patch.print_retraction == FirmwareRetractionPatch(
        0.6, 24.0, 0.06, 25.0, None
    )
    assert patch.pressure_advance == PressureAdvancePatch(0.03, 0.04)
    assert result.tool_offsets == {ToolId(2): Vector3(0.2, 0.3, 0.4)}
    assert result.global_offset == Vector3(0.01, 0.02, 0.03)


def test_missing_fields_are_partial_patches_and_absent_global_is_zero():
    result = decode_persisted_profiles(
        _registry(),
        _decoded(
            {
                "ktcc_tool_retract_1": {"retract_length": 1.25},
                "ktcc_tool_pa_info_1": {"smooth_time": 0.08},
            }
        ),
    )

    assert result.patches[ToolId(1)].print_retraction == FirmwareRetractionPatch(
        retract_length=1.25
    )
    assert result.patches[ToolId(1)].pressure_advance == PressureAdvancePatch(
        smooth_time=0.08
    )
    assert result.global_offset == Vector3(0, 0, 0)


def test_result_is_deeply_immutable_and_detached_from_input_mappings():
    raw = {"ktcc_tool_retract_0": {"retract_length": 0.5}}
    result = decode_persisted_profiles(_registry(), _decoded(raw))
    raw["ktcc_tool_retract_1"] = {"retract_length": 99}

    with pytest.raises(TypeError):
        result.patches[ToolId(1)] = ToolProfile()  # type: ignore[index, assignment]
    with pytest.raises(TypeError):
        result.tool_offsets[ToolId(1)] = Vector3(1, 2, 3)  # type: ignore[index]
    with pytest.raises(FrozenInstanceError):
        result.global_offset.x = 1  # type: ignore[misc]
    assert ToolId(1) not in result.patches


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("ktcc_tool_retract_00", {"retract_length": 1.0}),
        ("ktcc_tool_retract_0", [1.0, 20.0, 0.0, 20.0]),
        ("ktcc_tool_pa_info_0", [0.03, 0.04]),
        ("ktcc_tool_retract_0", {"zhop": 0.2}),
        ("ktcc_tool_retract_0", {"surprise": 1.0}),
        ("ktcc_tool_pa_info_0", {"pressure_advance": True}),
        ("ktcc_tool_pa_info_0", {"pressure_advance": float("nan")}),
        ("ktcc_tool_pa_info_0", {"smooth_time": float("inf")}),
        ("ktcc_tool_pa_info_0", {"smooth_time": 0.21}),
        ("ktcc_tool_retract_0", {"retract_speed": 0.0}),
        ("ktcc_tool_offset_0", [0.0, False, 0.0]),
        ("ktcc_tool_offset_0", [0.0, 1.0]),
        ("ktcc_tool_offset_0", {"x": 0, "y": 0, "z": 0}),
        ("ktcc_global_offset", [0.0, float("-inf"), 0.0]),
        ("ktcc_global_offset_extra", [0.0, 0.0, 0.0]),
    ],
)
def test_malformed_persistence_is_rejected(key: str, value: Any):
    with pytest.raises(ProfileStorageError):
        decode_persisted_profiles(_registry(), _decoded({key: value}))


def test_non_string_profile_key_is_rejected():
    with pytest.raises(ProfileStorageError, match="keys must be strings"):
        decode_persisted_profiles(_registry(), _decoded({1: [0, 0, 0]}))  # type: ignore[dict-item]


def test_profile_for_removed_tool_is_ignored_without_startup_failure():
    result = decode_persisted_profiles(
        _registry(),
        _decoded({"ktcc_tool_retract_4": {"retract_length": 1.0}}),
    )

    assert result.patches == {}


def test_direct_data_construction_also_copies_mutable_mappings():
    patches = {ToolId(0): ToolProfilePatch()}
    offsets = {ToolId(0): Vector3(0, 0, 0)}
    result = PersistedProfileData(patches, offsets, Vector3(0, 0, 0))
    patches.clear()
    offsets.clear()

    assert ToolId(0) in result.patches
    assert ToolId(0) in result.tool_offsets
