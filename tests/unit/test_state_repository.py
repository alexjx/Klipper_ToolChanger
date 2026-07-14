from __future__ import annotations

from typing import Any, Mapping, Optional

import pytest

from ktcc.persistence import StateRepository
from ktcc.persistence.codec import CURRENT_TOOL_KEY
from ktcc.toolchange.state import ChangerMode, ChangerState
from ktcc.tools import ToolId, Vector3
from ktcc.profiles.models import FirmwareRetractionProfile, PressureAdvanceProfile
from ktcc.toolchange.recovery import FailureCode
from ktcc.ports import ProfilePersistencePort, StatePersistencePort


class RecordingVariableStore:
    def __init__(
        self,
        variables: Optional[Mapping[str, Any]] = None,
        *,
        fail_on: Optional[str] = None,
    ) -> None:
        self.variables = dict(variables or {})
        self.fail_on = fail_on
        self.effects: list[tuple[str, Any]] = []

    def read_variables(self) -> Mapping[str, Any]:
        self.effects.append(("read", None))
        return dict(self.variables)

    def write_variable(self, name: str, value: Any) -> None:
        self.effects.append((name, value))
        if name == self.fail_on:
            raise OSError(f"injected write failure for {name}")
        self.variables[name] = value


def test_repository_implements_typed_persistence_ports():
    repository = StateRepository(RecordingVariableStore())

    assert isinstance(repository, StatePersistencePort)
    assert isinstance(repository, ProfilePersistencePort)


def test_load_exposes_existing_current_key_and_profile_values():
    store = RecordingVariableStore(
        {
            CURRENT_TOOL_KEY: 1,
            "ktcc_tool_retract_3": {"retract_length": 0.7},
        }
    )

    decoded = StateRepository(store).load()

    assert decoded.state == ChangerState.idle_with_tool(1)
    assert decoded.profile_values == {
        "ktcc_tool_retract_3": {"retract_length": 0.7}
    }


def test_load_state_reads_existing_current_key():
    repository = StateRepository(RecordingVariableStore({CURRENT_TOOL_KEY: 2}))

    assert repository.load_state() == ChangerState.idle_with_tool(2)


def test_malformed_current_requires_recovery():
    repository = StateRepository(
        RecordingVariableStore({CURRENT_TOOL_KEY: {"tool": 3}})
    )

    decoded = repository.load()

    assert decoded.state.mode is ChangerMode.RECOVERY_REQUIRED
    assert decoded.state.failure is not None
    assert decoded.state.failure.error_code == FailureCode.MALFORMED_PERSISTENCE.value


def test_state_and_mirror_use_the_same_existing_current_key():
    store = RecordingVariableStore()
    repository = StateRepository(store)
    state = ChangerState.idle_with_tool(1, revision=4)

    repository.save_state(state)
    assert [name for name, _ in store.effects] == [CURRENT_TOOL_KEY]

    repository.mirror_tool_current(state.legacy_tool_current())

    assert [name for name, _ in store.effects] == [CURRENT_TOOL_KEY, CURRENT_TOOL_KEY]
    assert store.variables[CURRENT_TOOL_KEY] == 1


def test_profile_writes_freeze_deployed_legacy_value_shapes():
    store = RecordingVariableStore()
    repository = StateRepository(store)

    repository.save_retraction(
        ToolId(2),
        FirmwareRetractionProfile(0.6, 24.0, 0.06, 25.0, zhop=0.5),
    )
    repository.save_pressure_advance(ToolId(2), PressureAdvanceProfile(0.03, 0.04))
    repository.save_tool_offset(ToolId(2), Vector3(0.21, 0.22, 0.23))
    repository.save_global_offset(Vector3(0.001, 0.002, 0.003))

    assert store.variables == {
        "ktcc_tool_retract_2": {
            "retract_length": 0.6,
            "retract_speed": 24.0,
            "unretract_extra_length": 0.06,
            "unretract_speed": 25.0,
        },
        "ktcc_tool_pa_info_2": {
            "pressure_advance": 0.03,
            "smooth_time": 0.04,
        },
        "ktcc_tool_offset_2": [0.21, 0.22, 0.23],
        "ktcc_global_offset": [0.001, 0.002, 0.003],
    }


def test_four_tool_profile_keys_never_cross_tool_identity():
    store = RecordingVariableStore()
    repository = StateRepository(store)

    for number in range(4):
        tool_id = ToolId(number)
        repository.save_retraction(
            tool_id,
            FirmwareRetractionProfile(
                retract_length=number + 0.1,
                retract_speed=20.0 + number,
                unretract_extra_length=number + 0.01,
                unretract_speed=21.0 + number,
            ),
        )
        repository.save_pressure_advance(
            tool_id, PressureAdvanceProfile(number / 100.0, 0.03 + number / 100.0)
        )
        repository.save_tool_offset(tool_id, Vector3(number, number + 0.1, number + 0.2))

    for number in range(4):
        assert store.variables[f"ktcc_tool_retract_{number}"]["retract_length"] == pytest.approx(
            number + 0.1
        )
        assert store.variables[f"ktcc_tool_pa_info_{number}"]["pressure_advance"] == pytest.approx(
            number / 100.0
        )
        assert store.variables[f"ktcc_tool_offset_{number}"] == [
            number,
            number + 0.1,
            number + 0.2,
        ]


def test_failed_state_write_does_not_create_a_successful_value():
    previous = ChangerState.idle_without_tool(revision=0)
    attempted = ChangerState.idle_with_tool(0, revision=2)
    store = RecordingVariableStore(
    {CURRENT_TOOL_KEY: -1}, fail_on=CURRENT_TOOL_KEY
    )
    repository = StateRepository(store)

    with pytest.raises(OSError, match="injected write failure"):
        repository.save_state(attempted)

    assert store.variables[CURRENT_TOOL_KEY] == -1
    assert repository.load_state() == previous


def test_failed_current_write_keeps_previous_value():
    store = RecordingVariableStore({CURRENT_TOOL_KEY: -1}, fail_on=CURRENT_TOOL_KEY)
    repository = StateRepository(store)
    state = ChangerState.idle_with_tool(3, revision=5)

    with pytest.raises(OSError, match="injected write failure"):
        repository.save_state(state)

    assert store.variables[CURRENT_TOOL_KEY] == -1
    assert [name for name, _ in store.effects] == [CURRENT_TOOL_KEY]


@pytest.mark.parametrize("invalid", [-3, True, 1.5, "1"])
def test_current_mirror_rejects_invalid_values_before_raw_write(invalid):
    store = RecordingVariableStore()
    repository = StateRepository(store)

    with pytest.raises(ValueError):
        repository.mirror_tool_current(invalid)

    assert store.effects == []
