from __future__ import annotations

import ast
import shlex
from typing import Any, Mapping

import pytest

from ktcc.klipper.persistence import KlipperVariableStore
from ktcc.ports import VariableStorePort


class StatusStub:
    def __init__(self, variables: Mapping[str, Any]) -> None:
        self.variables = dict(variables)
        self.eventtimes: list[float] = []

    def get_status(self, eventtime: float) -> Mapping[str, Any]:
        self.eventtimes.append(eventtime)
        return {"variables": self.variables}


class GCodeStub:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.error: Exception | None = None

    def run_script_from_command(self, script: str) -> None:
        self.commands.append(script)
        if self.error is not None:
            raise self.error


def make_adapter(
    variables: Mapping[str, Any] | None = None,
) -> tuple[KlipperVariableStore, StatusStub, GCodeStub]:
    status = StatusStub(variables or {})
    gcode = GCodeStub()
    adapter = KlipperVariableStore(status, gcode, lambda: 12.5)
    return adapter, status, gcode


def test_adapter_satisfies_variable_store_port() -> None:
    adapter, _, _ = make_adapter()
    assert isinstance(adapter, VariableStorePort)


def test_read_uses_public_status_and_returns_deep_copy() -> None:
    adapter, status, _ = make_adapter({"profile": {"speeds": [20, 30]}})

    result = adapter.read_variables()
    result["profile"]["speeds"].append(40)

    assert status.eventtimes == [12.5]
    assert status.variables == {"profile": {"speeds": [20, 30]}}


def test_simple_write_command_encoding_is_frozen() -> None:
    adapter, _, gcode = make_adapter()

    adapter.write_variable("tool_current", 3)

    assert gcode.commands == ["SAVE_VARIABLE VARIABLE=tool_current VALUE=3"]


@pytest.mark.parametrize(
    "value",
    [
        None,
        False,
        7,
        0.042,
        [1, "two", None],
        {"pressure_advance": 0.042, "enabled": True, "note": None},
        "line one\n; SAVE_VARIABLE VARIABLE=owned VALUE=1 # comment",
    ],
)
def test_supported_values_round_trip_through_klipper_style_parsing(value: Any) -> None:
    adapter, _, gcode = make_adapter()

    adapter.write_variable("safe_name", value)

    command = gcode.commands[0]
    assert "\n" not in command
    lexer = shlex.shlex(command, posix=True)
    lexer.whitespace_split = True
    lexer.commenters = "#;"
    tokens = list(lexer)
    assert tokens[:2] == ["SAVE_VARIABLE", "VARIABLE=safe_name"]
    assert len(tokens) == 3
    encoded_value = tokens[2].removeprefix("VALUE=")
    decoded = ast.literal_eval(encoded_value)
    assert type(decoded) is type(value)
    assert decoded == value


@pytest.mark.parametrize(
    "name",
    ["", "Tool_Current", "tool-current", "tool current", "tool_current\nM112", "1tool"],
)
def test_dangerous_or_noncanonical_names_are_rejected(name: str) -> None:
    adapter, _, gcode = make_adapter()

    with pytest.raises(ValueError, match="lowercase identifier"):
        adapter.write_variable(name, 1)

    assert gcode.commands == []


@pytest.mark.parametrize(
    "value",
    [
        (1, 2),
        {1, 2},
        b"bytes",
        float("nan"),
        float("inf"),
        {"nested": object()},
    ],
)
def test_unsafe_or_unsupported_values_are_rejected(value: Any) -> None:
    adapter, _, gcode = make_adapter()

    with pytest.raises((TypeError, ValueError)):
        adapter.write_variable("safe_name", value)

    assert gcode.commands == []


def test_cyclic_and_excessively_nested_values_are_rejected_gracefully() -> None:
    cyclic: list[Any] = []
    cyclic.append(cyclic)
    nested: list[Any] = []
    for _ in range(66):
        nested = [nested]
    adapter, _, gcode = make_adapter()

    with pytest.raises(ValueError, match="cyclic"):
        adapter.write_variable("safe_name", cyclic)
    with pytest.raises(ValueError, match="nesting"):
        adapter.write_variable("safe_name", nested)

    assert gcode.commands == []


def test_dispatcher_exception_propagates_unchanged() -> None:
    adapter, _, gcode = make_adapter()
    failure = RuntimeError("Klipper write failed")
    gcode.error = failure

    with pytest.raises(RuntimeError) as caught:
        adapter.write_variable("tool_current", -2)

    assert caught.value is failure
