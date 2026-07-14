"""Klipper boundary for the public ``save_variables`` interface.

The adapter deliberately knows only the two stable integration surfaces needed
by :class:`ktcc.ports.VariableStorePort`: the object's status callback and the
G-code dispatcher's normal command path.  In particular, it never reaches into
``SaveVariables.allVariables`` or calls its command handler directly.
"""

from __future__ import annotations

import ast
import copy
import math
import re
import shlex
from typing import Any, Callable, Mapping, Protocol


class SaveVariablesStatus(Protocol):
    """Structural subset of Klipper's ``save_variables`` object."""

    def get_status(self, eventtime: float) -> Mapping[str, Any]: ...


class GCodeDispatcher(Protocol):
    """Structural subset of Klipper's G-code dispatcher used for writes."""

    def run_script_from_command(self, script: str) -> None: ...


EventtimeProvider = Callable[[], float]


_VARIABLE_NAME = re.compile(r"[a-z_][a-z0-9_]{0,127}\Z")
_LITERAL_TYPES = (dict, list, int, float, str, bool, type(None))


class KlipperVariableStore:
    """Implement the raw variable store using documented Klipper APIs.

    ``eventtime`` is injected instead of looking up a reactor, which keeps this
    module importable and testable without a Klipper installation.
    """

    def __init__(
        self,
        save_variables: SaveVariablesStatus,
        gcode: GCodeDispatcher,
        eventtime: EventtimeProvider,
    ) -> None:
        if not callable(getattr(save_variables, "get_status", None)):
            raise TypeError("save_variables must provide get_status(eventtime)")
        if not callable(getattr(gcode, "run_script_from_command", None)):
            raise TypeError("gcode must provide run_script_from_command(script)")
        if not callable(eventtime):
            raise TypeError("eventtime must be callable")
        self._save_variables = save_variables
        self._gcode = gcode
        self._eventtime = eventtime

    def read_variables(self) -> Mapping[str, Any]:
        status = self._save_variables.get_status(self._eventtime())
        if not isinstance(status, Mapping):
            raise TypeError("save_variables status must be a mapping")
        variables = status.get("variables")
        if not isinstance(variables, Mapping):
            raise TypeError("save_variables status must contain a variables mapping")
        # A deep copy prevents callers from mutating nested objects owned by
        # Klipper's status provider through this boundary.
        return copy.deepcopy(dict(variables))

    def write_variable(self, name: str, value: Any) -> None:
        if not isinstance(name, str) or _VARIABLE_NAME.fullmatch(name) is None:
            raise ValueError(
                "variable name must be a lowercase identifier of at most 128 characters"
            )

        _validate_literal_value(value)
        try:
            literal = repr(value)
            decoded = ast.literal_eval(literal)
        except (RecursionError, SyntaxError, TypeError, ValueError) as exc:
            raise ValueError("variable value does not have a safe literal representation") from exc
        if type(decoded) is not type(value) or decoded != value:
            raise ValueError("variable value does not round-trip through a Python literal")

        # Klipper parses extended commands with POSIX shlex before
        # save_variables applies ast.literal_eval.  shlex.quote makes the whole
        # Python literal one VALUE token and prevents whitespace, comments, or
        # command-looking string contents from becoming G-code syntax.
        command = (
            f"SAVE_VARIABLE VARIABLE={name} "
            f"VALUE={shlex.quote(literal)}"
        )
        self._gcode.run_script_from_command(command)


def _validate_literal_value(
    value: Any,
    *,
    ancestors: frozenset[int] = frozenset(),
    depth: int = 0,
) -> None:
    """Accept only the JSON-like literal families used by the repository."""

    if depth > 64:
        raise ValueError("variable value nesting exceeds 64 levels")
    if type(value) not in _LITERAL_TYPES:
        raise TypeError(
            "variable value must contain only dict, list, int, float, str, bool, or None"
        )
    if type(value) is float and not math.isfinite(value):
        raise ValueError("floating-point variable values must be finite")
    if type(value) in (list, dict):
        marker = id(value)
        if marker in ancestors:
            raise ValueError("cyclic variable values are not supported")
        ancestors = ancestors | {marker}
    if type(value) is list:
        for item in value:
            _validate_literal_value(item, ancestors=ancestors, depth=depth + 1)
    elif type(value) is dict:
        for key, item in value.items():
            _validate_literal_value(key, ancestors=ancestors, depth=depth + 1)
            _validate_literal_value(item, ancestors=ancestors, depth=depth + 1)
