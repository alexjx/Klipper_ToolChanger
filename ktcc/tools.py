"""Immutable physical-tool definitions.

These objects describe configuration facts only.  Mounted state, heater state,
Klipper objects, and mutable profile application state do not belong here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Iterable, Iterator, Mapping, Optional

from .profiles.models import ToolProfile


class ToolDefinitionError(ValueError):
    """A physical-tool definition violates an invariant."""


@dataclass(frozen=True, order=True)
class ToolId:
    value: int

    def __post_init__(self) -> None:
        if isinstance(self.value, bool) or not isinstance(self.value, int):
            raise ToolDefinitionError("tool id must be an integer")
        if self.value < 0:
            raise ToolDefinitionError("physical tool id must be non-negative")

    def __int__(self) -> int:
        return self.value

    def __str__(self) -> str:
        return str(self.value)


def _validate_ref(kind: str, value: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ToolDefinitionError(f"{kind} reference must be a non-empty string")
    if value != value.strip():
        raise ToolDefinitionError(f"{kind} reference must not have surrounding whitespace")


@dataclass(frozen=True, order=True)
class ExtruderRef:
    value: str

    def __post_init__(self) -> None:
        _validate_ref("extruder", self.value)

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, order=True)
class HeaterRef:
    value: str

    def __post_init__(self) -> None:
        _validate_ref("heater", self.value)

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True, order=True)
class FanRef:
    value: str

    def __post_init__(self) -> None:
        _validate_ref("fan", self.value)

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for axis in ("x", "y", "z"):
            value = getattr(self, axis)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ToolDefinitionError(
                    f"vector component {axis} must be a finite number"
                )


@dataclass(frozen=True)
class DockSpec:
    zone: Vector3
    park: Vector3


@dataclass(frozen=True)
class ToolActions:
    """Names of adapter-owned, precompiled actions."""

    pickup: str = ""
    dropoff: str = ""
    verify_mounted: Optional[str] = None
    verify_unmounted: Optional[str] = None
    recovery: Optional[str] = None

    def __post_init__(self) -> None:
        for field_name in ("pickup", "dropoff"):
            if not isinstance(getattr(self, field_name), str):
                raise ToolDefinitionError(f"{field_name} action must be a string")
        for field_name in ("verify_mounted", "verify_unmounted", "recovery"):
            value = getattr(self, field_name)
            if value is not None and not isinstance(value, str):
                raise ToolDefinitionError(f"{field_name} action must be a string or None")


@dataclass(frozen=True)
class ToolSpec:
    id: ToolId
    extruder: ExtruderRef
    heater: HeaterRef
    fan: Optional[FanRef]
    dock: DockSpec
    offset: Vector3
    actions: ToolActions
    configured_profile: ToolProfile


class ToolRegistry:
    """Immutable, ID-ordered collection of physical tools."""

    __slots__ = ("_by_id", "_tools")

    def __init__(self, tools: Iterable[ToolSpec]):
        by_id = {}
        for tool in tools:
            if tool.id in by_id:
                raise ToolDefinitionError(f"duplicate physical tool id: {tool.id}")
            by_id[tool.id] = tool
        ordered = tuple(by_id[key] for key in sorted(by_id))
        if not ordered:
            raise ToolDefinitionError("tool registry must contain at least one tool")
        self._tools = ordered
        self._by_id: Mapping[ToolId, ToolSpec] = MappingProxyType(
            {tool.id: tool for tool in ordered}
        )

    def __len__(self) -> int:
        return len(self._tools)

    def __iter__(self) -> Iterator[ToolSpec]:
        return iter(self._tools)

    def __getitem__(self, tool_id: ToolId | int) -> ToolSpec:
        key = tool_id if isinstance(tool_id, ToolId) else ToolId(tool_id)
        return self._by_id[key]

    @property
    def ids(self) -> tuple[ToolId, ...]:
        return tuple(tool.id for tool in self._tools)

    def get(self, tool_id: ToolId | int) -> Optional[ToolSpec]:
        key = tool_id if isinstance(tool_id, ToolId) else ToolId(tool_id)
        return self._by_id.get(key)
