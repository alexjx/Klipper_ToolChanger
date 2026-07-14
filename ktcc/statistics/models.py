"""Immutable public statistics values."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from types import MappingProxyType
from typing import Mapping

from ktcc.tools import ToolId


def _duration(name: str, value: float) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
    ):
        raise ValueError(f"{name} must be a non-negative finite number")


def _count(name: str, value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")


@dataclass(frozen=True)
class GlobalStatistics:
    total_time_spent_mounting: float = 0.0
    total_time_spent_unmounting: float = 0.0
    total_toollocks: int = 0
    total_toolunlocks: int = 0
    total_toolmounts: int = 0
    total_toolunmounts: int = 0

    def __post_init__(self) -> None:
        _duration("total_time_spent_mounting", self.total_time_spent_mounting)
        _duration("total_time_spent_unmounting", self.total_time_spent_unmounting)
        for name in (
            "total_toollocks",
            "total_toolunlocks",
            "total_toolmounts",
            "total_toolunmounts",
        ):
            _count(name, getattr(self, name))


@dataclass(frozen=True)
class ToolStatistics:
    toolmounts_completed: int = 0
    toolunmounts_completed: int = 0
    toolmounts_started: int = 0
    toolunmounts_started: int = 0
    time_selected: float = 0.0
    time_heater_active: float = 0.0
    time_heater_standby: float = 0.0
    total_time_spent_unmounting: float = 0.0
    total_time_spent_mounting: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "toolmounts_completed",
            "toolunmounts_completed",
            "toolmounts_started",
            "toolunmounts_started",
        ):
            _count(name, getattr(self, name))
        for name in (
            "time_selected",
            "time_heater_active",
            "time_heater_standby",
            "total_time_spent_unmounting",
            "total_time_spent_mounting",
        ):
            _duration(name, getattr(self, name))


@dataclass(frozen=True)
class StatisticsSnapshot:
    global_totals: GlobalStatistics = field(default_factory=GlobalStatistics)
    tools: Mapping[ToolId, ToolStatistics] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.global_totals, GlobalStatistics):
            raise TypeError("global_totals must be GlobalStatistics")
        if not isinstance(self.tools, Mapping):
            raise TypeError("tools must be a mapping")
        if any(not isinstance(key, ToolId) for key in self.tools):
            raise TypeError("statistics tool keys must be ToolId")
        if any(not isinstance(value, ToolStatistics) for value in self.tools.values()):
            raise TypeError("statistics tool values must be ToolStatistics")
        ordered = {key: self.tools[key] for key in sorted(self.tools)}
        object.__setattr__(self, "tools", MappingProxyType(ordered))

    @classmethod
    def empty(cls, tool_ids: tuple[ToolId, ...]) -> "StatisticsSnapshot":
        return cls(tools={tool_id: ToolStatistics() for tool_id in sorted(tool_ids)})

    def delta(self, baseline: "StatisticsSnapshot") -> "StatisticsSnapshot":
        def subtract(current: object, previous: object, names: tuple[str, ...]):
            return {name: max(0, getattr(current, name) - getattr(previous, name)) for name in names}

        global_names = tuple(GlobalStatistics.__dataclass_fields__)
        tool_names = tuple(ToolStatistics.__dataclass_fields__)
        tools = {}
        for tool_id, current in self.tools.items():
            previous = baseline.tools.get(tool_id, ToolStatistics())
            tools[tool_id] = ToolStatistics(**subtract(current, previous, tool_names))
        return StatisticsSnapshot(
            GlobalStatistics(**subtract(self.global_totals, baseline.global_totals, global_names)),
            tools,
        )
