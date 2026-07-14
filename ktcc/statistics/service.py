"""Event-driven statistics collection using monotonic process time."""

from __future__ import annotations

from dataclasses import replace
from enum import Enum
import math
from typing import Iterable, Optional

from ktcc.ports import Clock, ThermalMode
from ktcc.tools import ToolId

from .models import GlobalStatistics, StatisticsSnapshot, ToolStatistics
from .storage import StatisticsRepository


class MechanicalKind(str, Enum):
    MOUNT = "MOUNT"
    UNMOUNT = "UNMOUNT"


class StatisticsService:
    def __init__(self, tool_ids: Iterable[ToolId], clock: Clock,
                 repository: Optional[StatisticsRepository] = None,
                 initial: Optional[StatisticsSnapshot] = None) -> None:
        self._ids = tuple(sorted(tool_ids))
        if not self._ids or any(not isinstance(tool_id, ToolId) for tool_id in self._ids):
            raise ValueError("statistics require at least one physical ToolId")
        if len(self._ids) != len(set(self._ids)):
            raise ValueError("statistics tool ids must be unique")
        if not callable(getattr(clock, "monotonic", None)):
            raise TypeError("clock must provide monotonic()")
        self._clock = clock
        self._repository = repository
        self._snapshot = initial or StatisticsSnapshot.empty(self._ids)
        if tuple(self._snapshot.tools) != self._ids:
            raise ValueError("initial statistics must match configured physical tools")
        self._mechanical: dict[tuple[MechanicalKind, ToolId, str], float] = {}
        self._selected: dict[ToolId, float] = {}
        self._thermal: dict[ToolId, tuple[ThermalMode, float]] = {}
        self._print_baseline = self.snapshot()

    def _now(self) -> float:
        value = self._clock.monotonic()
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ValueError("statistics clock must be non-negative and finite")
        return float(value)

    def _known(self, tool_id: ToolId) -> None:
        if tool_id not in self._snapshot.tools:
            raise KeyError(f"unknown physical tool: {tool_id}")

    def _set_tool(self, tool_id: ToolId, stats: ToolStatistics) -> None:
        tools = dict(self._snapshot.tools)
        tools[tool_id] = stats
        self._snapshot = StatisticsSnapshot(self._snapshot.global_totals, tools)

    def mechanical_started(self, kind: MechanicalKind, tool_id: ToolId,
                           transition_id: str) -> None:
        self._known(tool_id)
        if not isinstance(transition_id, str) or not transition_id:
            raise ValueError("transition_id must be a non-empty string")
        key = (MechanicalKind(kind), tool_id, transition_id)
        if key in self._mechanical:
            return
        self._mechanical[key] = self._now()
        stats = self._snapshot.tools[tool_id]
        field = "toolmounts_started" if key[0] is MechanicalKind.MOUNT else "toolunmounts_started"
        self._set_tool(tool_id, replace(stats, **{field: getattr(stats, field) + 1}))

    def mechanical_completed(self, kind: MechanicalKind, tool_id: ToolId,
                             transition_id: str) -> None:
        self._known(tool_id)
        kind = MechanicalKind(kind)
        started = self._mechanical.pop((kind, tool_id, transition_id), None)
        if started is None:
            return
        elapsed = max(0.0, self._now() - started)
        stats = self._snapshot.tools[tool_id]
        totals = self._snapshot.global_totals
        if kind is MechanicalKind.MOUNT:
            stats = replace(stats, toolmounts_completed=stats.toolmounts_completed + 1,
                            total_time_spent_mounting=stats.total_time_spent_mounting + elapsed)
            totals = replace(totals, total_toolmounts=totals.total_toolmounts + 1,
                             total_time_spent_mounting=totals.total_time_spent_mounting + elapsed)
        else:
            stats = replace(stats, toolunmounts_completed=stats.toolunmounts_completed + 1,
                            total_time_spent_unmounting=stats.total_time_spent_unmounting + elapsed)
            totals = replace(totals, total_toolunmounts=totals.total_toolunmounts + 1,
                             total_time_spent_unmounting=totals.total_time_spent_unmounting + elapsed)
        self._snapshot = StatisticsSnapshot(totals, {**self._snapshot.tools, tool_id: stats})

    def selection_changed(self, previous: Optional[ToolId], current: Optional[ToolId]) -> None:
        now = self._now()
        if previous is not None:
            self._known(previous)
            started = self._selected.pop(previous, None)
            if started is not None:
                stats = self._snapshot.tools[previous]
                self._set_tool(previous, replace(stats, time_selected=stats.time_selected + max(0, now - started)))
        if current is not None:
            self._known(current)
            self._selected.setdefault(current, now)

    def lock_completed(self) -> None:
        totals = self._snapshot.global_totals
        self._snapshot = StatisticsSnapshot(replace(totals, total_toollocks=totals.total_toollocks + 1), self._snapshot.tools)

    def unlock_completed(self) -> None:
        totals = self._snapshot.global_totals
        self._snapshot = StatisticsSnapshot(replace(totals, total_toolunlocks=totals.total_toolunlocks + 1), self._snapshot.tools)

    def thermal_changed(self, tool_id: ToolId, previous: ThermalMode,
                        current: ThermalMode) -> None:
        self._known(tool_id)
        ThermalMode(previous)  # Validate the observer's complete transition.
        current = ThermalMode(current)
        now = self._now()
        tracked = self._thermal.pop(tool_id, None)
        if tracked is not None:
            old_mode, started = tracked
            elapsed = max(0.0, now - started)
            stats = self._snapshot.tools[tool_id]
            if old_mode is ThermalMode.ACTIVE:
                self._set_tool(tool_id, replace(stats, time_heater_active=stats.time_heater_active + elapsed))
            elif old_mode is ThermalMode.STANDBY:
                self._set_tool(tool_id, replace(stats, time_heater_standby=stats.time_heater_standby + elapsed))
        if current in (ThermalMode.ACTIVE, ThermalMode.STANDBY):
            self._thermal[tool_id] = (current, now)

    def snapshot(self) -> StatisticsSnapshot:
        now = self._now()
        tools = dict(self._snapshot.tools)
        for tool_id, started in self._selected.items():
            stats = tools[tool_id]
            tools[tool_id] = replace(stats, time_selected=stats.time_selected + max(0, now - started))
        for tool_id, (mode, started) in self._thermal.items():
            stats = tools[tool_id]
            elapsed = max(0, now - started)
            if mode is ThermalMode.ACTIVE:
                tools[tool_id] = replace(stats, time_heater_active=stats.time_heater_active + elapsed)
            elif mode is ThermalMode.STANDBY:
                tools[tool_id] = replace(stats, time_heater_standby=stats.time_heater_standby + elapsed)
        return StatisticsSnapshot(self._snapshot.global_totals, tools)

    def begin_print(self) -> None:
        self._print_baseline = self.snapshot()

    def print_snapshot(self) -> StatisticsSnapshot:
        return self.snapshot().delta(self._print_baseline)

    def reset(self) -> None:
        now = self._now()
        self._snapshot = StatisticsSnapshot.empty(self._ids)
        self._mechanical.clear()
        self._selected = {tool_id: now for tool_id in self._selected}
        self._thermal = {tool_id: (mode, now) for tool_id, (mode, _) in self._thermal.items()}
        self._print_baseline = self.snapshot()

    def flush(self) -> None:
        if self._repository is None:
            return
        self._repository.save(self.snapshot())
