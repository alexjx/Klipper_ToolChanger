"""Codec and repository for the deployed statistics save-variable shape."""

from __future__ import annotations

import math
from typing import Any, Iterable, Mapping

from ktcc.ports import VariableStorePort
from ktcc.tools import ToolId

from .models import GlobalStatistics, StatisticsSnapshot, ToolStatistics

SWAP_KEY = "ktcc_statistics_swaps"
TOOL_PREFIX = "ktcc_statistics_tool"


def _number(value: Any, *, integer: bool = False) -> int | float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return 0
    if value < 0:
        return 0
    return int(value) if integer else float(value)


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def decode_statistics(
    variables: Mapping[str, Any], tool_ids: Iterable[ToolId]
) -> StatisticsSnapshot:
    if not isinstance(variables, Mapping):
        raise TypeError("statistics variables must be a mapping")
    ids = tuple(sorted(tool_ids))
    if any(not isinstance(tool_id, ToolId) for tool_id in ids):
        raise TypeError("statistics tool ids must be ToolId instances")
    swap = _mapping(variables.get(SWAP_KEY))
    global_totals = GlobalStatistics(
        total_time_spent_mounting=_number(swap.get("total_time_spent_mounting")),
        total_time_spent_unmounting=_number(swap.get("total_time_spent_unmounting")),
        total_toollocks=_number(swap.get("total_toollocks"), integer=True),
        total_toolunlocks=_number(swap.get("total_toolunlocks"), integer=True),
        total_toolmounts=_number(swap.get("total_toolmounts"), integer=True),
        total_toolunmounts=_number(swap.get("total_toolunmounts"), integer=True),
    )
    tools = {}
    integer_fields = {
        "toolmounts_completed", "toolunmounts_completed",
        "toolmounts_started", "toolunmounts_started",
    }
    for tool_id in ids:
        raw = _mapping(variables.get(f"{TOOL_PREFIX}{tool_id.value}"))
        values = {
            name: _number(raw.get(name), integer=name in integer_fields)
            for name in ToolStatistics.__dataclass_fields__
        }
        tools[tool_id] = ToolStatistics(**values)
    return StatisticsSnapshot(global_totals, tools)


class StatisticsRepository:
    def __init__(self, store: VariableStorePort) -> None:
        if not isinstance(store, VariableStorePort):
            raise TypeError("store must implement VariableStorePort")
        self._store = store

    def load(self, tool_ids: Iterable[ToolId]) -> StatisticsSnapshot:
        return decode_statistics(self._store.read_variables(), tool_ids)

    def save(self, snapshot: StatisticsSnapshot) -> None:
        if not isinstance(snapshot, StatisticsSnapshot):
            raise TypeError("snapshot must be a StatisticsSnapshot")
        totals = snapshot.global_totals
        self._store.write_variable(SWAP_KEY, {
            name: round(getattr(totals, name), 1)
            if name.startswith("total_time") else getattr(totals, name)
            for name in GlobalStatistics.__dataclass_fields__
        })
        for tool_id, stats in snapshot.tools.items():
            value = {name: getattr(stats, name) for name in ToolStatistics.__dataclass_fields__}
            # Preserve old public dictionary fields without persisting process-local clocks.
            value.update({
                "tracked_start_time_selected": 0,
                "tracked_start_time_active": 0,
                "tracked_start_time_standby": 0,
                "tracked_unmount_start_time": 0,
                "tracked_mount_start_time": 0,
            })
            self._store.write_variable(f"{TOOL_PREFIX}{tool_id.value}", value)
