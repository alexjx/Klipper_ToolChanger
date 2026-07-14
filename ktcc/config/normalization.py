"""Normalization of accepted physical-tool field shapes."""

from __future__ import annotations

from typing import Any, Mapping

from .schema import ConfigError, RawTool, RawToolGroup, RemovedFeatureError


_REMOVED_KEYS = {
    "physical_parent",
    "virtual_toolload_gcode",
    "virtual_toolunload_gcode",
    "requires_pickup_for_virtual_load",
    "requires_pickup_for_virtual_unload",
    "unload_virtual_at_dropoff",
    "tool_remap",
    "remap",
}


def _section_id(section_name: str, expected: str) -> int:
    parts = section_name.strip().split()
    if len(parts) != 2 or parts[0].lower() != expected:
        raise ConfigError(f"expected section '[{expected} N]', got {section_name!r}")
    try:
        value = int(parts[1])
    except ValueError as exc:
        raise ConfigError(f"{expected} id must be an integer: {parts[1]!r}") from exc
    if value < 0:
        raise ConfigError(f"{expected} id must be non-negative")
    return value


def _truthy(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _reject_removed(values: Mapping[str, Any], section: str) -> None:
    if _truthy(values.get("is_virtual", False)):
        raise RemovedFeatureError(f"{section}: virtual tools are not supported")
    for key in _REMOVED_KEYS:
        if key not in values:
            continue
        value = values[key]
        is_inert = value is None or value is False or value == "" or value == -1
        if not is_inert:
            raise RemovedFeatureError(f"{section}: removed virtual/remap field {key!r}")


def normalize_tool_group_section(section_name: str, values: Mapping[str, Any]) -> RawToolGroup:
    group_id = _section_id(section_name, "toolgroup")
    _reject_removed(values, section_name)
    normalized = dict(values)
    normalized.pop("is_virtual", None)
    for key in _REMOVED_KEYS:
        normalized.pop(key, None)
    return RawToolGroup(group_id, normalized)


def normalize_tool_section(section_name: str, values: Mapping[str, Any]) -> RawTool:
    tool_id = _section_id(section_name, "tool")
    _reject_removed(values, section_name)
    normalized = dict(values)
    normalized.pop("is_virtual", None)
    for key in _REMOVED_KEYS:
        normalized.pop(key, None)
    group_value = normalized.pop("tool_group", 0)
    try:
        group_id = int(group_value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{section_name}: tool_group must be an integer") from exc
    return RawTool(tool_id, group_id, normalized)
