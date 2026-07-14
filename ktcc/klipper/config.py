"""Klipper-free bridge from loaded legacy facades to the pure config model.

Composition code may construct the snapshots below at ``klippy:connect`` or
``klippy:ready``.  The bridge deliberately accepts data and opaque, precompiled
templates only; it neither imports Klipper nor retains a ConfigWrapper.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Iterable, Mapping, TypeAlias

from ktcc.config.resolver import ConfigBuilder, ResourceCatalog
from ktcc.config.schema import ConfigError, RawTool, RawToolGroup, RemovedFeatureError
from ktcc.tools import ToolId, ToolRegistry


_SCALAR_FIELDS = frozenset(
    {
        "extruder",
        "heater",
        "fan",
        "zone",
        "park",
        "offset",
        "meltzonelength",
        "unload_length",
        "prime_length",
        "retract_length",
        "retract_speed",
        "unretract_extra_length",
        "unretract_speed",
        "zhop",
        "pressure_advance",
        "pressure_advance_smooth_time",
        "heater_active_temp",
        "heater_standby_temp",
        "idle_to_standby_time",
        "idle_to_powerdown_time",
        "select_wait_mode",
        "select_wait_tolerance",
        "select_wait_timeout",
        "select_wait_stable_time",
        "shaper_freq_x",
        "shaper_freq_y",
        "shaper_type_x",
        "shaper_type_y",
        "shaper_damping_ratio_x",
        "shaper_damping_ratio_y",
    }
)

_ACTION_FIELDS = MappingProxyType(
    {
        "pickup": "pickup_gcode",
        "dropoff": "dropoff_gcode",
        "verify_mounted": "verify_mounted_gcode",
        "verify_unmounted": "verify_unmounted_gcode",
        "recovery": "recovery_gcode",
    }
)

_REMOVED_FIELDS = frozenset(
    {
        "physical_parent",
        "physical_parent_id",
        "remap",
        "tool_remap",
        "virtual_toolload_gcode",
        "virtual_toolunload_gcode",
        "virtual_toolload_gcode_template",
        "virtual_toolunload_gcode_template",
        "requires_pickup_for_virtual_load",
        "requires_pickup_for_virtual_unload",
        "unload_virtual_at_dropoff",
    }
)


@dataclass(frozen=True)
class ToolGroupSnapshot:
    """Structural snapshot of one loaded physical ToolGroup facade."""

    id: int
    values: Mapping[str, Any] = field(default_factory=dict)
    action_templates: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ToolSnapshot:
    """Structural snapshot of one loaded physical Tool facade."""

    id: int
    group_id: int
    values: Mapping[str, Any] = field(default_factory=dict)
    action_templates: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class ActionBinding:
    """Adapter-owned association between a stable action ID and a template."""

    action_id: str
    tool_id: ToolId
    kind: str
    template: Any


@dataclass(frozen=True)
class ConfigBridgeResult:
    registry: ToolRegistry
    action_bindings: tuple[ActionBinding, ...]


GroupInput: TypeAlias = ToolGroupSnapshot | Mapping[str, Any]
ToolInput: TypeAlias = ToolSnapshot | Mapping[str, Any]


def _as_id(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ConfigError(f"{label} must be a non-negative integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{label} must be a non-negative integer") from exc
    if parsed < 0:
        raise ConfigError(f"{label} must be a non-negative integer")
    return parsed


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigError(f"{label} must be a mapping")
    if not all(isinstance(key, str) for key in value):
        raise ConfigError(f"{label} keys must be strings")
    return value


def _flat_group(snapshot: GroupInput) -> tuple[int, dict[str, Any], dict[str, Any]]:
    if isinstance(snapshot, ToolGroupSnapshot):
        return (
            _as_id(snapshot.id, "toolgroup id"),
            dict(_mapping(snapshot.values, "toolgroup values")),
            dict(_mapping(snapshot.action_templates, "toolgroup action_templates")),
        )
    source = dict(_mapping(snapshot, "toolgroup snapshot"))
    identifier = source.pop("id", source.pop("name", None))
    nested = source.pop("values", None)
    actions = source.pop("action_templates", {})
    if nested is not None:
        values = dict(_mapping(nested, "toolgroup values"))
        values.update(source)
    else:
        values = source
    return (
        _as_id(identifier, "toolgroup id"),
        values,
        dict(_mapping(actions, "toolgroup action_templates")),
    )


def _flat_tool(snapshot: ToolInput) -> tuple[int, int, dict[str, Any], dict[str, Any]]:
    if isinstance(snapshot, ToolSnapshot):
        return (
            _as_id(snapshot.id, "tool id"),
            _as_id(snapshot.group_id, "tool group_id"),
            dict(_mapping(snapshot.values, "tool values")),
            dict(_mapping(snapshot.action_templates, "tool action_templates")),
        )
    source = dict(_mapping(snapshot, "tool snapshot"))
    identifier = source.pop("id", source.pop("name", None))
    group_id = source.pop("group_id", source.pop("tool_group", None))
    nested = source.pop("values", None)
    actions = source.pop("action_templates", {})
    if nested is not None:
        values = dict(_mapping(nested, "tool values"))
        values.update(source)
    else:
        values = source
    return (
        _as_id(identifier, "tool id"),
        _as_id(group_id, "tool group_id"),
        values,
        dict(_mapping(actions, "tool action_templates")),
    )


def _truthy(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _inert_removed_value(value: Any) -> bool:
    return value is None or value is False or value == "" or value == -1


def _reject_removed(values: Mapping[str, Any], label: str) -> None:
    if _truthy(values.get("is_virtual", False)):
        raise RemovedFeatureError(f"{label}: virtual tools are not supported")
    for key in _REMOVED_FIELDS:
        if key in values and not _inert_removed_value(values[key]):
            raise RemovedFeatureError(f"{label}: removed virtual/remap field {key!r}")


def _split_values_and_actions(
    values: Mapping[str, Any], explicit_actions: Mapping[str, Any], label: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    _reject_removed(values, label)
    scalars = {key: value for key, value in values.items() if key in _SCALAR_FIELDS}
    actions = dict(explicit_actions)
    for kind in _ACTION_FIELDS:
        aliases = (kind, f"{kind}_template", f"{kind}_gcode_template")
        for alias in aliases:
            if alias in values:
                actions[kind] = values[alias]
    unknown_actions = set(actions).difference(_ACTION_FIELDS)
    if unknown_actions:
        names = ", ".join(sorted(unknown_actions))
        raise ConfigError(f"{label}: unknown action template kinds: {names}")
    return scalars, actions


def build_config_bridge(
    groups: Iterable[GroupInput],
    tools: Iterable[ToolInput],
    resources: ResourceCatalog = ResourceCatalog(),
) -> ConfigBridgeResult:
    """Build a pure registry and adapter-owned bindings in any input order."""

    group_actions: dict[int, dict[str, Any]] = {}
    builder = ConfigBuilder()

    for snapshot in groups:
        group_id, values, actions = _flat_group(snapshot)
        scalars, templates = _split_values_and_actions(
            values, actions, f"toolgroup {group_id}"
        )
        # Let ConfigBuilder own duplicate-section validation.
        builder.collect_group(RawToolGroup(group_id, scalars))
        group_actions[group_id] = templates

    bindings: list[ActionBinding] = []
    for snapshot in tools:
        tool_id, group_id, values, actions = _flat_tool(snapshot)
        scalars, templates = _split_values_and_actions(values, actions, f"tool {tool_id}")
        merged_templates = dict(group_actions.get(group_id, {}))
        merged_templates.update(templates)
        for kind, config_key in _ACTION_FIELDS.items():
            if kind not in merged_templates or merged_templates[kind] is None:
                continue
            action_id = f"tool:{tool_id}:{kind}"
            scalars[config_key] = action_id
            bindings.append(
                ActionBinding(action_id, ToolId(tool_id), kind, merged_templates[kind])
            )
        builder.collect_tool(RawTool(tool_id, group_id, scalars))

    registry = builder.resolve(resources)
    return ConfigBridgeResult(registry, tuple(bindings))


__all__ = [
    "ActionBinding",
    "ConfigBridgeResult",
    "ToolGroupSnapshot",
    "ToolSnapshot",
    "build_config_bridge",
]
