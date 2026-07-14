"""Resolve collected raw sections into an immutable physical tool registry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Mapping, Optional

from ktcc.tools import (
    DockSpec,
    ExtruderRef,
    FanRef,
    HeaterRef,
    ToolActions,
    ToolId,
    ToolRegistry,
    ToolSpec,
    Vector3,
)
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    MotionProfile,
    PressureAdvanceProfile,
    ThermalPolicy,
    ToolProfile,
    ToolchangeFilamentProfile,
)

from .schema import (
    ConfigError,
    DuplicateSectionError,
    InvalidShapeError,
    MissingReferenceError,
    RawTool,
    RawToolGroup,
)


@dataclass(frozen=True)
class ResourceCatalog:
    extruders: Optional[frozenset[str]] = None
    heaters: Optional[frozenset[str]] = None
    fans: Optional[frozenset[str]] = None


class ConfigBuilder:
    """Collect sections in any order, then resolve exactly once per call."""

    def __init__(self) -> None:
        self._groups: dict[int, RawToolGroup] = {}
        self._tools: dict[int, RawTool] = {}

    def collect_group(self, group: RawToolGroup) -> None:
        if group.id in self._groups:
            raise DuplicateSectionError(f"duplicate toolgroup id: {group.id}")
        self._groups[group.id] = group

    def collect_tool(self, tool: RawTool) -> None:
        if tool.id in self._tools:
            raise DuplicateSectionError(f"duplicate tool id: {tool.id}")
        self._tools[tool.id] = tool

    def resolve(self, resources: ResourceCatalog = ResourceCatalog()) -> ToolRegistry:
        return resolve_config(self._groups.values(), self._tools.values(), resources)


def _number(values: Mapping[str, Any], key: str, default: float) -> float:
    value = values.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{key} must be a number, got {value!r}") from exc


def _vector(values: Mapping[str, Any], key: str) -> Vector3:
    raw = values.get(key, "0,0,0")
    parts = [part.strip() for part in raw.split(",")] if isinstance(raw, str) else list(raw)
    if len(parts) != 3:
        raise InvalidShapeError(f"{key} must contain exactly x,y,z; got {raw!r}")
    try:
        return Vector3(*(float(part) for part in parts))
    except (TypeError, ValueError) as exc:
        raise InvalidShapeError(
            f"{key} must contain three finite numeric values; got {raw!r}"
        ) from exc


def _optional_string(values: Mapping[str, Any], key: str) -> Optional[str]:
    value = values.get(key)
    if value is None or str(value).strip().lower() in {"", "none"}:
        return None
    return str(value).strip()


def _require_resource(kind: str, name: str, available: Optional[frozenset[str]]) -> None:
    if available is not None and name not in available:
        raise MissingReferenceError(f"unknown {kind} reference: {name!r}")


def _profile(values: Mapping[str, Any]) -> ToolProfile:
    meltzone = _number(values, "meltzonelength", 0.0)
    motion_keys = {"shaper_freq_x", "shaper_freq_y", "shaper_type_x", "shaper_type_y", "shaper_damping_ratio_x", "shaper_damping_ratio_y"}
    motion = None
    if any(key in values for key in motion_keys):
        motion = MotionProfile(
            shaper_freq_x=_number(values, "shaper_freq_x", 0.0),
            shaper_freq_y=_number(values, "shaper_freq_y", 0.0),
            shaper_type_x=str(values.get("shaper_type_x", "mzv")),
            shaper_type_y=str(values.get("shaper_type_y", "mzv")),
            damping_ratio_x=_number(values, "shaper_damping_ratio_x", 0.1),
            damping_ratio_y=_number(values, "shaper_damping_ratio_y", 0.1),
        )
    return ToolProfile(
        print_retraction=FirmwareRetractionProfile(
            retract_length=_number(values, "retract_length", 0.0),
            retract_speed=_number(values, "retract_speed", 20.0),
            unretract_extra_length=_number(values, "unretract_extra_length", 0.0),
            unretract_speed=_number(values, "unretract_speed", 10.0),
            zhop=_number(values, "zhop", 0.0),
        ),
        pressure_advance=PressureAdvanceProfile(
            pressure_advance=_number(values, "pressure_advance", 0.0),
            smooth_time=_number(values, "pressure_advance_smooth_time", 0.04),
        ),
        toolchange_filament=ToolchangeFilamentProfile(
            unload_length=_number(values, "unload_length", meltzone),
            prime_length=_number(values, "prime_length", meltzone),
        ),
        thermal_policy=ThermalPolicy(
            default_active=_number(values, "heater_active_temp", 0.0),
            default_standby=_number(values, "heater_standby_temp", 0.0),
            idle_to_standby_time=_number(values, "idle_to_standby_time", 30.0),
            standby_to_off_time=_number(values, "idle_to_powerdown_time", 600.0),
            wait_mode=str(values.get("select_wait_mode", "HEAT")),
            tolerance=_number(values, "select_wait_tolerance", 1.0),
            timeout=_number(values, "select_wait_timeout", 900.0),
            stable_time=_number(values, "select_wait_stable_time", 0.0),
        ),
        motion=motion,
    )


def _resolve_tool(raw: RawTool, groups: Mapping[int, RawToolGroup], resources: ResourceCatalog) -> ToolSpec:
    group = groups.get(raw.group_id)
    if group is None:
        raise MissingReferenceError(f"tool {raw.id} references missing toolgroup {raw.group_id}")
    values = dict(group.values)
    values.update(raw.values)
    extruder = _optional_string(values, "extruder")
    if extruder is None:
        raise MissingReferenceError(f"tool {raw.id} has no extruder reference")
    heater = _optional_string(values, "heater") or extruder
    fan = _optional_string(values, "fan")
    _require_resource("extruder", extruder, resources.extruders)
    _require_resource("heater", heater, resources.heaters)
    if fan is not None:
        _require_resource("fan", fan, resources.fans)
    return ToolSpec(
        id=ToolId(raw.id),
        extruder=ExtruderRef(extruder),
        heater=HeaterRef(heater),
        fan=FanRef(fan) if fan is not None else None,
        dock=DockSpec(zone=_vector(values, "zone"), park=_vector(values, "park")),
        offset=_vector(values, "offset"),
        actions=ToolActions(
            pickup=str(values.get("pickup_gcode", "")),
            dropoff=str(values.get("dropoff_gcode", "")),
            verify_mounted=_optional_string(values, "verify_mounted_gcode"),
            verify_unmounted=_optional_string(values, "verify_unmounted_gcode"),
            recovery=_optional_string(values, "recovery_gcode"),
        ),
        configured_profile=_profile(values),
    )


def resolve_config(
    groups: Iterable[RawToolGroup],
    tools: Iterable[RawTool],
    resources: ResourceCatalog = ResourceCatalog(),
) -> ToolRegistry:
    group_map: dict[int, RawToolGroup] = {}
    for group in groups:
        if group.id in group_map:
            raise DuplicateSectionError(f"duplicate toolgroup id: {group.id}")
        group_map[group.id] = group
    tool_map: dict[int, RawTool] = {}
    for tool in tools:
        if tool.id in tool_map:
            raise DuplicateSectionError(f"duplicate tool id: {tool.id}")
        tool_map[tool.id] = tool
    if not tool_map:
        raise ConfigError("configuration must define at least one physical tool")
    return ToolRegistry(
        _resolve_tool(tool_map[tool_id], group_map, resources)
        for tool_id in sorted(tool_map)
    )
