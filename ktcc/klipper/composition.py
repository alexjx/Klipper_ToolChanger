"""Klipper lifecycle composition for toolchanger features.

This is the only module that walks loaded legacy facade objects.  It snapshots
their configuration at ``klippy:ready`` and then wires feature services to the
narrow Klipper integrations. It intentionally performs no heater
or mechanical action during construction.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Iterable, Mapping

from ktcc.diagnostics import LoggingEventSink
from ktcc.klipper.config import (
    ConfigBridgeResult,
    ToolGroupSnapshot,
    ToolSnapshot,
    build_config_bridge,
)
from ktcc.klipper.persistence import KlipperVariableStore
from ktcc.klipper.runtime import (
    KlipperActionPort,
    KlipperExtrusionProfilePort,
    KlipperMachinePort,
    KlipperReadinessPort,
    KlipperSchedulerPort,
    KlipperShutdownPort,
    KlipperThermalPort,
)
from ktcc.profiles.service import ProfileService
from ktcc.statistics.service import StatisticsService
from ktcc.statistics.storage import StatisticsRepository
from ktcc.persistence import StateRepository
from ktcc.persistence.codec import reconcile_snapshot
from ktcc.thermal import ThermalService
from ktcc.toolchange.service import ToolChangeService
from ktcc.profiles.storage import PersistedProfileData, decode_persisted_profiles
from ktcc.toolchange.state import ChangerMode, MountedKind
from ktcc.tools import HeaterRef, ToolId, Vector3


@dataclass(frozen=True)
class RuntimeServices:
    """The authoritative service graph retained by the thin facades."""

    bridge: ConfigBridgeResult
    repository: StateRepository
    persisted_profiles: PersistedProfileData
    profiles: ProfileService
    thermal: ThermalService
    toolchange: ToolChangeService
    statistics: StatisticsService
    machine: KlipperMachinePort
    diagnostics: LoggingEventSink


class _ScopedTemplate:
    """Mark nested legacy lock commands as part of an owned action."""

    def __init__(self, template: Any, facade: Any) -> None:
        self._template = template
        self._facade = facade

    def run_gcode_from_command(self, context=None):
        self._facade._action_depth = getattr(self._facade, "_action_depth", 0) + 1
        try:
            return self._template.run_gcode_from_command(context)
        finally:
            self._facade._action_depth -= 1


def _tool_values(tool: Any) -> dict[str, Any]:
    values = {
        "extruder": tool.extruder,
        # In deployed Klipper configurations the extruder object owns its
        # heater.  An explicit future heater attribute remains supported.
        "heater": getattr(tool, "heater", None) or tool.extruder,
        "fan": tool.fan,
        "zone": tuple(tool.zone),
        "park": tuple(tool.park),
        "offset": tuple(tool.config_offset or tool.offset),
        "meltzonelength": tool.meltzonelength,
        "unload_length": getattr(tool, "unload_length", tool.meltzonelength),
        "prime_length": getattr(tool, "prime_length", tool.meltzonelength),
        "retract_length": tool.retract_length,
        "retract_speed": tool.retract_speed,
        "unretract_extra_length": tool.unretract_extra_length,
        "unretract_speed": tool.unretract_speed,
        "zhop": tool.zhop,
        "pressure_advance": tool.pressure_advance,
        "pressure_advance_smooth_time": tool.pressure_advance_smooth_time,
        "heater_active_temp": tool.heater_active_temp,
        "heater_standby_temp": tool.heater_standby_temp,
        "idle_to_standby_time": tool.idle_to_standby_time,
        "idle_to_powerdown_time": tool.idle_to_powerdown_time,
        "select_wait_mode": getattr(tool, "select_wait_mode", "HEAT"),
        "select_wait_tolerance": getattr(tool, "select_wait_tolerance", 1.0),
        "select_wait_timeout": getattr(tool, "select_wait_timeout", 900.0),
        "select_wait_stable_time": getattr(tool, "select_wait_stable_time", 0.0),
    }
    if float(tool.shaper_freq_x) != 0.0 or float(tool.shaper_freq_y) != 0.0:
        values.update(
            {
                "shaper_freq_x": tool.shaper_freq_x,
                "shaper_freq_y": tool.shaper_freq_y,
                "shaper_type_x": tool.shaper_type_x,
                "shaper_type_y": tool.shaper_type_y,
                "shaper_damping_ratio_x": tool.shaper_damping_ratio_x,
                "shaper_damping_ratio_y": tool.shaper_damping_ratio_y,
            }
        )
    return values


def snapshot_facades(
    groups: Iterable[Any], tools: Iterable[Any]
) -> tuple[tuple[ToolGroupSnapshot, ...], tuple[ToolSnapshot, ...]]:
    """Detach the core from mutable ConfigWrapper/facade instances."""

    group_snapshots = tuple(
        ToolGroupSnapshot(group.name) for group in groups
    )
    tool_snapshots = tuple(
        ToolSnapshot(
            id=tool.name,
            group_id=tool.toolgroup.name,
            values=_tool_values(tool),
            action_templates={
                kind: template
                for kind, template in {
                    "pickup": tool.pickup_gcode_template,
                    "dropoff": tool.dropoff_gcode_template,
                    "verify_mounted": getattr(tool, "verify_mounted_gcode_template", None),
                    "verify_unmounted": getattr(tool, "verify_unmounted_gcode_template", None),
                    "recovery": getattr(tool, "recovery_gcode_template", None),
                }.items()
                if template is not None
            },
        )
        for tool in tools
    )
    return group_snapshots, tool_snapshots


def compose_runtime(printer: Any, facade: Any) -> RuntimeServices:
    """Build the complete runtime graph from already loaded Klipper objects."""

    reactor = printer.get_reactor()
    gcode = printer.lookup_object("gcode")
    tool_items = tuple(printer.lookup_objects("tool"))
    group_items = tuple(printer.lookup_objects("toolgroup"))
    tools = tuple(obj for _, obj in tool_items)
    groups = tuple(obj for _, obj in group_items)
    group_snapshots, tool_snapshots = snapshot_facades(groups, tools)
    bridge = build_config_bridge(group_snapshots, tool_snapshots)

    store = KlipperVariableStore(
        printer.lookup_object("save_variables"), gcode, reactor.monotonic
    )
    repository = StateRepository(store)
    decoded = reconcile_snapshot(repository.load(), bridge.registry)
    persisted = decode_persisted_profiles(bridge.registry, decoded)
    statistics_repository = StatisticsRepository(store)
    statistics = StatisticsService(
        bridge.registry.ids,
        reactor,
        statistics_repository,
        statistics_repository.load(bridge.registry.ids),
    )
    if (
        decoded.state.mode is ChangerMode.IDLE
        and decoded.state.mounted.kind is MountedKind.KNOWN
    ):
        statistics.selection_changed(
            None, ToolId(decoded.state.mounted.tool_id)
        )

    command_status = gcode.get_status(reactor.monotonic())
    commands = command_status.get("commands", {})
    set_retraction_command = (
        "SET_RETRACTION_ORIG"
        if "SET_RETRACTION_ORIG" in commands
        else "SET_RETRACTION"
    )
    extrusion = KlipperExtrusionProfilePort(
        gcode,
        printer.lookup_object("firmware_retraction"),
        set_retraction_command=set_retraction_command,
    )
    machine = KlipperMachinePort(gcode, printer.lookup_object("toolhead"))
    profiles = ProfileService(
        bridge.registry,
        extrusion,
        machine,
        repository,
        persisted_patches=persisted.patches,
        persisted_offsets=persisted.tool_offsets,
        global_offset=persisted.global_offset,
    )

    heaters = printer.lookup_object("heaters")
    physical_heaters = {}
    for spec in bridge.registry:
        # HeaterRef is independent from ExtruderRef. Klipper's heaters object
        # resolves both extruder-owned and generic heaters by their configured
        # heater name; never infer the physical heater from the extruder.
        physical_heaters[spec.heater] = heaters.lookup_heater(spec.heater.value)
    auxiliary_heaters = {}
    bed = printer.lookup_object("heater_bed", None)
    if bed is not None:
        bed_ref = HeaterRef("heater_bed")
        # PrinterHeaterBed intentionally exposes the Heater instance it wraps;
        # PrinterHeaters.set_temperature requires that physical object.
        physical_heaters[bed_ref] = bed.heater
        auxiliary_heaters[0] = bed_ref
    thermal_port = KlipperThermalPort(
        heaters,
        physical_heaters,
        reactor,
        printer.is_shutdown,
    )
    thermal = ThermalService(
        bridge.registry,
        thermal_port,
        KlipperSchedulerPort(reactor),
        reactor,
        auxiliary_heaters,
        statistics=statistics,
    )

    templates = {
        (binding.tool_id, binding.action_id): _ScopedTemplate(binding.template, facade)
        for binding in bridge.action_bindings
    }
    gcode_macro = printer.lookup_object("gcode_macro")

    def facade_context(context):
        tool = printer.lookup_object("tool %d" % context.tool.id.value)
        return MappingProxyType(
            {"myself": tool.get_status(), "toollock": facade.get_status()}
        )

    action = KlipperActionPort(
        templates,
        lambda template, context: template.run_gcode_from_command(context),
        gcode_macro.create_template_context,
        facade_context,
    )
    diagnostics = LoggingEventSink()
    toolchange = ToolChangeService(
        decoded.state,
        bridge.registry,
        KlipperReadinessPort(
            printer.lookup_object("toolhead"),
            reactor.monotonic,
            printer.command_error,
        ),
        action,
        thermal,
        profiles,
        extrusion,
        repository,
        diagnostics,
        KlipperShutdownPort(printer),
        reactor,
        statistics=statistics,
    )
    return RuntimeServices(
        bridge,
        repository,
        persisted,
        profiles,
        thermal,
        toolchange,
        statistics,
        machine,
        diagnostics,
    )


def vector_with_updates(
    current: Vector3,
    *,
    x: float | None = None,
    x_adjust: float | None = None,
    y: float | None = None,
    y_adjust: float | None = None,
    z: float | None = None,
    z_adjust: float | None = None,
) -> Vector3:
    """Apply the legacy absolute-or-adjust-per-axis command semantics."""

    return Vector3(
        x if x is not None else current.x + (x_adjust or 0.0),
        y if y is not None else current.y + (y_adjust or 0.0),
        z if z is not None else current.z + (z_adjust or 0.0),
    )


__all__ = [
    "RuntimeServices",
    "compose_runtime",
    "snapshot_facades",
    "vector_with_updates",
]
