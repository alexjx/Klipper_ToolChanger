from __future__ import annotations

from types import SimpleNamespace

import pytest

from ktcc.klipper.composition import (
    _ScopedTemplate,
    compose_runtime,
    vector_with_updates,
)
from ktcc.toolchange.service import ToolChangeStateError
from ktcc.toolchange.state import MountedKind
from ktcc.tools import Vector3
from ktcc.ports import ThermalMode
from toollock import ToolLock


class Reactor:
    NEVER = 1.0e30

    def __init__(self):
        self.now = 10.0
        self.timers = []

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = waketime
        return waketime

    def register_timer(self, callback, waketime):
        timer = (callback, waketime)
        self.timers.append(timer)
        return timer

    def unregister_timer(self, timer):
        self.timers.remove(timer)


class GCode:
    def __init__(self):
        self.commands = []
        self.status_commands = {"SET_RETRACTION": {}}

    def run_script_from_command(self, command):
        self.commands.append(command)

    def get_status(self, _eventtime):
        return {"commands": self.status_commands}


class Heater:
    def __init__(self):
        self.target = 0.0

    def get_status(self, _eventtime):
        return {
            "temperature": 25.0,
            "target": self.target,
            "min_extrude_temp": 170.0,
        }


class Heaters:
    def __init__(self):
        self.calls = []
        self.heaters = {}

    def lookup_heater(self, name):
        return self.heaters[name]

    def set_temperature(self, heater, target, wait=False):
        self.calls.append((heater, target, wait))
        heater.target = target


class Template:
    def __init__(self):
        self.contexts = []

    def run_gcode_from_command(self, context=None):
        self.contexts.append(context)


class Toolhead:
    def get_status(self, _eventtime):
        return {"homed_axes": "xyz"}

    def get_position(self):
        return [0.0, 0.0, 0.0, 0.0]


class SaveVariables:
    def __init__(self, variables):
        self.variables = variables

    def get_status(self, _eventtime):
        return {"variables": self.variables}


class FacadeTool:
    def __init__(self, tool_id, group, pickup, dropoff):
        self.name = tool_id
        self.toolgroup = group
        self.extruder = "extruder" if tool_id == 0 else "extruder%d" % tool_id
        self.fan = "part_fan_%d" % tool_id
        self.zone = [tool_id * 10, 200, 0]
        self.park = [tool_id * 10, 240, 0]
        self.offset = [0.0, 0.0, 0.0]
        self.config_offset = list(self.offset)
        self.meltzonelength = 18.0
        self.retract_length = 0.5 + tool_id
        self.retract_speed = 30.0
        self.unretract_extra_length = 0.0
        self.unretract_speed = 30.0
        self.zhop = 0.0
        self.pressure_advance = 0.01 * tool_id
        self.pressure_advance_smooth_time = 0.04
        self.heater_active_temp = 220.0
        self.heater_standby_temp = 150.0
        self.idle_to_standby_time = 30.0
        self.idle_to_powerdown_time = 600.0
        self.shaper_freq_x = 0.0
        self.shaper_freq_y = 0.0
        self.shaper_type_x = "mzv"
        self.shaper_type_y = "mzv"
        self.shaper_damping_ratio_x = 0.1
        self.shaper_damping_ratio_y = 0.1
        self.heater_state = 0
        self.pickup_gcode_template = pickup
        self.dropoff_gcode_template = dropoff

    def get_status(self, _eventtime=None):
        return {"name": self.name, "zone": self.zone, "park": self.park}


class Printer:
    def __init__(self, tools, variables):
        self.reactor = Reactor()
        self.gcode = GCode()
        self.heaters = Heaters()
        self.toolhead = Toolhead()
        self.objects = {
            "gcode": self.gcode,
            "save_variables": SaveVariables(variables),
            "firmware_retraction": SimpleNamespace(is_retracted=False),
            "toolhead": self.toolhead,
            "heaters": self.heaters,
            "gcode_macro": SimpleNamespace(create_template_context=lambda: {"printer": "status"}),
        }
        self.tool_items = []
        for tool in tools:
            heater = Heater()
            self.heaters.heaters[tool.extruder] = heater
            configured_heater = getattr(tool, "heater", tool.extruder)
            self.heaters.heaters[configured_heater] = heater
            self.objects[tool.extruder] = SimpleNamespace(get_heater=lambda heater=heater: heater)
            self.objects["tool %d" % tool.name] = tool
            self.tool_items.append(("tool %d" % tool.name, tool))
        self.group_items = [("toolgroup 0", tools[0].toolgroup)]
        self.shutdown_reasons = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=...):
        if name in self.objects:
            return self.objects[name]
        if default is not ...:
            return default
        raise KeyError(name)

    def lookup_objects(self, module):
        return self.tool_items if module == "tool" else self.group_items

    def is_shutdown(self):
        return False

    def command_error(self, message):
        return RuntimeError(message)

    def invoke_shutdown(self, reason):
        self.shutdown_reasons.append(reason)


class Facade:
    def __init__(self):
        self.runtime = None

    def get_status(self):
        current = -1 if self.runtime is None else self.runtime.toolchange.state.legacy_tool_current()
        return {"tool_current": str(current)}


class GCommand:
    def __init__(self, **parameters):
        self.parameters = parameters
        self.responses = []

    def get(self, name, default=...):
        if name in self.parameters:
            return str(self.parameters[name])
        if default is ...:
            raise RuntimeError("missing %s" % name)
        return default

    def get_int(self, name, default=..., **_limits):
        value = self.get(name, default)
        return value if value is None or value is ... else int(value)

    def get_float(self, name, default=..., **_limits):
        value = self.get(name, default)
        return value if value is None or value is ... else float(value)

    def respond_info(self, message):
        self.responses.append(message)

    def error(self, message):
        return RuntimeError(message)


def make_runtime(variables=None, *, status_commands=None, explicit_heater=None):
    group = SimpleNamespace(name=0)
    pickup = [Template(), Template()]
    dropoff = [Template(), Template()]
    tools = [
        FacadeTool(index, group, pickup[index], dropoff[index])
        for index in range(2)
    ]
    if explicit_heater is not None:
        tools[1].heater = explicit_heater
    printer = Printer(tools, {"tool_current": -1, **(variables or {})})
    if status_commands is not None:
        printer.gcode.status_commands = status_commands
    facade = Facade()
    runtime = compose_runtime(printer, facade)
    facade.runtime = runtime
    return runtime, facade, printer, tools, pickup, dropoff


def test_composition_loads_existing_state_and_profiles_without_heating_or_motion():
    runtime, _facade, printer, _tools, pickup, dropoff = make_runtime(
        {
            "ktcc_tool_retract_1": {
                "retract_length": 2.5,
                "retract_speed": 35.0,
                "unretract_extra_length": 0.1,
                "unretract_speed": 32.0,
            }
        }
    )

    assert runtime.toolchange.state.mounted.kind is MountedKind.NONE
    assert runtime.profiles.effective(1).profile.print_retraction.retract_length == 2.5
    assert printer.heaters.calls == []
    assert pickup[0].contexts == pickup[1].contexts == []
    assert dropoff[0].contexts == dropoff[1].contexts == []
    assert not any("ktcc_state_v2" in command for command in printer.gcode.commands)


@pytest.mark.parametrize("current", [9, "not-an-int"])
def test_invalid_or_removed_current_tool_enters_recovery_without_v2_write(current):
    _runtime, _facade, printer, _tools, _pickup, _dropoff = make_runtime(
        {"tool_current": current}
    )

    assert _runtime.toolchange.state.mode.value == "RECOVERY_REQUIRED"
    assert not any("ktcc_state_v2" in command for command in printer.gcode.commands)


def test_composition_binds_explicit_heater_instead_of_tool_extruder():
    runtime, _facade, printer, _tools, _pickup, _dropoff = make_runtime(
        explicit_heater="heater_generic/tool1"
    )

    runtime.thermal.set_temperature(1, active_target=205, mode=ThermalMode.ACTIVE)

    assert printer.heaters.calls[-1][0] is printer.heaters.heaters[
        "heater_generic/tool1"
    ]


def test_composition_uses_renamed_native_retraction_command_to_avoid_macro_recursion():
    runtime, _facade, printer, _tools, _pickup, _dropoff = make_runtime(
        status_commands={"SET_RETRACTION": {}, "SET_RETRACTION_ORIG": {}}
    )

    runtime.toolchange.select(0)

    assert any(
        command.startswith("SET_RETRACTION_ORIG ")
        for command in printer.gcode.commands
    )
    assert not any(
        command.startswith("SET_RETRACTION ")
        for command in printer.gcode.commands
    )


def test_composed_select_uses_transaction_and_legacy_jinja_alias():
    runtime, _facade, printer, _tools, pickup, _dropoff = make_runtime()

    state = runtime.toolchange.select(1)

    assert state.legacy_tool_current() == 1
    assert len(pickup[1].contexts) == 1
    context = pickup[1].contexts[0]
    assert context["myself"]["name"] == 1
    assert context["toollock"]["tool_current"] == "-2"
    assert context["tool"]["profile"]["print_retraction"]["retract_length"] == 1.5
    assert printer.shutdown_reasons == []


def test_vector_updates_preserve_absolute_over_adjust_precedence():
    assert vector_with_updates(
        Vector3(1, 2, 3), x=10, x_adjust=99, y_adjust=-0.5
    ) == Vector3(10, 1.5, 3)


def test_thin_toollock_facade_routes_thermal_and_profile_commands_to_services():
    runtime, _facade, printer, _tools, _pickup, _dropoff = make_runtime()
    facade = ToolLock.__new__(ToolLock)
    facade.runtime = runtime
    facade.printer = printer
    facade.reactor = printer.reactor
    facade.gcode = printer.gcode
    facade.global_offset = [0.0, 0.0, 0.0]
    facade.saved_fan_speed = 0.0
    facade.tool_current = "-1"
    facade.purge_on_toolchange = True
    facade.saved_position = None
    facade.restore_position_on_toolchange_type = 0
    facade.last_endstop_query = {}

    facade.cmd_SET_TOOL_TEMPERATURE(
        GCommand(TOOL=1, ACTV_TMP=235, STDB_TMP=165, CHNG_STATE=2)
    )
    facade.cmd_KTCC_SET_TOOL_RETRACTION(
        GCommand(TOOL=1, LENGTH=2.2, SPEED=40, EXTRA=0.1, PRIME_SPEED=35)
    )
    facade.cmd_KTCC_SET_TOOL_PRESSURE_ADVANCE(
        GCommand(TOOL=1, ADVANCE=0.055, SMOOTH_TIME=0.045)
    )

    assert runtime.thermal.snapshot(1).mode is ThermalMode.ACTIVE
    assert runtime.thermal.snapshot(1).active_target == 235
    profile = runtime.profiles.effective(1).profile
    assert profile.print_retraction.retract_length == 2.2
    assert profile.pressure_advance.pressure_advance == 0.055
    assert any("VARIABLE=ktcc_tool_retract_1" in command for command in printer.gcode.commands)
    assert any(command.startswith("SET_PRESSURE_ADVANCE EXTRUDER=extruder1") for command in printer.gcode.commands)


def test_explicit_preheat_facade_and_recovery_guard_share_authoritative_state():
    runtime, _facade, printer, _tools, _pickup, _dropoff = make_runtime()
    facade = ToolLock.__new__(ToolLock)
    facade.runtime = runtime
    facade.printer = printer
    facade.reactor = printer.reactor
    facade.gcode = printer.gcode
    facade.global_offset = [0.0, 0.0, 0.0]
    facade.saved_fan_speed = 0.0
    facade.tool_current = "-1"
    facade.purge_on_toolchange = True
    facade.saved_position = None
    facade.restore_position_on_toolchange_type = 0
    facade.last_endstop_query = {}

    facade.cmd_KTCC_PREHEAT_TOOL(GCommand(TOOL=0, MODE="ACTIVE", TEMP=215))
    assert runtime.thermal.snapshot(0).mode is ThermalMode.PREHEAT
    assert runtime.thermal.snapshot(0).active_target == 215
    facade.cmd_KTCC_CANCEL_PREHEAT(GCommand(TOOL=0))
    assert runtime.thermal.snapshot(0).mode is ThermalMode.OFF

    runtime.toolchange.declare_current_tool(-2)
    with pytest.raises(ToolChangeStateError, match="RECOVERY_REQUIRED"):
        facade.cmd_SET_TOOL_TEMPERATURE(
            GCommand(TOOL=0, ACTV_TMP=220, CHNG_STATE=2)
        )

    # The same compatibility command is allowed only while an adapter-owned
    # recovery action is synchronously executing.
    facade._action_depth = 1
    facade.cmd_SET_TOOL_TEMPERATURE(
        GCommand(TOOL=0, ACTV_TMP=205, CHNG_STATE=2)
    )
    facade._action_depth = 0
    assert runtime.thermal.snapshot(0).active_target == 205


def test_scoped_template_cleans_action_ownership_even_when_template_raises():
    facade = SimpleNamespace(_action_depth=0)

    class FailingTemplate:
        def run_gcode_from_command(self, context=None):
            assert facade._action_depth == 1
            raise RuntimeError("macro failed")

    with pytest.raises(RuntimeError, match="macro failed"):
        _ScopedTemplate(FailingTemplate(), facade).run_gcode_from_command({})

    assert facade._action_depth == 0
