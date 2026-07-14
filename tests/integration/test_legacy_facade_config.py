"""Load an actual-shaped four-head section through the thin config facades."""

from ktcc.klipper.composition import snapshot_facades
from ktcc.klipper.config import build_config_bridge
from tool import Tool
from toolgroup import ToolGroup


class ConfigError(Exception):
    pass


class Config:
    def __init__(self, printer, name, values, sections):
        self.printer = printer
        self.name = name
        self.values = values
        self.sections = sections
        self.accessed = set()

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def has_section(self, name):
        return name in self.sections

    def error(self, message):
        return ConfigError(message)

    def get(self, name, default=None):
        self.accessed.add(name)
        return self.values.get(name, default)

    def getint(self, name, default=None, **_limits):
        value = self.get(name, default)
        return None if value is None else int(value)

    def getfloat(self, name, default=None, **_limits):
        value = self.get(name, default)
        return None if value is None else float(value)

    def getboolean(self, name, default=None):
        value = self.get(name, default)
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)


class Template:
    def __init__(self, script):
        self.script = script


class GCodeMacro:
    def load_template(self, config, option, default=None):
        return Template(config.get(option, default))


class GCode:
    def __init__(self):
        self.commands = {}

    def register_command(self, name, handler, **_options):
        self.commands[name] = handler


class Printer:
    def __init__(self):
        self.gcode = GCode()
        self.macro = GCodeMacro()
        self.objects = {"gcode": self.gcode, "toollock": object()}

    def lookup_object(self, name):
        return self.objects[name]

    def load_object(self, _config, name):
        assert name == "gcode_macro"
        return self.macro


def test_deployed_shape_consumes_fields_and_resolves_nonzero_motion_profile():
    printer = Printer()
    sections = {"toolgroup 0", "tool 0"}
    group_values = {
        "idle_to_standby_time": 300,
        "idle_to_powerdown_time": 600,
        "pickup_gcode": "_TOOLCHANGE_PICKUP T={myself.name}",
        "dropoff_gcode": "_TOOLCHANGE_DROPOFF T={myself.name}",
    }
    group_config = Config(printer, "toolgroup 0", group_values, sections)
    group = ToolGroup(group_config)
    printer.objects["toolgroup 0"] = group

    tool_values = {
        "tool_group": 0,
        "extruder": "extruder",
        "fan": "part_fan_0",
        "meltzonelength": 25,
        "zone": "-12.0, 200.0, 0.0",
        "park": "-12.0, 239.5, 0.0",
        "offset": "0.0, -39.0, 10.0",
        "shaper_freq_x": 67.6,
        "shaper_type_x": "mzv",
        "shaper_damping_ratio_x": 0.067,
        "shaper_freq_y": 81.0,
        "shaper_type_y": "2hump_ei",
        "shaper_damping_ratio_y": 0.046,
    }
    tool_config = Config(printer, "tool 0", tool_values, sections)
    tool = Tool(tool_config)

    groups, tools = snapshot_facades([group], [tool])
    bridge = build_config_bridge(groups, tools)
    profile = bridge.registry[0].configured_profile

    assert set(group_values) <= group_config.accessed
    assert set(tool_values) <= tool_config.accessed
    assert profile.motion is not None
    assert profile.motion.shaper_freq_y == 81.0
    assert profile.motion.shaper_type_y == "2hump_ei"
    assert bridge.registry[0].configured_profile.thermal_policy.idle_to_standby_time == 300
