"""Optional contract test against the user's read-only local Klipper checkout."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from ktcc.klipper.runtime import KlipperExtrusionProfilePort
from ktcc.profiles.models import FirmwareRetractionProfile


KLIPPER_RETRACTION = (
    Path.home() / "klipper" / "klippy" / "extras" / "firmware_retraction.py"
)


class Command:
    def __init__(self, parameters):
        self.parameters = parameters

    def get_float(self, name, default, **_limits):
        return float(self.parameters.get(name, default))


class GCode:
    def __init__(self):
        self.handlers = {}
        self.scripts = []

    def register_command(self, name, handler, **_options):
        self.handlers[name] = handler

    def run_script_from_command(self, script):
        self.scripts.append(script)
        line = script.strip()
        if "\n" in line:
            return
        parts = line.split()
        handler = self.handlers.get(parts[0])
        if handler is None:
            return
        parameters = dict(part.split("=", 1) for part in parts[1:])
        handler(Command(parameters))


class Printer:
    def __init__(self, gcode):
        self.gcode = gcode

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode


class Config:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def getfloat(self, _name, default, **_limits):
        return default


@pytest.mark.skipif(
    not KLIPPER_RETRACTION.exists(),
    reason="optional local ~/klipper source contract is unavailable",
)
def test_adapter_matches_local_firmware_retraction_executor_contract():
    spec = importlib.util.spec_from_file_location(
        "ktcc_local_firmware_retraction", KLIPPER_RETRACTION
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    gcode = GCode()
    executor = module.FirmwareRetraction(Config(Printer(gcode)))
    adapter = KlipperExtrusionProfilePort(gcode, executor)

    adapter.apply_firmware_retraction(
        FirmwareRetractionProfile(0.9, 35.0, 0.1, 32.0)
    )
    assert executor.get_status(0) == {
        "retract_length": 0.9,
        "retract_speed": 35.0,
        "unretract_extra_length": 0.1,
        "unretract_speed": 32.0,
    }

    gcode.handlers["G10"](Command({}))
    assert adapter.observe_retracted() is True
    adapter.unretract()
    assert adapter.observe_retracted() is False
