from __future__ import annotations

import ast
import configparser
from pathlib import Path

from conftest import ROOT


TOOL_SECTIONS = tuple(f"tool {tool_id}" for tool_id in range(4))
PROFILE_FIELDS = (
    "extruder",
    "fan",
    "offset",
    "meltzonelength",
    "idle_to_standby_time",
    "idle_to_powerdown_time",
    "shaper_freq_x",
    "shaper_freq_y",
)


def test_fixture_is_exactly_four_independent_physical_tools(tools_config):
    assert tuple(section for section in tools_config if section.startswith("tool ")) == TOOL_SECTIONS
    assert tools_config.sections().count("toolgroup 0") == 1
    assert tools_config["toolgroup 0"].getboolean("is_virtual") is False

    for section in TOOL_SECTIONS:
        assert tools_config[section].get("tool_group") == "0"
        assert "physical_parent" not in tools_config[section]
        assert "is_virtual" not in tools_config[section]


def test_every_tool_has_distinct_resources_and_profile_values(tools_config):
    for field in PROFILE_FIELDS:
        values = [tools_config[section][field] for section in TOOL_SECTIONS]
        assert len(set(values)) == 4, f"{field} must expose cross-tool leakage"


def test_persisted_tuning_is_distinct_for_every_tool(variables_config):
    variables = variables_config["Variables"]
    for prefix in ("ktcc_tool_offset_", "ktcc_tool_retract_", "ktcc_tool_pa_info_"):
        parsed = [ast.literal_eval(variables[f"{prefix}{tool_id}"]) for tool_id in range(4)]
        canonical = [
            tuple(sorted(value.items())) if isinstance(value, dict) else tuple(value)
            for value in parsed
        ]
        assert len(set(canonical)) == 4

    retraction = ast.literal_eval(variables["ktcc_tool_retract_0"])
    pressure_advance = ast.literal_eval(variables["ktcc_tool_pa_info_0"])
    assert set(retraction) == {
        "retract_length",
        "retract_speed",
        "unretract_extra_length",
        "unretract_speed",
    }
    assert set(pressure_advance) == {"pressure_advance", "smooth_time"}

    assert "ktcc_global_offset" in variables
    assert variables.getint("tool_current") == -1


def test_fixture_contains_no_machine_identity_or_real_actions(fixture_root: Path):
    combined = "\n".join(path.read_text(encoding="utf-8") for path in fixture_root.glob("*.cfg"))
    lowered = combined.lower()

    assert "synthetic" in lowered
    assert "manual_stepper" not in lowered
    assert "[mcu" not in lowered
    assert "serial:" not in lowered
    assert "pin:" not in lowered
    assert "/dev/" not in lowered
    assert "/home/" not in lowered
    assert "~/" not in lowered
    assert " g0 " not in f" {lowered} "
    assert " g1 " not in f" {lowered} "


def test_fixture_omits_removed_logging_and_virtual_tool_surfaces(fixture_root: Path):
    combined = "\n".join(path.read_text(encoding="utf-8") for path in fixture_root.glob("*.cfg"))
    lowered = combined.lower()

    for removed in (
        "[ktcclog]",
        "ktcc_log_",
        "ktcc_set_log_level",
        "ktcc_remap_tool",
        "physical_parent",
        "virtual_tool",
    ):
        assert removed not in lowered


def test_fixture_uses_repo_relative_includes(fixture_root: Path):
    printer = (fixture_root / "printer.cfg").read_text(encoding="utf-8")
    assert "[include toolchange_macros.cfg]" in printer
    assert "[include compatibility_macros.cfg]" in printer
    assert "[include tools.cfg]" in printer
    assert "filename: tests/fixtures/four_physical/variables.cfg" in printer


def test_recording_actions_expose_stable_template_context(tools_config):
    group = tools_config["toolgroup 0"]
    assert "{myself.name}" in group["pickup_gcode"]
    assert "{myself.name}" in group["dropoff_gcode"]
    assert "EVENT=dropoff" in group["dropoff_gcode"]
    assert group["pickup_gcode"].index("EVENT=pickup") < group["pickup_gcode"].index("EVENT=wait")
    assert group["pickup_gcode"].index("EVENT=wait") < group["pickup_gcode"].index("EVENT=offset")


def test_fixture_has_four_distinct_non_heating_thermal_profiles(fixture_root: Path):
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(fixture_root / "toolchange_macros.cfg", encoding="utf-8")
    profile = parser["gcode_macro KTCC_SYNTHETIC_THERMAL_PROFILES"]

    active = [profile.getfloat(f"variable_active_{tool_id}") for tool_id in range(4)]
    standby = [profile.getfloat(f"variable_standby_{tool_id}") for tool_id in range(4)]
    assert len(set(active)) == 4
    assert len(set(standby)) == 4
    assert all(standby_temp < active_temp for standby_temp, active_temp in zip(standby, active))
    assert "SET_HEATER_TEMPERATURE" not in profile["gcode"]
    assert "SET_TOOL_TEMPERATURE" not in profile["gcode"]


def test_selection_effect_contract_is_semantic_and_ordered(fixture_root: Path):
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(fixture_root / "toolchange_macros.cfg", encoding="utf-8")
    body = parser["gcode_macro KTCC_EXPECTED_SELECT_TRACE"]["gcode"]
    events = [
        line.split("EVENT=", 1)[1].split(maxsplit=1)[0]
        for line in body.splitlines()
        if "EVENT=" in line
    ]
    assert events == [
        "preheat",
        "dropoff",
        "activate",
        "apply_retraction",
        "apply_pa",
        "pickup",
        "wait",
        "offset",
        "commit",
    ]
