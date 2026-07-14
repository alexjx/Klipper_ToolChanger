from __future__ import annotations

import ast
import re
from pathlib import Path

from conftest import ROOT


PRESERVED_TOOLLOCK_COMMANDS = {
    "SAVE_CURRENT_TOOL",
    "TOOL_LOCK",
    "TOOL_UNLOCK",
    "KTCC_TOOL_DROPOFF_ALL",
    "SET_AND_SAVE_FAN_SPEED",
    "TEMPERATURE_WAIT_WITH_TOLERANCE",
    "SET_TOOL_TEMPERATURE",
    "SET_GLOBAL_OFFSET",
    "SET_TOOL_OFFSET",
    "GET_GLOBAL_OFFSET",
    "GET_TOOL_OFFSET",
    "SET_PURGE_ON_TOOLCHANGE",
    "SAVE_POSITION",
    "SAVE_CURRENT_POSITION",
    "RESTORE_POSITION",
    "KTCC_SET_GCODE_OFFSET_FOR_CURRENT_TOOL",
    "KTCC_ENDSTOP_QUERY",
    "KTCC_SET_ALL_TOOL_HEATERS_OFF",
    "KTCC_RESUME_ALL_TOOL_HEATERS",
    "KTCC_SET_TOOL_RETRACTION",
    "KTCC_GET_TOOL_RETRACTION",
    "KTCC_SAVE_TOOL_OFFSET",
    "KTCC_SET_TOOL_PRESSURE_ADVANCE",
    "KTCC_GET_TOOL_PRESSURE_ADVANCE",
    "KTCC_SAVE_TOOL_PRESSURE_ADVANCE",
    "KTCC_APPLY_TOOL_RETRACTION",
}

REMOVED_OBSERVABILITY_COMMANDS = {
    "KTCC_LOG_TRACE",
    "KTCC_LOG_DEBUG",
    "KTCC_LOG_INFO",
    "KTCC_LOG_ALWAYS",
    "KTCC_SET_LOG_LEVEL",
}

STATISTICS_COMMANDS = {
    "KTCC_DUMP_STATS",
    "KTCC_RESET_STATS",
    "KTCC_INIT_PRINT_STATS",
    "KTCC_DUMP_PRINT_STATS",
}

M109_SOURCE = (ROOT / "klipper_macros" / "M109.cfg").read_text(encoding="utf-8")


def _string_list_assigned_to(source: str, variable: str) -> set[str]:
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == variable for target in node.targets):
            continue
        if isinstance(node.value, (ast.List, ast.Tuple)):
            return {
                item.value
                for item in node.value.elts
                if isinstance(item, ast.Constant) and isinstance(item.value, str)
            }
    raise AssertionError(f"could not find literal {variable} command list")


def test_preserved_toollock_commands_are_registered():
    source = (ROOT / "toollock.py").read_text(encoding="utf-8")
    assert PRESERVED_TOOLLOCK_COMMANDS <= _string_list_assigned_to(source, "handlers")
    assert STATISTICS_COMMANDS <= _string_list_assigned_to(source, "handlers")


def test_four_tool_selection_commands_remain_part_of_fixture(fixture_root: Path):
    macros = (fixture_root / "toolchange_macros.cfg").read_text(encoding="utf-8")
    for tool_id in range(4):
        assert f"[gcode_macro T{tool_id}]" in macros
        assert f"KTCC_T{tool_id} {{rawparams}}" in macros


def test_slicer_facing_macro_names_remain_part_of_fixture(fixture_root: Path):
    macros = (fixture_root / "compatibility_macros.cfg").read_text(encoding="utf-8")
    assert set(re.findall(r"^\[gcode_macro ([^]]+)]$", macros, flags=re.MULTILINE)) == {
        "M104",
        "M109",
        "M116",
        "M568",
        "M106",
        "M107",
    }


def test_legacy_persistence_key_families_are_frozen(variables_config):
    keys = set(variables_config["Variables"])
    expected = {"tool_current", "ktcc_global_offset"}
    for tool_id in range(4):
        expected.update(
            {
                f"ktcc_tool_offset_{tool_id}",
                f"ktcc_tool_retract_{tool_id}",
                f"ktcc_tool_pa_info_{tool_id}",
            }
        )
        expected.add(f"ktcc_statistics_tool{tool_id}")
    expected.add("ktcc_statistics_swaps")
    assert keys == expected


def test_stable_jinja_status_fields_are_exercised(fixture_root: Path):
    combined = "\n".join(path.read_text(encoding="utf-8") for path in fixture_root.glob("*.cfg"))
    assert "{myself.name}" in combined
    assert "printer.toollock.tool_current" in combined


def test_custom_logging_surface_is_absent_but_statistics_are_independent():
    assert not (ROOT / "ktcclog.py").exists()
    runtime_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in ROOT.glob("*.py")
        if path.name != "ktcclog.py"
    ).lower()
    assert "ktcclog" not in runtime_source
    assert not any(command.lower() in runtime_source for command in REMOVED_OBSERVABILITY_COMMANDS)
    assert all(command.lower() in runtime_source for command in STATISTICS_COMMANDS)


def test_legacy_runtime_uses_standard_logging():
    for filename in ("tool.py", "toollock.py", "alignment.py"):
        source = (ROOT / filename).read_text(encoding="utf-8")
        assert "logging.getLogger('ktcc." in source
        assert ".trace(" not in source
        assert ".always(" not in source


def test_repo_m109_routes_set_active_and_heat_wait_as_one_service_command():
    assert "KTCC_TOOL_M109{tool} TEMP={params.S} TOLERANCE={tolerance}" in M109_SOURCE
    assert "S=0 turns the requested tool OFF and intentionally does not wait" in M109_SOURCE
    assert M109_SOURCE.index("KTCC_TOOL_M109") < M109_SOURCE.index(
        "TEMPERATURE_WAIT_WITH_TOLERANCE"
    )


def test_g10_is_reserved_for_firmware_retraction_not_temperature_aliasing():
    macro_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "klipper_macros").glob("*.cfg")
    )
    slicer_source = (ROOT / "config" / "SuperSlicer_Custom_Gcode.md").read_text(
        encoding="utf-8"
    )
    assert "[gcode_macro G10]" not in macro_source
    assert re.search(r"^G10\s+P", slicer_source, flags=re.MULTILINE) is None


def test_installer_deploys_both_entry_modules_and_ktcc_package():
    source = (ROOT / "install.sh").read_text(encoding="utf-8")
    for module in ("alignment.py", "tool.py", "toolgroup.py", "toollock.py"):
        assert module in source
    assert '"${SRCDIR}/ktcc"' in source
    assert 'klippy/extras/ktcc"' in source
