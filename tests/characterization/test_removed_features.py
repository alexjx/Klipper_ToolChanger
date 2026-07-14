from pathlib import Path

import pytest

from tool import Tool
from toolgroup import ToolGroup


ROOT = Path(__file__).resolve().parents[2]


class ConfigError(Exception):
    pass


class FakeConfig:
    def __init__(self, name, values=None):
        self._name = name
        self._values = values or {}

    def get_name(self):
        return self._name

    def get_printer(self):
        return object()

    def error(self, message):
        return ConfigError(message)

    def get(self, option, default=None):
        return self._values.get(option, default)

    def getboolean(self, option, default=None):
        return self._values.get(option, default)

    def getint(self, option, default=None):
        return self._values.get(option, default)

    def getfloat(self, option, default=None, **_kwargs):
        return self._values.get(option, default)


@pytest.mark.parametrize(
    "values, message",
    [
        ({"is_virtual": True}, "virtual tools"),
        ({"physical_parent": 0}, "physical_parent"),
        ({"physical_parent_id": 0}, "physical_parent_id"),
        ({"remap": "0:1"}, "remap"),
        ({"tool_remap": {0: 1}}, "tool_remap"),
        ({"virtual_toolload_gcode": "LOAD_FILAMENT"}, "virtual_toolload_gcode"),
        ({"virtual_toolunload_gcode": "UNLOAD_FILAMENT"}, "virtual_toolunload_gcode"),
        ({"requires_pickup_for_virtual_load": True}, "requires_pickup_for_virtual_load"),
    ],
)
def test_tool_rejects_removed_virtual_configuration(values, message):
    with pytest.raises(ConfigError, match=message):
        Tool._reject_removed_features(FakeConfig("tool 0", values))


def test_inert_legacy_markers_are_accepted():
    config = FakeConfig(
        "tool 0",
        {
            "is_virtual": False,
            "physical_parent": -1,
            "virtual_toolload_gcode": "",
            "virtual_toolunload_gcode": "",
            "requires_pickup_for_virtual_load": False,
            "requires_pickup_for_virtual_unload": False,
            "unload_virtual_at_dropoff": False,
        },
    )

    Tool._reject_removed_features(config)


def test_toolgroup_remains_a_shared_physical_configuration_source():
    group = ToolGroup(
        FakeConfig(
            "toolgroup 0",
            {
                "pickup_gcode": "PICKUP_SHARED",
                "dropoff_gcode": "DROPOFF_SHARED",
            },
        )
    )

    assert group.get_config("pickup_gcode") == "PICKUP_SHARED"
    assert group.get_config("dropoff_gcode") == "DROPOFF_SHARED"
    assert "is_virtual" not in group.get_status()
    assert "physical_parent_id" not in group.get_status()


def test_removed_remap_commands_and_virtual_methods_are_absent_from_runtime():
    toollock_source = (ROOT / "toollock.py").read_text()

    assert "'KTCC_REMAP_TOOL'" not in toollock_source
    assert "'KTCC_DISPLAY_TOOL_MAP'" not in toollock_source
    assert "ktcc_state_tool_remap" not in toollock_source
    assert not hasattr(Tool, "LoadVirtual")
    assert not hasattr(Tool, "UnloadVirtual")
    assert not hasattr(Tool, "set_virtual_loaded")


def test_physical_selection_delegates_to_authoritative_transaction_service():
    effects = []

    class ToolChange:
        state = type(
            "State", (),
            {"mounted": type("Mounted", (), {"kind": type("Kind", (), {"value": "NONE"})(), "tool_id": None})()},
        )()

        def select(self, tool_id):
            effects.append("select:%s" % tool_id)

    class Registry:
        def __getitem__(self, tool_id):
            return type("Spec", (), {"fan": None})()

    class ToolLock:
        runtime = type(
            "Runtime", (),
            {"toolchange": ToolChange(), "bridge": type("Bridge", (), {"registry": Registry()})(),
             "machine": object()},
        )()
        saved_fan_speed = 0

        def _sync_legacy_views(self):
            effects.append("sync")

        def _flush_statistics(self):
            effects.append("flush_statistics")

    tool = Tool()
    tool.name = 2
    tool.toollock = ToolLock()

    tool.select_tool_actual()

    assert effects == ["select:2", "sync", "flush_statistics"]
