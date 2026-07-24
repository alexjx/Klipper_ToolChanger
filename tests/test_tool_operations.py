import unittest

from toollock import ToolLock
from tool import Tool


class FakeLog:
    def __init__(self):
        self.events = []

    def info(self, message):
        self.events.append(("info", message))

    def __getattr__(self, name):
        return lambda *args, **kwargs: self.events.append((name, args))


class FakeGcode:
    def __init__(self):
        self.commands = []

    def run_script_from_command(self, command):
        self.commands.append(command)


class FakePrinter:
    class CommandError(Exception):
        pass

    def __init__(self, homed=True, parent=None):
        self.homed = homed
        self.parent = parent

    def command_error(self, message):
        return self.CommandError(message)

    def lookup_object(self, name):
        if name == "tool 0":
            return self.parent
        if name == "toolhead":
            return self
        raise KeyError(name)

    def wait_moves(self):
        pass


class FakeTemplate:
    def __init__(self, events, failure=None):
        self.events = events
        self.failure = failure

    def create_template_context(self):
        self.events.append("context")
        return {}

    def run_gcode_from_command(self, context):
        self.events.append("template")
        if self.failure is not None:
            raise self.failure


def make_toollock(tool_current="-1", mode=ToolLock.CHANGER_IDLE):
    toollock = ToolLock.__new__(ToolLock)
    toollock.log = FakeLog()
    toollock.tool_current = str(tool_current)
    toollock.changer_mode = mode
    toollock._changer_depth = 0
    toollock._changer_previous_mode = mode
    toollock._changer_operation = None
    toollock._changer_risk_started = False
    toollock._changer_failed = False
    return toollock


def make_tool(toollock, *, virtual=False, homed=True, events=None,
              template_failure=None, parent=None):
    events = [] if events is None else events
    tool = Tool.__new__(Tool)
    tool.name = 1
    tool.is_virtual = virtual
    tool.physical_parent_id = 0
    tool.extruder = None
    tool.fan = None
    tool.lazy_home_when_parking = 0
    tool.unload_virtual_at_dropoff = False
    tool.shaper_freq_x = 0
    tool.shaper_freq_y = 0
    tool.shaper_type_x = "mzv"
    tool.shaper_type_y = "mzv"
    tool.shaper_damping_ratio_x = 0.1
    tool.shaper_damping_ratio_y = 0.1
    tool.toollock = toollock
    tool.log = FakeLog()
    tool.gcode = FakeGcode()
    tool.printer = FakePrinter(homed=homed, parent=parent)
    tool.get_status = lambda eventtime=None: {}
    tool.apply_retract_options = lambda: None
    tool.pickup_gcode_template = FakeTemplate(events, template_failure)
    tool.dropoff_gcode_template = FakeTemplate(events, template_failure)
    tool.virtual_toolload_gcode_template = FakeTemplate(events, template_failure)
    tool.virtual_toolunload_gcode_template = FakeTemplate(events, template_failure)
    toollock.PrinterIsHomedForToolchange = lambda: homed
    toollock.get_status = lambda eventtime=None: {
        "tool_current": toollock.tool_current,
        "saved_fan_speed": 0,
    }
    toollock.SaveCurrentTool = lambda value: setattr(
        toollock, "tool_current", str(value))
    return tool


class ToolOperationTests(unittest.TestCase):
    def test_select_noop_is_transactional_and_idle(self):
        toollock = make_toollock(tool_current="1")
        tool = make_tool(toollock)

        tool.select_tool_actual()

        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)
        self.assertEqual(0, toollock._changer_depth)

    def test_select_nested_dropoff_and_pickup_stay_changing(self):
        toollock = make_toollock(tool_current="0")
        observed_modes = []

        class CurrentTool:
            def get_status(self):
                return {"physical_parent_id": -1}

            def Dropoff(self):
                with toollock.changer_operation("dropoff"):
                    toollock.mark_changer_risk()
                    observed_modes.append(toollock.changer_mode)
                    toollock.SaveCurrentTool(ToolLock.TOOL_UNLOCKED)

        class ModeTemplate(FakeTemplate):
            def run_gcode_from_command(self, context):
                observed_modes.append(toollock.changer_mode)
                super().run_gcode_from_command(context)

        target = make_tool(toollock, parent=CurrentTool())
        target.physical_parent_id = -1
        target.pickup_gcode_template = ModeTemplate([])

        target.select_tool_actual()

        self.assertEqual(
            [ToolLock.CHANGER_CHANGING, ToolLock.CHANGER_CHANGING],
            observed_modes)
        self.assertEqual("1", toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_select_unknown_tool_rejection_preserves_recovery(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNKNOWN,
            mode=ToolLock.CHANGER_RECOVERY_REQUIRED)
        tool = make_tool(toollock)

        with self.assertRaises(FakePrinter.CommandError):
            tool.select_tool_actual()

        self.assertEqual(
            ToolLock.CHANGER_RECOVERY_REQUIRED, toollock.changer_mode)
        self.assertEqual(0, toollock._changer_depth)

    def test_pickup_unhomed_rejects_before_tracking_or_risk(self):
        toollock = make_toollock()
        events = []
        tool = make_tool(toollock, homed=False, events=events)

        with self.assertRaises(FakePrinter.CommandError):
            tool.Pickup()

        self.assertEqual([], events)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_pickup_marks_risk_before_template(self):
        toollock = make_toollock()
        events = []
        original_mark = toollock.mark_changer_risk
        toollock.mark_changer_risk = lambda: (events.append("risk"), original_mark())[1]
        tool = make_tool(toollock, events=events)

        tool.Pickup()

        self.assertLess(events.index("risk"), events.index("template"))
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_dropoff_unhomed_raises_before_tracking_or_mutation(self):
        toollock = make_toollock(tool_current="1")
        tool = make_tool(toollock, homed=False)

        with self.assertRaises(FakePrinter.CommandError):
            tool.Dropoff()

        self.assertEqual("1", toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_dropoff_template_failure_requires_recovery(self):
        toollock = make_toollock(tool_current="1")
        failure = RuntimeError("dropoff failed")
        tool = make_tool(toollock, template_failure=failure)

        with self.assertRaisesRegex(Exception, "dropoff failed"):
            tool.Dropoff()

        self.assertEqual("1", toollock.tool_current)
        self.assertEqual(
            ToolLock.CHANGER_RECOVERY_REQUIRED, toollock.changer_mode)

    def test_virtual_operations_are_nested_safe(self):
        toollock = make_toollock(tool_current="0")
        parent = type("Parent", (), {"set_virtual_loaded": lambda self, value: None})()
        events = []
        tool = make_tool(toollock, virtual=True, events=events, parent=parent)

        with toollock.changer_operation("select"):
            tool.LoadVirtual()
            self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
            tool.UnloadVirtual()
            self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)

        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)
        self.assertEqual(2, events.count("template"))


if __name__ == "__main__":
    unittest.main()
