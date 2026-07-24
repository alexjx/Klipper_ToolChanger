import unittest
from unittest import mock

from toollock import ToolLock


class FakeLog:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def __getattr__(self, name):
        return lambda *args, **kwargs: None


class FakeTemplate:
    def __init__(self, action=None):
        self.action = action
        self.calls = 0

    def run_gcode_from_command(self):
        self.calls += 1
        if self.action is not None:
            self.action()


def make_toollock(tool_current="-1", mode=None):
    toollock = ToolLock.__new__(ToolLock)
    toollock.printer = mock.Mock()
    toollock.log = FakeLog()
    toollock.tool_current = str(tool_current)
    toollock.changer_mode = (
        ToolLock.CHANGER_SYNCHRONIZING if mode is None else mode)
    toollock._changer_depth = 0
    toollock._changer_previous_mode = toollock.changer_mode
    toollock._changer_operation = None
    toollock._changer_risk_started = False
    toollock._changer_failed = False
    toollock._changer_failure = None
    return toollock


class ChangerStateTests(unittest.TestCase):
    def test_constructor_starts_in_synchronizing_mode(self):
        config = mock.Mock()
        printer = config.get_printer.return_value
        printer.get_reactor.return_value = mock.Mock()
        printer.lookup_object.return_value = mock.Mock()
        gcode_macro = printer.load_object.return_value
        gcode_macro.load_template.return_value = mock.Mock()
        config.getboolean.side_effect = lambda name, default: default

        toollock = ToolLock(config)

        self.assertEqual(
            ToolLock.CHANGER_SYNCHRONIZING, toollock.changer_mode)

    def test_get_status_exposes_changer_mode(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        toollock.global_offset = [0, 0, 0]
        toollock.saved_fan_speed = 0
        toollock.purge_on_toolchange = True
        toollock.restore_position_on_toolchange_type = 0
        toollock.saved_position = None
        toollock.last_endstop_query = {}
        self.assertEqual(
            ToolLock.CHANGER_IDLE,
            toollock.get_status()["changer_mode"])

    def test_transition_rejects_values_outside_public_enum(self):
        toollock = make_toollock()
        for mode in ToolLock.CHANGER_MODES:
            toollock._set_changer_mode(mode)
            self.assertEqual(mode, toollock.changer_mode)
        with self.assertRaises(ValueError):
            toollock._set_changer_mode("BROKEN")

    def test_transition_logs_once_only_when_value_changes(self):
        toollock = make_toollock()

        toollock._set_changer_mode(ToolLock.CHANGER_IDLE)
        toollock._set_changer_mode(ToolLock.CHANGER_IDLE)

        self.assertEqual(1, len(toollock.log.messages))

    def test_normal_outer_transaction(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with toollock.changer_operation("select"):
            self.assertEqual(
                ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_nested_transaction_has_no_intermediate_idle(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with toollock.changer_operation("select"):
            with toollock.changer_operation("pickup"):
                self.assertEqual(
                    ToolLock.CHANGER_CHANGING, toollock.changer_mode)
            self.assertEqual(
                ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_depth_is_restored_after_exception(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with self.assertRaisesRegex(RuntimeError, "failed"):
            with toollock.changer_operation("select"):
                with toollock.changer_operation("pickup"):
                    raise RuntimeError("failed")
        self.assertEqual(0, toollock._changer_depth)

    def test_pre_risk_failure_restores_entry_mode(self):
        for entry_mode in (
                ToolLock.CHANGER_IDLE,
                ToolLock.CHANGER_SYNCHRONIZING):
            toollock = make_toollock(mode=entry_mode)
            with self.assertRaises(ValueError):
                with toollock.changer_operation("select"):
                    raise ValueError("validation")
            self.assertEqual(entry_mode, toollock.changer_mode)

    def test_post_risk_failure_shuts_down_klippy(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with self.assertRaises(RuntimeError):
            with toollock.changer_operation("pickup"):
                toollock.mark_changer_risk()
                raise RuntimeError("template failed")
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        toollock.printer.invoke_shutdown.assert_called_once()

    def test_unknown_tool_on_success_ends_idle(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNKNOWN,
            mode=ToolLock.CHANGER_IDLE)
        with toollock.changer_operation("lock"):
            pass
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_successful_standalone_lock_ends_idle_with_unknown_tool(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNLOCKED,
            mode=ToolLock.CHANGER_IDLE)
        toollock.tool_lock_gcode_template = FakeTemplate()
        toollock.SaveCurrentTool = lambda tool: setattr(
            toollock, "tool_current", str(tool))

        toollock.ToolLock()

        self.assertEqual(str(ToolLock.TOOL_UNKNOWN), toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_failed_lock_after_risk_shuts_down_klippy(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNLOCKED,
            mode=ToolLock.CHANGER_IDLE)
        toollock.tool_lock_gcode_template = FakeTemplate(
            lambda: (_ for _ in ()).throw(RuntimeError("lock failed")))
        toollock.SaveCurrentTool = lambda tool: setattr(
            toollock, "tool_current", str(tool))

        with self.assertRaisesRegex(RuntimeError, "lock failed"):
            toollock.ToolLock()

        self.assertEqual(str(ToolLock.TOOL_UNLOCKED), toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        toollock.printer.invoke_shutdown.assert_called_once()

    def test_noop_lock_preserves_idle_without_running_template(self):
        toollock = make_toollock(
            tool_current="3",
            mode=ToolLock.CHANGER_IDLE)
        toollock.tool_lock_gcode_template = FakeTemplate()

        toollock.ToolLock()

        self.assertEqual("3", toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)
        self.assertEqual(0, toollock.tool_lock_gcode_template.calls)

    def test_drop_all_unknown_rejection_preserves_idle(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNKNOWN,
            mode=ToolLock.CHANGER_IDLE)
        error = RuntimeError("unknown tool")
        toollock.printer.command_error.return_value = error

        with self.assertRaisesRegex(RuntimeError, "unknown tool"):
            toollock.cmd_KTCC_TOOL_DROPOFF_ALL()

        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)
        toollock.printer.lookup_object.assert_not_called()

    def test_marking_risk_outside_transaction_is_error(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with self.assertRaises(RuntimeError):
            toollock.mark_changer_risk()

    def test_caught_nested_post_risk_failure_still_shuts_down(self):
        toollock = make_toollock(mode=ToolLock.CHANGER_IDLE)
        with toollock.changer_operation("select"):
            try:
                with toollock.changer_operation("pickup"):
                    toollock.mark_changer_risk()
                    raise RuntimeError("caught")
            except RuntimeError:
                pass
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        toollock.printer.invoke_shutdown.assert_called_once()

    def test_successful_unlock_ends_idle(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNKNOWN,
            mode=ToolLock.CHANGER_IDLE)
        toollock.tool_unlock_gcode_template = FakeTemplate()
        toollock.SaveCurrentTool = lambda tool: setattr(
            toollock, "tool_current", str(tool))

        toollock.cmd_TOOL_UNLOCK()

        self.assertEqual(str(ToolLock.TOOL_UNLOCKED), toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_failed_unlock_after_risk_shuts_down_klippy(self):
        toollock = make_toollock(
            tool_current=ToolLock.TOOL_UNKNOWN,
            mode=ToolLock.CHANGER_IDLE)
        failure = RuntimeError("unlock failed")
        toollock.tool_unlock_gcode_template = FakeTemplate(
            lambda: (_ for _ in ()).throw(failure))
        toollock.SaveCurrentTool = lambda tool: setattr(
            toollock, "tool_current", str(tool))

        with self.assertRaisesRegex(RuntimeError, "unlock failed"):
            toollock.cmd_TOOL_UNLOCK()

        self.assertEqual(str(ToolLock.TOOL_UNKNOWN), toollock.tool_current)
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        toollock.printer.invoke_shutdown.assert_called_once()

    def test_bootstrap_any_resolved_or_unknown_tool_ends_idle(self):
        for tool_current in ("-2", "-1", "3"):
            toollock = make_toollock(tool_current=ToolLock.TOOL_UNKNOWN)
            toollock.tool_map = {}
            toollock.load_tool_offsets = lambda: None
            toollock.load_global_offset = lambda: None
            toollock.load_tool_retractions = lambda: None
            toollock.load_tool_pressure_advance = lambda: None
            toollock.Initialize_Tool_Lock = lambda value=tool_current: setattr(
                toollock, "tool_current", value)

            toollock._bootup_tasks(0)

            self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)

    def test_bootstrap_exception_shuts_down_klippy(self):
        toollock = make_toollock(tool_current=ToolLock.TOOL_UNKNOWN)
        toollock.tool_map = {}
        toollock.load_tool_offsets = lambda: None
        toollock.load_global_offset = lambda: None
        toollock.load_tool_retractions = lambda: None
        toollock.load_tool_pressure_advance = lambda: None
        toollock.Initialize_Tool_Lock = lambda: (
            _ for _ in ()).throw(RuntimeError("bootstrap failed"))

        toollock._bootup_tasks(0)

        self.assertEqual(
            ToolLock.CHANGER_SYNCHRONIZING,
            toollock.changer_mode)
        toollock.printer.invoke_shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
