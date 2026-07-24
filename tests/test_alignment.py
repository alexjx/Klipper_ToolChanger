import unittest
from unittest import mock

import alignment
from toollock import ToolLock


class CommandError(RuntimeError):
    pass


class FakeGcmd:
    def __init__(self, params):
        self.params = {key.upper(): value for key, value in params.items()}

    def get(self, name, default=None):
        return self.params.get(name.upper(), default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        value = self.get(name, default)
        if value is None:
            return value
        value = int(value)
        if minval is not None and value < minval:
            raise self.error(f'{name} below minimum')
        return value

    def get_float(self, name, default=None, minval=None, maxval=None):
        value = self.get(name, default)
        if value is None:
            return value
        value = float(value)
        if minval is not None and value < minval:
            raise self.error(f'{name} below minimum')
        return value

    def error(self, message):
        return CommandError(message)


class FakeGcode:
    def __init__(self):
        self.messages = []
        self.scripts = []

    def respond_info(self, message):
        self.messages.append(message)

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def error(self, message):
        return CommandError(message)


class FakeTool:
    def __init__(self, offsets=(0.0, 0.0, 0.0)):
        self.config_offset = list(offsets)


class FakeKinematics:
    def get_status(self, eventtime):
        return {'homed_axes': 'xyz'}


class FakeToolhead:
    def get_kinematics(self):
        return FakeKinematics()


class NullContext:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class FakeKtcclog:
    def info(self, message):
        pass

    def disable_save(self):
        return NullContext()


class FakePrinter:
    def __init__(self, objects):
        self.objects = objects
        self.shutdowns = []

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def get_reactor(self):
        return mock.Mock(monotonic=lambda: 0.0)

    def invoke_shutdown(self, message):
        self.shutdowns.append(message)


class FakeLog:
    def info(self, message):
        pass


def make_toollock():
    toollock = ToolLock.__new__(ToolLock)
    toollock.log = FakeLog()
    toollock.tool_current = str(ToolLock.TOOL_UNLOCKED)
    toollock.changer_mode = ToolLock.CHANGER_IDLE
    toollock._changer_depth = 0
    toollock._changer_previous_mode = ToolLock.CHANGER_IDLE
    toollock._changer_operation = None
    toollock._changer_risk_started = False
    toollock._changer_failed = False
    toollock._changer_failure = None
    return toollock


def make_alignment(helper_mode):
    gcode = FakeGcode()
    toollock = make_toollock()
    printer = FakePrinter({
        'tool 0': FakeTool(),
        'toolhead': FakeToolhead(),
        'ktcclog': FakeKtcclog(),
        'toollock': toollock,
    })
    toollock.printer = printer
    instance = alignment.Alignment.__new__(alignment.Alignment)
    instance.printer = printer
    instance.gcode = gcode
    instance.probes = object()
    instance.lower_stepper_current = lambda: NullContext()

    helper_count = {'n': 0}

    class FakeHelper:
        def __init__(self, tool_id, probe_point, probes, helper_printer):
            self.tool_id = tool_id
            self.index = helper_count['n']
            helper_count['n'] += 1

        def prepare(self):
            if helper_mode == 'prepare_failure':
                raise RuntimeError('motion failed')

        def probe_z(self, n_samples):
            if helper_mode == 'probe_failure':
                raise RuntimeError('probe failed')
            if helper_mode == 'retry_exhaustion':
                self.z_samples = [0.0, 1.0, 0.0]
            else:
                self.z_samples = [0.0, 0.0, 0.0]

        def probe_xy(self):
            if helper_mode == 'retry_exhaustion':
                self.xy_samples = [0.0, 1.0, 0.0, 1.0]
            else:
                self.xy_samples = [0.0, 0.0, 0.0, 0.0]

        def get_xy(self):
            if helper_mode == 'retry_exhaustion':
                value = float(self.index % 2)
                return (value, value)
            return (0.0, 0.0)

        def get_z(self):
            if helper_mode == 'retry_exhaustion':
                return float(self.index % 2)
            return sum(self.z_samples) / len(self.z_samples)

        def finish(self):
            pass

    return instance, gcode, toollock, FakeHelper


class AlignmentTransactionTests(unittest.TestCase):
    def test_validation_failure_is_before_transaction(self):
        instance, _, toollock, helper = make_alignment('success')
        gcmd = FakeGcmd({'TOOLS': '1', 'PROBE_POINT': '0,0'})
        with mock.patch.object(alignment, 'AlignemntHelper', helper):
            with self.assertRaisesRegex(CommandError, 'No tool with id 1'):
                instance.cmd_KTCC_ALIGN_TOOLS(gcmd)
        self.assertEqual(ToolLock.CHANGER_IDLE, toollock.changer_mode)
        self.assertEqual(0, toollock._changer_depth)

    def test_motion_failure_after_risk_shuts_down_klippy(self):
        instance, _, toollock, helper = make_alignment('probe_failure')
        gcmd = FakeGcmd({'TOOLS': '0', 'PROBE_POINT': '0,0', 'SAMPLES': '1'})
        with mock.patch.object(alignment, 'AlignemntHelper', helper):
            with self.assertRaisesRegex(RuntimeError, 'probe failed'):
                instance.cmd_KTCC_ALIGN_TOOLS(gcmd)
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        self.assertEqual(1, len(toollock.printer.shutdowns))
        self.assertEqual(0, toollock._changer_depth)

    def test_retry_exhaustion_raises_and_shuts_down_klippy(self):
        instance, _, toollock, helper = make_alignment('retry_exhaustion')
        gcmd = FakeGcmd({
            'TOOLS': '0',
            'PROBE_POINT': '0,0',
            'SAMPLES': '2',
            'RETRIES': '2',
        })
        with mock.patch.object(alignment, 'AlignemntHelper', helper):
            with self.assertRaisesRegex(CommandError, 'tool 0: alignment failed'):
                instance.cmd_KTCC_ALIGN_TOOLS(gcmd)
        self.assertEqual(ToolLock.CHANGER_CHANGING, toollock.changer_mode)
        self.assertEqual(1, len(toollock.printer.shutdowns))
        self.assertEqual(0, toollock._changer_depth)


if __name__ == '__main__':
    unittest.main()
