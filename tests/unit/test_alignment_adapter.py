import pytest

from alignment import Alignment, AlignmentHelper, AlignemntHelper


class GCode:
    def __init__(self):
        self.commands = []
        self.responses = []

    def run_script_from_command(self, command):
        self.commands.append(command)

    def respond_info(self, message):
        self.responses.append(message)


def test_alignment_helper_math_and_spelling_compatibility_alias():
    helper = AlignmentHelper.__new__(AlignmentHelper)
    helper.xy_samples = [9.8, 20.4, 10.2, 19.6]
    helper.probe_point = (10.0, 20.0)
    helper.tool = type("Tool", (), {"offset": [1.0, -2.0, 3.0]})()

    assert helper.get_xy() == pytest.approx((0.0, 0.0))
    assert helper.transform_to_machine_position([4.0, 5.0, 6.0]) == [5.0, 3.0, 9.0]
    assert helper.tranform_to_machine_position([4.0, 5.0, 6.0]) == [5.0, 3.0, 9.0]
    assert AlignemntHelper is AlignmentHelper


def test_finish_drops_tool_and_restores_saved_gcode_state():
    helper = AlignmentHelper.__new__(AlignmentHelper)
    helper.gcode = GCode()

    helper.finish()

    assert helper.gcode.commands == [
        "KTCC_TOOL_DROPOFF_ALL\nRESTORE_GCODE_STATE NAME=alignment_state MOVE=0\n"
    ]


def test_lower_stepper_current_restores_every_changed_stepper_on_failure():
    alignment = Alignment.__new__(Alignment)
    alignment.gcode = GCode()
    alignment.steppers = {
        "stepper_x": type("Stepper", (), {"get_status": lambda self: {"run_current": 1.0}})(),
        "stepper_z1": type("Stepper", (), {"get_status": lambda self: {"run_current": 0.8}})(),
    }

    with pytest.raises(RuntimeError, match="probe failed"):
        with alignment.lower_stepper_current():
            raise RuntimeError("probe failed")

    commands = "".join(alignment.gcode.commands)
    assert "STEPPER=stepper_x CURRENT=0.85" in commands
    assert "STEPPER=stepper_z1 CURRENT=0.68" in commands
    assert "STEPPER=stepper_x CURRENT=1.0" in commands
    assert "STEPPER=stepper_z1 CURRENT=0.8" in commands
