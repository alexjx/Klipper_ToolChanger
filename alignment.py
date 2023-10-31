import logging
import re

from contextlib import contextmanager
from collections import namedtuple

DEFAULT_X_OFFSET = 0.0
DEFAULT_Y_OFFSET = -39.0
DEFAULT_Z_OFFSET = 10.0

PROBE_SPEED = 0.1
FAST_PROBE_SPEED = 5.0

FAST_MOVE_SPEED_XY = 300.0
FAST_MOVE_SPEED_Z = 10.0

PROBE_BACKOFF = 0.5
XY_PROBE_DEPTH = 1.0
XY_PROBE_OFFSET = 10.0
DWELL_TIME = 0.8 # this used to avoid klipper timing issue

Probes = namedtuple('Probes', ['x', 'y', 'z'])

class AlignemntHelper:
    def __init__(self, tool_id: int, probe_point: tuple[float, float], probes: Probes, printer) -> None:
        self.printer = printer
        self.tool_id = tool_id
        self.tool = self.printer.lookup_object(f'tool {tool_id}', None)
        if self.tool is None:
            raise Exception(f'No tool with id {tool_id} found')
        self.toollock = self.printer.lookup_object(f'toollock')
        self.probe_point = probe_point
        self.probes = probes
        self.gcode = self.printer.lookup_object('gcode')
        self.phoming = self.printer.lookup_object('homing')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.reactor = self.printer.get_reactor()

        # set default offsets
        default_offsets = self.tool.get_offset()
        self._default_x_offset = default_offsets[0]
        self._default_y_offset = default_offsets[1]
        self._default_z_offset = default_offsets[2]

        self.z_samples = []
        self.xy_samples = []

    def get_z(self):
        if len(self.z_samples) == 0:
            raise Exception('z axis not sampled')
        return sum(self.z_samples) / len(self.z_samples) # type: ignore

    def get_xy(self):
        if len(self.xy_samples) == 0:
            raise Exception('xy axis not sampled')
        x = (self.xy_samples[0] + self.xy_samples[2]) / 2.0
        y = (self.xy_samples[1] + self.xy_samples[3]) / 2.0

        # klipper apply following tranform with offsets
        #   x = toolhead_x - offset_x
        #   y = toolhead_y - offset_y
        # where toolhead_x and toolhead_y are the machine coordinates
        # so when probing, it will be the probed position
        # and x, y will be the probe point
        # so we need to apply the inverse transform
        #   offset_x = toolhead_x - x = probed_x - probe_point_x
        #   offset_y = toolhead_y - y = probed_y - probe_point_y

        return (x - self.probe_point[0], y - self.probe_point[1])

    def tranform_to_machine_position(self, user_pos):
        return [p + o for p, o in zip(user_pos, self.tool.offset)]

    def prepare(self):
        self.gcode.respond_info(f'preparing alignment for tool {self.tool_id}')
        self.toollock.saved_position = None # remove saved position
        self.gcode.run_script_from_command(
            'SAVE_GCODE_STATE NAME=alignment_state\n'
            'KTCC_TOOL_DROPOFF_ALL\n'
            'SAVE_POSITION\n' # remove saved position
            'BED_MESH_CLEAR\n'
            'G90\n'
            'G28 Z\n'
            f'SET_TOOL_OFFSET TOOL={self.tool_id} X={self._default_x_offset:.6f} Y={self._default_y_offset:.6f} Z={self._default_z_offset:.6f}\n'
            f'KTCC_T{self.tool_id}\n'
            f'G0 X{self.probe_point[0]} Y{self.probe_point[1]} F12000\n'
            'M400\n'
        )

    def finish(self):
        self.gcode.run_script_from_command(
            'KTCC_TOOL_DROPOFF_ALL\n'
        )

    def move_in_user_pos(self, pos, speed, wait=True):
        transformed_pos = self.tranform_to_machine_position(pos)
        self.toolhead.manual_move(transformed_pos, speed)
        if wait:
            self.toolhead.wait_moves()

    def probe_xy(self):
        # for each tool we mesure 4 points, left, front, right, back
        results = []

        # we should have z sampled already
        z_offset = self.get_z()
        probe_x, probe_y = self.tranform_to_machine_position([self.probe_point[0], self.probe_point[1]])
        probe_z = z_offset - XY_PROBE_DEPTH
        pos = self.toolhead.get_position()
        probe_center = [probe_x, probe_y, probe_z, pos[3]]

        # raise z axis if necessary
        if pos[2] <= z_offset:
            self.gcode.respond_info(f'tool {self.tool_id}: raising z axis')
            self.toolhead.manual_move([None, None, z_offset + 1.0], FAST_MOVE_SPEED_Z)
            self.toolhead.wait_moves()

        # move to first probe point (left)
        start_pos = self.tranform_to_machine_position([self.probe_point[0] - XY_PROBE_OFFSET, self.probe_point[1]])
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        self.toolhead.manual_move([None, None, probe_z], FAST_MOVE_SPEED_Z)
        self.toolhead.wait_moves()

        # probe left
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.x, probe_center, FAST_PROBE_SPEED)
        self.toolhead.manual_move([epos[0] - PROBE_BACKOFF, None], FAST_MOVE_SPEED_XY) # back X a bit
        self.toolhead.wait_moves()
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.x, probe_center, PROBE_SPEED)
        self.gcode.respond_info(f'tool {self.tool_id}: left probed at X={epos[0]:.4f}')
        results.append(epos[0])

        # move to next point (front)
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)    # return to start   ( probe_center - XY_PROBE_OFFSET,   probe_center                   )
        start_pos[1] -= XY_PROBE_OFFSET                             # move to           ( probe_center - XY_PROBE_OFFSET,   probe_center - XY_PROBE_OFFSET )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        start_pos[0] += XY_PROBE_OFFSET                             # move to           ( probe_center,                     probe_center - XY_PROBE_OFFSET )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        self.toolhead.wait_moves()

        # probe front
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.y, probe_center, FAST_PROBE_SPEED)
        self.toolhead.manual_move([None, epos[1] - PROBE_BACKOFF], FAST_MOVE_SPEED_XY) # back Y a bit
        self.toolhead.wait_moves()
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.y, probe_center, PROBE_SPEED)
        self.gcode.respond_info(f'tool {self.tool_id}: front probed at Y={epos[1]:.4f}')
        results.append(epos[1])

        # move to next point (right)
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)    # return to start   ( probe_center,                     probe_center - XY_PROBE_OFFSET )
        start_pos[0] += XY_PROBE_OFFSET                             # move to           ( probe_center + XY_PROBE_OFFSET,   probe_center - XY_PROBE_OFFSET )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        start_pos[1] += XY_PROBE_OFFSET                             # move to           ( probe_center + XY_PROBE_OFFSET,   probe_center                   )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        self.toolhead.wait_moves()

        # probe right
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.x, probe_center, FAST_PROBE_SPEED)
        self.toolhead.manual_move([epos[0] + PROBE_BACKOFF, None], FAST_MOVE_SPEED_XY)
        self.toolhead.wait_moves()
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.x, probe_center, PROBE_SPEED)
        self.gcode.respond_info(f'tool {self.tool_id}: right probed at X={epos[0]:.4f}')
        results.append(epos[0])

        # move to next point (back)
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)    # return to start   ( probe_center + XY_PROBE_OFFSET,   probe_center                   )
        start_pos[1] += XY_PROBE_OFFSET                             # move to           ( probe_center + XY_PROBE_OFFSET,   probe_center + XY_PROBE_OFFSET )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        start_pos[0] -= XY_PROBE_OFFSET                             # move to           ( probe_center,                     probe_center + XY_PROBE_OFFSET )
        self.toolhead.manual_move(start_pos, FAST_MOVE_SPEED_XY)
        self.toolhead.wait_moves()

        # probe back
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.y, probe_center, FAST_PROBE_SPEED)
        self.toolhead.manual_move([None, epos[1] + PROBE_BACKOFF], FAST_MOVE_SPEED_XY)
        self.toolhead.wait_moves()
        self.toolhead.dwell(DWELL_TIME)
        epos = self.phoming.probing_move(self.probes.y, probe_center, PROBE_SPEED)
        self.gcode.respond_info(f'tool {self.tool_id}: back probed at Y={epos[1]:.4f}')
        results.append(epos[1])

        self.xy_samples = results
        return results

    def probe_z(self, n_samples: int = 3):
        # toolhead position is the machine position in RRF
        probe_tgt = self.tranform_to_machine_position([self.probe_point[0], self.probe_point[1], -DEFAULT_Z_OFFSET - 1.0])
        logging.info(f'tool {self.tool_id}: probing z from location {probe_tgt}')

        pos = self.toolhead.get_position()
        probe_tgt.append(pos[3]) # probing_move requires 4th axis

        def _do_probe(probe_sp, lift_sp):
            epos = self.phoming.probing_move(self.probes.z, probe_tgt, probe_sp)
            result_z = epos[2]
            self.toolhead.manual_move([None, None, result_z + PROBE_BACKOFF], lift_sp)
            self.toolhead.wait_moves()
            return result_z

        # move to probe point fast
        self.toolhead.manual_move([probe_tgt[0], probe_tgt[1]], FAST_MOVE_SPEED_XY)

        # establish z axis position
        self.gcode.respond_info(f'tool {self.tool_id}: establishing z axis position')
        _do_probe(FAST_PROBE_SPEED, FAST_MOVE_SPEED_Z)

        # probe z axis
        self.z_samples = []
        for i in range(n_samples):
            self.gcode.respond_info(f'tool {self.tool_id}: probing z axis, sample {i+1}/{n_samples}')
            z_result = _do_probe(PROBE_SPEED, FAST_MOVE_SPEED_Z)
            self.gcode.respond_info(f'tool {self.tool_id}: z axis triggered at {z_result:.4f}')
            self.z_samples.append(z_result)

        return self.z_samples

class Alignment:
    TMC_STEPPERS = re.compile(r'tmc[0-9]+ stepper_[xyz]')

    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # steppers
        self.steppers = {}
        for name, obj in self.printer.lookup_objects():
            if self.TMC_STEPPERS.match(name):
                axis = name[-1]
                self.steppers[axis] = obj

        # Create an "endstop" object to handle the probe pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        logging.info(f'alignment: using pin {pin}')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']

        mcu_endstop_x = mcu.setup_pin('endstop', pin_params)
        mcu_endstop_y = mcu.setup_pin('endstop', pin_params)
        mcu_endstop_z = mcu.setup_pin('endstop', pin_params)

        query_endstops = self.printer.load_object(config, 'query_endstops')
        query_endstops.register_endstop(mcu_endstop_x, 'alignment_probe_x')
        query_endstops.register_endstop(mcu_endstop_y, 'alignment_probe_y')
        query_endstops.register_endstop(mcu_endstop_z, 'alignment_probe_z')

        self.probes = Probes(mcu_endstop_x, mcu_endstop_y, mcu_endstop_z)

        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)

        self.gcode.register_command(
            'KTCC_ALIGN_TOOLS',
            self.cmd_KTCC_ALIGN_TOOLS,
            False,
            self.cmd_KTCC_ALIGN_TOOLS_help,
        )

    def _handle_mcu_identify(self):
        # since we are doing alignment, we will register all 3 axes
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('x'):
                self.probes.x.add_stepper(stepper)
            if stepper.is_active_axis('y'):
                self.probes.y.add_stepper(stepper)
            if stepper.is_active_axis('z'):
                self.probes.z.add_stepper(stepper)

    @contextmanager
    def lower_stepper_current(self):
        # lower stepper current
        run_current = {}
        for axis, stepper in self.steppers.items():
            self.gcode.respond_info(f'lowering stepper current for axis {axis}')
            run_current[axis] = stepper.get_status()['run_current']
            if axis == 'z':
                self.gcode.run_script_from_command(
                    f'SET_TMC_CURRENT STEPPER=stepper_{axis} CURRENT={run_current[axis] * 0.6:.2f}\n'
                    'G4 P600\n'
                )
            else:
                self.gcode.run_script_from_command(
                    f'SET_TMC_CURRENT STEPPER=stepper_{axis} CURRENT={run_current[axis] * 0.4:.2f}\n'
                    'G4 P600\n'
                )

        yield

        # restore stepper current
        for axis, stepper in self.steppers.items():
            self.gcode.respond_info(f'restoring stepper current for axis {axis}')
            self.gcode.run_script_from_command(
                f'SET_TMC_CURRENT STEPPER=stepper_{axis} CURRENT={run_current[axis]}\n'
                'G4 P600\n'
            )

    cmd_KTCC_ALIGN_TOOLS_help = "aligns mutiple tools"
    def cmd_KTCC_ALIGN_TOOLS(self, gcmd):
        tools_to_probe_str = gcmd.get('TOOLS', None)
        if tools_to_probe_str is not None:
            tools_to_probe = [int(t) for t in tools_to_probe_str.split(',')]
        else:
            tools_to_probe = []
            for i in range(99):
                if self.printer.lookup_object(f'tool {i}', None) is not None:
                    tools_to_probe.append(i)
        n_samples = gcmd.get_int('SAMPLES', 3, minval=1)
        n_retries = gcmd.get_int('RETRIES', 3, minval=1)
        probe_point_str = gcmd.get('PROBE_POINT', None)
        if probe_point_str is None:
            raise gcmd.error("missing probe point")
        probe_point = tuple(float(p) for p in probe_point_str.split(','))
        if len(probe_point) != 2:
            raise gcmd.error("invalid probe point")
        tolerance = gcmd.get_float('TOLERANCE', 0.02, minval=0.0)
        save_it = gcmd.get_int('SAVE', 0)

        # check if we have homed all axes
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        kin = toolhead.get_kinematics()
        if kin.get_status(curtime)['homed_axes'] != 'xyz':
            raise self.gcode.error("all axes must be homed before align")

        self.gcode.respond_info(f'aligning tools {tools_to_probe} at probe point {probe_point} with {n_samples} samples and {n_retries} retries')

        ktcclog = self.printer.lookup_object('ktcclog')

        for tool_id in tools_to_probe:
            for t in range(n_retries):
                ktcclog.info(f'aligning tool {tool_id} for {t+1}/{n_retries}')
                samples = []
                for i in range(n_samples):
                    self.gcode.respond_info(f'probing tool {tool_id}, sample {i+1}/{n_samples}')
                    helper = AlignemntHelper(tool_id, probe_point, self.probes, self.printer)

                    helper.prepare()
                    # ktcc log will mesh up the probe results
                    with self.lower_stepper_current(), ktcclog.disable_save():
                        helper.probe_z(3)
                        helper.probe_xy()
                    helper.finish()

                    x, y = helper.get_xy()
                    z = helper.get_z()

                    self.gcode.respond_info(f'tool {tool_id}: sample {i+1}/{n_samples}: x={x:.4f}, y={y:.4f}, z={z:.4f}')
                    samples.append((x, y, z))

                # calculate average
                x_avg = sum([s[0] for s in samples]) / len(samples)
                y_avg = sum([s[1] for s in samples]) / len(samples)
                z_avg = sum([s[2] for s in samples]) / len(samples)
                # calculate mean absolute deviation
                x_mad = sum([abs(s[0] - x_avg) for s in samples]) / len(samples)
                y_mad = sum([abs(s[1] - y_avg) for s in samples]) / len(samples)
                z_mad = sum([abs(s[2] - z_avg) for s in samples]) / len(samples)
                # print out results
                self.gcode.respond_info(f'tool {tool_id}: x={x_avg:.4f}, y={y_avg:.4f}, z={z_avg:.4f} (x_mad={x_mad:.6f}, y_mad={y_mad:.6f}, z_mad={z_mad:.6f})')

                # check if we are within tolerance
                if x_mad <= tolerance and y_mad <= tolerance and z_mad <= tolerance:
                    self.gcode.respond_info(f'tool {tool_id}: alignment successful')
                    self.gcode.run_script_from_command(f'SET_TOOL_OFFSET TOOL={tool_id} X={x_avg:.4f} Y={y_avg:.4f} Z={z_avg:.4f}')
                    if save_it:
                        self.gcode.run_script_from_command(f'KTCC_SAVE_TOOL_OFFSET TOOL={tool_id}')
                    break
            else:
                self.gcode.error(f'tool {tool_id}: alignment failed')


def load_config(config):
    return Alignment(config)
