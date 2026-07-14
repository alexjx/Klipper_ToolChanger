# Copyright (C) 2023 Andrei Ignat <andrei@ignat.se>
# SPDX-License-Identifier: GPL-3.0-only

"""Thin Klipper facade for one configured physical tool.

The object remains a Klipper config-prefix entry point and preserves the
documented status/Jinja surface.  Runtime state and all effects are owned by
the services composed by :mod:`toollock` at ``klippy:ready``.
"""

import logging


class Tool:
    TOOL_UNKNOWN = -2
    TOOL_UNLOCKED = -1
    HEATER_STATE_ACTIVE = 2
    HEATER_STATE_STANDBY = 1
    HEATER_STATE_OFF = 0

    def __init__(self, config=None):
        self.name = None
        self.config = config
        self.log = logging.getLogger('ktcc.tool')
        if config is None:
            return

        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.toollock = self.printer.lookup_object('toollock')

        try:
            _, raw_name = config.get_name().split(' ', 1)
            self.name = int(raw_name)
        except ValueError:
            raise config.error(
                "Name of section '%s' contains illegal characters. "
                "Use only integer tool number." % config.get_name()
            )

        group_name = 'toolgroup ' + str(config.getint('tool_group'))
        if not config.has_section(group_name):
            raise config.error(
                "ToolGroup of T'%s' is not defined. It must be configured "
                "before the tool." % config.get_name()
            )
        self.toolgroup = self.printer.lookup_object(group_name)
        self._reject_removed_features(config)

        self.extruder = self._value('extruder')
        self.heater = self._value('heater', self.extruder)
        self.fan = self._value('fan')
        self.meltzonelength = self._value('meltzonelength', 0)
        self.unload_length = self._float(
            'unload_length', float(self.meltzonelength)
        )
        self.prime_length = self._float(
            'prime_length', float(self.meltzonelength)
        )
        self.lazy_home_when_parking = self._boolean(
            'lazy_home_when_parking', False
        )
        self.zone = self._vector('zone')
        self.park = self._vector('park')
        self.offset = self._numeric_vector('offset')
        self.config_offset = list(self.offset)

        self.heater_state = self.HEATER_STATE_OFF
        self.heater_active_temp = self._float('heater_active_temp', 0.0)
        self.heater_standby_temp = self._float('heater_standby_temp', 0.0)
        self.idle_to_standby_time = self._float('idle_to_standby_time', 30.0)
        self.idle_to_powerdown_time = self._float('idle_to_powerdown_time', 600.0)
        self.select_wait_mode = self._value('select_wait_mode', 'HEAT')
        self.select_wait_tolerance = self._float('select_wait_tolerance', 1.0)
        self.select_wait_timeout = self._float('select_wait_timeout', 900.0)
        self.select_wait_stable_time = self._float('select_wait_stable_time', 0.0)

        self.retract_length = self._float('retract_length', 0.0)
        self.retract_speed = self._float('retract_speed', 20.0)
        self.unretract_extra_length = self._float(
            'unretract_extra_length', 0.0
        )
        self.unretract_speed = self._float('unretract_speed', 20.0)
        self.zhop = self._float('zhop', 0.0)
        self.pressure_advance = self._float('pressure_advance', 0.0)
        self.pressure_advance_smooth_time = self._float(
            'pressure_advance_smooth_time', 0.04
        )

        self.shaper_freq_x = self._float('shaper_freq_x', 0.0)
        self.shaper_freq_y = self._float('shaper_freq_y', 0.0)
        self.shaper_type_x = self._value('shaper_type_x', 'mzv')
        self.shaper_type_y = self._value('shaper_type_y', 'mzv')
        self.shaper_damping_ratio_x = self._float(
            'shaper_damping_ratio_x', 0.1
        )
        self.shaper_damping_ratio_y = self._float(
            'shaper_damping_ratio_y', 0.1
        )

        self.pickup_gcode_template = self._template('pickup_gcode')
        self.dropoff_gcode_template = self._template('dropoff_gcode')
        self.verify_mounted_gcode_template = self._optional_template(
            'verify_mounted_gcode'
        )
        self.verify_unmounted_gcode_template = self._optional_template(
            'verify_unmounted_gcode'
        )
        self.recovery_gcode_template = self._optional_template('recovery_gcode')

        self.gcode.register_command(
            'KTCC_T%d' % self.name,
            self.cmd_SelectTool,
            desc=self.cmd_SelectTool_help,
        )

    @staticmethod
    def _reject_removed_features(config):
        if config.getboolean('is_virtual', False):
            raise config.error(
                "Section '%s' enables virtual tools, which are no longer supported."
                % config.get_name()
            )
        parent = config.getint('physical_parent', None)
        if parent not in (None, -1):
            raise config.error(
                "Section '%s' sets physical_parent, which is no longer supported."
                % config.get_name()
            )
        parent_id = config.getint('physical_parent_id', None)
        if parent_id not in (None, -1):
            raise config.error(
                "Section '%s' sets physical_parent_id, which is no longer supported."
                % config.get_name()
            )
        for option in ('remap', 'tool_remap'):
            value = config.get(option, None)
            if value not in (None, False, '', -1):
                raise config.error(
                    "Section '%s' sets %s, but runtime remapping is no longer supported."
                    % (config.get_name(), option)
                )
        for option in ('virtual_toolload_gcode', 'virtual_toolunload_gcode'):
            if str(config.get(option, '')).strip():
                raise config.error(
                    "Section '%s' sets %s, but virtual tools are no longer supported."
                    % (config.get_name(), option)
                )
        for option in (
            'requires_pickup_for_virtual_load',
            'requires_pickup_for_virtual_unload',
            'unload_virtual_at_dropoff',
        ):
            if config.getboolean(option, False):
                raise config.error(
                    "Section '%s' enables %s, but virtual tools are no longer supported."
                    % (config.get_name(), option)
                )

    def _value(self, name, default=None):
        return self.config.get(name, self.toolgroup.get_config(name, default))

    def _boolean(self, name, default=False):
        return self.config.getboolean(
            name, self.toolgroup.get_config(name, default)
        )

    def _float(self, name, default):
        return float(self._value(name, default))

    def _vector(self, name):
        value = self._value(name, '0,0,0')
        parts = list(value) if isinstance(value, (list, tuple)) else str(value).split(',')
        parts = [str(item).strip() for item in parts]
        if len(parts) != 3:
            raise self.config.error(
                "%s of section '%s' must contain exactly X,Y,Z"
                % (name, self.config.get_name())
            )
        return parts

    def _numeric_vector(self, name):
        try:
            return [float(item) for item in self._vector(name)]
        except ValueError as error:
            raise self.config.error(
                "%s of section '%s' must contain finite numbers: %s"
                % (name, self.config.get_name(), error)
            )

    def _template(self, name):
        inherited = self.toolgroup.get_config(name, '')
        return self.gcode_macro.load_template(self.config, name, inherited)

    def _optional_template(self, name):
        raw = self._value(name, None)
        if raw is None or not str(raw).strip():
            return None
        return self.gcode_macro.load_template(self.config, name, raw)

    def get_config(self, name, default=None):
        if self.config is None:
            return default
        return self.config.get(name, default)

    cmd_SelectTool_help = 'Select physical tool'

    def cmd_SelectTool(self, gcmd):
        runtime = getattr(self.toollock, 'runtime', None)
        if runtime is None:
            raise gcmd.error('KTCC runtime is not ready')
        restore = gcmd.get_int('R', None, minval=0, maxval=2)
        if restore in (1, 2):
            self.toollock.SaveCurrentPosition(restore)
        elif restore == 0:
            self.toollock.SavePosition()
        self._select(runtime)

    def select_tool_actual(self, param=None):
        """Compatibility Python entry point backed by the transaction service."""
        runtime = getattr(self.toollock, 'runtime', None)
        if runtime is None:
            raise self.printer.command_error('KTCC runtime is not ready')
        if param in (1, 2):
            self.toollock.SaveCurrentPosition(param)
        elif param == 0:
            self.toollock.SavePosition()
        return self._select(runtime)

    def _select(self, runtime):
        before = runtime.toolchange.state.mounted
        try:
            state = runtime.toolchange.select(self.name)
            source_id = before.tool_id if before.kind.value == 'KNOWN' else None
            if source_id is not None and source_id != self.name:
                source_fan = runtime.bridge.registry[source_id].fan
                if source_fan is not None:
                    runtime.machine.set_fan(source_fan, 0.0)
            target_fan = runtime.bridge.registry[self.name].fan
            if target_fan is not None:
                runtime.machine.set_fan(
                    target_fan, float(self.toollock.saved_fan_speed)
                )
            return state
        finally:
            self.toollock._sync_legacy_views()
            self.toollock._flush_statistics()

    def set_offset(self, **updates):
        axes = {'x': 0, 'y': 1, 'z': 2}
        for axis, index in axes.items():
            absolute = updates.get(axis + '_pos')
            adjust = updates.get(axis + '_adjust')
            if absolute is not None:
                self.offset[index] = float(absolute)
            elif adjust is not None:
                self.offset[index] += float(adjust)

    def get_offset(self):
        return [float(value) for value in self.offset]

    def get_status(self, eventtime=None):
        del eventtime
        return {
            'name': self.name,
            'extruder': self.extruder,
            'heater': self.heater,
            'fan': self.fan,
            'lazy_home_when_parking': self.lazy_home_when_parking,
            'meltzonelength': self.meltzonelength,
            'unload_length': self.unload_length,
            'prime_length': self.prime_length,
            'zone': self.zone,
            'park': self.park,
            'offset': self.offset,
            'heater_state': self.heater_state,
            'heater_active_temp': self.heater_active_temp,
            'heater_standby_temp': self.heater_standby_temp,
            'idle_to_standby_time': self.idle_to_standby_time,
            'idle_to_powerdown_next_wake': self.idle_to_powerdown_time,
            'shaper_freq_x': self.shaper_freq_x,
            'shaper_freq_y': self.shaper_freq_y,
            'shaper_type_x': self.shaper_type_x,
            'shaper_type_y': self.shaper_type_y,
            'shaper_damping_ratio_x': self.shaper_damping_ratio_x,
            'shaper_damping_ratio_y': self.shaper_damping_ratio_y,
            'retract_length': self.retract_length,
            'retract_speed': self.retract_speed,
            'unretract_extra_length': self.unretract_extra_length,
            'unretract_speed': self.unretract_speed,
            'zhop': self.zhop,
            'pressure_advance': self.pressure_advance,
            'pressure_advance_smooth_time': self.pressure_advance_smooth_time,
        }


def load_config_prefix(config):
    return Tool(config)
