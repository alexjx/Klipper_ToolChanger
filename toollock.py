# Copyright (C) 2023 Andrei Ignat <andrei@ignat.se>
# SPDX-License-Identifier: GPL-3.0-only

"""Klipper G-code facade and composition root for KTCC.

This module parses legacy command parameters and exposes stable status fields.
Changer state, profiles, thermal policy, persistence, and transactions live in
the ``ktcc`` application services; no second implementation is kept here.
"""

import logging

from ktcc.klipper.composition import compose_runtime
from ktcc.statistics.reporting import format_print_statistics, format_statistics
from ktcc.thermal import WaitTarget
from ktcc.toolchange.state import ChangerMode, LockState, MountedKind, MountedState
from ktcc.tools import Vector3
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    PressureAdvanceProfile,
    WaitMode,
)
from ktcc.toolchange.recovery import CarriageEstimate
from ktcc.ports import ThermalMode


class ToolLock:
    TOOL_UNKNOWN = -2
    TOOL_UNLOCKED = -1
    BOOT_DELAY = 1.5

    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.log = logging.getLogger('ktcc.toollock')
        self.runtime = None
        self._action_depth = 0

        self.global_offset = [0.0, 0.0, 0.0]
        self.saved_fan_speed = 0.0
        self.tool_current = '-2'
        # Retained as a parsed compatibility option.  The refactored runtime
        # never performs lock/unlock motion at startup; persisted state is an
        # estimate and is reconciled transactionally on the first operation.
        self.init_printer_to_last_tool = config.getboolean(
            'init_printer_to_last_tool', True
        )
        self.purge_on_toolchange = config.getboolean('purge_on_toolchange', True)
        self.saved_position = None
        self.restore_position_on_toolchange_type = 0
        self.last_endstop_query = {}
        self._suspended_heater_modes = {}

        self.tool_lock_gcode_template = gcode_macro.load_template(
            config, 'tool_lock_gcode', ''
        )
        self.tool_unlock_gcode_template = gcode_macro.load_template(
            config, 'tool_unlock_gcode', ''
        )

        handlers = [
            'SAVE_CURRENT_TOOL', 'TOOL_LOCK', 'TOOL_UNLOCK',
            'KTCC_TOOL_DROPOFF_ALL', 'SET_AND_SAVE_FAN_SPEED',
            'TEMPERATURE_WAIT_WITH_TOLERANCE', 'SET_TOOL_TEMPERATURE',
            'KTCC_TOOL_M109', 'SET_GLOBAL_OFFSET', 'SET_TOOL_OFFSET',
            'GET_GLOBAL_OFFSET', 'GET_TOOL_OFFSET',
            'SET_PURGE_ON_TOOLCHANGE', 'SAVE_POSITION',
            'SAVE_CURRENT_POSITION', 'RESTORE_POSITION',
            'KTCC_SET_GCODE_OFFSET_FOR_CURRENT_TOOL', 'KTCC_ENDSTOP_QUERY',
            'KTCC_SET_ALL_TOOL_HEATERS_OFF', 'KTCC_RESUME_ALL_TOOL_HEATERS',
            'KTCC_SET_TOOL_RETRACTION', 'KTCC_GET_TOOL_RETRACTION',
            'KTCC_SAVE_TOOL_OFFSET', 'KTCC_SET_TOOL_PRESSURE_ADVANCE',
            'KTCC_GET_TOOL_PRESSURE_ADVANCE',
            'KTCC_SAVE_TOOL_PRESSURE_ADVANCE',
            'KTCC_APPLY_TOOL_RETRACTION', 'KTCC_TRANSITION_CHECKPOINT',
            'KTCC_RECOVERY_STATUS', 'KTCC_RECOVERY_ABORT',
            'KTCC_RECOVERY_RUN', 'KTCC_RECOVERY_RECONCILE',
            'KTCC_RECOVERY_CONFIRM', 'KTCC_PREHEAT_TOOL',
            'KTCC_CANCEL_PREHEAT', 'KTCC_WAIT_TOOL',
            'KTCC_APPLY_TOOL_PROFILE',
            'KTCC_DUMP_STATS', 'KTCC_RESET_STATS',
            'KTCC_INIT_PRINT_STATS', 'KTCC_DUMP_PRINT_STATS',
        ]
        for command in handlers:
            handler = getattr(self, 'cmd_' + command)
            description = getattr(self, 'cmd_' + command + '_help', None)
            self.gcode.register_command(
                command, handler, when_not_ready=False, desc=description
            )
        self.printer.register_event_handler('klippy:ready', self.handle_ready)

    # Lifecycle and compatibility projection -------------------------

    def handle_ready(self):
        self.reactor.register_callback(
            self._bootup_tasks, self.reactor.monotonic() + self.BOOT_DELAY
        )

    def _bootup_tasks(self, eventtime):
        del eventtime
        self.runtime = compose_runtime(self.printer, self)
        state = self.runtime.toolchange.state
        if state.mode is ChangerMode.IDLE and state.mounted.kind is MountedKind.KNOWN:
            self.runtime.profiles.activate(state.mounted.tool_id)
            self.runtime.profiles.apply_offset(state.mounted.tool_id)
        self._sync_legacy_views()

    def _require_runtime(self, gcmd=None):
        if self.runtime is None:
            if gcmd is not None:
                raise gcmd.error('KTCC runtime is not ready')
            raise self.printer.command_error('KTCC runtime is not ready')
        return self.runtime

    def _require_normal(self, gcmd=None):
        runtime = self._require_runtime(gcmd)
        # An adapter-owned action runs synchronously under Klipper's G-code
        # mutex and may call compatibility commands as part of its recipe.
        # Console/top-level commands still obey the recovery guard.
        if getattr(self, '_action_depth', 0) == 0:
            runtime.toolchange.require_ready()
        return runtime

    def _flush_statistics(self):
        """Persist observations without making printer operation depend on them."""
        if self.runtime is None:
            return
        try:
            self.runtime.statistics.flush()
        except Exception:
            self.log.warning('Unable to persist KTCC statistics.', exc_info=True)

    def _record_statistics(self, callback):
        """Keep a statistics observer failure from changing machine behavior."""
        try:
            callback()
        except Exception:
            self.log.warning('Unable to record KTCC statistics.', exc_info=True)

    def _mounted_tool_id(self):
        if self.runtime is None:
            value = int(self.tool_current)
            return value if value >= 0 else None
        mounted = self.runtime.toolchange.state.mounted
        return mounted.tool_id if mounted.kind is MountedKind.KNOWN else None

    def _transaction_is_active(self):
        return (
            self._action_depth > 0
            or (
                self.runtime is not None
                and self.runtime.toolchange.state.mode is ChangerMode.CHANGING
            )
        )

    def _sync_legacy_views(self):
        if self.runtime is None:
            return
        self.tool_current = str(self.runtime.toolchange.state.legacy_tool_current())
        first = self.runtime.bridge.registry.ids[0]
        global_offset = self.runtime.profiles.offset_state(first).global_offset
        self.global_offset = [global_offset.x, global_offset.y, global_offset.z]
        for spec in self.runtime.bridge.registry:
            tool = self.printer.lookup_object('tool %d' % spec.id.value)
            profile = self.runtime.profiles.effective(spec.id).profile
            offset = self.runtime.profiles.offset_state(spec.id)
            local = offset.session or offset.persisted or offset.configured
            tool.offset = [local.x, local.y, local.z]
            retract = profile.print_retraction
            tool.retract_length = retract.retract_length
            tool.retract_speed = retract.retract_speed
            tool.unretract_extra_length = retract.unretract_extra_length
            tool.unretract_speed = retract.unretract_speed
            tool.zhop = retract.zhop
            pressure = profile.pressure_advance
            tool.pressure_advance = pressure.pressure_advance
            tool.pressure_advance_smooth_time = pressure.smooth_time
            thermal = self.runtime.thermal.snapshot(spec.id)
            tool.heater_active_temp = thermal.active_target
            tool.heater_standby_temp = thermal.standby_target
            tool.heater_state = {
                ThermalMode.OFF: 0,
                ThermalMode.STANDBY: 1,
                ThermalMode.PREHEAT: 2,
                ThermalMode.ACTIVE: 2,
            }[thermal.mode]

    def get_status(self, eventtime=None):
        del eventtime
        self._sync_legacy_views()
        return {
            'global_offset': self.global_offset,
            'tool_current': self.tool_current,
            'saved_fan_speed': self.saved_fan_speed,
            'purge_on_toolchange': self.purge_on_toolchange,
            'restore_position_on_toolchange_type': self.restore_position_on_toolchange_type,
            'saved_position': self.saved_position,
            'last_endstop_query': self.last_endstop_query,
            'changer_mode': (
                None if self.runtime is None
                else self.runtime.toolchange.state.mode.value
            ),
        }

    # Changer and lock commands ---------------------------------------

    def SaveCurrentTool(self, value):
        value = int(value)
        if self._transaction_is_active():
            return
        self._require_runtime().toolchange.declare_current_tool(value)
        self._sync_legacy_views()
        self._flush_statistics()

    def cmd_SAVE_CURRENT_TOOL(self, gcmd):
        value = gcmd.get_int('T', None, minval=-2)
        if value is not None:
            self.SaveCurrentTool(value)

    def cmd_TOOL_LOCK(self, gcmd=None):
        self.ToolLock()

    def ToolLock(self, ignore_locked=False):
        if self._transaction_is_active():
            self.tool_lock_gcode_template.run_gcode_from_command()
            self._record_statistics(
                self._require_runtime().statistics.lock_completed
            )
            return
        self._require_normal()
        if not ignore_locked and int(self.tool_current) != self.TOOL_UNLOCKED:
            self.log.info('TOOL_LOCK is already locked with tool %s.', self.tool_current)
            return
        self.tool_lock_gcode_template.run_gcode_from_command()
        self._record_statistics(self._require_runtime().statistics.lock_completed)
        self.SaveCurrentTool(self.TOOL_UNKNOWN)

    def cmd_TOOL_UNLOCK(self, gcmd=None):
        if not self._transaction_is_active():
            self._require_normal(gcmd)
        self.tool_unlock_gcode_template.run_gcode_from_command()
        self._record_statistics(self._require_runtime().statistics.unlock_completed)
        if not self._transaction_is_active():
            self.SaveCurrentTool(self.TOOL_UNLOCKED)

    def cmd_KTCC_TOOL_DROPOFF_ALL(self, gcmd=None):
        runtime = self._require_normal(gcmd)
        source = self._mounted_tool_id()
        try:
            runtime.toolchange.drop_all()
            if source is not None:
                fan = runtime.bridge.registry[source].fan
                if fan is not None:
                    runtime.machine.set_fan(fan, 0.0)
        finally:
            self._sync_legacy_views()
            self._flush_statistics()

    # Statistics commands --------------------------------------------

    def cmd_KTCC_DUMP_STATS(self, gcmd):
        runtime = self._require_runtime(gcmd)
        self._flush_statistics()
        gcmd.respond_info(format_statistics(runtime.statistics.snapshot()))

    def cmd_KTCC_RESET_STATS(self, gcmd):
        runtime = self._require_runtime(gcmd)
        if gcmd.get('SURE', '').strip().upper() != 'YES':
            gcmd.respond_info('Run KTCC_RESET_STATS SURE=YES to confirm reset.')
            return
        runtime.statistics.reset()
        self._flush_statistics()
        gcmd.respond_info(format_statistics(runtime.statistics.snapshot()))
        gcmd.respond_info('Statistics RESET.')

    def cmd_KTCC_INIT_PRINT_STATS(self, gcmd):
        runtime = self._require_runtime(gcmd)
        runtime.statistics.begin_print()

    def cmd_KTCC_DUMP_PRINT_STATS(self, gcmd):
        runtime = self._require_runtime(gcmd)
        self._flush_statistics()
        gcmd.respond_info(
            format_print_statistics(runtime.statistics.print_snapshot())
        )

    def PrinterIsHomedForToolchange(self, lazy_home_when_parking=0):
        status = self.printer.lookup_object('toolhead').get_status(
            self.reactor.monotonic()
        )
        homed = status['homed_axes'].lower()
        missing = ''.join(axis for axis in 'xyz' if axis not in homed)
        if not missing:
            return True
        if lazy_home_when_parking == 0 or (
            lazy_home_when_parking == 1 and 'z' in missing
        ):
            return False
        self.gcode.run_script_from_command('G28 ' + missing.upper())
        return True

    # Profile commands ------------------------------------------------

    def _tool_id(self, gcmd, *, allow_extruder=False):
        tool_id = gcmd.get_int('TOOL', None)
        if tool_id is None and allow_extruder:
            extruder = gcmd.get('EXTRUDER', None)
            if extruder is not None:
                matches = [
                    spec.id.value for spec in self.runtime.bridge.registry
                    if spec.extruder.value == extruder
                ]
                if len(matches) != 1:
                    raise gcmd.error('invalid extruder specified')
                return matches[0]
        if tool_id is None:
            tool_id = self._mounted_tool_id()
        if tool_id is None or self.runtime.bridge.registry.get(tool_id) is None:
            raise gcmd.error('missing or invalid tool id')
        return tool_id

    def cmd_KTCC_SET_TOOL_RETRACTION(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        current = runtime.profiles.effective(tool_id).profile.print_retraction
        profile = FirmwareRetractionProfile(
            gcmd.get_float('LENGTH', current.retract_length, minval=0),
            gcmd.get_float('SPEED', current.retract_speed, above=0),
            gcmd.get_float('EXTRA', current.unretract_extra_length, minval=0),
            gcmd.get_float('PRIME_SPEED', current.unretract_speed, above=0),
            gcmd.get_float('ZHOP', current.zhop, minval=0),
        )
        runtime.profiles.update_retraction(
            tool_id, profile, mounted_tool=self._mounted_tool_id()
        )
        self._sync_legacy_views()

    def cmd_KTCC_APPLY_TOOL_RETRACTION(self, gcmd):
        runtime = self._require_normal(gcmd)
        mounted = self._mounted_tool_id()
        if mounted is not None:
            runtime.profiles.reapply(mounted)
            self._sync_legacy_views()

    def cmd_KTCC_GET_TOOL_RETRACTION(self, gcmd):
        runtime = self._require_normal(gcmd)
        requested = gcmd.get_int('TOOL', None, minval=0)
        tool_ids = (
            [requested] if requested is not None
            else [spec.id.value for spec in runtime.bridge.registry]
        )
        for tool_id in tool_ids:
            profile = runtime.profiles.effective(tool_id).profile.print_retraction
            gcmd.respond_info(
                'TOOL %d: RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f '
                'UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f ZHOP=%.5f'
                % (tool_id, profile.retract_length, profile.retract_speed,
                   profile.unretract_extra_length, profile.unretract_speed,
                   profile.zhop)
            )

    def cmd_KTCC_SET_TOOL_PRESSURE_ADVANCE(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd, allow_extruder=True)
        current = runtime.profiles.effective(tool_id).profile.pressure_advance
        advance = gcmd.get_float('ADVANCE', None, minval=0)
        smooth = gcmd.get_float('SMOOTH_TIME', None, minval=0, maxval=0.2)
        if advance is None and smooth is None:
            raise gcmd.error('missing ADVANCE or SMOOTH_TIME')
        runtime.profiles.update_pressure_advance(
            tool_id,
            PressureAdvanceProfile(
                current.pressure_advance if advance is None else advance,
                current.smooth_time if smooth is None else smooth,
            ),
        )
        self._sync_legacy_views()

    def cmd_KTCC_GET_TOOL_PRESSURE_ADVANCE(self, gcmd):
        runtime = self._require_normal(gcmd)
        requested = gcmd.get_int('TOOL', None, minval=0)
        tool_ids = (
            [requested] if requested is not None
            else [spec.id.value for spec in runtime.bridge.registry]
        )
        for tool_id in tool_ids:
            profile = runtime.profiles.effective(tool_id).profile.pressure_advance
            gcmd.respond_info(
                'TOOL %d: ADVANCE=%.5f SMOOTH_TIME=%.5f'
                % (tool_id, profile.pressure_advance, profile.smooth_time)
            )

    def cmd_KTCC_SAVE_TOOL_PRESSURE_ADVANCE(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        profile = runtime.profiles.effective(tool_id).profile.pressure_advance
        runtime.profiles.update_pressure_advance(tool_id, profile, persist=True)
        self._sync_legacy_views()

    def cmd_SET_TOOL_OFFSET(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        state = runtime.profiles.offset_state(tool_id)
        local = state.session or state.persisted or state.configured
        values = []
        for axis, original in zip('XYZ', (local.x, local.y, local.z)):
            absolute = gcmd.get_float(axis, None)
            adjust = gcmd.get_float(axis + '_ADJUST', None)
            values.append(
                absolute if absolute is not None
                else original + (0.0 if adjust is None else adjust)
            )
        runtime.profiles.update_session_tool_offset(
            tool_id, Vector3(*values)
        )
        self._sync_legacy_views()

    def cmd_KTCC_SAVE_TOOL_OFFSET(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        state = runtime.profiles.offset_state(tool_id)
        offset = state.session or state.persisted or state.configured
        runtime.profiles.update_tool_offset(
            tool_id, offset, mounted_tool=self._mounted_tool_id()
        )
        self._sync_legacy_views()

    def cmd_GET_TOOL_OFFSET(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        state = runtime.profiles.offset_state(tool_id)
        local = state.session or state.persisted or state.configured
        gcmd.respond_info(
            'TOOL %d: OFFSET=%g, %g, %g' % (tool_id, local.x, local.y, local.z)
        )

    def cmd_SET_GLOBAL_OFFSET(self, gcmd):
        runtime = self._require_normal(gcmd)
        current = Vector3(*self.global_offset)
        values = []
        for axis, original in zip('XYZ', (current.x, current.y, current.z)):
            absolute = gcmd.get_float(axis, None)
            adjust = gcmd.get_float(axis + '_ADJUST', None)
            values.append(
                absolute if absolute is not None
                else original + (0.0 if adjust is None else adjust)
            )
        runtime.profiles.update_global_offset(
            Vector3(*values), mounted_tool=self._mounted_tool_id()
        )
        self._sync_legacy_views()

    def cmd_GET_GLOBAL_OFFSET(self, gcmd):
        gcmd.respond_info('Offset is: %g %g %g' % tuple(self.global_offset))

    # Thermal commands ------------------------------------------------

    def cmd_SET_TOOL_TEMPERATURE(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        standby = gcmd.get_float('STDB_TMP', None, minval=0)
        active = gcmd.get_float('ACTV_TMP', None, minval=0)
        state = gcmd.get_int('CHNG_STATE', None, minval=0, maxval=2)
        idle = gcmd.get_float('STDB_TIMEOUT', None, minval=0.1)
        off = gcmd.get_float('SHTDWN_TIMEOUT', None, minval=0.1)
        mode = None if state is None else {
            0: ThermalMode.OFF,
            1: ThermalMode.STANDBY,
            2: ThermalMode.ACTIVE,
        }[state]
        snapshot = runtime.thermal.set_tool_temperature(
            tool_id,
            active_target=active,
            standby_target=standby,
            mode=mode,
            idle_to_standby_time=idle,
            standby_to_off_time=off,
        )
        if mode is ThermalMode.STANDBY:
            runtime.thermal.schedule_inactivity(tool_id)
        self._sync_legacy_views()
        if all(value is None for value in (standby, active, state, idle, off)):
            gcmd.respond_info(
                'T%d Current Temperature Settings\n Active %s %.1f*C'
                '\n Standby %s %.1f*C'
                % (tool_id,
                   '*' if snapshot.mode is ThermalMode.ACTIVE else ' ',
                   snapshot.active_target,
                   '*' if snapshot.mode is ThermalMode.STANDBY else ' ',
                   snapshot.standby_target)
            )

    def cmd_KTCC_TOOL_M109(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        runtime.thermal.m109(
            tool_id,
            gcmd.get_float('TEMP', minval=0),
            tolerance=gcmd.get_float('TOLERANCE', None, minval=0),
        )
        self._sync_legacy_views()

    def cmd_TEMPERATURE_WAIT_WITH_TOLERANCE(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = gcmd.get_int('TOOL', None, minval=0)
        heater_id = gcmd.get_int('HEATER', None, minval=0)
        if tool_id is not None and heater_id is not None:
            raise gcmd.error('TOOL and HEATER are mutually exclusive')
        runtime.thermal.temperature_wait_with_tolerance(
            tool_id if tool_id is not None else self._mounted_tool_id(),
            heater_id=heater_id,
            tolerance=gcmd.get_float('TOLERANCE', 1.0, minval=0, maxval=50),
        )

    def cmd_KTCC_SET_ALL_TOOL_HEATERS_OFF(self, gcmd):
        runtime = self._require_normal(gcmd)
        self._suspended_heater_modes = {
            spec.id.value: runtime.thermal.snapshot(spec.id).mode
            for spec in runtime.bridge.registry
            if runtime.thermal.snapshot(spec.id).mode is not ThermalMode.OFF
        }
        runtime.thermal.off_all()
        self._sync_legacy_views()

    def cmd_KTCC_RESUME_ALL_TOOL_HEATERS(self, gcmd):
        runtime = self._require_normal(gcmd)
        for desired in (ThermalMode.STANDBY, ThermalMode.PREHEAT, ThermalMode.ACTIVE):
            for tool_id, mode in self._suspended_heater_modes.items():
                if mode is desired:
                    runtime.thermal.set_temperature(tool_id, mode=mode)
        self._sync_legacy_views()

    def cmd_KTCC_PREHEAT_TOOL(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        try:
            target = WaitTarget(gcmd.get('MODE', 'ACTIVE').strip().upper())
        except ValueError as error:
            raise gcmd.error(str(error))
        if target not in (WaitTarget.ACTIVE, WaitTarget.STANDBY):
            raise gcmd.error('MODE must be ACTIVE or STANDBY')
        runtime.thermal.preheat(
            tool_id,
            target=target,
            temperature=gcmd.get_float('TEMP', None, minval=0),
            wait=bool(gcmd.get_int('WAIT', 0, minval=0, maxval=1)),
            tolerance=gcmd.get_float('TOLERANCE', None, minval=0),
            timeout=gcmd.get_float('TIMEOUT', None, above=0),
        )
        self._sync_legacy_views()

    def cmd_KTCC_CANCEL_PREHEAT(self, gcmd):
        runtime = self._require_normal(gcmd)
        runtime.thermal.cancel_preheat(self._tool_id(gcmd))
        self._sync_legacy_views()

    def cmd_KTCC_WAIT_TOOL(self, gcmd):
        runtime = self._require_normal(gcmd)
        try:
            target = WaitTarget(gcmd.get('TARGET', 'CURRENT').strip().upper())
            mode = WaitMode(gcmd.get('MODE', 'HEAT').strip().upper())
        except ValueError as error:
            raise gcmd.error(str(error))
        runtime.thermal.wait(
            self._tool_id(gcmd),
            target=target,
            mode=mode,
            tolerance=gcmd.get_float('TOLERANCE', None, minval=0),
            timeout=gcmd.get_float('TIMEOUT', None, above=0),
            stable_time=gcmd.get_float('STABLE_TIME', None, minval=0),
        )

    def cmd_KTCC_APPLY_TOOL_PROFILE(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = self._tool_id(gcmd)
        if tool_id != self._mounted_tool_id():
            raise gcmd.error('profile can only be applied to the mounted tool')
        runtime.profiles.reapply(tool_id)
        runtime.profiles.apply_offset(tool_id)
        self._sync_legacy_views()

    # Fan, offsets, and position compatibility ------------------------

    def cmd_SET_AND_SAVE_FAN_SPEED(self, gcmd):
        runtime = self._require_normal(gcmd)
        tool_id = gcmd.get_int('P', self._mounted_tool_id(), minval=0)
        if tool_id is None:
            raise gcmd.error('missing tool id')
        speed = gcmd.get_float('S', 1.0, minval=0, maxval=255)
        speed = speed / 255.0 if speed > 1.0 else speed
        fan = runtime.bridge.registry[tool_id].fan
        if fan is not None:
            self.saved_fan_speed = speed
            runtime.machine.set_fan(fan, speed)

    def SetAndSaveFanSpeed(self, tool_id, fanspeed):
        runtime = self._require_normal()
        fan = runtime.bridge.registry[int(tool_id)].fan
        if fan is not None:
            self.saved_fan_speed = float(fanspeed)
            runtime.machine.set_fan(fan, self.saved_fan_speed)

    def SaveFanSpeed(self, fanspeed):
        self.saved_fan_speed = float(fanspeed)

    def cmd_SET_PURGE_ON_TOOLCHANGE(self, gcmd):
        value = gcmd.get('VALUE', 'FALSE').upper()
        self.purge_on_toolchange = value not in ('FALSE', '0')

    def cmd_SAVE_POSITION(self, gcmd):
        self.SavePosition(
            gcmd.get_float('X', None),
            gcmd.get_float('Y', None),
            gcmd.get_float('Z', None),
        )
        restore = gcmd.get_int(
            'RESTORE_POSITION_TYPE', None, minval=0, maxval=2
        )
        if restore is not None:
            self.restore_position_on_toolchange_type = restore

    def SavePosition(self, param_X=None, param_Y=None, param_Z=None):
        if param_X is None or param_Y is None:
            self.restore_position_on_toolchange_type = 0
            return
        self.restore_position_on_toolchange_type = 1 if param_Z is None else 2
        self.saved_position = [param_X, param_Y, param_Z]

    def cmd_SAVE_CURRENT_POSITION(self, gcmd):
        self.SaveCurrentPosition(
            gcmd.get_int('RESTORE_POSITION_TYPE', None, minval=0, maxval=2)
        )

    def SaveCurrentPosition(self, restore_position_type=None):
        if restore_position_type is not None:
            self.restore_position_on_toolchange_type = restore_position_type
        status = self.printer.lookup_object('gcode_move').get_status(
            self.reactor.monotonic()
        )
        position = status.get('gcode_position')
        if not isinstance(position, (list, tuple)) or len(position) < 3:
            raise self.printer.command_error('gcode_position status is unavailable')
        self.saved_position = list(position)

    def cmd_RESTORE_POSITION(self, gcmd):
        restore = gcmd.get_int(
            'RESTORE_POSITION_TYPE', None, minval=0, maxval=2
        )
        if restore is not None:
            self.restore_position_on_toolchange_type = restore
        if self.restore_position_on_toolchange_type == 0:
            return
        if self.saved_position is None:
            raise gcmd.error('No previously saved g-code position.')
        feedrate = gcmd.get_int('F', None, minval=0)
        if feedrate:
            self.gcode.run_script_from_command('G0 F%d' % feedrate)
        self.gcode.run_script_from_command(
            'G0 X%.3f Y%.3f' % (self.saved_position[0], self.saved_position[1])
        )
        if self.restore_position_on_toolchange_type >= 2:
            self.gcode.run_script_from_command('G0 Z%.3f' % self.saved_position[2])

    def cmd_KTCC_SET_GCODE_OFFSET_FOR_CURRENT_TOOL(self, gcmd):
        runtime = self._require_runtime(gcmd)
        tool_id = self._mounted_tool_id()
        if tool_id is None:
            raise gcmd.error("A known mounted tool is required to apply offsets")
        move = gcmd.get_int('MOVE', 0, minval=0, maxval=1)
        state = runtime.profiles.offset_state(tool_id)
        if move == 0:
            runtime.profiles.apply_offset(tool_id)
        else:
            offset = state.effective
            self.gcode.run_script_from_command(
                'SET_GCODE_OFFSET X=%g Y=%g Z=%g MOVE=1'
                % (offset.x, offset.y, offset.z)
            )

    # Transaction checkpoints and recovery ----------------------------

    def cmd_KTCC_TRANSITION_CHECKPOINT(self, gcmd):
        runtime = self._require_runtime(gcmd)
        runtime.toolchange.checkpoint(
            gcmd.get('TRANSITION_ID'),
            gcmd.get('CAPABILITY'),
            gcmd.get('NAME'),
            completed=bool(gcmd.get_int('COMPLETED', 1, minval=0, maxval=1)),
        )
        self._sync_legacy_views()

    def cmd_KTCC_RECOVERY_STATUS(self, gcmd):
        status = self._require_runtime(gcmd).toolchange.recovery_status()
        failure = status.state.failure
        gcmd.respond_info(
            'KTCC recovery failure_id=%s code=%s operation=%s source=%s '
            'target=%s phase=%s action=%s checkpoint=%s mounted_estimate=%s '
            'lock_estimate=%s carriage_estimate=%s ack=%s commands=%s '
            'actions=%s heaters=%s history=%s'
            % (failure.failure_id, failure.error_code, failure.operation.value,
               failure.source_mounted.kind.value, failure.requested_target,
               failure.phase, failure.action_name,
               failure.last_completed_checkpoint,
               failure.mounted_estimate.kind.value,
               failure.lock_estimate.value,
               failure.carriage_estimate.value,
               status.acknowledgment_token,
               ','.join(status.permitted_commands),
               ','.join(status.available_actions),
               ','.join('%s:%g' % item for item in failure.heater_snapshot),
               ','.join(failure.recovery_action_history))
        )

    def cmd_KTCC_RECOVERY_ABORT(self, gcmd):
        heaters = gcmd.get('HEATERS', 'OFF').strip().upper()
        if heaters != 'OFF':
            raise gcmd.error('this machine policy accepts only HEATERS=OFF')
        self._require_runtime(gcmd).toolchange.recovery_abort(
            gcmd.get('FAILURE_ID')
        )
        self._sync_legacy_views()

    def cmd_KTCC_RECOVERY_RUN(self, gcmd):
        self._require_runtime(gcmd).toolchange.run_recovery_action(
            gcmd.get('FAILURE_ID'), gcmd.get('ACTION')
        )
        self._sync_legacy_views()

    def cmd_KTCC_RECOVERY_RECONCILE(self, gcmd):
        runtime = self._require_runtime(gcmd)
        raw = gcmd.get('MOUNTED').strip().upper()
        if raw == 'NONE':
            mounted = MountedState.none()
        elif raw == 'UNKNOWN':
            mounted = MountedState.unknown()
        elif raw.startswith('T') and raw[1:].isdigit():
            mounted = MountedState.known(int(raw[1:]))
        else:
            raise gcmd.error('MOUNTED must be NONE, UNKNOWN, or T<id>')
        try:
            lock = LockState(gcmd.get('LOCK').strip().upper())
            carriage = CarriageEstimate(
                gcmd.get('CARRIAGE', 'UNKNOWN').strip().upper()
            )
        except ValueError as error:
            raise gcmd.error(str(error))
        runtime.toolchange.reconcile(
            gcmd.get('FAILURE_ID'),
            mounted,
            lock,
            carriage=carriage,
            acknowledgment=gcmd.get('ACK'),
            note=gcmd.get('NOTE', None),
        )
        self._sync_legacy_views()

    def cmd_KTCC_RECOVERY_CONFIRM(self, gcmd):
        self._require_runtime(gcmd).toolchange.confirm_recovery(
            gcmd.get('FAILURE_ID')
        )
        self._sync_legacy_views()

    # Endstop helper retained as an explicit hardware adapter ----------

    def cmd_KTCC_ENDSTOP_QUERY(self, gcmd):
        self.query_endstop(
            gcmd.get('ENDSTOP'),
            bool(gcmd.get_int('TRIGGERED', 1, minval=0, maxval=1)),
            gcmd.get_int('ATEMPTS', -1),
        )

    def query_endstop(self, endstop_name, should_be_triggered=True, atempts=-1):
        endstop = None
        for candidate, name in self.printer.lookup_object('query_endstops').endstops:
            if name == endstop_name:
                endstop = candidate
                break
        if endstop is None:
            raise self.printer.command_error("Unknown endstop '%s'" % endstop_name)
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.reactor.monotonic()
        dwell = 1.0 if atempts == -1 else 0.1
        attempts = 0
        triggered = False
        while not self.printer.is_shutdown():
            attempts += 1
            triggered = bool(endstop.query_endstop(toolhead.get_last_move_time()))
            if triggered == should_be_triggered:
                break
            if atempts > 0 and attempts >= atempts:
                break
            eventtime = self.reactor.pause(eventtime + dwell)
        self.last_endstop_query[endstop_name] = triggered


def load_config(config):
    return ToolLock(config)
