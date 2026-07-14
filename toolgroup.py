# Copyright (C) 2023 Andrei Ignat <andrei@ignat.se>
# SPDX-License-Identifier: GPL-3.0-only

"""Shared physical-tool configuration facade for Klipper.

Tool groups no longer model inheritance or runtime identity.  They remain a
small compatibility surface for sharing pickup/dropoff actions and defaults
between independently configured physical tools.
"""


class ToolGroup:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        try:
            _, raw_name = config.get_name().split(' ', 1)
            self.name = int(raw_name)
        except ValueError:
            raise config.error(
                "Name of section '%s' contains illegal characters. "
                "Use only integer ToolGroup number." % config.get_name()
            )

        self._reject_removed_features(config)
        self.lazy_home_when_parking = config.get(
            'lazy_home_when_parking', 0
        )
        self.pickup_gcode = config.get('pickup_gcode', '')
        self.dropoff_gcode = config.get('dropoff_gcode', '')
        self.meltzonelength = config.get('meltzonelength', 0)
        self.idle_to_standby_time = config.getfloat(
            'idle_to_standby_time', 30, minval=0.1
        )
        self.idle_to_powerdown_time = config.getfloat(
            'idle_to_powerdown_time', 600, minval=0.1
        )

    @staticmethod
    def _reject_removed_features(config):
        if config.getboolean('is_virtual', False):
            raise config.error(
                "Section '%s' enables virtual tools, which are no longer supported."
                % config.get_name()
            )
        for option in ('physical_parent', 'physical_parent_id'):
            parent = config.getint(option, None)
            if parent not in (None, -1):
                raise config.error(
                    "Section '%s' sets %s, which is no longer supported."
                    % (config.get_name(), option)
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

    def get_config(self, config_param, default=None):
        return self.config.get(config_param, default)

    def get_status(self, eventtime=None):
        del eventtime
        return {
            'lazy_home_when_parking': self.lazy_home_when_parking,
            'meltzonelength': self.meltzonelength,
            'idle_to_standby_time': self.idle_to_standby_time,
            'idle_to_powerdown_time': self.idle_to_powerdown_time,
        }


def load_config_prefix(config):
    return ToolGroup(config)
