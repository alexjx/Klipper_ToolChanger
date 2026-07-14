# Configuration reference

This repository implements a framework for independent physical tools. A
`[toolgroup N]` is a source of shared defaults and actions; it does not create a
runtime parent or a second tool-changing layer.

See `example_simple/` for a compact two-tool setup and `example_complex/` for a
four-tool setup with distinct retraction, pressure advance and thermal policy.

## Configuration requirements

- Configure `[save_variables]` before `[toollock]` for persistent tool state and
  profile overrides.
- Configure `[input_shaper]` when per-tool input-shaper values are used.
- Define every referenced Klipper extruder and fan in the printer config.

## `[toollock]`

The lock section owns the physical carriage lock/unlock actions:

```ini
[toollock]
purge_on_toolchange: True
init_printer_to_last_tool: True
tool_lock_gcode:
  # Hardware-specific lock action.
tool_unlock_gcode:
  # Hardware-specific unlock action.
```

Lock and unlock actions may call other macros. They must either finish
successfully or raise a Klipper error; the tool-change transaction uses that
result to decide whether it may commit the new current-tool state.

## `[toolgroup N]`

A group provides configuration defaults to multiple physical tools. A value in
`[tool N]` overrides the group value.

```ini
[toolgroup 0]
pickup_gcode:
  SUB_TOOL_PICKUP T={myself.name}
dropoff_gcode:
  SUB_TOOL_DROPOFF T={myself.name}
idle_to_standby_time: 30
idle_to_powerdown_time: 600
lazy_home_when_parking: 0
meltzonelength: 18
select_wait_mode: HEAT
select_wait_tolerance: 1
select_wait_timeout: 900
select_wait_stable_time: 0
```

`select_wait_mode` accepts `HEAT`, `RANGE`, or `STABLE`:

- `HEAT` waits only while heating toward the target.
- `RANGE` waits until the temperature is inside the configured tolerance.
- `STABLE` additionally requires it to remain inside the range for
  `select_wait_stable_time` seconds.

## `[tool N]`

Every tool section describes one physical tool. `tool_group` is required. Define
the referenced group before its tools so the current Klipper facade can resolve
the shared configuration while loading sections.

```ini
[tool 0]
tool_group: 0

# Klipper resources. By default the heater has the extruder's name.
extruder: extruder
heater: extruder
fan: partfan_t0

# Physical geometry, always X,Y,Z.
zone: 550,5,0
park: 598,5,0
offset: 0,0,0

# Firmware retraction profile owned by this tool.
retract_length: 0.8
retract_speed: 35
unretract_extra_length: 0
unretract_speed: 35
zhop: 0

# Pressure advance profile owned by this tool.
pressure_advance: 0.025
pressure_advance_smooth_time: 0.04

# Tool-change filament distances. If omitted, both default to meltzonelength.
meltzonelength: 18
unload_length: 18
prime_length: 18

# Thermal defaults, timers, and select-time wait policy.
heater_active_temp: 220
heater_standby_temp: 170
idle_to_standby_time: 30
idle_to_powerdown_time: 600
select_wait_mode: HEAT
select_wait_tolerance: 1
select_wait_timeout: 900
select_wait_stable_time: 0

# Optional per-tool motion profile.
shaper_freq_x: 0
shaper_freq_y: 0
shaper_type_x: mzv
shaper_type_y: mzv
shaper_damping_ratio_x: 0.1
shaper_damping_ratio_y: 0.1

# Optional overrides for the group's physical actions and verification hooks.
pickup_gcode:
dropoff_gcode:
verify_mounted_gcode:
verify_unmounted_gcode:
recovery_gcode:
```

Tools remain independently configurable: four tools may use four different
extruders, fans, offsets, thermal defaults, retraction profiles and pressure
advance profiles while sharing the same physical dock choreography.

## Deliberately unsupported configuration

Virtual tools, physical-parent inheritance and runtime tool remapping were
removed. Non-inert declarations of their former fields are configuration
errors. Remove those fields and model each selectable head as its own physical
`[tool N]`.

Runtime diagnostics use Klipper's standard Python logging and appear in
`klippy.log`; no separate logging section is configured.
