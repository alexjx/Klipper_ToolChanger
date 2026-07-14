# KTCC: physical toolchanger framework for Klipper

KTCC is a Klipper extension for multi-head toolchangers. Each `[tool N]`
represents one independent physical head with its own extruder, heater, fan,
offset, firmware-retraction profile, pressure advance, thermal targets, and
temperature-wait policy. Hardware-specific pickup, dropoff, lock, unlock, and
recovery motion remains user-defined G-code.

This repository originated from Andrei Ignat's open-source Klipper ToolChanger
project and retains its GPLv3 source-file licensing.

## Supported model

- Any number of independently configured physical tools; the included
  reference fixture and complex example use four.
- `[toolgroup N]` as a shared configuration/action source. A group is not a
  runtime parent and does not own changer state.
- Existing `KTCC_Tn`, temperature, fan, offset, retraction, PA, position,
  lock/unlock, and alignment command names.
- Existing action templates using `myself` and `toollock`, plus a versioned
  immutable action context for new integrations.
- Existing `save_variables` keys, kept as the only persistent representation.

Virtual tools, `physical_parent`, and runtime remapping are intentionally not
supported. Non-inert legacy declarations fail configuration loading instead of
being silently ignored.

## Architecture

The root Klipper modules are thin facades. Code under `ktcc/` is organized by
toolchanger feature instead of generic architectural layers:

```text
ktcc/
├── tools.py             physical tool definitions and registry
├── profiles/            per-tool parameters and profile application
├── toolchange/          transaction state, recovery, and execution
├── thermal.py           heater ownership, preheat, and waiting
├── persistence/         state repository and current-key codec
├── ports.py             typed machine integration contracts
├── klipper/             Klipper implementations and composition
└── config/              typed config resolution and old-field normalization
```

Compatibility is an external behavior, not a parallel runtime layer. The
existing root facades keep the same command/status/Jinja look and feel while
delegating to the same registry and services used by new functionality. The
existing save-variable and config shapes are read directly by their owning
features.

There is one authoritative `ToolChangeService`, one profile service, and one
thermal controller per physical heater. Machine actions, persistence, timers,
logging, and shutdown are injected ports, so the core is testable without
printer hardware.

See [`docs/architecture.md`](docs/architecture.md) for the functional
capabilities, their observable contracts, the code that owns them, runtime
state flow, and extension points.

## Installation

The installer links the four Klipper entry modules and the `ktcc` package into
`klippy/extras`, then restarts Klipper:

```bash
git clone https://github.com/alexjx/Klipper_ToolChanger.git \
  ~/Klipper_ToolChanger
~/Klipper_ToolChanger/install.sh
```

Use `-k` when Klipper is not at `~/klipper`:

```bash
~/Klipper_ToolChanger/install.sh -k /path/to/klipper
```

For Moonraker update management:

```ini
[update_manager client klipper_toolchanger]
type: git_repo
path: ~/Klipper_ToolChanger
origin: https://github.com/alexjx/Klipper_ToolChanger.git
install_script: install.sh
is_system_service: False
```

For a manual installation, copy/link `alignment.py`, `tool.py`, `toolgroup.py`,
`toollock.py`, and the complete `ktcc/` directory into `klippy/extras`. Copying
only the root Python files is insufficient.

## Configuration

Start with:

- [`config/example_simple`](config/example_simple) for two physical tools;
- [`config/example_complex`](config/example_complex) for four independent
  physical tools;
- [`config/readme.md`](config/readme.md) for every supported field.

Minimal shape:

```ini
[save_variables]
filename: ~/variables.cfg

[toollock]
tool_lock_gcode:
  # Hardware-specific lock action
tool_unlock_gcode:
  # Hardware-specific unlock action

[toolgroup 0]
pickup_gcode:
  SUB_TOOL_PICKUP T={myself.name}
dropoff_gcode:
  SUB_TOOL_DROPOFF T={myself.name}

[tool 0]
tool_group: 0
extruder: extruder
fan: partfan_t0
zone: 550,5,0
park: 598,5,0
offset: 0,0,0
retract_length: 0.8
retract_speed: 35
unretract_extra_length: 0
unretract_speed: 35
pressure_advance: 0.025
pressure_advance_smooth_time: 0.04
heater_active_temp: 220
heater_standby_temp: 170
```

`init_printer_to_last_tool` is still accepted for configuration compatibility,
but startup never performs automatic lock/unlock motion. Persisted state is an
estimate; interrupted or malformed state enters recovery conservatively.

## Per-tool profile lifecycle

Each tool resolves an immutable effective profile in this order:

1. configured defaults from `[toolgroup]` and `[tool]`;
2. persisted retraction, PA, and local offset overrides;
3. non-persistent job/session overrides.

Retraction and PA are applied when their tool becomes active. Editing the
mounted tool repairs the relevant hardware component immediately; editing an
unmounted tool stores its desired value without changing another extruder.
Optional input-shaper values are applied as part of the same activation.

Useful commands:

- `KTCC_SET_TOOL_RETRACTION TOOL=<id> LENGTH=<mm> SPEED=<mm/s>
  EXTRA=<mm> PRIME_SPEED=<mm/s> [ZHOP=<mm>]` — update and persist that tool's
  firmware-retraction profile.
- `KTCC_GET_TOOL_RETRACTION [TOOL=<id>]`.
- `KTCC_SET_TOOL_PRESSURE_ADVANCE [TOOL=<id>|EXTRUDER=<name>]
  ADVANCE=<value> [SMOOTH_TIME=<seconds>]` — session update.
- `KTCC_SAVE_TOOL_PRESSURE_ADVANCE TOOL=<id>` — promote current PA to the
  persisted layer.
- `KTCC_APPLY_TOOL_PROFILE [TOOL=<mounted id>]` — reapply retraction, PA,
  optional motion profile, and offset to the mounted tool.
- `SET_TOOL_OFFSET ...` creates a session offset;
  `KTCC_SAVE_TOOL_OFFSET TOOL=<id>` persists it.

## Thermal, preheat, and waiting

Every physical heater has one controller and explicit owner. Session active and
standby targets remain independent for every tool and are never persisted.
Parked tools follow `ACTIVE -> STANDBY -> OFF` using generation-safe timers.

Compatibility commands remain available:

- `SET_TOOL_TEMPERATURE TOOL=<id> [ACTV_TMP=<C>] [STDB_TMP=<C>]
  [CHNG_STATE=0|1|2] [STDB_TIMEOUT=<s>] [SHTDWN_TIMEOUT=<s>]`;
- `TEMPERATURE_WAIT_WITH_TOLERANCE [TOOL=<id>|HEATER=<id>]
  [TOLERANCE=<C>]`;
- the repo `M104`, `M109`, `M116`, and `M568` macros.

`M109 Tn Sx` atomically sets the active target and performs a heat-only wait
against a frozen target. `M109 S0` turns the heater off and does not wait.
`G10` is deliberately left to Klipper firmware retraction; use `M568` for the
older RepRap-style tool-temperature operation.

Explicit framework commands:

- `KTCC_PREHEAT_TOOL TOOL=<id> MODE=ACTIVE|STANDBY [TEMP=<C>]
  [WAIT=0|1] [TOLERANCE=<C>] [TIMEOUT=<s>]`;
- `KTCC_CANCEL_PREHEAT TOOL=<id>`;
- `KTCC_WAIT_TOOL TOOL=<id> TARGET=CURRENT|ACTIVE|STANDBY
  MODE=HEAT|RANGE|STABLE [TOLERANCE=<C>] [TIMEOUT=<s>]
  [STABLE_TIME=<s>]`.

`HEAT` never waits for cooling after overshoot, `RANGE` accepts one observation
inside the tolerance, and `STABLE` requires a continuous in-range duration. A
target changed by another actor aborts a frozen-target wait.

## Tool-change transaction and recovery

`KTCC_Tn` and `KTCC_TOOL_DROPOFF_ALL` execute durable transactions:

1. validate state and homing;
2. automatically issue `G11` if the known source is firmware-retracted;
3. preheat the target;
4. persist `CHANGING` before the first opaque mechanical action;
5. drop, activate/apply the target profile, pick up, verify when configured,
   apply offset, and commit;
6. write `tool_current=-2` before mechanical risk, then write the final tool ID
   (or `-1`) after commit. No new persistent state schema is introduced.

An error after mechanical risk turns all tool heaters off, writes the existing
`tool_current=-2` recovery sentinel, and requests Klipper shutdown. A restart
with `tool_current=-2` also enters recovery. Normal KTCC commands are blocked
until recovery completes. Detailed transaction phases and checkpoints are
runtime-only because the persistent model is intentionally unchanged.

Recovery commands:

- `KTCC_RECOVERY_STATUS` — failure/action/checkpoint/estimate data and the
  required acknowledgment token;
- `KTCC_RECOVERY_ABORT FAILURE_ID=<id> [HEATERS=OFF]` — no reverse motion;
- `KTCC_RECOVERY_RUN FAILURE_ID=<id> ACTION=<configured action id>`;
- `KTCC_RECOVERY_RECONCILE FAILURE_ID=<id> MOUNTED=NONE|UNKNOWN|T<id>
  LOCK=LOCKED|UNLOCKED|UNKNOWN CARRIAGE=<estimate> ACK=<token> [NOTE=<text>]`;
- `KTCC_RECOVERY_CONFIRM FAILURE_ID=<id>` — verify/reapply and return to idle.

Action macros may durably report sub-steps with:

```text
KTCC_TRANSITION_CHECKPOINT
  TRANSITION_ID={transition.id}
  CAPABILITY={transition.checkpoint_capability}
  NAME=<checkpoint>
  COMPLETED=0|1
```

The capability exists only in the active action context; console commands
cannot forge a checkpoint.

## Action template context

Existing templates retain:

- `myself` — the legacy tool status mapping (`name`, `zone`, `park`, `offset`,
  thermal/profile fields, and so on);
- `toollock` — current compatibility status.

New templates also receive immutable `tool`, `thermal`, `changer`, and
`transition` mappings. `tool.profile` is the effective per-tool profile, not
only the configured defaults.

## Logging and tool-change statistics

KTCC uses Klipper's standard Python logging pipeline and writes contextual
events to normal `klippy.log`. The former `[ktcclog]` file/thread/rotation
behavior and log-level commands were removed. A no-op `[ktcclog]` configuration
adapter remains so the same printer configuration can boot when switching
between old and refactored revisions; its options have no effect.

Tool-change statistics remain available as an independent feature. They count
physical mount/unmount attempts and completions, lock cycles, selected time,
and active/standby heater time. `KTCC_DUMP_STATS` reports lifetime totals and
`KTCC_RESET_STATS SURE=YES` clears them. To report one print, call
`KTCC_INIT_PRINT_STATS` from `PRINT_START` and `KTCC_DUMP_PRINT_STATS` from
`PRINT_END`. Totals use the existing `ktcc_statistics_*` save-variable keys;
no `[ktcclog]` section is required by the new runtime.

The former repo macro that redirected `G10` to `M568` was also removed because
it conflicts with Klipper's `G10`/`G11` firmware-retraction pair. Slicer
temperature snippets must use `M568` explicitly.

External configuration migration checklist (this repository does not edit
`~/klipper_config`):

1. optionally remove `[ktcclog]` and its inactive options; retain statistics calls if desired;
2. replace tool-temperature forms such as `G10 P...` with `M568 P...`;
3. use this repo's `M109.cfg`, or route an existing `M109 S...` macro to
   `KTCC_TOOL_M109 ... TEMP=...`; the old sequence that only changed
   `ACTV_TMP` and then called a tolerance wait did not activate an OFF heater.

## Development and verification

The repository uses `uv` and tests without printer hardware:

```bash
uv sync --locked
uv run pytest -q
uv run python -m py_compile tool.py toolgroup.py toollock.py alignment.py
bash -n install.sh
git diff --check
```

The local unit suite covers model invariants, persistence, four-tool profile
isolation, thermal timers/waits, transaction failure boundaries, recovery,
Klipper integration, facade composition, removed features, and alignment
cleanup. Real pickup/dropoff motion and probing still require careful
validation on the target machine.
