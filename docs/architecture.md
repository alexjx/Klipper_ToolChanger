# KTCC Functional Architecture

KTCC is a framework for a Klipper printer with several independent physical
tool heads. This document starts with the capabilities a printer owner uses;
the code organization is described as the implementation of each capability.
That order is intentional: a module is useful only when its ownership of a
user-visible behavior is clear.

## Product boundary

The framework receives three kinds of input:

1. Klipper configuration describing physical tools, resources, and G-code
   actions;
2. existing `save_variables` values describing the last known tool and saved
   per-tool settings; and
3. G-code commands or action templates requesting an operation.

It produces machine effects (heater, extruder, fan, offset, motion, and
configured G-code actions), status/Jinja data, persistence writes, and normal
Klipper log events. It does not edit `~/klipper_config`, move hardware while
the runtime graph is being constructed, or infer a physical result that a
verification action has contradicted.

The central data flow is:

```text
configuration + save_variables
            │
            ▼
physical registry + effective profiles + authoritative changer state
            │
            ▼
G-code facade ──► feature service ──► typed machine port ──► Klipper
                         │
                         └──────────► existing save_variables keys
```

## Capability 1: define independent physical heads

### User-facing behavior

The user can define any number of physical heads, including the four-head
fixture used by this repository. Each `[tool N]` owns its own extruder, heater,
optional fan, dock coordinates, offset, thermal defaults, profile parameters,
and pickup/dropoff/verification actions. A `[toolgroup N]` is only a reusable
source of defaults and actions; it is not a runtime parent and never owns
mounted state.

The configured `heater` is an explicit physical resource. It may be different
from the configured `extruder`, and must still control the named Klipper heater.
Duplicate IDs, invalid resource references, malformed vectors, and enabled
virtual/parent/remap options fail configuration loading with an actionable
error.

### Code that owns the behavior

- `tool.py` and `toolgroup.py` are Klipper config-prefix facades. They read the
  deployed sections and expose the existing status/Jinja shape.
- `ktcc/klipper/composition.py` snapshots those facades at `klippy:ready`; the
  core never retains mutable `ConfigWrapper` objects.
- `ktcc/klipper/config.py` and `ktcc/config/` collect sections without relying
  on include order, normalize old field shapes, merge group defaults, and
  validate references.
- `ktcc/tools.py` contains immutable `ToolSpec` values and the ID-ordered
  `ToolRegistry`. The registry is configuration truth, not live hardware.

Action templates are compiled and bound to stable `(tool, action)` IDs by the
config bridge. The action adapter later receives the selected tool and an
immutable context, so a template cannot accidentally use another head's
profile or resource.

## Capability 2: maintain and apply a per-head process profile

### User-facing behavior

Every physical head can have independent:

- firmware-retraction values (length, speeds, extra length, and Z-hop);
- pressure advance and smoothing time;
- tool-change filament lengths and scalar template variables;
- active/standby thermal policy and wait policy; and
- optional X/Y input-shaper settings.

Offsets have configured, persisted, session, and global components. A profile
is resolved in this order:

```text
configured defaults → persisted per-tool values → session/job overrides
```

Changing an unmounted tool changes its desired profile only. Changing a
mounted tool applies the affected hardware component when that is safe; an
explicit profile-apply command reapplies retraction, pressure advance, motion,
and offset together. Fan speed is also associated with the selected physical
tool: the old tool is stopped and the new tool receives the saved speed during
selection.

### Code that owns the behavior

- `ktcc/profiles/models.py` defines immutable profiles, patches, validation
  limits, and deterministic revisions.
- `ktcc/profiles/service.py` owns configured/persisted/session layers and the
  applied-revision bookkeeping. It is the only place that decides whether an
  effective profile component is current.
- `ktcc/profiles/storage.py` decodes the deployed retraction, PA, and offset
  value shapes; unknown removed-tool profile keys are ignored.
- `ktcc/persistence/service.py` writes the same deployed dictionaries/lists.
- `ktcc/ports.py` describes extrusion and machine effects; the implementations
  in `ktcc/klipper/runtime.py` translate them to public Klipper G-code/status
  APIs.

The root `toollock.py` handlers parse command arguments and delegate. They do
not merge profile layers or directly decide which extruder should be changed.

## Capability 3: control each head's heat, preheat, and waits

### User-facing behavior

Each tool has an independent session with active and standby targets. The user
can set a target, request `ACTIVE`, `STANDBY`, or `OFF`, preheat without
blocking, cancel a preheat, and wait for the current/active/standby target.
The wait policy supports:

- `HEAT`: reach the frozen target without requiring cooling after overshoot;
- `RANGE`: observe the target inside the requested tolerance; and
- `STABLE`: remain inside the tolerance for a continuous duration.

Parked tools can follow `ACTIVE → STANDBY → OFF` timers. Timer generations
make stale callbacks harmless, and a new demand cancels the old timer chain.
Two logical tools may not own one physical heater at the same time. Auxiliary
heaters such as the bed remain explicit adapter resources rather than fake
tools.

The preserved command surface includes `SET_TOOL_TEMPERATURE`, the repository
`M104`/`M109`/`M116`/`M568` macros, and
`TEMPERATURE_WAIT_WITH_TOLERANCE`. New callers can use
`KTCC_PREHEAT_TOOL`, `KTCC_CANCEL_PREHEAT`, and `KTCC_WAIT_TOOL`.
`M109` sets the active target and performs one heat-only wait; `S0` turns the
requested heater off without waiting. `G10`/`G11` remain Klipper firmware
retraction and are not reinterpreted as temperature commands.

### Code that owns the behavior

- `ktcc/thermal.py` owns sessions, heater ownership, demand transitions,
  preheat markers, timer scheduling, and wait policy. It never parses G-code.
- `ktcc/klipper/runtime.py::KlipperThermalPort` observes and sets the actual
  heater objects and implements bounded waits over the reactor.
- `ktcc/klipper/composition.py` resolves every `ToolSpec.heater` directly from
  Klipper's heater registry; it never infers a heater from `extruder`.
- `toollock.py` and the macro files are the command syntax boundary.

## Capability 4: perform a safe physical tool change

### User-facing behavior

`KTCC_Tn` selects a physical head and `KTCC_TOOL_DROPOFF_ALL` leaves the
carriage empty. A normal selection follows this behavioral contract:

1. verify that the changer is idle, the target exists, and the machine is ready
   for motion;
2. if the known source is firmware-retracted, activate its extruder and issue
   the equivalent of `G11`; abort before mechanical risk if it remains
   retracted;
3. request active preheat for the target; any blocking temperature wait is an
   explicit thermal operation;
4. persist the in-progress intent before invoking an opaque mechanical action;
5. run source dropoff, activate the target extruder, apply the target profile,
   run target pickup, and execute configured verification;
6. apply the target offset, activate its heater, update fan ownership, and
   commit the final mounted state.

The same transaction machinery handles dropping the current tool without a
replacement. It does not automatically perform pickup/dropoff motion at
startup. User-provided actions remain the hardware-specific part of the
framework and receive `myself`, `toollock`, and the new immutable context.

### Code that owns the behavior

- `ktcc/toolchange/service.py::ToolChangeService` is the sole normal-operation
  owner. It sequences readiness, G11 repair, preheat, persistence, actions,
  profile/offset application, verification, and commit.
- `ktcc/toolchange/state.py` defines the allowed modes, mounted/lock invariants,
  transition phases, and authenticated checkpoints.
- `ktcc/klipper/runtime.py::KlipperActionPort` runs the precompiled action for
  the correct physical tool and creates a fresh template context for every
  invocation.
- `tool.py` and `toollock.py` preserve the public `KTCC_Tn`, lock/unlock, fan,
  and position commands while delegating all stateful selection work.

The risk boundary is monotonic: once a dropoff or pickup action starts, a
failure is never reported as a successful normal selection.

### Selection-wait boundary

The profile model already carries `select_wait_mode`, tolerance, timeout, and
stable-time values, and `ThermalService.wait()` implements the corresponding
policies. The current transaction implementation intentionally calls
`preheat(..., wait=False)`; it does not silently block a `KTCC_Tn` command on a
temperature wait. Therefore the current guarantee is “selection starts
preheat,” while `KTCC_WAIT_TOOL` and `M109` provide explicit blocking waits.

If the product contract is changed to “selection must wait before pickup,” the
change belongs in `ToolChangeService` as a pre-risk wait stage, followed by
failure/thermal rollback tests. It must not be implemented by adding an
unbounded wait to a G-code facade or by hiding the wait inside a pickup macro.

## Capability 5: recover after interruption or physical uncertainty

### User-facing behavior

After a failure beyond the mechanical risk boundary, KTCC turns configured
heaters off, records an actionable failure, writes `tool_current=-2`, requests
Klipper shutdown, and blocks all normal commands. A restart with `-2`, a
malformed current value, or a current tool removed from configuration also
enters recovery instead of raising a startup `KeyError`.

The operator can inspect `KTCC_RECOVERY_STATUS`, turn heaters off with
`KTCC_RECOVERY_ABORT`, run a registered recovery action, then submit an
acknowledged physical observation with `KTCC_RECOVERY_RECONCILE`. Confirmation
verifies the observation, reapplies the known tool's profile and offset when
needed, and returns to `IDLE`. A recovery action invalidates an earlier
observation, so reconciliation must be repeated.

Transition phases and checkpoints are kept in memory. For changer transaction
state, the existing persistence model stores only the current-tool projection;
profile and offset values remain in their separate existing keys. After a
restart, KTCC can safely require reconciliation but cannot claim to know the
exact interrupted checkpoint.

### Code that owns the behavior

- `ktcc/toolchange/recovery.py` defines bounded failure records, recovery
  estimates, acknowledgment rules, and synchronization transitions.
- `ToolChangeService` records failures and owns the recovery command workflow;
  it is also the only component allowed to clear recovery.
- `ktcc/persistence/codec.py` maps the existing `tool_current` value to startup
  state and reconciles IDs that are no longer present in the registry.
- `toollock.py` exposes the recovery commands and keeps legacy status fields in
  sync with the authoritative service state.

## Capability 6: preserve the Klipper-facing contract

The refactor keeps the established command names, status/Jinja look and feel,
action-template aliases (`myself` and `toollock`), and save-variable keys. This
is not implemented as a parallel legacy runtime: the root facades call the
same registry and feature services used by new commands. Input decoding stays
beside the feature that owns it (`config/`, `profiles/`, or `persistence/`).

The persistent keys remain exactly:

```text
tool_current
ktcc_global_offset
ktcc_tool_offset_<id>
ktcc_tool_retract_<id>
ktcc_tool_pa_info_<id>
ktcc_statistics_swaps
ktcc_statistics_tool<id>
```

KTCC uses Klipper's standard Python logging pipeline through
`ktcc/diagnostics.py`; the former separate `ktcclog` subsystem is intentionally
absent. Virtual tools, physical parents, and runtime remapping are explicit
non-goals and fail configuration when enabled.

## Capability 7: measure toolchanger usage

KTCC records mount/unmount attempts and completions, physical lock/unlock
cycles, time selected, and active/standby heater time for each configured
physical head. Lifetime reports are exposed through `KTCC_DUMP_STATS`; reset
requires `KTCC_RESET_STATS SURE=YES`. Print reports are explicit snapshots:
`PRINT_START` calls `KTCC_INIT_PRINT_STATS`, and `PRINT_END` calls
`KTCC_DUMP_PRINT_STATS`. This avoids guessing which user macro defines a print.

`ktcc/statistics/` owns immutable counters, monotonic interval accounting,
report formatting, and the deployed save-variable codec. `ToolChangeService`
and `ThermalService` publish semantic state transitions through
`StatisticsPort`; they do not format reports or write persistence. The thin
`toollock.py` facade owns G-code parsing and flushes observations only at safe
command boundaries. Observer and persistence failures are logged through
Klipper's standard logger and never turn a successful hardware action into a
failed transaction.

## Capability 8: align and calibrate physical heads

`KTCC_ALIGN_TOOLS` provides a hardware-bound calibration workflow. It requires
homed axes, temporarily drops/picks each requested tool, probes Z and four XY
directions with configurable samples/retries/tolerance, restores stepper
currents and G-code state in `finally` paths, and publishes the measured offset
through `SET_TOOL_OFFSET`. `KTCC_SAVE_TOOL_OFFSET` promotes it to the existing
persistent key.

`alignment.py` remains a Klipper-facing hardware adapter because probing,
stepper current, and toolhead motion are machine-specific. The reusable
profile/offset policy still belongs to `ProfileService`; alignment invokes that
policy through the preserved facade commands rather than maintaining a second
offset store.

## Startup and runtime composition

`toollock.py` registers commands and waits for `klippy:ready`. The composition
root then performs this effect-free setup:

1. snapshot loaded tool/group facades;
2. build and validate the immutable registry and action bindings;
3. load and reconcile the existing save-variable snapshot;
4. decode persisted profile/offset layers;
5. bind heaters, extrusion, machine, scheduler, readiness, persistence,
   verification, shutdown, actions, and diagnostics ports; and
6. retain one `RuntimeServices` graph for all root facades.

Construction never heats, moves, or changes an extruder. Only the ready-time
state reconciliation may reapply a known mounted profile/offset; recovery or
unknown state cannot trigger automatic mechanical motion.

## Extension model

To add a framework capability:

1. state the user-visible command/status behavior and its invariants;
2. add immutable value objects and a feature-owned service next to the closest
   existing capability;
3. express machine effects as a narrow protocol in `ktcc/ports.py`;
4. implement the Klipper boundary in `ktcc/klipper/runtime.py` and wire it only
   in `composition.py`;
5. expose the behavior through a thin facade or action-context field; and
6. add pure service tests, boundary tests, and a public-contract test when an
   existing command or persistence shape is involved.

Do not add policy to a G-code handler, make a feature service import Klipper,
or create a second state owner. A new persistent key or restart guarantee is a
design decision, not an incidental implementation detail.

## Functional verification

The test layout follows the same capability map:

- configuration and four-head isolation: `tests/unit/test_config_profiles.py`,
  `test_klipper_config_bridge.py`, and `tests/characterization/`;
- profile layers and persistence shapes: `test_profile_service.py`,
  `test_profile_storage.py`, and `test_state_repository.py`;
- thermal ownership, timers, preheat, and waits: `test_thermal_service.py` and
  `test_klipper_runtime_adapters.py`;
- change transactions and recovery: `test_toolchange_service.py` and
  `test_changer_recovery.py`;
- public commands, macros, status, and removed features:
  `tests/characterization/test_public_contract.py` and
  `test_removed_features.py`;
- alignment cleanup: `test_alignment_adapter.py`.

The local suite proves software contracts and deterministic effect ordering. It
does not prove the target printer's pickup/dropoff geometry, heater wiring,
probe repeatability, or shutdown behavior; those remain controlled hardware
acceptance tests.
