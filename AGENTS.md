# Repository Guidelines

## Project Structure & Module Organization

`ktcc/` is organized by toolchanger feature. Physical definitions live in
`ktcc/tools.py`; per-head parameters in `ktcc/profiles/`; transactions and
recovery in `ktcc/toolchange/`; thermal behavior in `ktcc/thermal.py`; and
Klipper-specific effects in `ktcc/klipper/`. Existing config/save-variable
shapes are decoded beside their owner (`config/`, `profiles/`, or
`persistence/`); they are not a separate runtime layer.
The root modules (`tool.py`, `toolgroup.py`, `toollock.py`, and `alignment.py`)
are Klipper entry-point facades and should remain thin. `ktcclog.py` is a
configuration-only rollback adapter and must not regain runtime behavior.
Printer macros live in `klipper_macros/`; supported sample configurations live
in `config/`. Tests are
split into `tests/unit/`, `tests/integration/`, and `tests/characterization/`,
with reusable printer data under `tests/fixtures/`.

## Build, Test, and Development Commands

Use the uv-managed environment; do not invoke system Python directly.

- `uv sync --group dev` installs the development environment.
- `uv run pytest -q` runs the complete Python 3.10+ test suite.
- `uv run pytest -q tests/unit/test_thermal_service.py` runs one focused module.
- `uv run python -m py_compile tool.py toolgroup.py toollock.py alignment.py $(find ktcc -name '*.py')` checks production syntax.
- `uv lock --check`, `bash -n install.sh`, and `git diff --check` validate the lockfile, installer, and patch formatting.

`install.sh` changes a Klipper installation. Do not run it during ordinary test
work or against a live printer without explicit approval.

## Architecture and Coding Style

Use four-space indentation, descriptive `snake_case` functions, `PascalCase`
types, and uppercase enum/constants. Add type hints to new core code. Prefer
immutable state/value objects and dependency injection through `ktcc/ports.py`; avoid
accessing private Klipper fields when a public status or G-code interface exists.
Preserve existing public G-code, configuration, Jinja, and persistence contracts
unless a breaking change is explicitly approved and documented.

## Testing Guidelines

Pytest uses strict markers and strict xfails. Name files `test_*.py` and tests
`test_<observable_behavior>`. Pure policy belongs in unit tests; adapter/API
contracts belong in integration tests; compatibility surfaces belong in
characterization tests. Cover success, invalid input, persistence ordering, and
failure recovery. Hardware motion and heater behavior must be reported as
unverified unless tested on a printer.

## Configuration Safety

Treat `~/klipper` and `~/klipper_config` as read-only reference trees. Never
commit machine-specific pins, serial identifiers, saved variables, logs, or
credentials. Use sanitized fixtures and repo-owned examples instead.
