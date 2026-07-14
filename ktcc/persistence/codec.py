"""Codec for the repository's existing ``save_variables`` representation.

The persisted model intentionally remains the deployed one: ``tool_current``,
the global offset, and one profile value per tool. Transaction phases and
failure details are runtime state; ``tool_current=-2`` is the durable recovery
sentinel.
"""

from dataclasses import dataclass, replace
import re
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple

from ktcc.toolchange.recovery import FailureCode, synthetic_failure
from ktcc.toolchange.state import (
    ChangerMode,
    ChangerState,
    LockState,
    MountedKind,
    MountedState,
)
from ktcc.tools import ToolRegistry


CURRENT_TOOL_KEY = "tool_current"
GLOBAL_OFFSET_KEY = "ktcc_global_offset"
_PROFILE_KEY = re.compile(r"^ktcc_tool_(offset|retract|pa_info)_([0-9]+)$")


@dataclass(frozen=True)
class PersistenceSnapshot:
    """One decoded save-variable snapshot and its in-memory changer state."""

    state: ChangerState
    profile_values: Mapping[str, Any]
    warnings: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "profile_values", MappingProxyType(dict(self.profile_values))
        )


def decode_variables(values: Mapping[str, Any]) -> PersistenceSnapshot:
    """Decode the existing keys without introducing a second state schema."""

    if not isinstance(values, Mapping):
        raise TypeError("save_variables values must be a mapping")
    raw = values.get(CURRENT_TOOL_KEY, -1)
    current = _parse_current(raw)
    if current is None or current < -2:
        state = _recovery_state(
            FailureCode.MALFORMED_PERSISTENCE,
            "tool_current is malformed: %r" % (raw,),
        )
        warning = "malformed tool_current requires reconciliation"
    elif current == -2:
        state = _recovery_state(
            FailureCode.PERSISTED_RECOVERY,
            "tool_current recovery sentinel requires reconciliation",
        )
        warning = "tool_current=-2 requires reconciliation"
    elif current == -1:
        state = ChangerState.idle_without_tool()
        warning = "tool_current=-1 restored IDLE without a mounted tool"
    else:
        state = ChangerState.idle_with_tool(current)
        warning = "tool_current restored IDLE with the recorded tool locked"
    return PersistenceSnapshot(
        state=state,
        profile_values=_extract_profile_values(values),
        warnings=(warning,),
    )


def tool_current_for_state(state: ChangerState) -> int:
    """Project runtime state to the existing durable current-tool key."""

    if not isinstance(state, ChangerState):
        raise TypeError("state must be a ChangerState")
    if state.mode is not ChangerMode.IDLE:
        return -2
    return state.legacy_tool_current()


def reconcile_snapshot(
    snapshot: PersistenceSnapshot, registry: ToolRegistry
) -> PersistenceSnapshot:
    """Turn a removed persisted tool into safe recovery instead of a lookup error."""

    if not isinstance(snapshot, PersistenceSnapshot):
        raise TypeError("snapshot must be a PersistenceSnapshot")
    if not isinstance(registry, ToolRegistry):
        raise TypeError("registry must be a ToolRegistry")
    mounted = snapshot.state.mounted
    if (
        snapshot.state.mode is ChangerMode.IDLE
        and mounted.kind is MountedKind.KNOWN
        and registry.get(mounted.tool_id) is None
    ):
        failure = synthetic_failure(
            FailureCode.UNKNOWN_PERSISTED_TOOL,
            "tool_current refers to tool %s, which is not configured"
            % mounted.tool_id,
            revision=snapshot.state.revision,
            mounted=mounted,
            lock=snapshot.state.lock,
        )
        state = ChangerState(
            schema_version=2,
            revision=snapshot.state.revision + 1,
            mode=ChangerMode.RECOVERY_REQUIRED,
            mounted=MountedState.unknown(),
            lock=LockState.UNKNOWN,
            selected=None,
            failure=failure,
        )
        return replace(
            snapshot,
            state=state,
            warnings=snapshot.warnings
            + ("tool_current refers to an unknown configured tool",),
        )
    return snapshot


def _extract_profile_values(values: Mapping[str, Any]) -> dict[str, Any]:
    result = {}
    if GLOBAL_OFFSET_KEY in values:
        result[GLOBAL_OFFSET_KEY] = values[GLOBAL_OFFSET_KEY]
    for key, value in values.items():
        if isinstance(key, str) and _PROFILE_KEY.fullmatch(key):
            result[key] = value
    return result


def _parse_current(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        try:
            return int(value.strip())
        except ValueError:
            return None
    return None


def _recovery_state(code: FailureCode, message: str) -> ChangerState:
    failure = synthetic_failure(code, message)
    return ChangerState(
        schema_version=2,
        revision=0,
        mode=ChangerMode.RECOVERY_REQUIRED,
        mounted=MountedState.unknown(),
        lock=LockState.UNKNOWN,
        selected=None,
        failure=failure,
    )
