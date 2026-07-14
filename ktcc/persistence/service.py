"""Toolchanger persistence repository.

This is the only feature component which understands save-variable keys.
The raw Klipper boundary is deliberately reduced to :class:`VariableStorePort`;
there are no printer, G-code, or Klipper imports here.
"""

from __future__ import annotations

from typing import Optional

from .codec import (
    CURRENT_TOOL_KEY,
    GLOBAL_OFFSET_KEY,
    PersistenceSnapshot,
    decode_variables,
    tool_current_for_state,
)
from ktcc.toolchange.state import ChangerState
from ktcc.tools import ToolId, Vector3
from ktcc.profiles.models import FirmwareRetractionProfile, PressureAdvanceProfile
from ktcc.ports import VariableStorePort


_TOOL_OFFSET_PREFIX = "ktcc_tool_offset_"
_TOOL_RETRACTION_PREFIX = "ktcc_tool_retract_"
_TOOL_PA_PREFIX = "ktcc_tool_pa_info_"


class StateRepository:
    """Translate typed state/profile operations to raw save-variable writes.

    The repository is intentionally stateless.  In particular, it never caches
    a requested write as though it had succeeded; the variable store remains
    the only persistence truth after an exception.
    """

    def __init__(self, store: VariableStorePort) -> None:
        if not isinstance(store, VariableStorePort):
            raise TypeError("store must implement VariableStorePort")
        self._store = store

    def load(self) -> PersistenceSnapshot:
        """Read and decode one complete persistence snapshot."""
        return decode_variables(self._store.read_variables())

    def load_state(self) -> Optional[ChangerState]:
        """Implement StatePersistencePort while retaining the richer load API."""
        return self.load().state

    def save_state(self, state: ChangerState) -> None:
        if not isinstance(state, ChangerState):
            raise TypeError("state must be a ChangerState")
        self._store.write_variable(CURRENT_TOOL_KEY, tool_current_for_state(state))

    def mirror_tool_current(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or value < -2:
            raise ValueError("tool_current must be an integer at least -2")
        self._store.write_variable(CURRENT_TOOL_KEY, value)

    def save_retraction(
        self, tool_id: ToolId, profile: FirmwareRetractionProfile
    ) -> None:
        self._require_tool_id(tool_id)
        if not isinstance(profile, FirmwareRetractionProfile):
            raise TypeError("profile must be a FirmwareRetractionProfile")
        # Keep the exact dictionary shape written by the deployed implementation.
        # zhop is not part of Klipper's firmware-retraction executor or the
        # deployed persistence value.
        value = {
            "retract_length": profile.retract_length,
            "retract_speed": profile.retract_speed,
            "unretract_extra_length": profile.unretract_extra_length,
            "unretract_speed": profile.unretract_speed,
        }
        self._store.write_variable(f"{_TOOL_RETRACTION_PREFIX}{tool_id.value}", value)

    def save_pressure_advance(
        self, tool_id: ToolId, profile: PressureAdvanceProfile
    ) -> None:
        self._require_tool_id(tool_id)
        if not isinstance(profile, PressureAdvanceProfile):
            raise TypeError("profile must be a PressureAdvanceProfile")
        value = {
            "pressure_advance": profile.pressure_advance,
            "smooth_time": profile.smooth_time,
        }
        self._store.write_variable(f"{_TOOL_PA_PREFIX}{tool_id.value}", value)

    def save_tool_offset(self, tool_id: ToolId, offset: Vector3) -> None:
        self._require_tool_id(tool_id)
        self._require_offset(offset)
        self._store.write_variable(
            f"{_TOOL_OFFSET_PREFIX}{tool_id.value}",
            [offset.x, offset.y, offset.z],
        )

    def save_global_offset(self, offset: Vector3) -> None:
        self._require_offset(offset)
        self._store.write_variable(GLOBAL_OFFSET_KEY, [offset.x, offset.y, offset.z])

    @staticmethod
    def _require_tool_id(tool_id: ToolId) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")

    @staticmethod
    def _require_offset(offset: Vector3) -> None:
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
