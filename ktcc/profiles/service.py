"""Per-tool profile lifecycle and hardware orchestration.

The service owns mutable profile layers and applied revisions.  Values crossing
the boundary remain immutable value objects; persistence and Klipper effects
are delegated to explicit ports.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Mapping, Optional

from ktcc.tools import ToolId, ToolRegistry, Vector3
from ktcc.profiles.models import (
    EffectiveToolProfile,
    FirmwareRetractionPatch,
    FirmwareRetractionProfile,
    PressureAdvancePatch,
    PressureAdvanceProfile,
    ProfileLayers,
    ToolProfile,
    ToolProfilePatch,
)
from ktcc.ports import ExtrusionProfilePort, MachinePort, ProfilePersistencePort


@dataclass(frozen=True)
class AppliedToolProfile:
    """Desired profile plus independently observable hardware revisions."""

    tool_id: ToolId
    desired: EffectiveToolProfile
    applied_revision: Optional[str]
    pressure_advance_revision: Optional[str]
    firmware_retraction_revision: Optional[str]
    motion_revision: Optional[str]

    @property
    def is_current(self) -> bool:
        return self.applied_revision == self.desired.revision


@dataclass(frozen=True)
class OffsetState:
    tool_id: ToolId
    configured: Vector3
    persisted: Optional[Vector3]
    session: Optional[Vector3]
    global_offset: Vector3
    effective: Vector3
    applied: Optional[Vector3]

    @property
    def is_current(self) -> bool:
        return self.applied == self.effective


def _retraction_patch(profile: FirmwareRetractionProfile) -> FirmwareRetractionPatch:
    return FirmwareRetractionPatch(
        retract_length=profile.retract_length,
        retract_speed=profile.retract_speed,
        unretract_extra_length=profile.unretract_extra_length,
        unretract_speed=profile.unretract_speed,
        zhop=profile.zhop,
    )


def _pa_patch(profile: PressureAdvanceProfile) -> PressureAdvancePatch:
    return PressureAdvancePatch(
        pressure_advance=profile.pressure_advance,
        smooth_time=profile.smooth_time,
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return Vector3(left.x + right.x, left.y + right.y, left.z + right.z)


class ProfileService:
    """Resolve, persist, apply, and report independent physical-tool profiles."""

    def __init__(
        self,
        registry: ToolRegistry,
        extrusion: ExtrusionProfilePort,
        machine: MachinePort,
        persistence: ProfilePersistencePort,
        *,
        persisted_patches: Optional[Mapping[ToolId, ToolProfilePatch]] = None,
        persisted_offsets: Optional[Mapping[ToolId, Vector3]] = None,
        global_offset: Vector3 = Vector3(0.0, 0.0, 0.0),
    ) -> None:
        self._registry = registry
        self._extrusion = extrusion
        self._machine = machine
        self._persistence = persistence
        patches = persisted_patches or {}
        offsets = persisted_offsets or {}
        self._layers = {
            tool.id: ProfileLayers(
                configured=tool.configured_profile,
                persisted=patches.get(tool.id, ToolProfilePatch()),
            )
            for tool in registry
        }
        self._tool_offsets = {
            tool_id: offset for tool_id, offset in offsets.items() if registry.get(tool_id)
        }
        self._session_tool_offsets: dict[ToolId, Vector3] = {}
        unknown_offsets = set(offsets) - set(self._tool_offsets)
        unknown_patches = set(patches) - set(self._layers)
        if unknown_offsets or unknown_patches:
            unknown = sorted(int(tool_id) for tool_id in unknown_offsets | unknown_patches)
            raise KeyError("persisted data refers to unknown tools: %s" % unknown)
        self._global_offset = global_offset
        self._applied: dict[ToolId, str] = {}
        self._pa_applied: dict[ToolId, str] = {}
        self._retraction_applied: dict[ToolId, str] = {}
        self._motion_applied: dict[ToolId, str] = {}
        self._offset_applied: dict[ToolId, Vector3] = {}

    def _id(self, tool_id: ToolId | int) -> ToolId:
        spec = self._registry[tool_id]
        return spec.id

    def layers(self, tool_id: ToolId | int) -> ProfileLayers:
        return self._layers[self._id(tool_id)]

    def desired(self, tool_id: ToolId | int) -> ToolProfile:
        return self.effective(tool_id).profile

    def effective(self, tool_id: ToolId | int) -> EffectiveToolProfile:
        return self.layers(tool_id).resolve()

    def applied(self, tool_id: ToolId | int) -> AppliedToolProfile:
        key = self._id(tool_id)
        desired = self._layers[key].resolve()
        return AppliedToolProfile(
            tool_id=key,
            desired=desired,
            applied_revision=self._applied.get(key),
            pressure_advance_revision=self._pa_applied.get(key),
            firmware_retraction_revision=self._retraction_applied.get(key),
            motion_revision=self._motion_applied.get(key),
        )

    def _replace_layers(self, tool_id: ToolId, layers: ProfileLayers) -> None:
        old = self._layers[tool_id].resolve()
        new = layers.resolve()
        self._layers[tool_id] = layers
        # An unchanged component remains applied even though the aggregate
        # profile revision changed because another component was edited.
        if (
            self._pa_applied.get(tool_id) == old.revision
            and old.profile.pressure_advance == new.profile.pressure_advance
        ):
            self._pa_applied[tool_id] = new.revision
        if (
            self._retraction_applied.get(tool_id) == old.revision
            and old.profile.print_retraction == new.profile.print_retraction
        ):
            self._retraction_applied[tool_id] = new.revision
        if (
            self._motion_applied.get(tool_id) == old.revision
            and old.profile.motion == new.profile.motion
        ):
            self._motion_applied[tool_id] = new.revision
        self._commit_if_complete(tool_id, new.revision)

    def _commit_if_complete(self, tool_id: ToolId, revision: str) -> None:
        if (
            self._pa_applied.get(tool_id) == revision
            and self._retraction_applied.get(tool_id) == revision
            and self._motion_applied.get(tool_id) == revision
        ):
            self._applied[tool_id] = revision

    def update_persisted_patch(
        self, tool_id: ToolId | int, patch: ToolProfilePatch
    ) -> EffectiveToolProfile:
        """Replace the decoded persisted layer without causing machine effects."""

        if not isinstance(patch, ToolProfilePatch):
            raise TypeError("patch must be a ToolProfilePatch")
        key = self._id(tool_id)
        self._replace_layers(key, replace(self._layers[key], persisted=patch))
        return self.effective(key)

    def update_session_patch(
        self, tool_id: ToolId | int, patch: ToolProfilePatch
    ) -> EffectiveToolProfile:
        """Replace a job-scoped layer without persistence or implicit effects."""

        if not isinstance(patch, ToolProfilePatch):
            raise TypeError("patch must be a ToolProfilePatch")
        key = self._id(tool_id)
        self._replace_layers(key, replace(self._layers[key], session=patch))
        return self.effective(key)

    def clear_session_patch(self, tool_id: ToolId | int) -> EffectiveToolProfile:
        return self.update_session_patch(tool_id, ToolProfilePatch())

    def update_retraction(
        self,
        tool_id: ToolId | int,
        profile: FirmwareRetractionProfile,
        *,
        mounted_tool: Optional[ToolId],
    ) -> AppliedToolProfile:
        if not isinstance(profile, FirmwareRetractionProfile):
            raise TypeError("profile must be a FirmwareRetractionProfile")
        key = self._id(tool_id)
        mounted = None if mounted_tool is None else self._id(mounted_tool)
        self._persistence.save_retraction(key, profile)
        layers = self._layers[key]
        persisted = replace(layers.persisted, print_retraction=_retraction_patch(profile))
        session = replace(layers.session, print_retraction=None)
        self._replace_layers(key, replace(layers, persisted=persisted, session=session))
        if mounted == key:
            self._extrusion.apply_firmware_retraction(profile)
            revision = self.effective(key).revision
            self._retraction_applied[key] = revision
            self._commit_if_complete(key, revision)
        return self.applied(key)

    def update_pressure_advance(
        self,
        tool_id: ToolId | int,
        profile: PressureAdvanceProfile,
        *,
        persist: bool = False,
    ) -> AppliedToolProfile:
        if not isinstance(profile, PressureAdvanceProfile):
            raise TypeError("profile must be a PressureAdvanceProfile")
        key = self._id(tool_id)
        layers = self._layers[key]
        if persist:
            self._persistence.save_pressure_advance(key, profile)
            persisted = replace(layers.persisted, pressure_advance=_pa_patch(profile))
            session = replace(layers.session, pressure_advance=None)
            replacement = replace(layers, persisted=persisted, session=session)
        else:
            session = replace(layers.session, pressure_advance=_pa_patch(profile))
            replacement = replace(layers, session=session)
        self._replace_layers(key, replacement)
        self._extrusion.apply_pressure_advance(self._registry[key].extruder, profile)
        revision = self.effective(key).revision
        self._pa_applied[key] = revision
        self._commit_if_complete(key, revision)
        return self.applied(key)

    def activate(self, tool_id: ToolId | int) -> AppliedToolProfile:
        key = self._id(tool_id)
        self._extrusion.activate_extruder(self._registry[key].extruder)
        return self.reapply(key)

    def reapply(self, tool_id: ToolId | int) -> AppliedToolProfile:
        key = self._id(tool_id)
        effective = self.effective(key)
        # Commit both applied revisions only after the complete candidate was
        # accepted.  A mid-sequence hardware failure is therefore observable.
        self._extrusion.apply_firmware_retraction(effective.profile.print_retraction)
        self._extrusion.apply_pressure_advance(
            self._registry[key].extruder, effective.profile.pressure_advance
        )
        if effective.profile.motion is not None:
            self._machine.apply_motion_profile(effective.profile.motion)
        self._retraction_applied[key] = effective.revision
        self._pa_applied[key] = effective.revision
        self._motion_applied[key] = effective.revision
        self._applied[key] = effective.revision
        return self.applied(key)

    def offset_state(self, tool_id: ToolId | int) -> OffsetState:
        key = self._id(tool_id)
        configured = self._registry[key].offset
        persisted = self._tool_offsets.get(key)
        session = self._session_tool_offsets.get(key)
        local = session if session is not None else (
            persisted if persisted is not None else configured
        )
        return OffsetState(
            key,
            configured,
            persisted,
            session,
            self._global_offset,
            _add(local, self._global_offset),
            self._offset_applied.get(key),
        )

    def apply_offset(self, tool_id: ToolId | int) -> OffsetState:
        state = self.offset_state(tool_id)
        self._machine.apply_offset(state.tool_id, state.effective)
        self._offset_applied[state.tool_id] = state.effective
        return self.offset_state(state.tool_id)

    def update_tool_offset(
        self,
        tool_id: ToolId | int,
        offset: Vector3,
        *,
        mounted_tool: Optional[ToolId],
    ) -> OffsetState:
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        key = self._id(tool_id)
        mounted = None if mounted_tool is None else self._id(mounted_tool)
        self._persistence.save_tool_offset(key, offset)
        self._tool_offsets[key] = offset
        self._session_tool_offsets.pop(key, None)
        if mounted == key:
            effective = self.offset_state(key).effective
            self._machine.apply_offset(key, effective)
            self._offset_applied[key] = effective
        return self.offset_state(key)

    def update_session_tool_offset(
        self, tool_id: ToolId | int, offset: Vector3
    ) -> OffsetState:
        """Replace the non-persistent local offset without machine effects."""

        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        key = self._id(tool_id)
        self._session_tool_offsets[key] = offset
        return self.offset_state(key)

    def update_global_offset(
        self, offset: Vector3, *, mounted_tool: Optional[ToolId]
    ) -> Vector3:
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        mounted = None if mounted_tool is None else self._id(mounted_tool)
        self._persistence.save_global_offset(offset)
        self._global_offset = offset
        if mounted is not None:
            effective = self.offset_state(mounted).effective
            self._machine.apply_offset(mounted, effective)
            self._offset_applied[mounted] = effective
        return self._global_offset
