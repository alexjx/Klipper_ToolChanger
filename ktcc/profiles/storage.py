"""Strict decoder for deployed per-tool save-variable profile values.

The persistence codec preserves the configured key/value shapes. This module
is the pure boundary which turns those values into typed profile patches and
offsets for feature composition.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace
from types import MappingProxyType
from typing import Any, Mapping

from ktcc.persistence.codec import GLOBAL_OFFSET_KEY, PersistenceSnapshot
from ktcc.tools import ToolId, ToolRegistry, Vector3
from ktcc.profiles.models import (
    FirmwareRetractionPatch,
    PressureAdvancePatch,
    ToolProfilePatch,
)


_PROFILE_KEY = re.compile(r"^ktcc_tool_(offset|retract|pa_info)_([0-9]+)$")
_RETRACTION_FIELDS = frozenset(
    {
        "retract_length",
        "retract_speed",
        "unretract_extra_length",
        "unretract_speed",
    }
)
_PA_FIELDS = frozenset({"pressure_advance", "smooth_time"})


class ProfileStorageError(ValueError):
    """A persisted profile value cannot be associated or applied safely."""


@dataclass(frozen=True)
class PersistedProfileData:
    """Immutable typed profile data recovered from one persistence snapshot."""

    patches: Mapping[ToolId, ToolProfilePatch]
    tool_offsets: Mapping[ToolId, Vector3]
    global_offset: Vector3

    def __post_init__(self) -> None:
        # Copy before wrapping so mutation of caller-owned mappings cannot alter
        # the decoded snapshot after construction.
        object.__setattr__(self, "patches", MappingProxyType(dict(self.patches)))
        object.__setattr__(
            self, "tool_offsets", MappingProxyType(dict(self.tool_offsets))
        )


def decode_persisted_profiles(
    registry: ToolRegistry, decoded: PersistenceSnapshot
) -> PersistedProfileData:
    """Decode stored profile variables against configured physical tools.

    Missing dictionary fields are intentional partial overrides.  Unknown
    fields and unknown tool ids are rejected rather than ignored because either
    can otherwise apply a setting to an unintended physical tool.
    """

    if not isinstance(registry, ToolRegistry):
        raise TypeError("registry must be a ToolRegistry")
    if not isinstance(decoded, PersistenceSnapshot):
        raise TypeError("decoded must be a PersistenceSnapshot")

    patches: dict[ToolId, ToolProfilePatch] = {}
    offsets: dict[ToolId, Vector3] = {}
    global_offset = Vector3(0.0, 0.0, 0.0)

    for key, raw in decoded.profile_values.items():
        if not isinstance(key, str):
            raise ProfileStorageError("persisted profile keys must be strings")
        if key == GLOBAL_OFFSET_KEY:
            global_offset = _vector(raw, key)
            continue

        match = _PROFILE_KEY.fullmatch(key)
        if match is None:
            raise ProfileStorageError(f"unknown persisted profile key: {key!r}")
        kind, suffix = match.groups()
        if suffix != str(int(suffix)):
            raise ProfileStorageError(f"non-canonical tool key: {key!r}")
        tool_id = ToolId(int(suffix))
        if registry.get(tool_id) is None:
            # Removing a physical tool must not make the printer fail during
            # startup. Its old profile keys are harmless and remain untouched.
            continue

        if kind == "offset":
            if tool_id in offsets:
                raise ProfileStorageError(f"duplicate offset for tool {tool_id}")
            offsets[tool_id] = _vector(raw, key)
        elif kind == "retract":
            section = _retraction_patch(raw, key)
            current = patches.get(tool_id, ToolProfilePatch())
            if current.print_retraction is not None:
                raise ProfileStorageError(
                    f"duplicate retraction profile for tool {tool_id}"
                )
            patches[tool_id] = replace(current, print_retraction=section)
        else:
            section = _pressure_advance_patch(raw, key)
            current = patches.get(tool_id, ToolProfilePatch())
            if current.pressure_advance is not None:
                raise ProfileStorageError(
                    f"duplicate pressure-advance profile for tool {tool_id}"
                )
            patches[tool_id] = replace(current, pressure_advance=section)

    return PersistedProfileData(
        patches={tool_id: patches[tool_id] for tool_id in sorted(patches)},
        tool_offsets={tool_id: offsets[tool_id] for tool_id in sorted(offsets)},
        global_offset=global_offset,
    )


def _retraction_patch(raw: Any, key: str) -> FirmwareRetractionPatch:
    values = _profile_mapping(raw, key, _RETRACTION_FIELDS)
    return FirmwareRetractionPatch(
        retract_length=_optional_number(values, "retract_length", key, minimum=0.0),
        retract_speed=_optional_number(values, "retract_speed", key, minimum=1.0),
        unretract_extra_length=_optional_number(
            values, "unretract_extra_length", key, minimum=0.0
        ),
        unretract_speed=_optional_number(
            values, "unretract_speed", key, minimum=1.0
        ),
        # The deployed writer never persisted zhop.  None preserves the
        # configured value when this partial patch is composed.
        zhop=None,
    )


def _pressure_advance_patch(raw: Any, key: str) -> PressureAdvancePatch:
    values = _profile_mapping(raw, key, _PA_FIELDS)
    return PressureAdvancePatch(
        pressure_advance=_optional_number(
            values, "pressure_advance", key, minimum=0.0
        ),
        smooth_time=_optional_number(
            values, "smooth_time", key, minimum=0.0, maximum=0.2
        ),
    )


def _profile_mapping(
    raw: Any, key: str, allowed_fields: frozenset[str]
) -> Mapping[str, Any]:
    if not isinstance(raw, Mapping):
        raise ProfileStorageError(f"{key} must be a mapping")
    unknown = set(raw) - allowed_fields
    if unknown:
        try:
            rendered = sorted(repr(field) for field in unknown)
        except TypeError:
            rendered = [repr(field) for field in unknown]
        raise ProfileStorageError(
            f"{key} contains unknown fields: {', '.join(rendered)}"
        )
    return raw


def _optional_number(
    values: Mapping[str, Any],
    field: str,
    key: str,
    *,
    minimum: float,
    maximum: float | None = None,
) -> float | None:
    if field not in values:
        return None
    value = values[field]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ProfileStorageError(f"{key}.{field} must be a finite number")
    normalized = float(value)
    if normalized < minimum:
        raise ProfileStorageError(f"{key}.{field} must be at least {minimum}")
    if maximum is not None and normalized > maximum:
        raise ProfileStorageError(f"{key}.{field} must not exceed {maximum}")
    return normalized


def _vector(raw: Any, key: str) -> Vector3:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ProfileStorageError(f"{key} must be a three-number sequence")
    values = []
    for index, value in enumerate(raw):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ProfileStorageError(
                f"{key}[{index}] must be a finite number"
            )
        values.append(float(value))
    return Vector3(*values)
