"""Klipper-free records collected from configuration sections."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping


class ConfigError(ValueError):
    """Base class for a typed configuration failure."""


class DuplicateSectionError(ConfigError):
    pass


class MissingReferenceError(ConfigError):
    pass


class RemovedFeatureError(ConfigError):
    pass


class InvalidShapeError(ConfigError):
    pass


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, set):
        return frozenset(_freeze(item) for item in value)
    return value


def immutable_values(values: Mapping[str, Any]) -> Mapping[str, Any]:
    return _freeze(values)


@dataclass(frozen=True)
class RawToolGroup:
    id: int
    values: Mapping[str, Any]

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", immutable_values(self.values))


@dataclass(frozen=True)
class RawTool:
    id: int
    group_id: int
    values: Mapping[str, Any]

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", immutable_values(self.values))
