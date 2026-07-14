"""Structural adapters for Klipper's public runtime integration surfaces.

The module intentionally imports no ``klippy`` package.  Composition code owns
object lookup and injects the small structural objects used here, which keeps
the boundary testable against the Klipper API without installing Klipper as a
Python dependency.
"""

from __future__ import annotations

import math
import re
from dataclasses import fields, is_dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any, Callable, Mapping, Protocol

from ktcc.tools import ExtruderRef, FanRef, HeaterRef, ToolId, Vector3
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    MotionProfile,
    PressureAdvanceProfile,
    WaitMode,
)
from ktcc.ports import (
    ActionContext,
    HeaterObservation,
    ThermalWaitRequest,
    TimerHandle,
    VerificationRequest,
    VerificationResult,
)


class ThermalWaitError(RuntimeError):
    """Base class for an adapter-owned thermal wait failure."""


class ThermalWaitTimeout(ThermalWaitError):
    """The frozen target was not satisfied before the requested deadline."""


class ThermalTargetChanged(ThermalWaitError):
    """Another actor changed the heater target during a frozen-target wait."""


class ThermalWaitShutdown(ThermalWaitError):
    """Klipper entered shutdown while a thermal wait was in progress."""


class ToolchangeNotReadyError(RuntimeError):
    """The machine lacks the homing state required for tool-changing motion."""


class _CommandTemplate(Protocol):
    def run_gcode_from_command(self, context: Mapping[str, Any]) -> None: ...


class _Heater(Protocol):
    def get_status(self, eventtime: float) -> Mapping[str, Any]: ...


class _PrinterHeaters(Protocol):
    def set_temperature(self, heater: _Heater, target: float, wait: bool = False) -> None: ...


class _Reactor(Protocol):
    NEVER: float

    def monotonic(self) -> float: ...

    def pause(self, waketime: float) -> float: ...

    def register_timer(self, callback: Callable[[float], float], waketime: float) -> Any: ...

    def unregister_timer(self, timer: Any) -> None: ...


class _GCode(Protocol):
    def run_script_from_command(self, script: str) -> None: ...


class _Toolhead(Protocol):
    def get_status(self, eventtime: float) -> Mapping[str, Any]: ...

    def get_position(self) -> list[float]: ...


class _Printer(Protocol):
    def invoke_shutdown(self, reason: str) -> None: ...


_RESOURCE_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_.-]{0,127}\Z")
_ENUM_TOKEN = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}\Z")


def _resource(kind: str, value: str) -> str:
    if not isinstance(value, str) or _RESOURCE_NAME.fullmatch(value) is None:
        raise ValueError(f"{kind} name cannot be encoded safely in G-code")
    return value


def _enum_token(kind: str, value: str) -> str:
    if not isinstance(value, str) or _ENUM_TOKEN.fullmatch(value) is None:
        raise ValueError(f"{kind} cannot be encoded safely in G-code")
    return value


def _number(name: str, value: float, *, low: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if low is not None and value < low:
        raise ValueError(f"{name} must be at least {low}")
    return value


def _gcode_number(value: float) -> str:
    # Model validation has already rejected NaN/inf.  ``.12g`` is compact,
    # locale-independent, and retains substantially more precision than the
    # physical settings represented here require.
    return format(float(value), ".12g")


def _snapshot(value: Any) -> Any:
    """Recursively copy a value object into an immutable Jinja snapshot."""

    if is_dataclass(value):
        return MappingProxyType(
            {field.name: _snapshot(getattr(value, field.name)) for field in fields(value)}
        )
    if isinstance(value, Mapping):
        return MappingProxyType({key: _snapshot(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_snapshot(item) for item in value)
    if isinstance(value, Enum):
        return value.value
    return value


def _vector_snapshot(value: Vector3) -> Mapping[str, float]:
    return MappingProxyType({"x": value.x, "y": value.y, "z": value.z})


def _tool_snapshot(context: ActionContext) -> Mapping[str, Any]:
    tool = context.tool
    return MappingProxyType(
        {
            "id": tool.id.value,
            "extruder": tool.extruder.value,
            "heater": tool.heater.value,
            "fan": None if tool.fan is None else tool.fan.value,
            "zone": _vector_snapshot(tool.dock.zone),
            "park": _vector_snapshot(tool.dock.park),
            "offset": _vector_snapshot(tool.offset),
            "profile": _snapshot(
                context.effective_profile or tool.configured_profile
            ),
        }
    )


class KlipperActionPort:
    """Run a tool-specific precompiled template with a fresh v1 context."""

    def __init__(
        self,
        templates: Mapping[tuple[int | ToolId, str], _CommandTemplate],
        callback_runner: Callable[[_CommandTemplate, Mapping[str, Any]], None],
        context_factory: Callable[[], Mapping[str, Any]] = dict,
        facade_context_factory: Callable[[ActionContext], Mapping[str, Any]] | None = None,
    ) -> None:
        if not callable(callback_runner):
            raise TypeError("callback_runner must be callable")
        if not callable(context_factory):
            raise TypeError("context_factory must be callable")
        if facade_context_factory is not None and not callable(facade_context_factory):
            raise TypeError("facade_context_factory must be callable")
        normalized: dict[tuple[int, str], _CommandTemplate] = {}
        for (tool_id, action), template in templates.items():
            number = tool_id.value if isinstance(tool_id, ToolId) else tool_id
            if isinstance(number, bool) or not isinstance(number, int) or number < 0:
                raise ValueError("template tool id must be a non-negative integer")
            if not isinstance(action, str) or not action:
                raise ValueError("template action must be a non-empty string")
            key = (number, action)
            if key in normalized:
                raise ValueError(f"duplicate action template: tool={number} action={action}")
            normalized[key] = template
        self._templates = MappingProxyType(normalized)
        self._callback_runner = callback_runner
        self._context_factory = context_factory
        self._facade_context_factory = facade_context_factory

    def run_from_command(self, action: str, context: ActionContext) -> None:
        template = self._template(action, context)
        template.run_gcode_from_command(self._context(context))

    def run_from_callback(self, action: str, context: ActionContext) -> None:
        template = self._template(action, context)
        self._callback_runner(template, self._context(context))

    def _template(self, action: str, context: ActionContext) -> _CommandTemplate:
        if not isinstance(context, ActionContext):
            raise TypeError("context must be an ActionContext")
        if not isinstance(action, str) or not action:
            raise ValueError("action must be a non-empty string")
        try:
            return self._templates[(context.tool.id.value, action)]
        except KeyError:
            raise KeyError(
                f"unknown action template: tool={context.tool.id.value} action={action}"
            ) from None

    def _context(self, context: ActionContext) -> Mapping[str, Any]:
        base = self._context_factory()
        if not isinstance(base, Mapping):
            raise TypeError("context_factory must return a mapping")
        result = dict(base)
        if self._facade_context_factory is not None:
            aliases = self._facade_context_factory(context)
            if not isinstance(aliases, Mapping):
                raise TypeError("facade_context_factory must return a mapping")
            result.update(aliases)
        # These names are deliberately replaced, never updated in-place, so a
        # stale factory snapshot cannot shadow authoritative transition data.
        result.update(
            {
                "schema_version": context.schema_version,
                "tool": _tool_snapshot(context),
                "thermal": MappingProxyType(
                    {
                        "active": context.thermal.active_target,
                        "standby": context.thermal.standby_target,
                        "mode": context.thermal.mode.value,
                    }
                ),
                "changer": MappingProxyType(
                    {
                        "before": _snapshot(context.changer_before),
                        "expected_after": _snapshot(context.expected_after),
                    }
                ),
                "transition": MappingProxyType(
                    {
                        "id": context.transition_id,
                        "phase": context.phase.value,
                        "last_checkpoint": context.last_checkpoint,
                        "checkpoint_capability": context.checkpoint_capability,
                    }
                ),
            }
        )
        return MappingProxyType(result)


class KlipperThermalPort:
    """Heater control and bounded waits over the public heater/reactor APIs."""

    def __init__(
        self,
        printer_heaters: _PrinterHeaters,
        heaters: Mapping[HeaterRef | str, _Heater],
        reactor: _Reactor,
        is_shutdown: Callable[[], bool],
        *,
        poll_interval: float = 0.25,
    ) -> None:
        if not callable(is_shutdown):
            raise TypeError("is_shutdown must be callable")
        self._poll_interval = _number("poll_interval", poll_interval, low=0.001)
        normalized: dict[HeaterRef, _Heater] = {}
        for reference, heater in heaters.items():
            key = reference if isinstance(reference, HeaterRef) else HeaterRef(reference)
            if key in normalized:
                raise ValueError(f"duplicate heater reference: {key}")
            if not callable(getattr(heater, "get_status", None)):
                raise TypeError(f"heater {key} must provide get_status(eventtime)")
            normalized[key] = heater
        self._printer_heaters = printer_heaters
        self._heaters = MappingProxyType(normalized)
        self._reactor = reactor
        self._is_shutdown = is_shutdown

    def set_target(self, heater: HeaterRef, target: float) -> None:
        target = _number("target", target, low=0.0)
        self._printer_heaters.set_temperature(self._heater(heater), target, wait=False)

    def observe(self, heater: HeaterRef) -> HeaterObservation:
        return self._observe_at(heater, self._reactor.monotonic())

    def wait(self, heater: HeaterRef, request: ThermalWaitRequest) -> HeaterObservation:
        if not isinstance(request, ThermalWaitRequest):
            raise TypeError("request must be a ThermalWaitRequest")
        physical = self._heater(heater)
        start = self._reactor.monotonic()
        deadline = start + request.timeout
        stable_since: float | None = None

        while True:
            if self._is_shutdown():
                raise ThermalWaitShutdown(f"shutdown while waiting for heater {heater}")
            now = self._reactor.monotonic()
            observation = self._observe_physical(physical, now)
            if not math.isclose(
                observation.target, request.frozen_target, rel_tol=0.0, abs_tol=1e-6
            ):
                raise ThermalTargetChanged(
                    f"heater {heater} target changed from "
                    f"{request.frozen_target:g} to {observation.target:g}"
                )

            within = abs(observation.temperature - request.frozen_target) <= request.tolerance
            if request.mode is WaitMode.HEAT:
                if observation.temperature >= request.frozen_target - request.tolerance:
                    return observation
            elif request.mode is WaitMode.RANGE:
                if within:
                    return observation
            else:
                if within:
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= request.stable_time:
                        return observation
                else:
                    stable_since = None

            if now >= deadline:
                raise ThermalWaitTimeout(
                    f"heater {heater} did not reach {request.frozen_target:g} "
                    f"within {request.timeout:g}s"
                )
            self._reactor.pause(min(now + self._poll_interval, deadline))

    def _heater(self, reference: HeaterRef) -> _Heater:
        if not isinstance(reference, HeaterRef):
            raise TypeError("heater must be a HeaterRef")
        try:
            return self._heaters[reference]
        except KeyError:
            raise KeyError(f"unknown heater: {reference}") from None

    def _observe_at(self, reference: HeaterRef, eventtime: float) -> HeaterObservation:
        return self._observe_physical(self._heater(reference), eventtime)

    @staticmethod
    def _observe_physical(heater: _Heater, eventtime: float) -> HeaterObservation:
        status = heater.get_status(eventtime)
        if not isinstance(status, Mapping):
            raise TypeError("heater status must be a mapping")
        try:
            temperature = _number("heater temperature", status["temperature"])
            target = _number("heater target", status["target"], low=0.0)
        except KeyError as exc:
            raise TypeError(f"heater status is missing {exc.args[0]}") from None
        can_extrude = status.get("can_extrude")
        if can_extrude is None:
            minimum = status.get("min_extrude_temp")
            if minimum is None:
                raise TypeError(
                    "heater status must contain can_extrude or min_extrude_temp"
                )
            can_extrude = temperature >= _number("min_extrude_temp", minimum)
        if not isinstance(can_extrude, bool):
            raise TypeError("heater can_extrude status must be boolean")
        return HeaterObservation(temperature, target, can_extrude)


class KlipperExtrusionProfilePort:
    """Apply per-tool extrusion profiles through Klipper G-code commands."""

    def __init__(
        self,
        gcode: _GCode,
        firmware_retraction: Any,
        *,
        set_retraction_command: str = "SET_RETRACTION",
    ) -> None:
        if not callable(getattr(gcode, "run_script_from_command", None)):
            raise TypeError("gcode must provide run_script_from_command(script)")
        if not hasattr(firmware_retraction, "is_retracted"):
            raise TypeError("firmware_retraction must expose is_retracted")
        self._gcode = gcode
        self._firmware_retraction = firmware_retraction
        self._set_retraction_command = _resource(
            "set-retraction command", set_retraction_command
        )

    def activate_extruder(self, extruder: ExtruderRef) -> None:
        name = self._extruder(extruder)
        self._run(f"ACTIVATE_EXTRUDER EXTRUDER={name}")

    def apply_pressure_advance(
        self, extruder: ExtruderRef, profile: PressureAdvanceProfile
    ) -> None:
        name = self._extruder(extruder)
        if not isinstance(profile, PressureAdvanceProfile):
            raise TypeError("profile must be a PressureAdvanceProfile")
        self._run(
            "SET_PRESSURE_ADVANCE "
            f"EXTRUDER={name} ADVANCE={_gcode_number(profile.pressure_advance)} "
            f"SMOOTH_TIME={_gcode_number(profile.smooth_time)}"
        )

    def apply_firmware_retraction(self, profile: FirmwareRetractionProfile) -> None:
        if not isinstance(profile, FirmwareRetractionProfile):
            raise TypeError("profile must be a FirmwareRetractionProfile")
        self._run(
            f"{self._set_retraction_command} "
            f"RETRACT_LENGTH={_gcode_number(profile.retract_length)} "
            f"RETRACT_SPEED={_gcode_number(profile.retract_speed)} "
            f"UNRETRACT_EXTRA_LENGTH={_gcode_number(profile.unretract_extra_length)} "
            f"UNRETRACT_SPEED={_gcode_number(profile.unretract_speed)}"
        )

    def observe_retracted(self) -> bool:
        # Klipper 2026-07 exposes retraction parameters in get_status(), but not
        # this state bit.  This is the single intentionally narrow audited
        # attribute read in the runtime adapter.
        value = self._firmware_retraction.is_retracted
        if not isinstance(value, bool):
            raise TypeError("firmware_retraction.is_retracted must be boolean")
        return value

    def unretract(self) -> None:
        self._run("G11")

    @staticmethod
    def _extruder(extruder: ExtruderRef) -> str:
        if not isinstance(extruder, ExtruderRef):
            raise TypeError("extruder must be an ExtruderRef")
        return _resource("extruder", extruder.value)

    def _run(self, command: str) -> None:
        self._gcode.run_script_from_command(command)


class KlipperSchedulerPort:
    """Map one-shot application timers onto Klipper reactor timers."""

    def __init__(self, reactor: _Reactor) -> None:
        self._reactor = reactor
        self._next_handle = 0
        self._timers: dict[TimerHandle, Any] = {}

    def schedule_at(
        self, callback_id: str, deadline: float, callback: Callable[[], None]
    ) -> TimerHandle:
        if not isinstance(callback_id, str) or not callback_id:
            raise ValueError("callback_id must be a non-empty string")
        deadline = _number("deadline", deadline, low=0.0)
        if not callable(callback):
            raise TypeError("callback must be callable")
        handle = TimerHandle(self._next_handle)
        self._next_handle += 1

        def invoke(eventtime: float) -> float:
            del eventtime
            self._timers.pop(handle, None)
            callback()
            return self._reactor.NEVER

        timer = self._reactor.register_timer(invoke, deadline)
        self._timers[handle] = timer
        return handle

    def cancel(self, handle: TimerHandle) -> None:
        if not isinstance(handle, TimerHandle):
            raise TypeError("handle must be a TimerHandle")
        try:
            timer = self._timers.pop(handle)
        except KeyError:
            raise KeyError(f"unknown or completed timer: {handle.value}") from None
        self._reactor.unregister_timer(timer)


class KlipperReadinessPort:
    """Require XYZ homing without performing motion or changing machine state."""

    def __init__(
        self,
        toolhead: _Toolhead,
        eventtime: Callable[[], float],
        error_factory: Callable[[str], BaseException] = ToolchangeNotReadyError,
    ) -> None:
        if not callable(eventtime) or not callable(error_factory):
            raise TypeError("eventtime and error_factory must be callable")
        self._toolhead = toolhead
        self._eventtime = eventtime
        self._error_factory = error_factory

    def require_ready(self) -> None:
        status = self._toolhead.get_status(self._eventtime())
        if not isinstance(status, Mapping):
            raise TypeError("toolhead status must be a mapping")
        axes = status.get("homed_axes")
        if not isinstance(axes, str):
            raise TypeError("toolhead status must contain string homed_axes")
        missing = "".join(axis for axis in "xyz" if axis not in axes.lower())
        if missing:
            raise self._error_factory(
                f"tool change requires homed XYZ axes; missing {missing.upper()}"
            )


class KlipperMachinePort:
    """Small machine-effects adapter implemented with explicit G-code."""

    def __init__(self, gcode: _GCode, toolhead: _Toolhead) -> None:
        self._gcode = gcode
        self._toolhead = toolhead

    def apply_offset(self, tool_id: ToolId, offset: Vector3) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        self._gcode.run_script_from_command(
            "SET_GCODE_OFFSET "
            f"X={_gcode_number(offset.x)} Y={_gcode_number(offset.y)} "
            f"Z={_gcode_number(offset.z)} MOVE=0"
        )

    def set_fan(self, fan: FanRef, speed: float) -> None:
        if not isinstance(fan, FanRef):
            raise TypeError("fan must be a FanRef")
        speed = _number("fan speed", speed, low=0.0)
        if speed > 1.0:
            raise ValueError("fan speed must not exceed 1.0")
        self._gcode.run_script_from_command(
            f"SET_FAN_SPEED FAN={_resource('fan', fan.value)} "
            f"SPEED={_gcode_number(speed)}"
        )

    def observe_position(self) -> Vector3:
        position = self._toolhead.get_position()
        if not isinstance(position, (list, tuple)) or len(position) < 3:
            raise TypeError("toolhead position must contain at least XYZ")
        return Vector3(position[0], position[1], position[2])

    def apply_motion_profile(self, profile: MotionProfile) -> None:
        if not isinstance(profile, MotionProfile):
            raise TypeError("profile must be a MotionProfile")
        self._gcode.run_script_from_command(
            "SET_INPUT_SHAPER "
            f"SHAPER_FREQ_X={_gcode_number(profile.shaper_freq_x)} "
            f"SHAPER_FREQ_Y={_gcode_number(profile.shaper_freq_y)} "
            f"DAMPING_RATIO_X={_gcode_number(profile.damping_ratio_x)} "
            f"DAMPING_RATIO_Y={_gcode_number(profile.damping_ratio_y)} "
            f"SHAPER_TYPE_X={_enum_token('shaper type', profile.shaper_type_x)} "
            f"SHAPER_TYPE_Y={_enum_token('shaper type', profile.shaper_type_y)}"
        )


class KlipperShutdownPort:
    def __init__(self, printer: _Printer) -> None:
        self._printer = printer

    def shutdown(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason:
            raise ValueError("shutdown reason must be a non-empty string")
        self._printer.invoke_shutdown(reason)


class KlipperVerificationPort:
    """Delegate verification only to an explicitly configured callback."""

    def __init__(
        self, callback: Callable[[VerificationRequest], VerificationResult]
    ) -> None:
        if not callable(callback):
            raise TypeError("verification callback must be callable")
        self._callback = callback

    def verify(self, request: VerificationRequest) -> VerificationResult:
        if not isinstance(request, VerificationRequest):
            raise TypeError("request must be a VerificationRequest")
        result = self._callback(request)
        if not isinstance(result, VerificationResult):
            raise TypeError("verification callback must return VerificationResult")
        if result.request != request:
            raise ValueError("verification result request does not match invocation")
        return result
