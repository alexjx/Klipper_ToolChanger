"""Reusable strict fakes for feature and integration contract tests."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Set, Tuple

from ktcc.toolchange.state import ChangerState
from ktcc.statistics import MechanicalKind
from ktcc.tools import ExtruderRef, FanRef, HeaterRef, ToolId, Vector3
from ktcc.profiles.models import (
    FirmwareRetractionProfile,
    MotionProfile,
    PressureAdvanceProfile,
)
from ktcc.ports import (
    ActionContext,
    ToolchangerEvent,
    HeaterObservation,
    ThermalWaitRequest,
    TimerCallback,
    TimerHandle,
    VerificationRequest,
    VerificationResult,
)


@dataclass(frozen=True)
class SemanticEffect:
    kind: str
    subject: str
    details: Tuple[object, ...] = ()


class InjectedFailure(RuntimeError):
    pass


class _StrictRecorder:
    def __init__(self, trace: Optional[List[SemanticEffect]] = None) -> None:
        self.trace = trace if trace is not None else []
        self._failures: Dict[Tuple[str, str], List[BaseException]] = {}

    def inject_failure(
        self,
        kind: str,
        subject: str,
        error: Optional[BaseException] = None,
    ) -> None:
        if not kind or not subject:
            raise ValueError("failure kind and subject must be non-empty")
        failure = error or InjectedFailure(f"injected failure: {kind}:{subject}")
        self._failures.setdefault((kind, subject), []).append(failure)

    def _record(self, kind: str, subject: str, *details: object) -> None:
        self.trace.append(SemanticEffect(kind, subject, tuple(details)))
        failures = self._failures.get((kind, subject))
        if failures:
            error = failures.pop(0)
            if not failures:
                del self._failures[(kind, subject)]
            raise error


class FakeActionPort(_StrictRecorder):
    def __init__(
        self, actions: Iterable[str], trace: Optional[List[SemanticEffect]] = None
    ) -> None:
        super().__init__(trace)
        self.actions = frozenset(actions)
        if not self.actions or any(not action for action in self.actions):
            raise ValueError("at least one non-empty action is required")

    def _validate(self, action: str, context: ActionContext) -> None:
        if action not in self.actions:
            raise KeyError(f"unknown action: {action}")
        if not isinstance(context, ActionContext):
            raise TypeError("context must be an ActionContext")

    def run_from_command(self, action: str, context: ActionContext) -> None:
        self._validate(action, context)
        self._record("action.command", action, context.tool.id, context.transition_id)

    def run_from_callback(self, action: str, context: ActionContext) -> None:
        self._validate(action, context)
        self._record("action.callback", action, context.tool.id, context.transition_id)


class FakeThermalPort(_StrictRecorder):
    def __init__(
        self,
        observations: Dict[HeaterRef, HeaterObservation],
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        if not observations:
            raise ValueError("at least one known heater is required")
        self.observations = dict(observations)

    def _known(self, heater: HeaterRef) -> HeaterObservation:
        if not isinstance(heater, HeaterRef):
            raise TypeError("heater must be a HeaterRef")
        try:
            return self.observations[heater]
        except KeyError as exc:
            raise KeyError(f"unknown heater: {heater}") from exc

    def set_target(self, heater: HeaterRef, target: float) -> None:
        observation = self._known(heater)
        if isinstance(target, bool) or not isinstance(target, (int, float)):
            raise TypeError("target must be a number")
        if not math.isfinite(target) or target < 0:
            raise ValueError("target must be a non-negative finite number")
        self._record("thermal.set_target", str(heater), float(target))
        self.observations[heater] = replace(observation, target=float(target))

    def wait(
        self, heater: HeaterRef, request: ThermalWaitRequest
    ) -> HeaterObservation:
        observation = self._known(heater)
        if not isinstance(request, ThermalWaitRequest):
            raise TypeError("request must be a ThermalWaitRequest")
        if observation.target != request.frozen_target:
            raise ValueError("heater target differs from frozen wait target")
        self._record("thermal.wait", str(heater), request)
        return observation

    def observe(self, heater: HeaterRef) -> HeaterObservation:
        observation = self._known(heater)
        self._record("thermal.observe", str(heater))
        return observation


class FakeExtrusionProfilePort(_StrictRecorder):
    def __init__(
        self,
        extruders: Iterable[ExtruderRef],
        trace: Optional[List[SemanticEffect]] = None,
        *,
        retracted: bool = False,
    ) -> None:
        super().__init__(trace)
        self.extruders = frozenset(extruders)
        if not self.extruders:
            raise ValueError("at least one known extruder is required")
        if not isinstance(retracted, bool):
            raise TypeError("retracted must be boolean")
        self.active: Optional[ExtruderRef] = None
        self.retracted = retracted

    def _known(self, extruder: ExtruderRef) -> None:
        if not isinstance(extruder, ExtruderRef):
            raise TypeError("extruder must be an ExtruderRef")
        if extruder not in self.extruders:
            raise KeyError(f"unknown extruder: {extruder}")

    def activate_extruder(self, extruder: ExtruderRef) -> None:
        self._known(extruder)
        self._record("extrusion.activate", str(extruder))
        self.active = extruder

    def apply_pressure_advance(
        self, extruder: ExtruderRef, profile: PressureAdvanceProfile
    ) -> None:
        self._known(extruder)
        if not isinstance(profile, PressureAdvanceProfile):
            raise TypeError("profile must be a PressureAdvanceProfile")
        self._record("extrusion.apply_pa", str(extruder), profile)

    def apply_firmware_retraction(self, profile: FirmwareRetractionProfile) -> None:
        if not isinstance(profile, FirmwareRetractionProfile):
            raise TypeError("profile must be a FirmwareRetractionProfile")
        self._record("extrusion.apply_retraction", "firmware_retraction", profile)

    def observe_retracted(self) -> bool:
        self._record("extrusion.observe_retracted", "firmware_retraction")
        return self.retracted

    def unretract(self) -> None:
        if self.active is None:
            raise RuntimeError("cannot unretract without an active extruder")
        self._record("extrusion.unretract", str(self.active))
        self.retracted = False


class FakeMachinePort(_StrictRecorder):
    def __init__(
        self,
        tool_ids: Iterable[ToolId],
        fans: Iterable[FanRef],
        trace: Optional[List[SemanticEffect]] = None,
        *,
        position: Vector3 = Vector3(0.0, 0.0, 0.0),
    ) -> None:
        super().__init__(trace)
        self.tool_ids = frozenset(tool_ids)
        self.fans = frozenset(fans)
        self.position = position

    def apply_offset(self, tool_id: ToolId, offset: Vector3) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")
        if tool_id not in self.tool_ids:
            raise KeyError(f"unknown tool: {tool_id}")
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        self._record("machine.apply_offset", str(tool_id), offset)

    def set_fan(self, fan: FanRef, speed: float) -> None:
        if not isinstance(fan, FanRef):
            raise TypeError("fan must be a FanRef")
        if fan not in self.fans:
            raise KeyError(f"unknown fan: {fan}")
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            raise TypeError("speed must be a number")
        if not math.isfinite(speed) or not 0 <= speed <= 1:
            raise ValueError("fan speed must be between 0 and 1")
        self._record("machine.set_fan", str(fan), float(speed))

    def observe_position(self) -> Vector3:
        self._record("machine.observe_position", "toolhead")
        return self.position

    def apply_motion_profile(self, profile: MotionProfile) -> None:
        if not isinstance(profile, MotionProfile):
            raise TypeError("profile must be a MotionProfile")
        self._record("machine.apply_motion", "input_shaper", profile)


class FakeToolchangeReadinessPort(_StrictRecorder):
    def require_ready(self) -> None:
        self._record("readiness.require", "toolchange")


@dataclass
class _Scheduled:
    callback_id: str
    deadline: float
    callback: TimerCallback


class FakeSchedulerPort(_StrictRecorder):
    def __init__(
        self,
        callback_ids: Iterable[str],
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        self.callback_ids = frozenset(callback_ids)
        if not self.callback_ids or any(not item for item in self.callback_ids):
            raise ValueError("at least one non-empty callback id is required")
        self._next_handle = 0
        self._scheduled: Dict[TimerHandle, _Scheduled] = {}

    def schedule_at(
        self, callback_id: str, deadline: float, callback: TimerCallback
    ) -> TimerHandle:
        if callback_id not in self.callback_ids:
            raise KeyError(f"unknown callback id: {callback_id}")
        if isinstance(deadline, bool) or not isinstance(deadline, (int, float)):
            raise TypeError("deadline must be a number")
        if not math.isfinite(deadline) or deadline < 0:
            raise ValueError("deadline must be a non-negative finite number")
        if not callable(callback):
            raise TypeError("callback must be callable")
        handle = TimerHandle(self._next_handle)
        self._next_handle += 1
        self._record("scheduler.schedule", callback_id, float(deadline), handle)
        self._scheduled[handle] = _Scheduled(callback_id, float(deadline), callback)
        return handle

    def cancel(self, handle: TimerHandle) -> None:
        if not isinstance(handle, TimerHandle):
            raise TypeError("handle must be a TimerHandle")
        try:
            scheduled = self._scheduled.pop(handle)
        except KeyError as exc:
            raise KeyError(f"unknown or inactive timer handle: {handle.value}") from exc
        self._record("scheduler.cancel", scheduled.callback_id, handle)

    def run_due(self, now: float) -> None:
        if isinstance(now, bool) or not isinstance(now, (int, float)):
            raise TypeError("now must be a number")
        if not math.isfinite(now) or now < 0:
            raise ValueError("now must be a non-negative finite number")
        due = sorted(
            (
                (scheduled.deadline, handle, scheduled)
                for handle, scheduled in self._scheduled.items()
                if scheduled.deadline <= now
            ),
            key=lambda item: (item[0], item[1]),
        )
        for _, handle, scheduled in due:
            del self._scheduled[handle]
            self._record("scheduler.fire", scheduled.callback_id, handle)
            scheduled.callback()


class FakeStatePersistencePort(_StrictRecorder):
    def __init__(
        self,
        state: Optional[ChangerState] = None,
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        self.state = state
        self.mirrored_tool_current: Optional[int] = None

    def load_state(self) -> Optional[ChangerState]:
        self._record("persistence.load_state", "changer")
        return self.state

    def save_state(self, state: ChangerState) -> None:
        if not isinstance(state, ChangerState):
            raise TypeError("state must be a ChangerState")
        self._record("persistence.save_state", "changer", state)
        self.state = state

    def mirror_tool_current(self, value: int) -> None:
        if isinstance(value, bool) or not isinstance(value, int) or value < -2:
            raise ValueError("legacy tool_current must be an integer at least -2")
        self._record("persistence.mirror_current", "tool_current", value)
        self.mirrored_tool_current = value


class FakeProfilePersistencePort(_StrictRecorder):
    def save_retraction(
        self, tool_id: ToolId, profile: FirmwareRetractionProfile
    ) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")
        if not isinstance(profile, FirmwareRetractionProfile):
            raise TypeError("profile must be a FirmwareRetractionProfile")
        self._record("persistence.save_retraction", str(tool_id), profile)

    def save_pressure_advance(
        self, tool_id: ToolId, profile: PressureAdvanceProfile
    ) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")
        if not isinstance(profile, PressureAdvanceProfile):
            raise TypeError("profile must be a PressureAdvanceProfile")
        self._record("persistence.save_pa", str(tool_id), profile)

    def save_tool_offset(self, tool_id: ToolId, offset: Vector3) -> None:
        if not isinstance(tool_id, ToolId):
            raise TypeError("tool_id must be a ToolId")
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        self._record("persistence.save_tool_offset", str(tool_id), offset)

    def save_global_offset(self, offset: Vector3) -> None:
        if not isinstance(offset, Vector3):
            raise TypeError("offset must be a Vector3")
        self._record("persistence.save_global_offset", "global", offset)


class FakeStatisticsPort(_StrictRecorder):
    def mechanical_started(
        self, kind: MechanicalKind, tool_id: ToolId, transition_id: str
    ) -> None:
        self._record("statistics.mechanical_started", kind.value, tool_id, transition_id)

    def mechanical_completed(
        self, kind: MechanicalKind, tool_id: ToolId, transition_id: str
    ) -> None:
        self._record("statistics.mechanical_completed", kind.value, tool_id, transition_id)

    def selection_changed(
        self, previous: Optional[ToolId], current: Optional[ToolId]
    ) -> None:
        self._record("statistics.selection_changed", "selection", previous, current)

    def lock_completed(self) -> None:
        self._record("statistics.lock_completed", "lock")

    def unlock_completed(self) -> None:
        self._record("statistics.unlock_completed", "lock")

    def thermal_changed(
        self, tool_id: ToolId, previous: ThermalMode, current: ThermalMode
    ) -> None:
        self._record("statistics.thermal_changed", str(tool_id), previous, current)


class FakeVariableStorePort(_StrictRecorder):
    def __init__(
        self,
        variables: Optional[Mapping[str, Any]] = None,
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        self.variables = dict(variables or {})

    def read_variables(self) -> Mapping[str, Any]:
        self._record("variables.read", "save_variables")
        return dict(self.variables)

    def write_variable(self, name: str, value: Any) -> None:
        if not isinstance(name, str) or not name:
            raise ValueError("variable name must be non-empty")
        self._record("variables.write", name, value)
        self.variables[name] = value


class FakeVerificationPort(_StrictRecorder):
    def __init__(
        self,
        results: Dict[VerificationRequest, VerificationResult],
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        self.results = dict(results)
        if any(request != result.request for request, result in self.results.items()):
            raise ValueError("verification result must match its request")

    def verify(self, request: VerificationRequest) -> VerificationResult:
        if not isinstance(request, VerificationRequest):
            raise TypeError("request must be a VerificationRequest")
        try:
            result = self.results[request]
        except KeyError as exc:
            raise KeyError(f"no configured verification result for {request}") from exc
        subject = request.kind.value
        if request.tool_id is not None:
            subject = f"{subject}:T{request.tool_id}"
        self._record("verification.verify", subject, result)
        return result


class FakeEventSink(_StrictRecorder):
    def __init__(
        self,
        event_codes: Iterable[str],
        trace: Optional[List[SemanticEffect]] = None,
    ) -> None:
        super().__init__(trace)
        self.event_codes = frozenset(event_codes)
        if not self.event_codes or any(not code for code in self.event_codes):
            raise ValueError("at least one non-empty event code is required")

    def emit(self, event: ToolchangerEvent) -> None:
        if not isinstance(event, ToolchangerEvent):
            raise TypeError("event must be a ToolchangerEvent")
        if event.code not in self.event_codes:
            raise KeyError(f"unknown event code: {event.code}")
        self._record("event.emit", event.code, event)


class FakeShutdownPort(_StrictRecorder):
    def shutdown(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("shutdown reason must be non-empty")
        self._record("shutdown", "klipper", reason)


class FakeClock:
    def __init__(self, now: float = 0.0) -> None:
        self._now = 0.0
        self.set(now)

    def monotonic(self) -> float:
        return self._now

    def set(self, value: float) -> None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("clock value must be a number")
        if not math.isfinite(value) or value < self._now:
            raise ValueError("clock cannot move backwards or become non-finite")
        self._now = float(value)

    def advance(self, seconds: float) -> None:
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
            raise TypeError("seconds must be a number")
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("advance must be a non-negative finite number")
        self._now += float(seconds)
