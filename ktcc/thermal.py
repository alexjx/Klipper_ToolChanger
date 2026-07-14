"""Per-tool thermal sessions, preheat, timers, and waits.

The service owns policy and sequencing.  ``ThermalPort`` owns the blocking
details of a wait and ``SchedulerPort`` owns reactor integration; neither
Klipper objects nor G-code parsing enter this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Mapping, Optional

from ktcc.tools import HeaterRef, ToolId, ToolRegistry, ToolSpec
from ktcc.profiles.models import ThermalPolicy, WaitMode
from ktcc.ports import (
    Clock,
    HeaterObservation,
    SchedulerPort,
    StatisticsPort,
    ThermalMode,
    ThermalPort,
    ThermalSnapshot,
    ThermalWaitRequest,
    TimerHandle,
)


class ThermalConflictError(RuntimeError):
    """Two logical tools attempted to control one physical heater."""


class ThermalStateError(RuntimeError):
    """A thermal operation is incompatible with the current demand."""


class WaitTarget(str, Enum):
    CURRENT = "CURRENT"
    ACTIVE = "ACTIVE"
    STANDBY = "STANDBY"


@dataclass(frozen=True)
class ControllerFailure:
    heater: HeaterRef
    operation: str
    generation: int
    error: BaseException


@dataclass
class _Session:
    active_target: float
    standby_target: float
    idle_to_standby_time: float
    standby_to_off_time: float


@dataclass
class _HeaterRuntime:
    heater: HeaterRef
    owner_tool: Optional[ToolId] = None
    mode: ThermalMode = ThermalMode.OFF
    target: float = 0.0
    generation: int = 0
    timer: Optional[TimerHandle] = None
    preheat_target: Optional[WaitTarget] = None


def _temperature(name: str, value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError(f"{name} must be a non-negative finite number")
    return result


def _duration(name: str, value: float) -> float:
    result = _temperature(name, value)
    if result < 0.1:
        raise ValueError(f"{name} must be at least 0.1 seconds")
    return result


class ThermalService:
    """Coordinates one controller per physical heater and one session per tool."""

    def __init__(
        self,
        registry: ToolRegistry,
        thermal: ThermalPort,
        scheduler: SchedulerPort,
        clock: Clock,
        auxiliary_heaters: Optional[Mapping[int, HeaterRef]] = None,
        statistics: Optional[StatisticsPort] = None,
    ) -> None:
        self._registry = registry
        self._thermal = thermal
        self._scheduler = scheduler
        self._clock = clock
        self._statistics = statistics
        self._auxiliary_heaters = dict(auxiliary_heaters or {})
        if any(
            isinstance(key, bool) or not isinstance(key, int) or key < 0
            for key in self._auxiliary_heaters
        ):
            raise ValueError("auxiliary heater ids must be non-negative integers")
        if any(not isinstance(heater, HeaterRef) for heater in self._auxiliary_heaters.values()):
            raise TypeError("auxiliary heater values must be HeaterRef instances")
        self._sessions: Dict[ToolId, _Session] = {}
        self._heaters: Dict[HeaterRef, _HeaterRuntime] = {}
        for tool in registry:
            policy = tool.configured_profile.thermal_policy
            self._sessions[tool.id] = _Session(
                active_target=policy.default_active,
                standby_target=policy.default_standby,
                idle_to_standby_time=policy.idle_to_standby_time,
                standby_to_off_time=policy.standby_to_off_time,
            )
            self._heaters.setdefault(tool.heater, _HeaterRuntime(tool.heater))
        self._failures: list[ControllerFailure] = []

    @staticmethod
    def standby_callback_id(heater: HeaterRef) -> str:
        return f"thermal.standby:{heater}"

    @staticmethod
    def off_callback_id(heater: HeaterRef) -> str:
        return f"thermal.off:{heater}"

    @property
    def controller_failures(self) -> tuple[ControllerFailure, ...]:
        return tuple(self._failures)

    def snapshot(self, tool_id: ToolId | int) -> ThermalSnapshot:
        tool = self._tool(tool_id)
        session = self._sessions[tool.id]
        runtime = self._heaters[tool.heater]
        mode = runtime.mode if runtime.owner_tool == tool.id else ThermalMode.OFF
        return ThermalSnapshot(session.active_target, session.standby_target, mode)

    def owner(self, heater: HeaterRef) -> Optional[ToolId]:
        return self._heaters[heater].owner_tool

    def preheat_target(self, tool_id: ToolId | int) -> Optional[WaitTarget]:
        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id:
            return None
        return runtime.preheat_target

    def set_temperature(
        self,
        tool_id: ToolId | int,
        *,
        active_target: Optional[float] = None,
        standby_target: Optional[float] = None,
        mode: Optional[ThermalMode] = None,
        idle_to_standby_time: Optional[float] = None,
        standby_to_off_time: Optional[float] = None,
    ) -> ThermalSnapshot:
        """Update non-persistent session values and optionally demand a mode."""

        tool = self._tool(tool_id)
        session = self._sessions[tool.id]
        active = session.active_target if active_target is None else _temperature(
            "active_target", active_target
        )
        standby = session.standby_target if standby_target is None else _temperature(
            "standby_target", standby_target
        )
        idle_timeout = (
            session.idle_to_standby_time
            if idle_to_standby_time is None
            else _duration("idle_to_standby_time", idle_to_standby_time)
        )
        off_timeout = (
            session.standby_to_off_time
            if standby_to_off_time is None
            else _duration("standby_to_off_time", standby_to_off_time)
        )
        if mode is not None and not isinstance(mode, ThermalMode):
            raise TypeError("mode must be a ThermalMode")

        if mode is not None:
            target = self._target_for_mode(mode, active, standby)
            self._apply_demand(tool, mode, target)
        else:
            self._update_current_target(tool, active, standby)
        session.active_target = active
        session.standby_target = standby
        session.idle_to_standby_time = idle_timeout
        session.standby_to_off_time = off_timeout
        if (
            mode is None
            and (idle_to_standby_time is not None or standby_to_off_time is not None)
        ):
            self._reschedule_pending(tool)
        return self.snapshot(tool.id)

    def preheat(
        self,
        tool_id: ToolId | int,
        *,
        target: WaitTarget = WaitTarget.ACTIVE,
        temperature: Optional[float] = None,
        wait: bool = False,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> Optional[HeaterObservation]:
        tool = self._tool(tool_id)
        if target not in (WaitTarget.ACTIVE, WaitTarget.STANDBY):
            raise ValueError("preheat target must be ACTIVE or STANDBY")
        session = self._sessions[tool.id]
        active = session.active_target
        standby = session.standby_target
        if temperature is not None:
            value = _temperature("temperature", temperature)
            if target is WaitTarget.ACTIVE:
                active = value
            else:
                standby = value
        desired = active if target is WaitTarget.ACTIVE else standby
        mode = (
            ThermalMode.PREHEAT
            if target is WaitTarget.ACTIVE
            else ThermalMode.STANDBY
        )
        self._apply_demand(tool, mode, desired)
        self._heaters[tool.heater].preheat_target = target
        session.active_target = active
        session.standby_target = standby
        if not wait:
            return None
        return self.wait(
            tool.id,
            target=target,
            mode=WaitMode.HEAT,
            tolerance=tolerance,
            timeout=timeout,
        )

    def cancel_preheat(self, tool_id: ToolId | int) -> None:
        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id:
            return
        if runtime.preheat_target is None:
            raise ThermalStateError("cannot cancel a non-preheat thermal demand")
        self._apply_demand(tool, ThermalMode.OFF, 0.0)

    def wait(
        self,
        tool_id: ToolId | int,
        *,
        target: WaitTarget = WaitTarget.CURRENT,
        mode: Optional[WaitMode] = None,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
        stable_time: Optional[float] = None,
    ) -> HeaterObservation:
        tool = self._tool(tool_id)
        policy = self._policy(tool)
        if not isinstance(target, WaitTarget):
            raise TypeError("target must be a WaitTarget")
        wait_mode = policy.wait_mode if mode is None else mode
        if not isinstance(wait_mode, WaitMode):
            raise TypeError("mode must be a WaitMode")
        frozen = self._wait_target(tool, target)
        request = ThermalWaitRequest(
            frozen_target=frozen,
            mode=wait_mode,
            tolerance=policy.tolerance if tolerance is None else tolerance,
            timeout=policy.timeout if timeout is None else timeout,
            stable_time=(
                policy.stable_time if stable_time is None and wait_mode is WaitMode.STABLE
                else (0.0 if stable_time is None else stable_time)
            ),
        )
        return self._thermal.wait(tool.heater, request)

    def schedule_idle(self, tool_id: ToolId | int) -> None:
        """Start the policy's ACTIVE -> STANDBY -> OFF timer chain."""

        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id:
            raise ThermalStateError("only the heater owner can schedule its idle policy")
        if runtime.mode is not ThermalMode.ACTIVE:
            raise ThermalStateError("idle policy requires an ACTIVE heater demand")
        self._cancel_timer(runtime)
        runtime.generation += 1
        generation = runtime.generation
        deadline = (
            self._clock.monotonic() + self._sessions[tool.id].idle_to_standby_time
        )
        runtime.timer = self._scheduler.schedule_at(
            self.standby_callback_id(tool.heater),
            deadline,
            lambda: self._standby_callback(tool.id, generation),
        )

    def schedule_inactivity(self, tool_id: ToolId | int) -> None:
        """Start or repair the parked-tool idle chain.

        Legacy dropoff actions may already have requested STANDBY.  In that
        case only the remaining STANDBY -> OFF timer is required.  New actions
        can omit thermal commands and let the normal ACTIVE chain run here.
        """

        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id or runtime.mode is ThermalMode.OFF:
            return
        if runtime.mode is ThermalMode.ACTIVE:
            self.schedule_idle(tool.id)
            return
        if runtime.mode is not ThermalMode.STANDBY:
            raise ThermalStateError(
                "parked-tool inactivity requires ACTIVE or STANDBY demand"
            )
        self._cancel_timer(runtime)
        runtime.generation += 1
        generation = runtime.generation
        deadline = (
            self._clock.monotonic() + self._sessions[tool.id].standby_to_off_time
        )
        runtime.timer = self._scheduler.schedule_at(
            self.off_callback_id(tool.heater),
            deadline,
            lambda: self._off_callback(tool.id, generation),
        )

    def activate(self, tool_id: ToolId | int) -> ThermalSnapshot:
        tool = self._tool(tool_id)
        session = self._sessions[tool.id]
        self._apply_demand(tool, ThermalMode.ACTIVE, session.active_target)
        return self.snapshot(tool.id)

    def off_all(self) -> None:
        """Immediately request OFF for every distinct configured tool heater."""

        for heater, runtime in self._heaters.items():
            previous_owner = runtime.owner_tool
            previous_mode = runtime.mode
            self._cancel_timer(runtime)
            self._thermal.set_target(heater, 0.0)
            runtime.generation += 1
            runtime.mode = ThermalMode.OFF
            runtime.target = 0.0
            runtime.owner_tool = None
            runtime.preheat_target = None
            self._notify_transition(
                previous_owner, previous_mode, runtime.owner_tool, runtime.mode
            )

    # Parsed compatibility command entry points.
    def m104(self, tool_id: ToolId | int, temperature: float) -> ThermalSnapshot:
        return self.set_temperature(
            tool_id, active_target=temperature, mode=ThermalMode.ACTIVE
        )

    def m109(
        self,
        tool_id: ToolId | int,
        temperature: float,
        *,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> Optional[HeaterObservation]:
        # This ordering is intentional: M109 always heats an OFF tool before wait.
        self.set_temperature(
            tool_id, active_target=temperature, mode=ThermalMode.ACTIVE
        )
        if float(temperature) == 0:
            return None
        return self.wait(
            tool_id,
            target=WaitTarget.ACTIVE,
            mode=WaitMode.HEAT,
            tolerance=tolerance,
            timeout=timeout,
        )

    def m116(
        self,
        tool_id: ToolId | int,
        *,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> HeaterObservation:
        tool = self._tool(tool_id)
        observation = self._thermal.observe(tool.heater)
        if int(observation.target) <= 40:
            return observation
        policy = self._policy(tool)
        return self._thermal.wait(
            tool.heater,
            ThermalWaitRequest(
                frozen_target=observation.target,
                mode=WaitMode.RANGE,
                tolerance=policy.tolerance if tolerance is None else tolerance,
                timeout=policy.timeout if timeout is None else timeout,
            ),
        )

    def m568(
        self,
        tool_id: ToolId | int,
        *,
        active_target: Optional[float] = None,
        standby_target: Optional[float] = None,
        mode: Optional[ThermalMode] = None,
        idle_to_standby_time: Optional[float] = None,
        standby_to_off_time: Optional[float] = None,
    ) -> ThermalSnapshot:
        return self.set_temperature(
            tool_id,
            active_target=active_target,
            standby_target=standby_target,
            mode=mode,
            idle_to_standby_time=idle_to_standby_time,
            standby_to_off_time=standby_to_off_time,
        )

    def set_tool_temperature(
        self,
        tool_id: ToolId | int,
        *,
        active_target: Optional[float] = None,
        standby_target: Optional[float] = None,
        mode: Optional[ThermalMode] = None,
        idle_to_standby_time: Optional[float] = None,
        standby_to_off_time: Optional[float] = None,
    ) -> ThermalSnapshot:
        return self.set_temperature(
            tool_id,
            active_target=active_target,
            standby_target=standby_target,
            mode=mode,
            idle_to_standby_time=idle_to_standby_time,
            standby_to_off_time=standby_to_off_time,
        )

    def temperature_wait_with_tolerance(
        self,
        current_tool: ToolId | int | None = None,
        *,
        heater_id: Optional[int] = None,
        tolerance: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> tuple[HeaterObservation, ...]:
        """Compatibility wait, including the legacy bed-before-tool ordering.

        Auxiliary numeric ids are adapter configuration, not fake Tool objects.
        With no explicit ``heater_id``, auxiliary id 0 is waited first when
        configured, followed by ``current_tool``.
        """

        observations = []
        if heater_id is not None:
            try:
                heater = self._auxiliary_heaters[heater_id]
            except KeyError as exc:
                raise KeyError(f"unknown auxiliary heater id: {heater_id}") from exc
            observations.append(
                self._wait_heater_current(heater, WaitMode.RANGE, tolerance, timeout)
            )
            return tuple(observations)
        if 0 in self._auxiliary_heaters:
            observations.append(
                self._wait_heater_current(
                    self._auxiliary_heaters[0], WaitMode.RANGE, tolerance, timeout
                )
            )
        if current_tool is not None:
            observations.append(
                self.m116(current_tool, tolerance=tolerance, timeout=timeout)
            )
        return tuple(observations)

    def _tool(self, tool_id: ToolId | int) -> ToolSpec:
        return self._registry[tool_id]

    @staticmethod
    def _policy(tool: ToolSpec) -> ThermalPolicy:
        return tool.configured_profile.thermal_policy

    @staticmethod
    def _target_for_mode(mode: ThermalMode, active: float, standby: float) -> float:
        if mode is ThermalMode.OFF:
            return 0.0
        if mode is ThermalMode.STANDBY:
            return standby
        return active

    def _claim(self, tool: ToolSpec) -> _HeaterRuntime:
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool is not None and runtime.owner_tool != tool.id:
            raise ThermalConflictError(
                f"heater {tool.heater} is owned by T{runtime.owner_tool}, not T{tool.id}"
            )
        return runtime

    def _apply_demand(self, tool: ToolSpec, mode: ThermalMode, target: float) -> None:
        target = _temperature("target", target)
        if target == 0:
            mode = ThermalMode.OFF
        runtime = self._claim(tool)
        previous_owner = runtime.owner_tool
        previous_mode = runtime.mode
        self._cancel_timer(runtime)
        self._thermal.set_target(tool.heater, target)
        runtime.generation += 1
        runtime.mode = mode
        runtime.target = target
        runtime.owner_tool = None if mode is ThermalMode.OFF else tool.id
        runtime.preheat_target = None
        self._notify_transition(
            previous_owner, previous_mode, runtime.owner_tool, runtime.mode
        )

    def _update_current_target(
        self, tool: ToolSpec, active_target: float, standby_target: float
    ) -> None:
        """Apply a matching session target without disturbing an idle chain."""

        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id:
            return
        if runtime.preheat_target is WaitTarget.ACTIVE:
            target = active_target
        elif runtime.preheat_target is WaitTarget.STANDBY:
            target = standby_target
        elif runtime.mode is ThermalMode.ACTIVE:
            target = active_target
        elif runtime.mode is ThermalMode.STANDBY:
            target = standby_target
        else:
            return
        if target == runtime.target:
            return
        previous_owner = runtime.owner_tool
        previous_mode = runtime.mode
        self._thermal.set_target(tool.heater, target)
        runtime.target = target
        if target == 0:
            self._cancel_timer(runtime)
            runtime.mode = ThermalMode.OFF
            runtime.owner_tool = None
            runtime.preheat_target = None
        self._notify_transition(
            previous_owner, previous_mode, runtime.owner_tool, runtime.mode
        )

    def _notify_transition(
        self,
        previous_owner: Optional[ToolId],
        previous_mode: ThermalMode,
        current_owner: Optional[ToolId],
        current_mode: ThermalMode,
    ) -> None:
        """Report ownership/mode changes without affecting heater control."""

        if self._statistics is None or (
            previous_owner == current_owner and previous_mode is current_mode
        ):
            return
        def emit(tool_id: ToolId, old: ThermalMode, new: ThermalMode) -> None:
            try:
                self._statistics.thermal_changed(tool_id, old, new)
            except Exception:
                # Statistics are observational and must never alter thermal behavior.
                pass

        if previous_owner is not None and previous_owner != current_owner:
            emit(previous_owner, previous_mode, ThermalMode.OFF)
        if current_owner is not None and previous_owner != current_owner:
            emit(current_owner, ThermalMode.OFF, current_mode)
        elif current_owner is not None:
            emit(current_owner, previous_mode, current_mode)

    def _reschedule_pending(self, tool: ToolSpec) -> None:
        runtime = self._heaters[tool.heater]
        if runtime.owner_tool != tool.id or runtime.timer is None:
            return
        if runtime.mode is ThermalMode.ACTIVE:
            callback_id = self.standby_callback_id(tool.heater)
            delay = self._sessions[tool.id].idle_to_standby_time
            callback_factory = self._standby_callback
        elif runtime.mode is ThermalMode.STANDBY:
            callback_id = self.off_callback_id(tool.heater)
            delay = self._sessions[tool.id].standby_to_off_time
            callback_factory = self._off_callback
        else:
            return
        self._cancel_timer(runtime)
        runtime.generation += 1
        generation = runtime.generation
        runtime.timer = self._scheduler.schedule_at(
            callback_id,
            self._clock.monotonic() + delay,
            lambda: callback_factory(tool.id, generation),
        )

    def _cancel_timer(self, runtime: _HeaterRuntime) -> None:
        if runtime.timer is not None:
            self._scheduler.cancel(runtime.timer)
            runtime.timer = None
        # Invalidate callbacks even when a scheduler has already dequeued one.
        runtime.generation += 1

    def _wait_target(self, tool: ToolSpec, target: WaitTarget) -> float:
        session = self._sessions[tool.id]
        if target is WaitTarget.ACTIVE:
            return session.active_target
        if target is WaitTarget.STANDBY:
            return session.standby_target
        return self._thermal.observe(tool.heater).target

    def _wait_heater_current(
        self,
        heater: HeaterRef,
        mode: WaitMode,
        tolerance: Optional[float],
        timeout: Optional[float],
    ) -> HeaterObservation:
        observation = self._thermal.observe(heater)
        if int(observation.target) <= 40:
            return observation
        # Auxiliary heaters have no ToolProfile. Compatibility defaults mirror
        # the profile defaults, while parsed overrides remain explicit.
        request = ThermalWaitRequest(
            frozen_target=observation.target,
            mode=mode,
            tolerance=1.0 if tolerance is None else tolerance,
            timeout=900.0 if timeout is None else timeout,
        )
        return self._thermal.wait(heater, request)

    def _standby_callback(self, tool_id: ToolId, generation: int) -> None:
        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.generation != generation or runtime.owner_tool != tool.id:
            return
        runtime.timer = None
        try:
            session = self._sessions[tool.id]
            previous_owner = runtime.owner_tool
            previous_mode = runtime.mode
            self._thermal.set_target(tool.heater, session.standby_target)
            runtime.mode = (
                ThermalMode.OFF
                if session.standby_target == 0
                else ThermalMode.STANDBY
            )
            runtime.target = session.standby_target
            runtime.preheat_target = None
            if session.standby_target == 0:
                runtime.owner_tool = None
                runtime.generation += 1
                self._notify_transition(
                    previous_owner, previous_mode, runtime.owner_tool, runtime.mode
                )
                return
            self._notify_transition(
                previous_owner, previous_mode, runtime.owner_tool, runtime.mode
            )
            runtime.generation += 1
            off_generation = runtime.generation
            runtime.timer = self._scheduler.schedule_at(
                self.off_callback_id(tool.heater),
                self._clock.monotonic()
                + self._sessions[tool.id].standby_to_off_time,
                lambda: self._off_callback(tool.id, off_generation),
            )
        except Exception as exc:  # reactor callback boundary
            self._failures.append(
                ControllerFailure(tool.heater, "standby", generation, exc)
            )

    def _off_callback(self, tool_id: ToolId, generation: int) -> None:
        tool = self._tool(tool_id)
        runtime = self._heaters[tool.heater]
        if runtime.generation != generation or runtime.owner_tool != tool.id:
            return
        runtime.timer = None
        try:
            previous_owner = runtime.owner_tool
            previous_mode = runtime.mode
            self._thermal.set_target(tool.heater, 0.0)
            runtime.mode = ThermalMode.OFF
            runtime.target = 0.0
            runtime.owner_tool = None
            runtime.preheat_target = None
            runtime.generation += 1
            self._notify_transition(
                previous_owner, previous_mode, runtime.owner_tool, runtime.mode
            )
        except Exception as exc:  # reactor callback boundary
            self._failures.append(ControllerFailure(tool.heater, "off", generation, exc))
