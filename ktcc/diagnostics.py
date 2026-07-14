"""Route toolchanger events through the process' standard logging setup.

Klipper already owns the root logging handlers and their asynchronous delivery.
This adapter deliberately adds no handler, file, queue, thread, or rotation
policy of its own.
"""

from __future__ import annotations

import logging
from typing import Protocol

from ktcc.ports import EventLevel, ToolchangerEvent


class _Logger(Protocol):
    def debug(self, message: str) -> None: ...

    def info(self, message: str) -> None: ...

    def warning(self, message: str) -> None: ...

    def error(self, message: str) -> None: ...


class LoggingEventSink:
    """An :class:`EventSink` backed by the named ``ktcc`` logger.

    Diagnostics are secondary effects.  A broken logging handler must not turn
    an otherwise successful tool change into a failed one, so emission errors
    are isolated and exposed only as an in-memory count for health inspection.
    """

    __slots__ = ("_logger", "_failure_count")

    def __init__(self, logger: _Logger | None = None) -> None:
        self._logger = logger if logger is not None else logging.getLogger("ktcc")
        self._failure_count = 0

    @property
    def failure_count(self) -> int:
        """Number of logger calls that raised since this sink was created."""

        return self._failure_count

    def emit(self, event: ToolchangerEvent) -> None:
        if not isinstance(event, ToolchangerEvent):
            raise TypeError("event must be a ToolchangerEvent")

        message = self._format(event)
        log = {
            EventLevel.DEBUG: self._logger.debug,
            EventLevel.INFO: self._logger.info,
            EventLevel.WARNING: self._logger.warning,
            EventLevel.ERROR: self._logger.error,
        }[event.level]
        try:
            log(message)
        except Exception:
            # Logging is observational and must never alter the operation result.
            # In particular, attempting to report this through logging again
            # could recurse into the same failed handler.
            self._failure_count += 1

    @staticmethod
    def _format(event: ToolchangerEvent) -> str:
        tool = "-" if event.tool_id is None else str(event.tool_id)
        transition = event.transition_id or "-"
        phase = "-" if event.phase is None else event.phase.value
        return (
            f"[code={event.code} tool={tool} transition={transition} phase={phase}] "
            f"{event.message}"
        )
