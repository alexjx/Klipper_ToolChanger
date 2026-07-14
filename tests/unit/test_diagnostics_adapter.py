from __future__ import annotations

import logging

import pytest

from ktcc.diagnostics import LoggingEventSink
from ktcc.toolchange.state import TransitionPhase
from ktcc.tools import ToolId
from ktcc.ports import EventLevel, EventSink, ToolchangerEvent


class FakeLogger:
    def __init__(self, *, fail: bool = False) -> None:
        self.fail = fail
        self.records: list[tuple[str, str]] = []

    def _record(self, level: str, message: str) -> None:
        if self.fail:
            raise RuntimeError("injected handler failure")
        self.records.append((level, message))

    def debug(self, message: str) -> None:
        self._record("DEBUG", message)

    def info(self, message: str) -> None:
        self._record("INFO", message)

    def warning(self, message: str) -> None:
        self._record("WARNING", message)

    def error(self, message: str) -> None:
        self._record("ERROR", message)


@pytest.mark.parametrize(
    ("event_level", "expected_level"),
    [
        (EventLevel.DEBUG, "DEBUG"),
        (EventLevel.INFO, "INFO"),
        (EventLevel.WARNING, "WARNING"),
        (EventLevel.ERROR, "ERROR"),
    ],
)
def test_maps_each_event_level_without_changing_context(
    event_level: EventLevel, expected_level: str
) -> None:
    logger = FakeLogger()
    sink = LoggingEventSink(logger)
    event = ToolchangerEvent(
        code="change.checkpoint",
        level=event_level,
        message="pickup action completed",
        tool_id=ToolId(3),
        transition_id="tx-0042",
        phase=TransitionPhase.PICKING_TARGET,
    )

    sink.emit(event)

    assert logger.records == [
        (
            expected_level,
            "[code=change.checkpoint tool=3 transition=tx-0042 "
            "phase=PICKING_TARGET] pickup action completed",
        )
    ]
    assert sink.failure_count == 0


def test_formats_absent_optional_context_explicitly() -> None:
    logger = FakeLogger()
    sink = LoggingEventSink(logger)

    sink.emit(ToolchangerEvent("boot.ready", EventLevel.INFO, "toolchanger ready"))

    assert logger.records == [
        (
            "INFO",
            "[code=boot.ready tool=- transition=- phase=-] toolchanger ready",
        )
    ]


def test_logger_failure_is_isolated_from_business_result() -> None:
    logger = FakeLogger(fail=True)
    sink = LoggingEventSink(logger)
    event = ToolchangerEvent("change.completed", EventLevel.INFO, "complete")

    assert sink.emit(event) is None
    assert sink.emit(event) is None
    assert sink.failure_count == 2


def test_rejects_non_toolchanger_event_before_calling_logger() -> None:
    logger = FakeLogger()
    sink = LoggingEventSink(logger)

    with pytest.raises(TypeError, match="event must be a ToolchangerEvent"):
        sink.emit(object())  # type: ignore[arg-type]

    assert logger.records == []


def test_satisfies_event_sink_protocol() -> None:
    assert isinstance(LoggingEventSink(FakeLogger()), EventSink)


def test_default_uses_named_logger_without_installing_handlers() -> None:
    logger = logging.getLogger("ktcc")
    handlers_before = tuple(logger.handlers)

    sink = LoggingEventSink()

    assert sink.failure_count == 0
    assert tuple(logger.handlers) == handlers_before
