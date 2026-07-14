from types import SimpleNamespace

from ktcc.statistics import StatisticsService
from ktcc.tools import ToolId
from toollock import ToolLock


class Clock:
    def monotonic(self):
        return 0.0


class Command:
    def __init__(self, parameters=None):
        self.parameters = parameters or {}
        self.responses = []

    def get(self, name, default=None):
        return self.parameters.get(name, default)

    def respond_info(self, message):
        self.responses.append(message)

    def error(self, message):
        return RuntimeError(message)


def facade():
    result = ToolLock.__new__(ToolLock)
    result.runtime = SimpleNamespace(
        statistics=StatisticsService([ToolId(0)], Clock())
    )
    result.log = SimpleNamespace(warning=lambda *args, **kwargs: None)
    return result


def test_statistics_dump_and_print_commands_respond_without_custom_logger():
    tool_lock = facade()
    total = Command()
    tool_lock.cmd_KTCC_DUMP_STATS(total)
    assert total.responses[0].startswith("ToolChanger Statistics:")

    tool_lock.cmd_KTCC_INIT_PRINT_STATS(Command())
    printed = Command()
    tool_lock.cmd_KTCC_DUMP_PRINT_STATS(printed)
    assert printed.responses[0].startswith(
        "ToolChanger Statistics for this print:"
    )


def test_statistics_reset_requires_explicit_confirmation():
    tool_lock = facade()
    tool_lock.runtime.statistics.lock_completed()

    refused = Command()
    tool_lock.cmd_KTCC_RESET_STATS(refused)
    assert "SURE=YES" in refused.responses[0]
    assert tool_lock.runtime.statistics.snapshot().global_totals.total_toollocks == 1

    confirmed = Command({"SURE": "yes"})
    tool_lock.cmd_KTCC_RESET_STATS(confirmed)
    assert tool_lock.runtime.statistics.snapshot().global_totals.total_toollocks == 0
    assert confirmed.responses[-1] == "Statistics RESET."
