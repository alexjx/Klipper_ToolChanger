from ktcc.statistics import StatisticsRepository, decode_statistics
from ktcc.statistics.storage import SWAP_KEY, TOOL_PREFIX
from ktcc.tools import ToolId


class Store:
    def __init__(self, variables=None, fail_on=None):
        self.variables = dict(variables or {})
        self.fail_on = fail_on

    def read_variables(self):
        return dict(self.variables)

    def write_variable(self, name, value):
        if name == self.fail_on:
            raise OSError("injected")
        self.variables[name] = value


def test_decode_tolerates_partial_malformed_values_and_ignores_removed_tools():
    decoded = decode_statistics({
        SWAP_KEY: {"total_toollocks": 4, "total_toolmounts": "bad"},
        f"{TOOL_PREFIX}2": {"toolmounts_completed": 3, "time_selected": None},
        f"{TOOL_PREFIX}99": {"toolmounts_completed": 100},
    }, [ToolId(2)])
    assert decoded.global_totals.total_toollocks == 4
    assert decoded.global_totals.total_toolmounts == 0
    assert decoded.tools[ToolId(2)].toolmounts_completed == 3
    assert decoded.tools[ToolId(2)].time_selected == 0
    assert ToolId(99) not in decoded.tools


def test_round_trip_uses_exact_keys_and_zeroes_old_tracking_fields():
    source = Store({
        SWAP_KEY: {"total_time_spent_mounting": 1.26},
        f"{TOOL_PREFIX}1": {"time_selected": 7.5},
    })
    repository = StatisticsRepository(source)
    repository.save(repository.load([ToolId(1)]))
    assert set(source.variables) == {SWAP_KEY, f"{TOOL_PREFIX}1"}
    assert source.variables[SWAP_KEY]["total_time_spent_mounting"] == 1.3
    tool = source.variables[f"{TOOL_PREFIX}1"]
    assert tool["time_selected"] == 7.5
    assert tool["tracked_start_time_selected"] == 0
    assert tool["tracked_mount_start_time"] == 0


def test_repository_write_failures_are_observable():
    repository = StatisticsRepository(Store(fail_on=SWAP_KEY))
    snapshot = decode_statistics({}, [ToolId(0)])
    try:
        repository.save(snapshot)
    except OSError as error:
        assert str(error) == "injected"
    else:
        raise AssertionError("storage error was swallowed")
