from ktcc.ports import ThermalMode
from ktcc.statistics import MechanicalKind, StatisticsService, format_print_statistics, format_statistics
from ktcc.tools import ToolId


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class Repository:
    def __init__(self):
        self.saved = []

    def save(self, snapshot):
        self.saved.append(snapshot)


def test_collects_completed_mechanics_selection_lock_and_thermal_intervals():
    clock = Clock()
    service = StatisticsService([ToolId(1)], clock)

    service.mechanical_started(MechanicalKind.MOUNT, ToolId(1), "change-1")
    service.selection_changed(None, ToolId(1))
    service.thermal_changed(ToolId(1), ThermalMode.OFF, ThermalMode.ACTIVE)
    clock.now = 4.0
    service.mechanical_completed(MechanicalKind.MOUNT, ToolId(1), "change-1")
    service.lock_completed()
    clock.now = 10.0
    projected = service.snapshot()

    assert projected.global_totals.total_toolmounts == 1
    assert projected.global_totals.total_time_spent_mounting == 4
    assert projected.global_totals.total_toollocks == 1
    assert projected.tools[ToolId(1)].toolmounts_started == 1
    assert projected.tools[ToolId(1)].toolmounts_completed == 1
    assert projected.tools[ToolId(1)].time_selected == 10
    assert projected.tools[ToolId(1)].time_heater_active == 10


def test_duplicate_and_unmatched_mechanical_events_are_safe():
    clock = Clock()
    service = StatisticsService([ToolId(0)], clock)
    service.mechanical_started(MechanicalKind.UNMOUNT, ToolId(0), "x")
    service.mechanical_started(MechanicalKind.UNMOUNT, ToolId(0), "x")
    clock.now = 3
    service.mechanical_completed(MechanicalKind.UNMOUNT, ToolId(0), "x")
    service.mechanical_completed(MechanicalKind.UNMOUNT, ToolId(0), "x")

    stats = service.snapshot()
    assert stats.global_totals.total_toolunmounts == 1
    assert stats.tools[ToolId(0)].toolunmounts_started == 1
    assert stats.tools[ToolId(0)].toolunmounts_completed == 1


def test_preheat_is_not_counted_and_open_active_interval_is_projected():
    clock = Clock()
    service = StatisticsService([ToolId(2)], clock)
    service.thermal_changed(ToolId(2), ThermalMode.OFF, ThermalMode.PREHEAT)
    clock.now = 8
    service.thermal_changed(ToolId(2), ThermalMode.PREHEAT, ThermalMode.ACTIVE)
    clock.now = 11
    assert service.snapshot().tools[ToolId(2)].time_heater_active == 3
    assert service.snapshot().tools[ToolId(2)].time_heater_standby == 0


def test_print_baseline_reset_and_flush_use_projected_values():
    clock = Clock()
    repository = Repository()
    service = StatisticsService([ToolId(0)], clock, repository=repository)
    service.selection_changed(None, ToolId(0))
    clock.now = 5
    service.begin_print()
    clock.now = 9
    assert service.print_snapshot().tools[ToolId(0)].time_selected == 4
    service.flush()
    assert repository.saved[-1].tools[ToolId(0)].time_selected == 9

    service.reset()
    clock.now = 12
    assert service.snapshot().tools[ToolId(0)].time_selected == 3


def test_reports_keep_old_vocabulary_and_numeric_tool_order():
    service = StatisticsService([ToolId(10), ToolId(2)], Clock())
    report = format_statistics(service.snapshot())
    print_report = format_print_statistics(service.print_snapshot())
    assert report.startswith("ToolChanger Statistics:")
    assert report.index("Tool#2:") < report.index("Tool#10:")
    assert "Completed 0 out of 0 mounts" in report
    assert print_report.startswith("ToolChanger Statistics for this print:")
