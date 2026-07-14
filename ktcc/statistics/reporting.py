"""Human-readable statistics reports; no logging or G-code side effects."""

import math

from .models import StatisticsSnapshot


def _duration(seconds: float) -> str:
    seconds = max(0, int(math.floor(seconds)))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    result = f"{hours} hours " if hours else ""
    if hours or minutes:
        result += f"{minutes} minutes "
    return result + f"{seconds} seconds"


def format_statistics(snapshot: StatisticsSnapshot, *, for_print: bool = False) -> str:
    suffix = " for this print" if for_print else ""
    totals = snapshot.global_totals
    lines = [f"ToolChanger Statistics{suffix}:", f"KTCC Statistics{suffix}:",
             f"{_duration(totals.total_time_spent_mounting)} spent mounting tools",
             f"{_duration(totals.total_time_spent_unmounting)} spent unmounting tools",
             f"{totals.total_toollocks} tool locks completed",
             f"{totals.total_toolunlocks} tool unlocks completed",
             f"{totals.total_toolmounts} tool mounts completed",
             f"{totals.total_toolunmounts} tool unmounts completed", "------------",
             f"Tool Statistics{suffix}:"]
    for tool_id, stats in snapshot.tools.items():
        mount_average = stats.total_time_spent_mounting / stats.toolmounts_completed if stats.toolmounts_completed else 0
        unmount_average = stats.total_time_spent_unmounting / stats.toolunmounts_completed if stats.toolunmounts_completed else 0
        lines.extend([
            f"Tool#{tool_id.value}:",
            f"Completed {stats.toolmounts_completed} out of {stats.toolmounts_started} mounts in {_duration(stats.total_time_spent_mounting)}. Average of {_duration(mount_average)} per toolmount.",
            f"Completed {stats.toolunmounts_completed} out of {stats.toolunmounts_started} unmounts in {_duration(stats.total_time_spent_unmounting)}. Average of {_duration(unmount_average)} per toolunmount.",
            f"{_duration(stats.time_selected)} spent selected. {_duration(stats.time_heater_active)} with active heater and {_duration(stats.time_heater_standby)} with standby heater.",
            "------------",
        ])
    return "\n".join(lines)


def format_print_statistics(snapshot: StatisticsSnapshot) -> str:
    return format_statistics(snapshot, for_print=True)
