#!/usr/bin/env python3

# BAM
from .diagnostic_status import DiagnosticStatus, DiagnosticStatusWrapper

# PYTHON
from collections import deque
import time
from typing import List, Optional, Union, SupportsIndex, TYPE_CHECKING

if TYPE_CHECKING:
    from .common_tasks import DiagnosticValue



def diagnostic_level_name(level: int) -> str:
    """Return the string name associated with a DiagnosticStatus level."""
    if level == DiagnosticStatus.OK:
        return "OK"
    elif level == DiagnosticStatus.WARN:
        return "WARN"
    elif level == DiagnosticStatus.ERROR:
        return "ERROR"
    elif level == DiagnosticStatus.STALE:
        return "STALE"
    else:
        return f"UNKNOWN({level})"

def summarise_diagnostic_list(stat_list: List[DiagnosticStatusWrapper], inital_summary: Optional[DiagnosticStatusWrapper] = None) -> DiagnosticStatusWrapper:
    """Combine multiple diagnostic statuses into a single summary
    https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_diagnostic_updater.py#L131
    """
    if inital_summary is None:
        inital_summary = DiagnosticStatusWrapper()
        inital_summary.clearSummary()

    combined_summary = inital_summary

    for stat in stat_list:
        combined_summary.mergeSummary(stat)

    return combined_summary

def check_thresholds(threshold: 'DiagnosticValue', value: float, stat_name: str, upper_bound: bool) -> DiagnosticStatusWrapper:
    """
    Check if the value exceeds the threshold and return a DiagnosticStatusWrapper.
    """
    stat = DiagnosticStatusWrapper()

    if threshold.error == 0 and threshold.warn == 0:
        stat.summary(DiagnosticStatus.OK, f"SKIPPING")
        # stat.summary(DiagnosticStatus.OK, f"{stat_name} thresholds are both 0, skipping check")
        return stat

    if upper_bound:
        if value > threshold.error:
            stat.summary(DiagnosticStatus.ERROR, f"{stat_name} exceeds error threshold: {value} > {threshold.error}")
        elif value > threshold.warn:
            stat.summary(DiagnosticStatus.WARN, f"{stat_name} exceeds warning threshold: {value} > {threshold.warn}")
        else:
            stat.summary(DiagnosticStatus.OK, f"OK")
    else:
        if value < threshold.error:
            stat.summary(DiagnosticStatus.ERROR, f"{stat_name} below error threshold: {value} < {threshold.error}")
        elif value < threshold.warn:
            stat.summary(DiagnosticStatus.WARN, f"{stat_name} below warning threshold: {value} < {threshold.warn}")
        else:
            stat.summary(DiagnosticStatus.OK, f"OK")
    return stat


def make_queue(length: int | float, type: str):
    """
    Make a queue with a max length.
    """
    if type == "timed":
        return TimedQueue[float](max_age_seconds=length)
    else:
        return deque(maxlen=int(length))

class TimedQueue[T]:
    """
    A queue that automatically prunes items that are older than a certain age.
    """
    def __init__(self, max_age_seconds: float):
        self.max_age = max_age_seconds
        self._queue: deque[tuple[T, float]] = deque()

    def append(self, item: T) -> None:
        now = time.time()
        self._queue.append((item, now))
        self._prune(now)

    def _prune(self, now=None):
        now = now or time.time()
        while self._queue and (now - self._queue[0][1]) > self.max_age:
            self._queue.popleft()

    def __getitem__(self, index: Union[SupportsIndex, slice]) -> Union[T, List[T]]:
        self._prune()
        if isinstance(index, slice):
            return [item for item, _ in list(self._queue)[index]]
        return self._queue[index][0]

    def __len__(self) -> int:
        self._prune()
        return len(self._queue)

    def __iter__(self):
        self._prune()
        return (item for item, _ in self._queue)

    def clear(self) -> None:
        self._queue.clear()