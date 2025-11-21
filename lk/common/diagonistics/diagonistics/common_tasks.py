#!/usr/bin/env python3

"""
Function structure has been inspired from:
https://github.com/ros/diagnostics/tree/ros2/diagnostic_updater/diagnostic_updater
https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_update_functions.py
"""

# BAM
from .diagnostic_status import DiagnosticStatus, DiagnosticTask, DiagnosticStatusWrapper
from .diagonstic_helper import check_thresholds, make_queue

# PYTHON
import numpy as np
import time
from typing import List, Tuple, Generic, TypeVar, get_type_hints, get_args
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field, asdict


@dataclass
class DiagnosticValue:
    """
    Diagnostic threshold value with warn and error levels.
    """

    warn: float = 0.0
    error: float = 0.0


@dataclass
class BoxChartThresholds:
    """
    Thresholds for BoxChart statistics (min, max, mean, std).
    """

    min: DiagnosticValue = field(default_factory=DiagnosticValue)
    max: DiagnosticValue = field(default_factory=DiagnosticValue)
    mean_min: DiagnosticValue = field(default_factory=DiagnosticValue)
    mean_max: DiagnosticValue = field(default_factory=DiagnosticValue)
    std: DiagnosticValue = field(default_factory=DiagnosticValue)


@dataclass
class ValueThresholds:
    """
    Thresholds for single value monitoring (upper/lower bounds).
    """

    upper_bound: DiagnosticValue = field(default_factory=DiagnosticValue)
    lower_bound: DiagnosticValue = field(default_factory=DiagnosticValue)


@dataclass
class CounterThresholds:
    """
    Thresholds for counter monitoring.
    """

    count: DiagnosticValue = field(default_factory=DiagnosticValue)


@dataclass
class PercentThresholds:
    """
    Thresholds for percentage monitoring.
    """

    percent: DiagnosticValue = field(default_factory=DiagnosticValue)


ThresholdsType = TypeVar(
    "ThresholdsType",
    ValueThresholds,
    CounterThresholds,
    PercentThresholds,
    BoxChartThresholds,
)


@dataclass
class UnifiedDiagnosticConfig(Generic[ThresholdsType]):
    """
    Configuration for diagnostic tasks.
    Combines thresholds with window settings for easy serialization.

    queue_val interpretation depends on queue_type:
    - "timed": queue_val is time window in seconds (e.g., 10.0 = last 10 seconds)
    - "count": queue_val is number of items (e.g., 100 = last 100 items)

    For EventPeriodTask, freq_queue_val and freq_queue_type can be used
    to set separate window settings for frequency calculation.
    """

    thresholds: ThresholdsType
    queue_val: int | float = 10
    queue_type: str = "count"
    freq_queue_val: int | float | None = (
        None  # Optional: for EventPeriodTask frequency calculation
    )
    freq_queue_type: str | None = (
        None  # Optional: for EventPeriodTask frequency calculation
    )


class UnifiedDiagnosticTask(ABC, DiagnosticTask, Generic[ThresholdsType]):
    """
    Base class that separates value calculation from threshold checking.

    This unified architecture provides:
    1. Combined value calculation and threshold checking in one place
    2. Consistent interface across all diagnostic task types
    3. Easy extensibility for new task types
    4. Centralized threshold checking logic
    5. Efficient: values are only calculated when needed for threshold checking

    Subclasses must implement:
    - _calculate_and_check_thresholds(): Returns List[Tuple[str, float, bool]] of (threshold_attr, calculated_value, upper_bound)

    Subclasses can optionally override:
    - _add_additional_diagnostics(): Add diagnostic values that aren't used for threshold checking (e.g., last_value)
    """

    def __init__(
        self, config: UnifiedDiagnosticConfig[ThresholdsType], name: str = None
    ):
        self.name = name or "unnamed_task"
        super().__init__(self.name)
        self.config = config
        self.thresholds = config.thresholds
        self.queue_val = config.queue_val
        self.queue_type = config.queue_type
        self.lock = threading.Lock()
        self.values = (
            make_queue(self.queue_val, self.queue_type) if self.queue_val > 0 else None
        )
        self.stat = DiagnosticStatusWrapper(
            level=DiagnosticStatus.STALE, name=self.name, message="__init__"
        )

    @abstractmethod
    def _calculate_and_check_thresholds(self) -> List[Tuple[str, float, bool]]:
        """
        Subclasses override this to calculate values and define threshold checks in one place.
        Returns: List of (threshold_attr, calculated_value, upper_bound) tuples

        Example:
            return [
                ("mean_max", 42.5, True),    # Check mean=42.5 against mean_max threshold (upper bound)
                ("mean_min", 42.5, False),   # Check mean=42.5 against mean_min threshold (lower bound)
                ("std", 12.3, True),         # Check std=12.3 against std threshold (upper bound)
                ("max", 67.8, True),         # Check max=67.8 against max threshold (upper bound)
                ("min", 25.0, False),        # Check min=25.0 against min threshold (lower bound)
            ]
        """

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        threshold_checks = self._calculate_and_check_thresholds()

        if not threshold_checks:
            # Return OK, while waiting for values to populate
            stat.summary(DiagnosticStatus.OK, "waiting for values...")
            # stat.summary(DiagnosticStatus.STALE, "No values to compute.")
            self.stat = stat
            return stat

        combined_status = DiagnosticStatusWrapper()

        for threshold_attr, value, upper_bound in threshold_checks:
            threshold_value = getattr(self.thresholds, threshold_attr)

            threshold_status = check_thresholds(
                threshold_value, value, threshold_attr, upper_bound
            )
            combined_status.mergeSummary(threshold_status)

            # Add to diagnostic output
            stat.add(
                f"{threshold_attr} [{threshold_value.warn}, {threshold_value.error}]",
                f"{value:.3f}",
            )

        # Add additional diagnostic values (like last_value) that aren't used for thresholds
        self._add_additional_diagnostics(stat)

        # BUG: you need to fill in the inital stat, as it holds the name info, you cannot create a new stat
        stat.summary(combined_status)
        self.stat = stat
        return stat

    def _add_additional_diagnostics(self, stat: DiagnosticStatusWrapper):
        """
        Subclasses can override this to add additional diagnostic values
        that aren't used for threshold checking (e.g., last_value).
        """

    def to_config(self) -> dict:
        """Export serializable config"""
        return asdict(self.config)

    @classmethod
    def from_config(cls, config_dict: dict, name: str = None):
        """
        Reconstruct task from config dict using dacite.
        Automatically determines threshold type from Generic parameter and
        handles nested dataclasses.
        """
        try:
            from dacite import from_dict, Config as DaciteConfig
        except ImportError:
            raise ImportError(
                "dacite is required for from_config(). Install with: pip install dacite"
            )

        # Get threshold type from Generic parameter
        threshold_class = get_args(cls.__orig_bases__[0])[0]

        # First use dacite to reconstruct the thresholds (with nested DiagnosticValue)
        thresholds = from_dict(
            data_class=threshold_class, data=config_dict["thresholds"]
        )

        # Now construct the config with the reconstructed thresholds
        config = UnifiedDiagnosticConfig(
            thresholds=thresholds,
            queue_val=config_dict.get("queue_val", 10),
            queue_type=config_dict.get("queue_type", "count"),
            freq_queue_val=config_dict.get("freq_queue_val"),
            freq_queue_type=config_dict.get("freq_queue_type"),
        )

        return cls(config=config, name=name)


class BoxChartTask(UnifiedDiagnosticTask[BoxChartThresholds]):
    """
    Unified implementation of BoxChartTask using the UnifiedDiagnosticTask pattern.
    Calculates statistics (mean, std, min, max, last_value) from a window of values.
    """

    def __init__(
        self, config: UnifiedDiagnosticConfig[BoxChartThresholds], name: str = None
    ):
        super().__init__(config, name)
        self.last_val = 0.0

    def append(self, value: float):
        with self.lock:
            if self.values is not None:
                self.values.append(value)

    def _calculate_and_check_thresholds(self) -> List[Tuple[str, float, bool]]:
        if not self.values or len(self.values) == 0:
            return []

        vals = np.array(self.values)
        mean_val = float(np.mean(vals))
        std_val = float(np.std(vals))
        min_val = float(np.min(vals))
        max_val = float(np.max(vals))
        self.last_val = float(vals[-1])

        return [
            ("mean_max", mean_val, True),
            ("mean_min", mean_val, False),
            ("std", std_val, True),
            ("max", max_val, True),
            ("min", min_val, False),
        ]

    def _add_additional_diagnostics(self, stat: DiagnosticStatusWrapper):
        stat.add("last_value", str(self.last_val))


class ValueTask(UnifiedDiagnosticTask[ValueThresholds]):

    def __init__(
        self, config: UnifiedDiagnosticConfig[ValueThresholds], name: str = None
    ):
        super().__init__(config, name)
        self.value = 0

    def set_value(self, value: float):
        with self.lock:
            self.value = value

    def _calculate_and_check_thresholds(self) -> List[Tuple[str, float, bool]]:
        return [("upper_bound", self.value, True), ("lower_bound", self.value, False)]


class CounterTask(UnifiedDiagnosticTask[CounterThresholds]):
    """
    Unified implementation of CounterTask using the UnifiedDiagnosticTask pattern.
    Counts events over time, supporting both window-based and all-time counting.
    """

    def __init__(
        self, config: UnifiedDiagnosticConfig[CounterThresholds], name: str = None
    ):
        super().__init__(config, name)
        self.all_time_count = 0

    def tick(self, increment: int = 1):
        with self.lock:
            self.all_time_count += increment
            if self.values is not None:
                self.values.append(increment)

    def _calculate_and_check_thresholds(self) -> List[Tuple[str, float, bool]]:
        if self.values is not None:
            count = sum(self.values)
        else:
            count = self.all_time_count
        return [("count", count, True)]


class PercentTask(UnifiedDiagnosticTask[PercentThresholds]):
    """
    Unified implementation of PercentTask using the UnifiedDiagnosticTask pattern.
    Calculates percentage of positive events over time.
    """

    def __init__(
        self, config: UnifiedDiagnosticConfig[PercentThresholds], name: str = None
    ):
        super().__init__(config, name)
        self.all_time_count = 0
        self.all_time_count_positive = 0

    def tick(self, positive_event: bool):
        with self.lock:
            if positive_event:
                self.all_time_count_positive += 1
            self.all_time_count += 1
            if self.values is not None:
                self.values.append(1 if positive_event else 0)

    def _calculate_and_check_thresholds(self) -> List[Tuple[str, float, bool]]:

        if self.values is not None:
            total = len(self.values)
            positive = sum(self.values)
        else:
            total = self.all_time_count
            positive = self.all_time_count_positive

        if total == 0:
            return [("percent", 0.0, True)]

        percent = (positive / total) * 100.0
        return [("percent", percent, True)]

    def _add_additional_diagnostics(self, stat: DiagnosticStatusWrapper):
        stat.add("all_time_count", str(self.all_time_count))
        stat.add("all_time_count_positive", str(self.all_time_count_positive))


class EventPeriodTask(BoxChartTask):
    """Design Notes:

    - Inspired by: https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_update_functions.py

    - Internally helps calculate the frequency of an event. The compomnent must call tick() to indicate the event has happened
    - It takes that frequency and then puts it into a BoxChartTask to check vs thresholds
    - It adds the period, hz, freq per min as key values, as well as the mean, std, min, max for the period. I like using period beacuse its an understandable unit, seconds.
    - add a warm up period so it reports OK? hmm no it should report stale... warm up period, it report ok and then stale...
    - A more complicated option would be to dynamically size the queue based on a certain time window instead of count window...
    - This does make a bit more sense though, Id
    - Right now don't worry about war up period.. todo
    """

    def __init__(
        self, config: UnifiedDiagnosticConfig[BoxChartThresholds], name: str = None
    ):
        """
        EventPeriodTask with separate window settings for frequency calculation.

        Args:
            config: Configuration (uses config.queue_val for stats,
                   config.freq_queue_val for frequency if provided)
            name: Task name (optional, can be set via DiagnosticTaskList)
        """
        super().__init__(config, name)

        # Use freq settings or fall back to main queue settings
        freq_val = (
            config.freq_queue_val
            if config.freq_queue_val is not None
            else config.queue_val
        )
        freq_type = (
            config.freq_queue_type
            if config.freq_queue_type is not None
            else config.queue_type
        )

        self.count = 0
        self.last_period = (
            1e6  # large default value to start, to avoid division by zero!
        )
        self.count_list = make_queue(freq_val, freq_type)
        self.stamp_list = make_queue(freq_val, freq_type)

    def tick(self, increment: int = 1):
        with self.lock:
            self.count += increment
            self.count_list.append(self.count)
            self.stamp_list.append(time.time())

            if len(self.count_list) < 2:
                return

            # Calculate period with moving avg filter. Set length of filter such that count_diff is reasonable value... ie for
            count_diff = self.count_list[-1] - self.count_list[0]  # type: ignore
            time_diff = self.stamp_list[-1] - self.stamp_list[0]  # type: ignore

            if count_diff == 0:  # don't divide by zero, return large number instead
                period = 1e6
            else:
                period = time_diff / count_diff

            self.values.append(period)  # type: ignore
            self.last_period = period
