#!/usr/bin/env python3

"""

Function structure has been inspired from: https://github.com/ros/diagnostics/tree/ros2/diagnostic_updater/diagnostic_updater

https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_update_functions.py

"""
# ROS
from rclpy.node import Node

from diagnostic_updater import DiagnosticStatus, DiagnosticTask, DiagnosticStatusWrapper

# BAM
# from bam_agent.agents.generic_agent import GenericAgent # Careful for circular import

# PYTHON
import numpy as np
from collections import deque
from typing import Dict


class DiagonisticValue:
    def __init__(self, warn: float = None, error: float = None):  # type: ignore
        self.warn = warn
        self.error = error

    def __str__(self):
        return f"warn: {self.warn}, error: {self.error}"


# use there combination task or.... seperate ones...
# beacuse its the same list of values. I don't want to have to append to all the multiple lists I do feel its simple to just do it in here...
# I think I can use there logic for combining though...


def check_thresholds(
    threshold: DiagonisticValue, value: float, stat_name: str, upper_bound: bool
) -> DiagnosticStatusWrapper:
    """
    Check if the value exceeds the threshold and return a DiagnosticStatusWrapper.
    """
    stat = DiagnosticStatusWrapper()
    if upper_bound:
        if value > threshold.error:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"{stat_name} exceeds error threshold: {value} > {threshold.error}",
            )
        elif value > threshold.warn:
            stat.summary(
                DiagnosticStatus.WARN,
                f"{stat_name} exceeds warning threshold: {value} > {threshold.warn}",
            )
    else:
        if value < threshold.error:
            stat.summary(
                DiagnosticStatus.ERROR,
                f"{stat_name} below error threshold: {value} < {threshold.error}",
            )
        elif value < threshold.warn:
            stat.summary(
                DiagnosticStatus.WARN,
                f"{stat_name} below warning threshold: {value} < {threshold.warn}",
            )
    return stat


class BoxChartThresholds:
    def __init__(self):
        self.mean_min = DiagonisticValue()
        self.mean_max = DiagonisticValue()
        self.std = DiagonisticValue()
        self.min = DiagonisticValue()
        self.max = DiagonisticValue()

    @staticmethod
    def load_from_node(node: Node, path: str) -> "BoxChartThresholds":
        """
        full_path -> path
        diagnostics.reward.mean_max.warn -> diagnostics.reward
        """
        params = BoxChartThresholds()
        attr_list = ["mean_min", "mean_max", "std", "min", "max"]
        level_list = ["warn", "error"]

        for attr in attr_list:
            for level in level_list:
                param_name = f"{path}.{attr}.{level}"
                setattr(
                    getattr(params, attr),
                    level,
                    node.get_parameter(param_name).get_parameter_value().double_value,
                )
        return params

    def __str__(self):
        return (
            f"BoxChartThresholds(\n"
            f"  mean_min: {self.mean_min}\n"
            f"  mean_max: {self.mean_max}\n"
            f"  std:      {self.std}\n"
            f"  min:      {self.min}\n"
            f"  max:      {self.max}\n"
            f")"
        )

    @staticmethod
    def declare_node_params(node: Node, path: str, defaults=[1e6, 1e6]):
        """
        Declare parameters for the BoxChartThresholds.
        """
        attr_list = ["mean_min", "mean_max", "std", "min", "max"]
        upper_bound = [-1, 1, 1, -1, 1]
        level_list = ["warn", "error"]

        for i, attr in enumerate(attr_list):
            for j, level in enumerate(level_list):
                param_name = f"{path}.{attr}.{level}"
                node.declare_parameter(param_name, upper_bound[i] * defaults[j])


class BoxChartTask(DiagnosticTask):

    def __init__(
        self, name="BoxChartTask", params=BoxChartThresholds(), window_len=100
    ):
        super().__init__(name)

        self.window_len = window_len

        self.values = deque(
            maxlen=window_len
        )  # Could be done by time, but this is reasonable as well...

        self.params = params

    def append(self, value: float):
        self.values.append(value)

    def _compute_statistics(self) -> Dict[str, float]:
        if len(self.values) == 0:
            return {}
        vals = np.array(self.values)
        return {
            "mean": float(np.mean(vals)),
            "std": float(np.std(vals)),
            "min": float(np.min(vals)),
            "max": float(np.max(vals)),
            "last_value": float(vals[-1]),
        }

    def run(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:

        statistics = self._compute_statistics()

        if not statistics:
            stat.summary(DiagnosticStatus.STALE, "No values to compute statistics.")
            return stat

        threshold_checks = [
            ("mean_max", True),  # mean_max threshold, upper bound
            ("mean_min", False),  # mean_min threshold, lower bound
            ("std", True),  # std threshold, upper bound
            ("max", True),  # max threshold, upper bound
            ("min", False),  # min threshold, lower bound
        ]

        # This section is based on:
        # https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_diagnostic_updater.py#L131

        combined_status = DiagnosticStatusWrapper()

        for param_attr, upper_bound in threshold_checks:
            thresholds = getattr(self.params, param_attr)
            statistics_key = param_attr.replace("_max", "").replace(
                "_min", ""
            )  # filter beacuse we use same mean for mean_max and mean_min
            statistics_val = statistics[statistics_key]

            threshold_status = check_thresholds(
                thresholds, statistics_val, statistics_key, upper_bound
            )

            combined_status.mergeSummary(threshold_status)

        stat.add("mean", str(statistics["mean"]))
        stat.add("std", str(statistics["std"]))
        stat.add("min", str(statistics["min"]))
        stat.add("max", str(statistics["max"]))
        stat.add("last_value", str(statistics["last_value"]))
        stat.summary(
            combined_status
        )  # copy into state instead of overriding, like example, to allow for further composition
        return stat
