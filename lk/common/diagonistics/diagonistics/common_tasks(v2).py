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
    def __init__(self, warn: float = None, error: float = None): # type: ignore
        self.warn = warn
        self.error = error

    def __str__(self):
        return f"warn: {self.warn}, error: {self.error}"

# use there combination task or.... seperate ones... 
# beacuse its the same list of values. I don't want to have to append to all the multiple lists I do feel its simple to just do it in here...
# I think I can use there logic for combining though...

def check_thresholds(threshold: DiagonisticValue, value: float, stat_name: str, upper_bound: bool) -> DiagnosticStatusWrapper:
    """
    Check if the value exceeds the threshold and return a DiagnosticStatusWrapper.
    """
    stat = DiagnosticStatusWrapper()
    if upper_bound:
        if value > threshold.error:
            stat.summary(DiagnosticStatus.ERROR, f"{stat_name} exceeds error threshold: {value} > {threshold.error}")
        elif value > threshold.warn:
            stat.summary(DiagnosticStatus.WARN, f"{stat_name} exceeds warning threshold: {value} > {threshold.warn}")
    else:
        if value < threshold.error:
            stat.summary(DiagnosticStatus.ERROR, f"{stat_name} below error threshold: {value} < {threshold.error}")
        elif value < threshold.warn:
            stat.summary(DiagnosticStatus.WARN, f"{stat_name} below warning threshold: {value} < {threshold.warn}")
    return stat


# def load_from_node(node: Node, path: str) -> DiagonisticValue:






# Helpers to generate standard nested schemas
def two_level_schema(level_1: list[str], level_2: list[str]) -> dict:
    """
    level_1 = ['mean_min', 'mean_max', 'std', 'min', 'max']
    level_2 = ['warn', 'error']
    """
    return {attr_1: {attr_2: {} for attr_2 in level_2} for attr_1 in level_1}

def box_chart_task_schema() -> dict:
    schema = {
        'mean_max': {
            'warn': {'default': 1e6, 'type': 'double_value'},
            'error': {'default': 1e6, 'type': 'double_value'}
        },
        'mean_min': {
            'warn': {'default': 1e6, 'type': 'double_value'},
            'error': {'default': 1e6, 'type': 'double_value'}
        },
        'std': {
            'warn': {'default': 1e6, 'type': 'double_value'},
            'error': {'default': 1e6, 'type': 'double_value'}
        },
        'min': {
            'warn': {'default': 1e6, 'type': 'double_value'},
            'error': {'default': 1e6, 'type': 'double_value'}
        },
        'max': {
            'warn': {'default': 1e6, 'type': 'double_value'},
            'error': {'default': 1e6, 'type': 'double_value'}
        }
    }
    return schema

class ParamLoader:
    verbose = False
    
    @staticmethod
    def _attr_schema() -> dict:
        # Should be overridden in child
        return {}

    @classmethod 
    def declare_node_params(cls,node: Node, prefix: str):
        """
        Declare parameters for the BoxChartThresholds using the class schema.
        """
        attr_schema = cls._attr_schema()
        cls._declare_node_params(node, attr_schema, prefix)

    @classmethod
    def _declare_node_params(cls, node: Node, schema: dict, prefix: str, level_index=0):
        """
        Recursively declare parameters on the node using the provided schema.
        The final dict value should be a dict with at least a 'default' key.
        """

        if prefix.endswith('.'):
            prefix = prefix[:-1]

        if cls.verbose:    
            print(f"[{level_index}] {prefix}")

        for key, value in schema.items():
            if isinstance(value, dict) and 'type' not in value:
                ParamLoader._declare_node_params(node, value, f"{prefix}.{key}", level_index + 1)
            elif isinstance(value, dict) and 'default' in value:
                param_name = f"{prefix}.{key}"
                node.declare_parameter(param_name, value['default'])
                if cls.verbose:
                    print(f"{param_name} = {value['default']}")
            else:
                raise ValueError(f"Unsupported schema leaf type: {type(value)} for key {key}")

    @classmethod 
    def load_from_node(cls, node: Node, prefix: str):
        # Get the schema from the child class
        # cls refers to child class of ParamLoader
        attr_schema = cls._attr_schema()
        instance = cls()
        instance._load_from_node(node, attr_schema, prefix)
        return instance


    def _load_from_node(self, node: Node, attr_schema: dict, prefix: str, level_instance=None, level_index=0):
        """
        Set values from node parameters recursively based on a nested attribute schema.

        Args:
            node: The ROS2 node to get parameters from.
            attr_schema: Nested dictionary schema describing parameter structure.
            prefix: The parameter namespace prefix (e.g., 'diagnostics.reward').
            level_instance: The current object whose attributes are being set (defaults to class).
            level_index: Current recursion depth (for internal use).

        Example:
            Given:
                prefix = 'diagnostics.reward'
                attr_schema = {
                    'mean_max': {
                        'warn': {'type': 'double_value', 'default': 1e6},
                        'error': {'type': 'double_value', 'default': 1e6}
                    }
                }
            This will set:
                self.mean_max.warn = node.get_parameter('diagnostics.reward.mean_max.warn').get_parameter_value().double_value
                self.mean_max.error = node.get_parameter('diagnostics.reward.mean_max.error').get_parameter_value().double_value

        The function traverses the schema, appending keys to the prefix to form the full parameter name,
        and sets the corresponding attribute on the object using the value from the node parameter.
        """

        if level_instance is None:
            level_instance = self

        # Convience checker so you don't need to add it yourself
        if prefix.endswith('.'):
            prefix = prefix[:-1]

        if self.verbose:    
            print(f"[{level_index}] {prefix} | {type(level_instance)}")

        for key, value in attr_schema.items():
            if isinstance(value, dict) and 'type' not in value:
                # Recurse into the next level, passing the corresponding attribute level_selfect
                self._load_from_node(node, value, f"{prefix}.{key}", getattr(level_instance, key), level_index + 1)
            elif isinstance(value, dict) and 'type' in value:
                # Leaf node: set the value using the specified value type
                param_name = f"{prefix}.{key}"
                param_value = getattr(node.get_parameter(param_name).get_parameter_value(), value['type'])
                setattr(level_instance, key, param_value)
                if self.verbose:
                    print(f"{param_name} = {param_value}")
            else:
                raise ValueError(f"Unsupported schema leaf type: {type(value)} for key {key}")

    def __str__(self):
        class_name = self.__class__.__name__
        attrs = [
            f"  {k}: {v}"
            for k, v in self.__dict__.items()
        ]
        return f"{class_name}(\n" + "\n".join(attrs) + "\n)"






class BoxChartThresholds(ParamLoader):

    def __init__(self):
        super().__init__()
        self.mean_min = DiagonisticValue()
        self.mean_max = DiagonisticValue()
        self.std = DiagonisticValue()
        self.min = DiagonisticValue()           
        self.max = DiagonisticValue()

    @staticmethod
    def _attr_schema() -> dict:
        return box_chart_task_schema()


                
class BoxChartTask(DiagnosticTask):

    def __init__(self, name = 'BoxChartTask', params = BoxChartThresholds(), window_len = 100 ):
        super().__init__(name)
        
        self.window_len = window_len

        self.values = deque(maxlen=window_len) # Could be done by time, but this is reasonable as well...

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
            ("mean_max", True),   # mean_max threshold, upper bound
            ("mean_min", False),  # mean_min threshold, lower bound
            ("std", True),        # std threshold, upper bound
            ("max", True),        # max threshold, upper bound
            ("min", False),       # min threshold, lower bound
        ]

        # This section is based on: 
        # https://github.com/ros/diagnostics/blob/ros2/diagnostic_updater/diagnostic_updater/_diagnostic_updater.py#L131

        combined_status = DiagnosticStatusWrapper()

        for param_attr, upper_bound in threshold_checks:
                thresholds = getattr(self.params, param_attr)
                statistics_key = param_attr.replace("_max", "").replace("_min", "") # filter beacuse we use same mean for mean_max and mean_min
                statistics_val = statistics[statistics_key]

                threshold_status = check_thresholds(thresholds, statistics_val, statistics_key, upper_bound)

                combined_status.mergeSummary(threshold_status) 


        stat.add("mean", str(statistics["mean"]))
        stat.add("std", str(statistics["std"]))
        stat.add("min", str(statistics["min"]))
        stat.add("max", str(statistics["max"]))
        stat.add("last_value", str(statistics["last_value"]))
        stat.summary(combined_status) # copy into state instead of overriding, like example, to allow for further composition
        return stat


        # INSERT_YOUR_CODE
if __name__ == "__main__":
    class DummyNode:
        def __init__(self, params=None):
            if params is None:
                params = {}
            self._params = params
            self._declared = {}

        def get_parameter(self, name):
            class Param:
                def __init__(self, value):
                    self._value = value
                def get_parameter_value(self):
                    class Value:
                        def __init__(self, v):
                            self.double_value = v
                    return Value(self._value)
            return Param(self._params.get(name, None))

        def declare_parameter(self, name, value):
            self._declared[name] = value
            # For test purposes, also set it in _params so get_parameter can find it
            self._params[name] = value
            return value

    # Simulate parameters for BoxChartThresholds
    params = {
        "test_path.mean_min.warn": 1.0,
        "test_path.mean_min.error": 0.5,
        "test_path.mean_max.warn": 10.0,
        "test_path.mean_max.error": 12.0,
        "test_path.std.warn": 2.0,
        "test_path.std.error": 3.0,
        "test_path.min.warn": 0.0,
        "test_path.min.error": -1.0,
        "test_path.max.warn": 15.0,
        "test_path.max.error": 20.0,
    }

    node = DummyNode(params)  # type: ignore

    # Test loading using the new generic loader
    test_params: BoxChartThresholds = BoxChartThresholds.load_from_node(node, "test_path")  # type: ignore

    print(test_params)
    print("mean_min:", test_params.mean_min)
    print("mean_max:", test_params.mean_max)
    print("std:", test_params.std)
    print("min:", test_params.min)
    print("max:", test_params.max)

    # Check values
    assert test_params.mean_min.warn == 1.0
    assert test_params.mean_min.error == 0.5
    assert test_params.mean_max.warn == 10.0
    assert test_params.mean_max.error == 12.0
    assert test_params.std.warn == 2.0
    assert test_params.std.error == 3.0
    assert test_params.min.warn == 0.0
    assert test_params.min.error == -1.0
    assert test_params.max.warn == 15.0
    assert test_params.max.error == 20.0
    print("All parameter loading tests passed.")

    # Test declaring parameters using the schema
    node2 = DummyNode()  # Start with empty params
    BoxChartThresholds.declare_node_params(node2, "test_path")  # type: ignore
    print("Declared parameters:")
    for k, v in sorted(node2._declared.items()):
        print(f"  {k}: {v}")
    # Optionally, check that all expected keys are present
    expected_keys = [
        "test_path.mean_min.warn", "test_path.mean_min.error",
        "test_path.mean_max.warn", "test_path.mean_max.error",
        "test_path.std.warn", "test_path.std.error",
        "test_path.min.warn", "test_path.min.error",
        "test_path.max.warn", "test_path.max.error",
    ]
    for k in expected_keys:
        assert k in node2._declared
    print("All parameter declaration tests passed.")