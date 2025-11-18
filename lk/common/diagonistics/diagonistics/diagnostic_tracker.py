#!/usr/bin/env python3

"""
See bam_utils/docs/diagnostic_tracker.md for more details.
"""
# ROS
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue, DiagnosticArray 
from std_msgs.msg import Header 
# BAM
# from bam_agent.metrics import MetricsTracker  # Commented out due to import error

# PYTHON
import numpy as np
from collections import deque
from typing import Dict, Optional, Tuple, List

# Can all diagonsitics be represented as a threshold?
# Yes, but you may need to do some computation inside of the node, to turn it into a threshold.
# At the end of the day, all diagonistics will come down to an asseration value.
# I can have bool metric in the DiagonsticTracker class...

class DiagonisticValue:
    def __init__(self, warn=None, error=None):
        self.warn = warn
        self.error = error

    def __str__(self):
        return f"warn: {self.warn}, error: {self.error}"


class DiagonisticThresholds:
    """
    Simple container for diagnostic thresholds. It looks like a ROS msg :)
    
    Example:
        thresholds = DiagonisticThresholds()
        thresholds.mean_max = DiagonisticValue(warn=1.5, error=2.0)
        thresholds.std = DiagonisticValue(warn=1.0, error=2.0)
        thresholds.max = DiagonisticValue(error=3.0)
    """
    def __init__(self):
        self.mean_max = DiagonisticValue()
        self.mean_min = DiagonisticValue()
        self.std = DiagonisticValue() 
        self.max = DiagonisticValue()
        self.min = DiagonisticValue()

class DiagnosticMetric:
    def __init__(
        self,
        name: str,
        max_len: int = 100,
        thresholds: Optional[DiagonisticThresholds] = None,
        level: str = "INFO",
        lazy: bool = False,
    ):
        self.name = name
        self.values = deque(maxlen=max_len)
        self.thresholds = thresholds or DiagonisticThresholds()
        self.level = level
        self.lazy = lazy
        self._cached_stats = {}

    def append(self, val: float):
        self.values.append(val)
        if not self.lazy:
            self._cached_stats = self._compute_stats()

    def _compute_stats(self) -> Dict[str, float]:
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

    def _check_threshold(self, threshold: DiagonisticValue, value: float, stat_name: str, upper_bound: bool = True) -> Tuple[str, str]:
        """Generic threshold checking function.
        
        Args:
            threshold: The threshold to check against
            value: The value to check
            stat_name: Name of the statistic (for error messages)
            upper_bound: If True, checks if value exceeds threshold (upper bound). 
                        If False, checks if value is below threshold (lower bound).
        
        Returns:
            Tuple of (level, message)
        """
        if threshold.warn is None and threshold.error is None:
            return "OK", ""
            
        if upper_bound:
            # For upper bound thresholds, check if value exceeds threshold
            if threshold.error is not None and value > threshold.error:
                return "ERROR", f"{stat_name} {value} exceeds (>) error threshold {threshold.error}"
            elif threshold.warn is not None and value > threshold.warn:
                return "WARN", f"{stat_name} {value} exceeds (>) warning threshold {threshold.warn}"
        else:
            # For lower bound thresholds, check if value is below threshold
            if threshold.error is not None and value < threshold.error:
                return "ERROR", f"{stat_name} {value} below (<) error threshold {threshold.error}"
            elif threshold.warn is not None and value < threshold.warn:
                return "WARN", f"{stat_name} {value} below (<) warning threshold {threshold.warn}"
        
        return "OK", ""

    def _update_worst_level(self, current: str, new: str) -> str:
        """Update the worst level (ERROR > WARN > OK)"""
        levels = {"OK": 0, "WARN": 1, "ERROR": 2}
        return new if levels[new] > levels[current] else current

    def evaluate(self) -> Tuple[str, str]:
        """Evaluate all thresholds against the current statistics.
        
        Returns the worst status (ERROR > WARN > OK) and a combined message.
        """
        if self.lazy or not self._cached_stats:
            self._cached_stats = self._compute_stats()
        
        stats = self._cached_stats
        
        # Return OK if no data available
        if not stats:
            return "OK", "No data available for evaluation"
        
        worst_level = "OK"
        messages = []
        
        # Define threshold checks: (threshold_attr, upper_bound)
        # All use the same name for threshold_attr, stat_key, and stat_name
        threshold_checks = [
            ("mean_max", True),   # mean_max threshold, upper bound
            ("mean_min", False),  # mean_min threshold, lower bound
            ("std", True),        # std threshold, upper bound
            ("max", True),        # max threshold, upper bound
            ("min", False),       # min threshold, lower bound
        ]
        
        # Check each threshold
        for threshold_attr, upper_bound in threshold_checks:
            threshold = getattr(self.thresholds, threshold_attr)
            stat_key = threshold_attr.replace("_max", "").replace("_min", "")
            
            # Skip if stat is not available
            if stat_key not in stats:
                continue
                
            stat_val = stats[stat_key]
            stat_name = stat_key.capitalize()
            level, msg = self._check_threshold(threshold, stat_val, stat_name, upper_bound)
            if level != "OK":
                messages.append(msg)
            worst_level = self._update_worst_level(worst_level, level)
        
        # Combine messages
        if messages:
            combined_message = "; ".join(messages)
        else:
            combined_message = "All metrics within acceptable ranges"
            
        return worst_level, combined_message

    def get_stats(self) -> Dict[str, float]:
        if self.lazy or not self._cached_stats:
            self._cached_stats = self._compute_stats()
        return self._cached_stats

    def get_last_value(self, default: float = 0.0) -> float:
        """Get the last value from the metric's values list.
        
        Args:
            default: Value to return if the list is empty
            
        Returns:
            The last value if the list is not empty, otherwise the default value
        """
        if len(self.values) == 0:
            return default
        return float(self.values[-1])

class DiagnosticTracker:
    def __init__(self, config: Optional[Dict] = None):
        self.metrics: Dict[str, DiagnosticMetric] = {}
        self.config = config or {}
        
        # Validate config if provided
        if self.config and 'metrics' not in self.config:
            raise ValueError("Config dictionary must contain 'metrics' section")

    def _create_thresholds_from_config(self, metric_name: str) -> DiagonisticThresholds:
        """Create thresholds from config for a given metric name."""
        # Strip leading slash for proper config matching in multi-environment cases
        clean_metric_name = metric_name.lstrip('/')
        
        if clean_metric_name not in self.config['metrics']:
            raise ValueError(f"Metric '{clean_metric_name}' not found in config")
        
        metric_config = self.config['metrics'][clean_metric_name]
        thresholds = DiagonisticThresholds()
        
        # Load each threshold type from config
        for threshold_type in ['mean_max', 'mean_min', 'std', 'max', 'min']:
            if threshold_type in metric_config:
                threshold_config = metric_config[threshold_type]
                warn = threshold_config.get('warn')
                error = threshold_config.get('error')
                
                # some threshold is better than no threshold!
                if warn is None and error is None:
                    raise ValueError(f"Threshold '{threshold_type}' for metric '{clean_metric_name}' has no warn or error values")
                
                # Set the threshold using direct assignment
                setattr(thresholds, threshold_type, DiagonisticValue(warn=warn, error=error))
        
        return thresholds

    def register(
        self,
        name: str,
        thresholds: Optional[DiagonisticThresholds] = None,
        level: str = "INFO",
        max_len: int = 100,
        lazy: bool = False,
    ):
        """Register a new metric.
        
        Args:
            name: Metric name
            thresholds: Optional thresholds to override config. If None, will load from config.
            level: Metric level (INFO, DEBUG, etc.)
            max_len: Maximum number of values to store
            lazy: Whether to compute stats lazily
        """
        # If no thresholds provided, try to load from config
        if thresholds is None:
            if not self.config:
                # Create metric without thresholds if no config available
                self.metrics[name] = DiagnosticMetric(
                    name=name,
                    max_len=max_len,
                    thresholds=None,
                    level=level,
                    lazy=lazy,
                )
                return
            thresholds = self._create_thresholds_from_config(name)
        
        self.metrics[name] = DiagnosticMetric(
            name=name,
            max_len=max_len,
            thresholds=thresholds,
            level=level,
            lazy=lazy,
        )

    def append(self, name: str, val: float):
        self.metrics[name].append(val)

    def evaluate(self, name: str) -> Tuple[str, str]:
        return self.metrics[name].evaluate()

    def get_last_value(self, name: str, default: float = 0.0) -> float:
        return self.metrics[name].get_last_value(default)

    def to_diagnostics(self) -> List[DiagnosticStatus]:
        """https://docs.ros2.org/foxy/api/diagnostic_msgs/index-msg.html"""
        msgs = []
        for metric in self.metrics.values():
            level_str, message = metric.evaluate()
            level = {
                "OK": DiagnosticStatus.OK,
                "WARN": DiagnosticStatus.WARN,
                "ERROR": DiagnosticStatus.ERROR,
            }.get(level_str, DiagnosticStatus.STALE)
            stat_msg = DiagnosticStatus()
            stat_msg.name = f"metric: {metric.name}"
            stat_msg.level = level
            stat_msg.message = message
            stat_msg.values = []
            # Create KeyValue objects and add them to the values list
            for key, val in metric.get_stats().items():
                kv = KeyValue()
                kv.key = key
                kv.value = str(val)
                # Use list assignment instead of append
                stat_msg.values.append(kv)
            msgs.append(stat_msg)
        return msgs

    def to_diagnostics_array(self, header: Optional[Header] = None) -> DiagnosticArray:

        msg = DiagnosticArray()
        if header is not None:
            msg.header = header
        msg.status = self.to_diagnostics()
        return msg

