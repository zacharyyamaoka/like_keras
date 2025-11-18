# Keep this empty. As these are included at various levels of imports, it gets complicated...

from .diagnostic_status import DiagnosticStatus, DiagnosticStatusWrapper, KeyValue, DiagnosticTask
from .common_tasks import (
    # Threshold dataclasses
    DiagnosticValue,
    BoxChartThresholds,
    ValueThresholds,
    CounterThresholds,
    PercentThresholds,
    # Config wrapper
    UnifiedDiagnosticConfig,
    # Task classes
    BoxChartTask,
    ValueTask,
    CounterTask,
    PercentTask,
    EventPeriodTask,
)
from .diagnostic_task_list import DiagnosticTaskList

# ROS-dependent imports (optional, only if ROS is available)
try:
    from .diagnostic_client import DiagonisticClient
except ImportError:
    DiagonisticClient = None  # ROS not available

def print_diagnostic_status(stat: DiagnosticStatusWrapper):
    """Print diagnostic status in a formatted way."""
    level_names = {
        DiagnosticStatus.OK: 'OK',
        DiagnosticStatus.WARN: 'WARN', 
        DiagnosticStatus.ERROR: 'ERROR',
        DiagnosticStatus.STALE: 'STALE'
    }
    
    level_name = level_names.get(stat.level, f'UNKNOWN({stat.level})')
    
    print(f"{stat.__class__.__name__}:\n"
          f"  name        : {stat.name}\n"
          f"  level       : {level_name}\n"
          f"  message     : {stat.message}\n"
          f"  hardware_id : {stat.hardware_id}\n"
          f"  values      : {len(stat.values)} items")
    
    item: KeyValue
    for item in stat.values:
        # Find the maximum key length to align all keys
        max_key_len = max(len(kv.key) for kv in stat.values) if stat.values else 0
        print(f"    {item.key:<{max_key_len}} : {item.value}")