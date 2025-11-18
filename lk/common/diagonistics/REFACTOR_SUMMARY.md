# Diagnostic Task Refactor Summary

## Overview
Successfully refactored the diagnostic system to use dataclasses with type-safe configuration, automatic serialization via dacite, and a reusable `DiagnosticTaskList` base class. **All ROS dependencies have been removed** from the core diagnostic system.

## Key Changes

### 1. Removed ROS Dependencies
- Created standalone `diagnostic_status.py` with lightweight replacements for ROS classes:
  - `DiagnosticStatus` (constants: OK, WARN, ERROR, STALE)
  - `DiagnosticStatusWrapper` (status container with merge/summary methods)
  - `DiagnosticTask` (base class for tasks)
  - `KeyValue` (simple key-value pair)
- Updated all imports to use local diagnostic classes
- Made `DiagnosticClient` optional (only imports if ROS is available)

### 2. Threshold Classes → Dataclasses
**Before:**
```python
class BoxChartThresholds(ParamLoader):
    def __init__(self):
        super().__init__()
        self.min = DiagonisticValue()
        self.max = DiagonisticValue()
        # ...
```

**After:**
```python
@dataclass
class DiagnosticValue:
    warn: float = 0.0
    error: float = 0.0

@dataclass
class BoxChartThresholds:
    min: DiagnosticValue = field(default_factory=DiagnosticValue)
    max: DiagnosticValue = field(default_factory=DiagnosticValue)
    mean_min: DiagnosticValue = field(default_factory=DiagnosticValue)
    mean_max: DiagnosticValue = field(default_factory=DiagnosticValue)
    std: DiagnosticValue = field(default_factory=DiagnosticValue)
```

**Benefits:**
- ✅ Full IDE autocomplete: `thresholds.mean_max.warn = 50.0`
- ✅ Type checking catches errors at development time
- ✅ Easy serialization with `asdict()` and `from_dict()`

### 3. UnifiedDiagnosticConfig
Created a wrapper dataclass to combine all configuration:

```python
@dataclass
class UnifiedDiagnosticConfig(Generic[ThresholdsType]):
    thresholds: ThresholdsType
    window_len: int = 10
    queue_type: str = "timed"
```

### 4. Updated Task Classes
**Before:**
```python
def __init__(self, name='BoxChartTask', thresholds=BoxChartThresholds(), window_len=10, queue_type="timed"):
    super().__init__(name, thresholds, window_len, queue_type)
```

**After:**
```python
def __init__(self, name: str, config: UnifiedDiagnosticConfig[BoxChartThresholds]):
    super().__init__(name, config)
```

**Added Methods:**
- `to_config()`: Serialize to dict using `asdict()`
- `from_config()`: Deserialize using dacite (handles nested dataclasses automatically)

### 5. DiagnosticTaskList Base Class
Created reusable base class for component diagnostic containers:

```python
class DiagnosticTaskList:
    def to_config(self) -> dict:
        """Convert all tasks to config dicts"""
        
    @classmethod
    def from_config(cls, config: dict):
        """Reconstruct all tasks from config using type introspection"""
        
    def prefix_task_names(self, component_id: str):
        """Add component_id prefix to all task names"""
```

### 6. Component Base Class Updates
- Updated `_create_diagnostics()` to automatically handle `DiagnosticTaskList`
- Added `dx` property as shorthand for `self.diagnostics`
- Automatically prefixes task names with `component_id`

### 7. EnvDiagnostics Example
Created type-safe diagnostic configuration for Env component:

```python
@dataclass
class EnvDiagnostics(DiagnosticTaskList):
    step_latency: BoxChartTask = field(default_factory=lambda: BoxChartTask(
        name="step_latency",
        config=UnifiedDiagnosticConfig(
            thresholds=BoxChartThresholds(
                mean_max=DiagnosticValue(warn=50.0, error=100.0),
                max=DiagnosticValue(warn=150.0, error=300.0)
            ),
            window_len=100
        )
    ))
    # ... more tasks ...
```

### 8. Component Usage
Components can now use diagnostics with full type safety:

```python
class RealEnv(Env):
    def step(self, action):
        start = time.time()
        # ... step logic ...
        self.dx.step_latency.append((time.time() - start) * 1000.0)
        
    def reset(self):
        start = time.time()
        # ... reset logic ...
        self.dx.reset_latency.append((time.time() - start) * 1000.0)
```

## Files Modified

### Core Diagnostic System
- `bam/common/diagonistics/diagonistics/diagnostic_status.py` (NEW) - Standalone diagnostic classes
- `bam/common/diagonistics/diagonistics/common_tasks.py` - Refactored to use dataclasses
- `bam/common/diagonistics/diagonistics/diagonstic_helper.py` - Updated imports
- `bam/common/diagonistics/diagonistics/diagnostic_task_list.py` (NEW) - Base class
- `bam/common/diagonistics/diagonistics/__init__.py` - Updated exports

### Component System
- `bam/common/component.py` - Added `dx` property and auto-prefixing

### Environment System
- `bam/env/env_config.py` - Added `EnvDiagnostics`
- `bam/env/real_env.py` - Added diagnostic tracking

### Examples
- `examples/ex_diagnostics_refactor.py` (NEW) - Comprehensive example

## Benefits

1. **No ROS Dependencies**: Core diagnostic system works standalone
2. **Type Safety**: Full IDE support and compile-time type checking
3. **Automatic Serialization**: `to_config()` / `from_config()` via dacite
4. **DRY Principle**: `DiagnosticTaskList` eliminates boilerplate
5. **Consistent API**: All components use same pattern with `self.dx`
6. **Easy to Extend**: Add new component diagnostics by creating a dataclass

## Testing

Run the example:
```bash
python examples/ex_diagnostics_refactor.py
```

Expected output:
- ✓ Task name prefixing works
- ✓ Data collection works
- ✓ Serialization works
- ✓ Deserialization works
- ✓ All todos completed

## Future Work

To add diagnostics to a new component:

1. Create diagnostic task list in component's config file:
```python
@dataclass
class ActorDiagnostics(DiagnosticTaskList):
    inference_latency: BoxChartTask = field(...)
    action_rate: EventPeriodTask = field(...)
```

2. Add to component config:
```python
@dataclass
class ActorConfig:
    # ... other config ...
    diagnostics: ActorDiagnostics = field(default_factory=ActorDiagnostics)
```

3. Use in component:
```python
def infer(self, obs):
    start = time.time()
    # ... inference ...
    self.dx.inference_latency.append((time.time() - start) * 1000.0)
```

No other code needed - everything is automatic!

