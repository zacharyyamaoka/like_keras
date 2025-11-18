# Config Utils - Dict ↔ Dataclass Conversion

Utilities for automatic reconstruction of dataclass configs from dictionaries using `dacite`.

## Key Features

✅ **Nested dict support** - Load configs from YAML/JSON files  
✅ **Flat dict support** - ROS parameter integration with dot-separated keys  
✅ **Type-safe** - Uses dacite for robust dataclass reconstruction  
✅ **Node-level config** - Reconstruct multiple component configs at once  
✅ **ConfigMixin** - Easy serialization/deserialization for dataclasses  

## Quick Start

### Basic Usage

```python
from bam.utils.config_utils import ConfigMixin
from dataclasses import dataclass

@dataclass
class MyConfig(ConfigMixin):
    value: int = 10
    name: str = "test"

# Serialize
cfg = MyConfig(value=20, name="hello")
d = cfg.to_dict()  # {'value': 20, 'name': 'hello'}

# Deserialize
cfg2 = MyConfig.from_dict(d)
```

### Flat (ROS-style) Params

```python
# Flat dict with dot-separated keys
flat_d = {
    'robot.sensor.rate': 60.0,
    'robot.sensor.topic': '/camera',
    'robot.controller.kp': 2.0
}

# Reconstruct from flat
cfg = MyConfig.from_dict(flat_d, is_flat=True)
```

### Node Config with Multiple Components

```python
from bam.common.node import Node
from bam.actor import ActorConfig
from bam.world_model import WorldModelConfig
from bam.env import EnvConfig

# Option 1: Manual (old way)
node_config = {
    'actor_node_actor': ActorConfig(policy_type="default"),
    'actor_node_worldmodel': WorldModelConfig(mode="local"),
}
node = Node(name="actor_node", config=node_config)

# Option 2: From dict (new way - good for YAML/JSON)
config_dict = {
    'actor_node_actor': {'policy_type': 'default'},
    'actor_node_worldmodel': {'mode': 'local'},
}
config_types = {
    'actor_node_actor': ActorConfig,
    'actor_node_worldmodel': WorldModelConfig,
}
node = Node(
    name="actor_node",
    config=config_dict,
    config_types=config_types
)

# Option 3: From flat params (ROS-style)
flat_params = {
    'actor_node_actor.policy_type': 'default',
    'actor_node_worldmodel.mode': 'local',
}
node = Node(
    name="actor_node",
    config=flat_params,
    config_types=config_types,
    is_flat=True
)
```

## Integration with DiagnosticTaskList

Similar pattern to `DiagnosticTaskList.from_config()`:

```python
from bam.common.diagonistics.diagonistics.diagnostic_task_list import DiagnosticTaskList
from bam.utils.config_utils import ConfigMixin

@dataclass
class MyConfig(ConfigMixin):
    """Same API as DiagnosticTaskList"""
    value: int = 10
    
    def to_dict(self):
        """Serialize to dict"""
        return super().to_dict()
    
    @classmethod
    def from_dict(cls, config: dict):
        """Reconstruct from dict"""
        return super().from_dict(config)
```

## YAML Integration

```python
import yaml
from bam.utils.config_utils import reconstruct_node_config

# Load YAML
with open('config.yaml', 'r') as f:
    config_dict = yaml.safe_load(f)

# Reconstruct all component configs
config_types = {
    'robot_node_sensor': SensorConfig,
    'robot_node_controller': ControllerConfig,
}
node_config = reconstruct_node_config(config_dict, config_types)
```

Example YAML:
```yaml
robot_node_sensor:
  rate_hz: 60.0
  topic: /camera/rgb
  
robot_node_controller:
  kp: 2.0
  ki: 0.2
```

## API Reference

### `to_config_dict(obj, flat=False, sep=".")`
Convert dataclass to dictionary.

### `from_config_dict(config_class, config_dict, is_flat=False, sep=".")`
Reconstruct dataclass from dictionary using dacite.

### `reconstruct_node_config(config_dict, config_types, is_flat=False, sep=".")`
Reconstruct all component configs in a node config dict.

### `ConfigMixin`
Mixin class for dataclasses providing `to_dict()` and `from_dict()` methods.

## Examples

- `examples/ex_config_utils_demo.py` - Comprehensive demonstration
- `examples/ex_config_reconstruction.py` - Node-level config examples
- `examples/ex_actor_node_with_config_reconstruction.py` - Real-world usage

## Under the Hood

Uses:
- **dacite** - Robust dict-to-dataclass conversion with type checking
- **dot_dict_utils** - Flatten/unflatten nested dicts with dot notation
- **dataclasses.asdict** - Dataclass-to-dict conversion

## Benefits

1. **Flexibility** - Load configs from any source (YAML, JSON, ROS params, etc.)
2. **Type Safety** - Dacite ensures proper type conversion
3. **DRY** - No need to write manual serialization code
4. **ROS Integration** - Native support for flat parameter format
5. **Backwards Compatible** - Still works with manual config construction

