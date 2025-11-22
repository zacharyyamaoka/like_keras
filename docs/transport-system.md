# Transport System (Message Passing)

## Overview

**Transport** = How messages are passed between components/nodes

like-keras uses **transport compilation** instead of implementing message passing:
- Define system once in Python
- Compile to transport format (Dora, ROS2, etc.)
- Let transport tools handle IPC

## Quick Start

```python
from lk.common.system import System
from lk.common.node import Node
from lk.agent import Agent
from lk.env import Env

# 1. Define system
node = Node(name="main")
agent = Agent.from_node(node)
env = Env.from_node(node)

system = System(
    nodes=[node],
    components=[agent, env],
)

# 2. Choose transport
system.launch()                    # Native (default, same process)
system.launch(transport="dora")    # Dora (multi-process, zero-copy)
system.launch(transport="ros2")    # ROS2 (DDS middleware)

# 3. Or compile without launching
dataflow = system.compile_to_transport("dora", "my_system.yml")
# Then: $ dora start my_system.yml
```

## Available Transports

### Native (Default)
- **IPC**: Direct Python calls
- **Process**: Single
- **Use**: Development, debugging
- **Launch**: `python script.py`

### Dora RS
- **IPC**: Zero-copy shared memory + network
- **Process**: Multi (one per node)
- **Use**: Production, performance
- **Launch**: `dora start dataflow.yml`

### ROS2
- **IPC**: DDS middleware
- **Process**: Multi (one per node)
- **Use**: ROS ecosystem
- **Launch**: `ros2 launch pkg launch.py`

## Transport Compilation

```python
# Compile to Dora
system.compile_to_transport("dora", "output.yml")
# → Generates dataflow.yml for Dora

# Compile to ROS2
system.compile_to_transport("ros2", "output.py")
# → Generates launch.py for ROS2
```

## CLI Pattern

```python
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--transport', default='native', 
                   choices=['native', 'dora', 'ros2'])
args = parser.parse_args()

system = MySystem()
system.launch(transport=args.transport)
```

Usage:
```bash
python my_system.py                    # Native
python my_system.py --transport=dora   # Dora
python my_system.py --transport=ros2   # ROS2
```

## Philosophy

**like-keras**: Define system structure (components, connections)
**Transport**: Handle message passing (serialization, IPC, discovery)

Just like:
- **Keras**: Define model → **TensorFlow/PyTorch**: Execute ops
- **like-keras**: Define system → **Dora/ROS2**: Pass messages

## Files

- `lk/common/transport.py` - Transport abstraction
- `lk/common/transports/dora_transport.py` - Dora compiler
- `lk/common/transports/ros2_transport.py` - ROS2 compiler (future)
- `examples/ex_06_transport_compilation.py` - Examples
- `docs/execution-engine.md` - Full documentation

## Next Steps

1. Install Dora: https://dora-rs.ai/docs/guides/installation
2. Try examples: `python examples/ex_06_transport_compilation.py`
3. Implement node operators for Dora
4. Test multi-process execution

## See Also

- [execution-engine.md](./execution-engine.md) - Detailed design
- [Dora RS docs](https://dora-rs.ai)
- [ROS2 launch docs](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/)

