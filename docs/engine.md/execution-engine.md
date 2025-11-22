# Execution Engine / Launch System

## Philosophy: Transport, Not Implementation

like-keras follows the **Keras philosophy** for execution:

> **Keras** doesn't implement tensor operations → compiles to TensorFlow, PyTorch, JAX
> 
> **like-keras** doesn't implement message passing → compiles to Dora RS, ROS2, etc.

**Key Terminology**: We call it "**transport**" because we're specifically talking about **how messages are passed** between components - not the entire execution stack.

This approach has several advantages:
- **Leverage existing tools**: Dora and ROS2 are battle-tested for robotics IPC
- **Maintainability**: Don't reinvent the wheel for message transport
- **Flexibility**: Support multiple transports with one codebase
- **Performance**: Use optimized implementations (Dora's zero-copy, ROS2's DDS)

---

## Key Abstraction

You define your system **once** in like-keras, then compile to different transports:

```python
# Define system in Python (high-level like-keras API)
system = MyRobotSystem()

# Option 1: Native transport (same process) for development
system.launch()

# Option 2: Dora RS transport (multi-process) for deployment
system.launch(transport="dora")

# Option 3: ROS2 transport (DDS middleware)
system.launch(transport="ros2")
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│  Python System Definition (like-keras)                  │
│  - Components: Agent, Env, Sensor, etc.                 │
│  - Nodes: Execution contexts                            │
│  - Connections: Data flow graph                         │
└───────────────────┬─────────────────────────────────────┘
                    │
                    │ Compile
                    ▼
    ┌───────────────────────────────────────┐
    │        Transport Compiler             │
    │  Translates graph → transport format  │
    └───────┬───────────────────────────────┘
            │
            ├─────────────┬─────────────┬────────────────┐
            ▼             ▼             ▼                ▼
     ┌──────────┐  ┌──────────┐  ┌──────────┐   ┌──────────┐
     │  Native  │  │  Dora RS │  │   ROS2   │   │  Custom  │
     │  Direct  │  │  Shared  │  │   DDS    │   │  ...     │
     │  Python  │  │  Memory  │  │  Network │   │          │
     └──────────┘  └──────────┘  └──────────┘   └──────────┘
```

---

## Transports (Message Passing Layers)

### 1. Native Transport (Default)

**What**: Direct Python function calls, same process
**Good for**: Development, debugging, fast iteration
**Launch**: `system.launch()`

```python
system = MyRobotSystem()
system.launch(iterations=100)  # Native transport
```

**Characteristics**:
- No serialization overhead
- Full Python debugging
- Single process
- Not suitable for multi-language or distributed systems

---

### 2. Dora RS Transport (Primary)

**What**: Zero-copy shared memory (local) + network (remote)
**Good for**: Production, performance, multi-process
**Launch**: `dora start dataflow.yml`

```python
# Compile to Dora transport
dataflow_path = system.compile_to_transport("dora")

# Or compile + launch
system.launch(transport="dora")
```

**Generated dataflow.yml**:
```yaml
nodes:
  - id: agent_node
    operator:
      python: nodes/agent.py
    inputs:
      obs:
        source: env_node/obs
    outputs:
      - action

  - id: env_node
    operator:
      python: nodes/env.py
    inputs:
      action_in:
        source: agent_node/action
    outputs:
      - obs
      - reward
      - done
```

**Dora handles**:
- Process spawning for each node
- Shared memory for zero-copy local IPC
- Network sockets for remote nodes
- Message serialization
- Event-driven execution

**References**:
- https://dora-rs.ai/dora/overview.html
- https://github.com/dora-rs/dora

---

### 3. ROS2 Transport (Secondary)

**What**: DDS middleware for message passing
**Good for**: ROS ecosystem integration
**Launch**: `ros2 launch pkg launch.py`

```python
system.compile_to_transport("ros2", "launch.py")
```

**Generated launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='agent_node',
            remappings=[('obs', '/env/obs')]
        ),
        Node(
            package='my_robot',
            executable='env_node',
        ),
    ])
```

**ROS2 handles**:
- DDS discovery across network
- Topic-based pub/sub
- QoS policies for reliability
- Multi-language support (Python, C++)

**References**:
- https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/
- https://github.com/ros2/launch

---

## Component Execution Contexts

### Same Process (Native Transport)
- **Language**: Python only
- **Process**: Single process
- **Machine**: Local
- **IPC**: Direct Python references (no serialization)
- **Use case**: Development, debugging

### Different Process (Dora/ROS2 Transport)
- **Language**: Python, C++, Rust, JavaScript
- **Process**: Multiple processes (one per node)
- **Machine**: Same or different
- **IPC**: Shared memory (local) or network (remote)
- **Use case**: Production, performance, distributed systems

---

## Handling Different Scenarios

### Same Language, Same Process, Same Machine
✓ **Use native transport**

```python
system.launch()  # Everything in one process
```

### Same Language, Different Process, Same Machine
✓ **Use Dora transport** - Zero-copy shared memory

```python
system.launch(transport="dora")
# → Compiles to dataflow.yml
# → Dora handles shared memory IPC
```

### Different Language, Different Process, Same Machine
✓ **Use Dora transport** - Multi-language support

```python
# Define nodes with different languages
node_python = Node(name="control")
node_cpp = Node(name="vision")
node_cpp.dora_operator = {"rust": "target/release/vision"}

system = System(nodes=[node_python, node_cpp])
system.launch(transport="dora")
```

### Different Machine (Distributed)
△ **Partial support** - Use Dora network transport

**Option 1**: Separate launch on each machine
```bash
# Machine A
$ dora start control_dataflow.yml

# Machine B  
$ dora start sensors_dataflow.yml
```

**Option 2**: Dora network transport (future)
- Dora can route messages over network
- Automatic discovery between machines
- Transparent local/remote communication

---

## Development to Deployment Workflow

### Phase 1: Development (Native Transport)
```python
# Fast iteration, full debugging
system.launch(iterations=100)
# → Direct Python calls
# → No serialization
# → Full debugger support
```

### Phase 2: Testing (Compile to Dora)
```python
# Generate dataflow for inspection
dataflow = system.compile_to_transport("dora", "test.yml")
# → Review YAML structure
# → Verify node connections
# → Test message passing
```

### Phase 3: Deployment (Dora Transport)
```python
# Launch with Dora transport
system.launch(transport="dora")

# Or deploy to robot
$ scp dataflow.yml robot@192.168.1.100:/opt/robot/
$ ssh robot@192.168.1.100 'cd /opt/robot && dora start dataflow.yml'
```

---

## System Configuration as Code

The system config file (Python) is like a Dockerfile or dataflow.yml:
- Contains all information to reproduce a system
- Version controlled
- Generates transport-specific files
- Can be parameterized and composed

**Example**:
```python
class ProductionSystem(System):
    def __init__(self):
        # Nodes (execution contexts)
        self.control_node = Node("control")
        self.sensors_node = Node("sensors")
        
        # Components (computation)
        self.agent = Agent.from_node(self.control_node)
        self.camera = Camera.from_node(self.sensors_node)
        
        super().__init__(config=System.Config(name="production"))

# Deploy
system = ProductionSystem()
system.compile_to_transport("dora", "production_v1.0.yml")
```

---

## Comparison: Transports for Message Passing

| Feature | Native | Dora RS | ROS2 |
|---------|--------|---------|------|
| Format | Python code | YAML | Python |
| IPC | Direct calls | Shared mem + network | DDS |
| Languages | Python | Python, Rust, C++ | Python, C++ |
| Zero-copy | ✓ (same process) | ✓ (shared mem) | △ (depends on DDS) |
| Multi-machine | ✗ | ✓ | ✓ |
| Launch | Direct | `dora start` | `ros2 launch` |
| Discovery | N/A | Coordinator | DDS discovery |
| Overhead | None | Minimal | Moderate (DDS) |

---

## References

### Dora RS
- Docs: https://dora-rs.ai
- GitHub: https://github.com/dora-rs/dora
- Philosophy: Event-driven dataflow, zero-copy IPC

### ROS2 Launch
- Docs: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/
- GitHub: https://github.com/ros2/launch
- Philosophy: DDS middleware, topic-based messaging

### Keras Multi-Backend
- Docs: https://keras.io/getting_started/
- How Keras compiles to multiple backends (TF, PyTorch, JAX)
- Inspiration for our approach

---

## Summary

like-keras is a **high-level description language** for robot systems that:

1. ✓ Provides intuitive Python API for system design
2. ✓ Compiles to transport-specific formats (dataflow.yml, launch.py)
3. ✓ Leverages specialized tools (Dora, ROS2) for message passing
4. ✓ Supports multi-process, multi-language, multi-machine
5. ✓ Enables seamless dev-to-deploy workflow

**Key insight**: Don't implement message transport ourselves - compile to formats that existing tools can execute.

**Terminology**: We call it "transport" (not "backend") because we're specifically referring to **how messages are passed** between components.

See also:
- [launch-system.md](./launch-system.md) - Detailed transport documentation
- [examples/ex_06_transport_compilation.py](../examples/ex_06_transport_compilation.py) - Working examples
