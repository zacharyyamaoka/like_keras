# Launch System & Backend Compilation

## Overview

**like-keras** follows the Keras philosophy: it's a high-level abstraction that **compiles** to specialized backends, rather than implementing execution itself.

```
┌─────────────────────────────────────┐
│  like-keras System (Python)         │
│  - Components, Nodes, Connections   │
│  - High-level, intuitive API        │
└──────────────┬──────────────────────┘
               │ Compile
               ▼
    ┌──────────────────────┐
    │   Backend Format     │
    ├──────────────────────┤
    │ • Dora: dataflow.yml │
    │ • ROS2: launch.py    │
    │ • Native: Python     │
    └──────────┬───────────┘
               │ Launch
               ▼
    ┌──────────────────────┐
    │  Execution Engine    │
    │  (Dora, ROS2, etc.)  │
    └──────────────────────┘
```

**Analogy:**
- Keras defines neural networks → compiles to TensorFlow/PyTorch/JAX
- like-keras defines robot systems → compiles to Dora/ROS2/etc.

---

## Supported Backends

### 1. Native (Default)

**What:** Runs everything in the same Python process
**Good for:** Development, debugging, prototyping
**Launch:** `system.launch()` or `system.launch(backend="native")`

```python
system = MyRobotSystem()
system.launch(iterations=100)  # Native launch
```

### 2. Dora RS (Recommended for Deployment)

**What:** Compiles to Dora dataflow.yml format
**Good for:** Production, multi-process, performance
**Launch:** `dora start dataflow.yml`

```python
# Compile only
dataflow_path = system.compile_to_backend("dora")

# Or compile + launch
system.launch(backend="dora")
```

**Generated dataflow.yml:**
```yaml
nodes:
  - id: agent_node
    operator:
      python: nodes/agent_node.py
    inputs:
      obs:
        source: env_node/obs
    outputs:
      - action
  
  - id: env_node
    operator:
      python: nodes/env_node.py
    inputs:
      action_in:
        source: agent_node/action
    outputs:
      - obs
      - reward
      - done
```

### 3. ROS2 (Future)

**What:** Compiles to ROS2 launch file
**Good for:** ROS ecosystem integration
**Launch:** `ros2 launch pkg launch.py`

```python
system.compile_to_backend("ros2", "launch.py")
```

---

## Architecture

### Components → Nodes → System

```python
# 1. Define Components (computational units)
class MyAgent(Agent):
    def forward(self, obs):
        return Action(...)

# 2. Assign to Nodes (execution contexts)
node = Node(name="agent_node")
agent = MyAgent.from_node(node)

# 3. Create System (orchestrator)
system = System(
    nodes=[node],
    components=[agent],
)

# 4. Compile to backend
system.compile_to_backend("dora")
```

### Mapping: like-keras → Dora

| like-keras | Dora RS |
|------------|---------|
| `System` | `dataflow` |
| `Node` | `node` (with operator) |
| `Component` | Python operator logic |
| `Connection` | `inputs` with `source` |
| `OutputPort` | `outputs` list |

---

## Typical Workflows

### Development Workflow

```python
# 1. Develop locally (fast iteration)
system.launch(iterations=100)  # Native backend

# 2. Test compilation
dataflow_path = system.compile_to_backend("dora")
# → Inspect generated dataflow.yml

# 3. Deploy with Dora
system.launch(backend="dora")
```

### Production Deployment

```bash
# 1. Generate dataflow on dev machine
$ python my_system.py --compile-only --backend=dora

# 2. Deploy to robot
$ scp dataflow.yml robot@192.168.1.100:/opt/robot/

# 3. Launch on robot
$ ssh robot@192.168.1.100
$ cd /opt/robot && dora start dataflow.yml
```

### Multi-Language Systems

```python
# Python components
node_python = Node(name="control")
agent = PythonAgent.from_node(node_python)

# C++ components (future)
node_cpp = Node(name="vision")
node_cpp.dora_operator = {
    "rust": "target/release/vision_node"
}

system = System(nodes=[node_python, node_cpp])
system.compile_to_backend("dora")
# → Generates dataflow with mixed Python/Rust operators
```

---

## Command-Line Interface

Make your systems backend-agnostic:

```python
#!/usr/bin/env python3
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', default='native',
                       choices=['native', 'dora', 'ros2'])
    parser.add_argument('--compile-only', action='store_true',
                       help='Compile but dont launch')
    parser.add_argument('--output', help='Output path for compiled file')
    args = parser.parse_args()
    
    # Create system
    system = MyRobotSystem()
    
    if args.compile_only:
        # Just compile
        path = system.compile_to_backend(args.backend, args.output)
        print(f"Compiled to: {path}")
    else:
        # Compile and launch
        system.launch(backend=args.backend)

if __name__ == "__main__":
    main()
```

**Usage:**
```bash
# Development
$ python robot_system.py

# Test compilation
$ python robot_system.py --compile-only --backend=dora

# Deploy with Dora
$ python robot_system.py --backend=dora

# Deploy with ROS2
$ python robot_system.py --backend=ros2
```

---

## Implementation Details

### System.launch() with Backends

```python
def launch(self, iterations=None, backend=None):
    """
    Launch system with specified backend.
    
    Args:
        iterations: Number of iterations (native only)
        backend: "native", "dora", "ros2", or None (default native)
    """
    if backend and backend != "native":
        return self._launch_with_backend(backend)
    
    # Native launch
    self.configure()
    self.activate()
    self.run(iterations)
    self.shutdown()
```

### Backend Compilation

```python
def compile_to_backend(self, backend, output_path=None):
    """
    Compile system to backend format.
    
    Returns:
        Path to compiled file
    """
    from lk.common.backend import create_backend, BackendConfig
    
    config = BackendConfig(
        backend_type=backend,
        output_file=output_path,
        auto_launch=False,
    )
    
    backend = create_backend(backend, config)
    return backend.compile(self)
```

---

## Dora Backend Details

### DoraBackend Class

```python
class DoraBackend(Backend):
    def compile(self, system: System) -> Path:
        """Convert System to dataflow.yml"""
        dataflow = self._build_dataflow(system)
        
        # Write YAML
        with open(output_path, 'w') as f:
            yaml.dump(dataflow, f)
        
        return output_path
    
    def launch(self, dataflow_path: Path):
        """Launch with: dora start dataflow.yml"""
        subprocess.run(['dora', 'start', str(dataflow_path)])
```

### Dataflow Generation

1. **Nodes**: Each `Node` → Dora node with operator
2. **Connections**: Each `Connection` → input with source
3. **Ports**: Output ports → outputs list

```python
def _build_dataflow(self, system):
    dataflow = {'nodes': []}
    
    for node in system.nodes:
        dataflow['nodes'].append({
            'id': node.name,
            'operator': self._build_operator(node),
            'inputs': self._build_inputs(node, system),
            'outputs': self._build_outputs(node),
        })
    
    return dataflow
```

---

## Multi-Machine Deployment (Future)

For distributed systems across multiple machines:

**Option 1: Separate Dataflows**
```bash
# Machine A (control)
$ dora start control_dataflow.yml

# Machine B (sensors)
$ dora start sensors_dataflow.yml
```

**Option 2: Remote SSH Launch**
```python
# Define remote nodes
node_remote = Node(name="sensors")
node_remote.machine_address = "192.168.1.100"

system.compile_to_backend("dora")
# → Generates dataflow with remote node specs
```

---

## Comparison: Dora RS vs ROS2 Launch

| Feature | Dora RS | ROS2 Launch |
|---------|---------|-------------|
| Format | YAML (dataflow.yml) | Python (launch.py) |
| Languages | Python, Rust, C++ | Python, C++ |
| IPC | Shared memory + network | DDS middleware |
| Launch Tool | `dora start` | `ros2 launch` |
| Discovery | Coordinator | DDS discovery |
| Zero-copy | ✓ (shared mem) | △ (depends on DDS) |

---

## FAQ

**Q: Why not implement our own launch system?**
A: Same reason Keras doesn't implement tensor ops - let specialized tools do what they do best. Dora and ROS2 are mature, optimized, and well-tested.

**Q: Can I mix backends?**
A: Not in a single system launch, but you can have different systems using different backends that communicate via network.

**Q: Do I need to install Dora/ROS2?**
A: Only if you want to use those backends. Native backend works out of the box.

**Q: How do I debug Dora-launched systems?**
A: Develop with native backend first, then compile to Dora once working.

**Q: Can I customize generated dataflow.yml?**
A: Yes! Compile with `--compile-only`, edit the YAML, then launch manually with `dora start`.

---

## Next Steps

1. **Install Dora RS**: https://dora-rs.ai/docs/guides/installation
2. **Try examples**: `python examples/ex_06_backend_compilation.py`
3. **Create your system**: Define in like-keras, compile to Dora
4. **Deploy**: Use `dora start` on production hardware

**Resources:**
- Dora RS Docs: https://dora-rs.ai
- ROS2 Launch: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Launch-Main.html
- like-keras Examples: `examples/`

