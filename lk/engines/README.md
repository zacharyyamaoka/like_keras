# Execution Engines

This directory contains execution engines for like_keras - backends that compile and run System graphs on different platforms.

## Overview

like_keras uses a **compilation approach** similar to Keras:
- Keras compiles models to TensorFlow or PyTorch
- like_keras compiles Systems to Dora, ROS2, or native Python

## Architecture

```
Component (atomic) + System (composition)
    ↓
Node assignment (WHERE to run)
    ↓
Engine.compile() (HOW to run)
    ↓
Engine.launch() (execute via engine CLI)
```

## Available Engines

### Native Engine (`native.py`)

Pure Python in-process execution. Best for development.

**Features:**
- No compilation step
- Single process
- Easy debugging
- Fast iteration

**Use for:** Development, testing, prototyping

### Dora Engine (`dora.py`)

Compiles to Dora-rs dataflow system. Best for production.

**Features:**
- Multi-machine deployment
- Zero-copy shared memory (local)
- Zenoh networking (distributed)  
- Multi-language support (Python, Rust, C++, C)

**Use for:** Production, distributed systems, performance

### ROS2 Engine (Future)

Will compile to ROS2 launch files.

## Usage

### Simple (Native)

```python
# No explicit nodes = native engine
camera = CameraComponent()
detector = DetectorComponent()

system = System(components=[camera, detector])
system.launch()  # Runs in-process
```

### Distributed (Dora)

```python
# Define deployment nodes
jetson = Node('jetson', machine='192.168.1.10', engine='dora')
nuc = Node('nuc', machine='192.168.1.11', engine='dora')

# Assign components to nodes
camera = CameraComponent(node=jetson)
planner = PlannerComponent(node=nuc)

# System auto-discovers nodes and engine
system = System(components=[camera, planner])
system.launch()  # Compiles to Dora and launches
```

## Adding New Engines

1. Create new engine file (e.g., `myengine.py`)
2. Inherit from `Engine` base class
3. Implement required methods:
   - `compile(system) -> Path`
   - `launch(system, config_path, **kwargs)`
   - `validate(system) -> tuple[bool, list[str]]`
4. Register in `__init__.py`:
   ```python
   from lk.engines.myengine import MyEngine
   register_engine('myengine', MyEngine)
   ```

## File Structure

```
engines/
├── __init__.py          # Engine registry
├── engine.py            # Abstract base class
├── native.py            # Native Python engine
├── dora.py              # Dora-rs engine
├── ros2.py              # ROS2 engine (future)
└── backends/            # Node backends
    ├── __init__.py
    ├── dora_node.py     # Generic Dora node
    └── ros2_node.py     # Generic ROS2 node (future)
```

## Key Design Decisions

### 1. Components are Atomic

No sub-components. Use normal Python inside components.

### 2. Nodes Define Deployment

Nodes specify WHERE (machine) and HOW (engine) to run.

### 3. Auto-Discovery

Systems auto-discover nodes from component assignments.

### 4. Leverage Existing Tools

Engines don't reimplement execution - they generate configs and call existing CLIs (Dora, ROS2, etc.).

## See Also

- [../docs/engines.md](../docs/engines.md) - Full documentation
- [../docs/IMPLEMENTATION_SUMMARY.md](../docs/IMPLEMENTATION_SUMMARY.md) - Implementation details
- [../examples/ex_05_multi_engine.py](../examples/ex_05_multi_engine.py) - Usage examples

