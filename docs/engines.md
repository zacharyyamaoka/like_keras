# Execution Engines

## Overview

like_keras uses a **compilation approach** to support different execution engines. Like Keras compiling to TensorFlow or PyTorch, like_keras compiles System graphs to different execution backends:

- **Native**: Pure Python in-process execution
- **Dora**: Dora-rs dataflow system for distributed robotics
- **ROS2**: ROS2 launch system (future)

## Architecture

```
System (like_keras API) → Define WHAT to compute
    ↓
Engine.compile() → Generate engine config files  
    ↓
Engine.launch() → Execute via engine CLI
```

## Two Primitives

like_keras uses a simple two-primitive architecture (like ResNet or behavior trees):

1. **Component** = Atomic computation (leaf node)
2. **System** = Composition (composite node)

### Component: Atomic Computation

Components are **atomic units** - they don't contain other components. Use normal Python inside:

```python
class VisionPipeline(Component):
    def __init__(self, node=None):
        super().__init__('vision', node=node)
        
        # Normal Python - use any libraries you want!
        self.camera = Camera()
        self.detector = YOLOv8()
        self.tracker = ByteTrack()
        self.model = torch.load('model.pth')
    
    def forward(self, image):
        # Normal Python calls (fast, in-process)
        detections = self.detector(image)
        tracked = self.tracker(detections)
        return tracked
```

**Key point:** If you need to distribute internal computation, split into separate Components.

### System: Composition

Systems compose atomic Components and assign them to nodes for deployment:

```python
# Define atomic components
camera = CameraComponent()
detector = DetectorComponent()
planner = PlannerComponent()

# Compose in system
robot = System(components=[camera, detector, planner])
```

## Node Assignment

Nodes define **where** components run (which machine, which engine). Assign nodes to components like port connections:

```python
# Define deployment nodes
jetson = Node('jetson', machine='192.168.1.10', engine='dora')
nuc = Node('nuc', machine='192.168.1.11', engine='dora')

# Assign at component creation (Pythonic!)
camera = CameraComponent(node=jetson)
detector = DetectorComponent(node=jetson)
planner = PlannerComponent(node=nuc)
controller = ControllerComponent(node=nuc)

# System auto-discovers nodes from components
robot = System(components=[camera, detector, planner, controller])

# Launch (automatically uses engine from nodes)
robot.launch()
```

## Native Engine

Pure Python in-process execution. Best for development and testing.

```python
# Components without explicit nodes use native engine
camera = CameraComponent()
detector = DetectorComponent()

system = System(components=[camera, detector])
system.launch(iterations=100)  # Runs in-process
```

**Use for:**
- Development and debugging
- Single-machine prototypes
- Testing

**Advantages:**
- Fast iteration (no compilation)
- Easy debugging (single process)
- Simple (no external dependencies)

## Dora Engine

Compiles to Dora-rs dataflow system. Best for production and distributed systems.

```python
# Define nodes with dora engine
gpu = Node('gpu', machine='jetson', engine='dora')
cpu = Node('cpu', machine='nuc', engine='dora')

# Assign components
vision = VisionComponent(node=gpu)
planner = PlannerComponent(node=cpu)

system = System(components=[vision, planner])

# Compile to Dora YAML
config_path = system.compile(engine='dora')
# Generates: .lk_build/dora/dataflow.yaml

# Or launch directly
system.launch()  # Calls: dora start dataflow.yaml
```

**Use for:**
- Production deployments
- Distributed systems
- High-performance applications
- Multi-machine robotics

**Features:**
- Zero-copy shared memory (same machine)
- Zenoh networking (distributed)
- Multi-language support (Python, Rust, C++, C)
- Coordinator/daemon architecture

### Distributed Execution

For multi-machine deployment, Dora uses a coordinator/daemon architecture:

```python
# 1. Start coordinator (on central machine)
# $ dora coordinator

# 2. Start daemon on each worker machine
# $ dora daemon --machine-id jetson --coordinator-addr 192.168.1.100
# $ dora daemon --machine-id nuc --coordinator-addr 192.168.1.100

# 3. Define system with distributed nodes
jetson = Node('jetson', machine='192.168.1.10', engine='dora')
nuc = Node('nuc', machine='192.168.1.11', engine='dora')

camera = CameraComponent(node=jetson)
planner = PlannerComponent(node=nuc)

system = System(components=[camera, planner])

# 4. Compile and launch (distributed)
system.launch(coordinator_addr='192.168.1.100')
```

## Switching Between Engines

Same computational graph, different deployments:

```python
# Same components
camera = CameraComponent()
detector = DetectorComponent()
planner = PlannerComponent()

# Development: native engine (all local)
dev_system = System(components=[camera, detector, planner])
dev_system.launch(engine='native', iterations=10)

# Production: dora engine (distributed)
jetson = Node('jetson', machine='jetson', engine='dora')
nuc = Node('nuc', machine='nuc', engine='dora')

camera.node = jetson
detector.node = jetson  
planner.node = nuc

prod_system = System(components=[camera, detector, planner])
prod_system.launch(engine='dora')
```

## Engine API

All engines implement the same interface:

```python
class Engine(ABC):
    def compile(self, system: System) -> Optional[Path]:
        """Generate engine-specific config files."""
        
    def launch(self, system: System, config_path: Optional[Path], **kwargs):
        """Execute via engine's native tooling."""
        
    def validate(self, system: System) -> tuple[bool, list[str]]:
        """Check system compatibility."""
```

## Best Practices

### 1. Component = Atomic

Don't nest components. Use normal Python for internal structure:

```python
# ✅ Good: atomic component with Python internals
class Pipeline(Component):
    def __init__(self):
        super().__init__('pipeline')
        self.stage1 = Stage1()  # Normal Python
        self.stage2 = Stage2()  # Normal Python

# ❌ Bad: trying to nest components
class Pipeline(Component):
    def __init__(self):
        super().__init__('pipeline')
        self.stage1 = Stage1Component()  # Don't do this!
        self.stage2 = Stage2Component()  # Use System instead
```

### 2. Explicit Node Assignment

Assign nodes explicitly for clarity:

```python
# ✅ Good: clear deployment
gpu = Node('gpu', machine='jetson', engine='dora')
vision = VisionComponent(node=gpu)

# ⚠️ Works but less clear
vision = VisionComponent()  # Defaults to local/native
```

### 3. Same Engine Per System

All nodes in a system must use the same engine (heterogeneous not supported yet):

```python
# ✅ Good: all dora
node1 = Node('n1', engine='dora')
node2 = Node('n2', engine='dora')

# ❌ Bad: mixed engines
node1 = Node('n1', engine='dora')
node2 = Node('n2', engine='ros2')  # Error!
```

### 4. Gradual Refinement

Start simple (native), optimize later (dora):

```python
# Phase 1: Prototype (native)
system = System(components=[...])
system.launch(engine='native')

# Phase 2: Profile and identify bottlenecks
# Add nodes only where needed

# Phase 3: Optimize (dora)
gpu_node = Node('gpu', engine='dora')
heavy_component.node = gpu_node
system.launch(engine='dora')
```

## See Also

- [Components Guide](components.md) - Component design patterns
- [Dora Engine Details](dora-engine.md) - Dora-specific features
- [Examples](../examples/) - Working examples

