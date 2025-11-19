# BAM Logger Module Implementation

## Overview

Create `bam_logger` as a thin multi-backend dispatcher for logging, mirroring the `bam_artist` architecture. Users can attach multiple logging backends (Python logging, ROS2 rclpy) and log messages that automatically include context (run_id, robot_id, etc.).

## Architecture Pattern

```python
from bam_logger import Logger

logger = Logger(run_id="exp_42", robot_id="BAM-01")  # Global context
logger.attach_python_backend()
logger.info("Controller started", component="arm.control", latency_ms=123)
```

Messages automatically include global context + per-call context via kwargs.

## File Structure

```
bam_logger/
├── CMakeLists.txt              # Minimal ROS2 CMake
├── package.xml                 # ROS2 package definition
├── pyproject.toml              # Python package definition
├── README.md                   # Usage guide
├── .order                      # BAM folder ordering
├── bam_logger/
│   ├── __init__.py            # Export Logger, backends
│   ├── backend.py             # Base Backend class (Protocol)
│   ├── logger.py              # Main Logger dispatcher
│   ├── python_backend.py      # Python stdlib logging backend
│   └── rclpy_backend.py       # ROS2 rclpy backend (stub)
└── examples/
    ├── ex_basic_usage.py      # Simple logging example
    ├── ex_context_injection.py # Context handling demo
    └── ex_multi_backend.py    # Multiple backends demo
```

## Core Components

### 1. Backend Base Class (`backend.py`)

Protocol defining the backend interface with methods: `debug()`, `info()`, `warning()`, `error()`, `critical()`. Each method takes `msg: str, **context`.

### 2. Logger Dispatcher (`logger.py`)

- Stores global context dict
- Manages multiple backends (dict of name -> backend)
- Methods: `debug()`, `info()`, `warning()`, `error()`, `critical()`
- Merges global context + per-call kwargs, dispatches to all backends
- Helper methods: `attach_python_backend()`, `attach_rclpy_backend(node)`

### 3. Python Backend (`python_backend.py`)

- Wraps Python's `logging.Logger` with custom formatter
- Supports structured context injection via LoggerAdapter
- Configurable format string that includes context fields
- Default format: `"%(asctime)s | %(levelname)s | %(name)s | %(message)s | %(context)s"`

### 4. ROS2 Backend (`rclpy_backend.py`)

Stub implementation for now with TODO comments. Will format messages with context and call `node.get_logger().info()` etc.

## Key Features

1. **Flexible context injection**: Both global (set once) and per-call (kwargs)
2. **Common context params**: Support specific params like `run_id`, `robot_id`, `component` in method signatures
3. **Free-form kwargs**: Any additional context via `**kwargs`
4. **Multi-backend**: Messages go to all attached backends simultaneously
5. **Clean API**: Simple `logger.info(msg, **context)` pattern

## Implementation Details

### Logger.info() signature:

```python
def info(self, msg: str, **context) -> None:
    """Log info message with optional context."""
    merged_context = {**self.global_context, **context}
    for backend in self.backends.values():
        backend.info(msg, **merged_context)
```

### Python Backend setup:

- Use `logging.LoggerAdapter` for context injection
- Custom formatter that pretty-prints context dict
- Setup queue-based async logging for performance (optional)
- Rate limiting filter for high-frequency loops (optional)

## Package Configuration

- **CMakeLists.txt**: Minimal, just `ament_python_install_package(bam_logger)`
- **package.xml**: Depends on `ament_cmake`, `ament_cmake_python`, `rclpy`
- **pyproject.toml**: Hatchling build system, no dependencies initially

## Examples

1. **ex_basic_usage.py**: Create logger, attach Python backend, log at various levels
2. **ex_context_injection.py**: Demonstrate global context + per-call kwargs
3. **ex_multi_backend.py**: Attach multiple backends, show messages going to all

## Following BAM Conventions

- Shebang on executable files
- Docstrings indented for collapsing
- Import order: `# BAM`, `# PYTHON`
- Type hints on all function signatures
- Region blocks for code organization
- Keep `__main__` examples minimal with imports inside the block
- No markdown documentation files created (user's memory)

## Testing

Create minimal `__main__` blocks in each file to demonstrate functionality without external dependencies.