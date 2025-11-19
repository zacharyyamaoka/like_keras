# BAM Logger

**Super lightweight multi-backend logger with context injection and generic message support.**

## Quick Start

```python
# Already configured in bam_logger/__init__.py - just import and use!
from bam_logger import get_logger

# Get loggers anywhere with standardized name field
arm_logger = get_logger(name="arm_controller", component="arm")
arm_logger.info("Arm ready", dof=6)

gripper_logger = get_logger(name="gripper_controller", component="gripper")
gripper_logger.info("Gripper ready", force_n=100)

# Common pattern: use __name__ for module identification
logger = get_logger(name=__name__)
logger.info("Module loaded")

# Override in dev/scripts if needed
from bam_logger import configure
configure(py=True, robot_id="DEV-01", py_level="DEBUG")
```

## New: Generic Message Support 🎉

The logger now accepts **any message type**, not just strings!

```python
from bam_logger import Logger
import bam.msgs.visual_objects as viz

logger = Logger()
logger.get_python_backend()        # For strings
logger.get_foxglove_backend()      # For live viz + MCAP recording
logger.get_artist_backend()        # For 3D visual objects

# String messages (still lightweight!)
logger.info("Task started")

# Visual objects → rendered in Artist
frame = viz.Frame.from_Pose(pose, axis_length=0.1)
logger.info(frame, topic="/viz/frames")

# Dataclass messages
logger.info(sensor_reading, topic="/sensors/temp")

# With topics and timestamps
logger.info(msg, topic="/robot/arm", log_time=time.time())
```

## Core API

### Recommended Pattern (Global Singleton)

```python
# Auto-configured in bam_logger/__init__.py - just use it!
from bam_logger import get_logger

# Standardized: use name field + additional context
logger = get_logger(name=__name__, component="arm")
logger.info("Ready", dof=6)

# Override if needed (dev/scripts)
from bam_logger import configure
configure(py=True, robot_id="DEV-01", py_level="DEBUG")
```

### Alternative Pattern (Explicit Root)

```python
# Create root explicitly (for more control)
from bam_logger import Logger

root = Logger(robot_id="BAM-01")
root.get_python_backend()

arm = root.get_logger(name="arm_controller")
arm.info("Ready")
```

## Features

### 1. Generic Message Types (NEW!)

Logger accepts **any message type** - automatically routes to appropriate backends:

```python
logger = Logger()
logger.get_python_backend()      # String conversion
logger.get_foxglove_backend()    # Live visualization
logger.get_artist_backend()      # 3D rendering

# Strings (most common - super lightweight!)
logger.info("System ready")

# Visual objects → Artist backend
logger.info(viz.Frame.from_Pose(pose), topic="/viz/robot")

# Dataclasses → Formatted automatically
logger.info(sensor_reading, topic="/sensors")

# Foxglove schemas → Direct to Foxglove
from foxglove.schemas import Log, LogLevel
logger.info(Log(level=LogLevel.Info, message="..."), topic="/log")
```

### 2. Context Injection

Auto-merges global + per-call context:

```python
configure(py=True, robot_id="BAM-01", run_id="exp_001")
logger = get_logger(component="arm")

# Has: robot_id + run_id + component
logger.info("Task done", task_id="pick_123", duration_ms=456)
# Output: "Task done | robot_id=BAM-01 | run_id=exp_001 | component=arm | task_id=pick_123 | duration_ms=456"
```

### 3. Multi-Backend Routing

```python
root = Logger(robot_id="BAM-01")
root.get_python_backend(name="console", level="WARNING")
root.get_python_backend(name="file", level="DEBUG", to_file="/tmp/robot.log")
root.get_foxglove_backend(mcap_file="/tmp/robot.mcap")

logger = root.get_logger(component="arm")
logger.warning("High CPU")  # Goes to all backends
```

### 4. Child Logger Inheritance

```python
configure(py=True, robot_id="BAM-01")

arm = get_logger(component="arm")
joints = arm.get_logger(subsystem="joints")
# joints has: robot_id + component + subsystem
```

### 5. ROS 2-Style Features

```python
# Throttle: Rate limit messages
logger.info("Control loop", throttle_duration_sec=1.0)

# Once: Log only first time
logger.info("Init complete", once=True)

# Skip first: Skip first occurrence
logger.info("Calibration", skip_first=True, value=sensor.read())
```

### 6. Call Site Tracking

```python
# Default: includes file and line number
configure(py=True)
logger = get_logger(component="arm")
logger.info("Message")  # Includes file=.../path/to/file.py, file_abs=/full/abs/path/to/file.py, line=42

# Disable if not needed
configure(py=True, include_call_site=False)
```

**Path Formatting:**
- `file`: Shortened path (30 chars from back, `~` for home directory) - for display
- `file_abs`: Full absolute path - for introspection and finding exact location
- `line`: Line number

Example output:
```
INFO | Message | file=.../bam_env_ws/src/reach.py | file_abs=/home/user/bam_env_ws/src/reach.py | line=42
```

### 7. Topic-Based Routing (NEW!)

```python
# Organize messages by topic (like ROS)
logger.info("Arm ready", topic="/robot/arm")
logger.info("Camera active", topic="/sensors/camera")
logger.info(visual_obj, topic="/viz/scene")

# Useful for filtering in Foxglove or organizing in MCAP files
```

### 8. Custom Timestamps (NEW!)

```python
# Log with specific timestamp (useful for historical data)
logger.info("Historical reading", log_time=past_timestamp, value=42.0)

# Default: uses current time
logger.info("Current reading", value=43.2)
```

## API Reference

### Global Configuration

```python
from bam_logger import configure, get_logger

# Configure global root
configure(
    py=True,                    # Enable Python backend
    py_level="INFO",            # Python log level
    py_file="/tmp/robot.log",  # Optional file
    include_call_site=True,     # Add file/line to context
    **global_context            # Global context (robot_id, run_id, etc.)
)

# Get loggers
logger = get_logger(**context)
```

### Logging Methods

```python
# NEW: msg accepts ANY type!
logger.debug(msg, topic=None, log_time=None, throttle_duration_sec=None, skip_first=False, once=False, **context)
logger.info(msg, topic=None, log_time=None, throttle_duration_sec=None, skip_first=False, once=False, **context)
logger.warning(msg, topic=None, log_time=None, throttle_duration_sec=None, skip_first=False, once=False, **context)
logger.error(msg, topic=None, log_time=None, throttle_duration_sec=None, skip_first=False, once=False, **context)
logger.critical(msg, topic=None, log_time=None, throttle_duration_sec=None, skip_first=False, once=False, **context)

# msg: Any type - str, dict, dataclass, visual object, Foxglove schema, etc.
# topic: Optional string for routing/organization (e.g., "/robot/arm")
# log_time: Optional timestamp (seconds since epoch)
```

### Backend Management

```python
# Get backends (they configure themselves)
root.get_python_backend(name="root", level="INFO", to_file=None, verbose=False)
root.get_rclpy_backend(node, name="rclpy")

# NEW: Foxglove backend (requires: pip install foxglove-sdk)
root.get_foxglove_backend(
    server_enabled=True,      # WebSocket server for live viz
    mcap_file="/tmp/log.mcap", # MCAP recording
    server_port=8765
)

# NEW: Artist backend (requires: bam_artist, visual_objects)
from bam_artist import Artist
artist = Artist()
artist.attach_viser_viewer()
root.get_artist_backend(artist=artist)
```

## Examples

**Basic Usage:**
- `quick_start.py` - Complete usage demo
- `ex_basic_usage.py` - Simple logging
- `ex_child_loggers.py` - Child logger pattern
- `ex_context_injection.py` - Context merging
- `ex_multi_backend.py` - Multiple backends
- `ex_throttle_once_skipfirst.py` - ROS 2 features
- `ex_call_site.py` - Call site tracking

**NEW - Generic Message Types:**
- `ex_generic_messages.py` - Different message types (str, dict, dataclass)
- `ex_foxglove_backend.py` - Live visualization + MCAP recording
- `ex_artist_backend.py` - 3D visual object rendering
- `ex_multi_backend_advanced.py` - All backends together (complete demo)

## Design

**Logger does FOUR things:**
1. Accepts any message type (strings, dicts, dataclasses, visual objects, etc.)
2. Merges context (global + per-call)
3. Routes to backends (multi-backend dispatch with type-specific handling)
4. Creates child loggers (inherit everything)

**Backends handle their own type conversion:**
- **Python backend:** Converts everything to strings for console/file output
- **Foxglove backend:** Converts to Foxglove schemas for live viz + MCAP
- **Artist backend:** Renders visual objects to viewers (Viser, O3D, Meshcat)
- **ROS 2 backend:** Uses `node.get_logger()` (stub)

**Key insight:** Different message types are automatically routed to appropriate backends. String messages remain super lightweight (no overhead), while visual objects and structured data get proper handling.

**Result:** ~500 lines core + ~300 lines per backend for complete multi-backend logging with generic message support.
