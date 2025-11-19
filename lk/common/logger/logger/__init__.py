
# Primary API: Lightweight multi-backend logger
from .logger import (
    Logger,
    get_logger,
    configure,
    get_root,
    get_logger_registry,
    set_logger_level
)

# Backends (handle their own setup)
from .backend import Backend
from .python_backend import PythonBackend
from .rclpy_backend import RclpyBackend

# Optional backends (require additional dependencies)
try:
    from .foxglove_backend import FoxgloveBackend
except ImportError:
    FoxgloveBackend = None

try:
    from .artist_backend import ArtistBackend
except ImportError:
    ArtistBackend = None

# Configure global logger with sensible defaults
# Users can override by calling configure() in their scripts
import os

configure(
    py=True,
    py_level=os.environ.get('BAM_LOG_LEVEL', 'INFO'),
    robot_id=os.environ.get('ROBOT_ID', None),
    include_call_site=True
)

