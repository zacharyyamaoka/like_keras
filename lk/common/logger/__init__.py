"""
BAM Logger Package

Top-level exports for bam_logger package.
"""

# Import and re-export to create top-level aliases
from .logger import (
    Logger,
    get_logger,
    configure,
    get_root,
    get_logger_registry,
    set_logger_level,
    Backend,
    PythonBackend,
    RclpyBackend,
)

# Optional backends (require additional dependencies)
try:
    from .logger import FoxgloveBackend
except ImportError:
    FoxgloveBackend = None

try:
    from .logger import ArtistBackend
except ImportError:
    ArtistBackend = None

# Create module alias in sys.modules for backward compatibility
import sys
sys.modules['bam_logger'] = sys.modules[__name__]

# Configure global logger with sensible defaults
# Users can override by calling configure() in their scripts
import os

configure(
    py=True,
    py_level=os.environ.get('BAM_LOG_LEVEL', 'INFO'),
    robot_id=os.environ.get('ROBOT_ID', None),
    include_call_site=True
)

