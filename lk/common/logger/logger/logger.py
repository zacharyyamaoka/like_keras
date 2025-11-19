#!/usr/bin/env python3

"""
    Logger - Lightweight multi-backend logger with context injection.
    
    Does TWO things:
    1. Auto-injects context (merges global + per-call)
    2. Routes calls to multiple backends
    
    Backends handle their own setup. Logger stays lightweight.
    
    Example:
        from bam_logger import Logger
        
        logger = Logger(run_id="exp_42", robot_id="BAM-01")
        logger.get_python_backend()  # Gets logger from global logging.getLogger()
        logger.info("Controller started", component="arm.control", latency_ms=123)
"""

# BAM
from .backend import Backend

# PYTHON
import time
import os
from typing import Optional, Any, TYPE_CHECKING
import inspect

if TYPE_CHECKING:
    from .python_backend import PythonBackend
    from .rclpy_backend import RclpyBackend


def _format_path_for_display(file_path: str, max_length: int = 30) -> str:
    """Format file path for display: shorten and replace home with ~.
    
    Args:
        file_path: Absolute file path
        max_length: Maximum length of displayed path (default: 30)
        
    Returns:
        Shortened path with home replaced by ~, truncated from the front if needed
        
    Example:
        _format_path_for_display("/home/user/very/long/path/to/bam_env_ws/src/reach.py")
        # Returns: ".../bam_env_ws/src/reach.py" (if max_length=30, truncated)
        
        _format_path_for_display("/home/user/file.py")
        # Returns: "~/file.py" (if max_length=30, short enough)
    """
    # Replace home directory with ~
    home = os.path.expanduser("~")
    if file_path.startswith(home):
        file_path = file_path.replace(home, "~", 1)
    
    # If path is already short enough, return as-is
    if len(file_path) <= max_length:
        return file_path
    
    # Truncate from the front, keeping the last max_length chars
    return "..." + file_path[-(max_length - 3):]


class Logger:
    """Lightweight multi-backend logger with context injection.
    
    Responsibilities:
    1. Merge global + per-call context
    2. Route to attached backends
    3. Create child loggers that inherit configuration
    
    Backends handle their own configuration.
    """
    
    def __init__(self, include_call_site: bool = True, **global_context):
        """Initialize lightweight logger with optional global context.
        
        Args:
            include_call_site: If True, adds file and line number to log context
            **global_context: Key-value pairs to include in all log messages
                             (e.g., run_id="exp_42", robot_id="BAM-01")
        """
        self.include_call_site = include_call_site
        self.global_context: dict[str, object] = global_context
        self.backends: dict[str, Backend] = {}
        self.parent: Optional['Logger'] = None
        
        # State for throttle/once/skip_first
        self._throttle_state: dict[str, float] = {}  # call_site -> last_time
        self._once_state: set[str] = set()  # call_sites that have fired
        self._skip_first_state: set[str] = set()  # call_sites to skip first
    
    #region - Child Logger Factory
    
    def get_logger(self, name: str, **context) -> 'Logger':
        """Create a child logger that inherits backends and context from this logger.
        
        Child loggers:
        - Inherit all backends from parent
        - Merge parent context with their own context
        - Share throttle/once/skip_first state with parent
        - Inherit include_call_site setting from parent
        - Hierarchical names with dot notation (like Python logging/rclpy)
        - Registered globally by name (same name = same instance)
        
        Args:
            name: Logger name (hierarchical with dots like "arm.joints")
            **context: Additional context for the child logger
            
        Returns:
            Child Logger instance (cached if same name)
            
        Example:
            # Configure root logger once
            root = Logger(robot_id="BAM-01")
            root.get_python_backend()
            
            # Get child loggers (hierarchical naming)
            arm = root.get_logger(name="arm")  # name="arm"
            joints = arm.get_logger(name="joints")  # name="arm.joints"
            
            # Same name returns same instance
            arm2 = root.get_logger(name="arm")  # arm2 is arm
        """
        # Build hierarchical name (like Python logging and rclpy)
        parent_name = self.global_context.get('name', '')
        if parent_name:
            # Parent has name, append child name with dot
            hierarchical_name = f"{parent_name}.{name}"
        else:
            # Parent has no name, use child name as-is
            hierarchical_name = name
        
        # Check registry - return existing logger if found (like logging.getLogger)
        global _logger_registry
        if hierarchical_name in _logger_registry:
            return _logger_registry[hierarchical_name]
        
        # Add name to context
        context['name'] = hierarchical_name
        
        # Create new child logger
        child = Logger(include_call_site=self.include_call_site, **context)
        child.parent = self
        
        # Register in global registry
        _logger_registry[hierarchical_name] = child
        
        return child
    
    def set_logger_level(self, level: str) -> None:
        """Set the log level for this logger's Python backend.
        
        Args:
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        """
        import logging
        for backend in self._get_all_backends().values():
            if hasattr(backend, 'logger'):  # Python backend
                backend.logger.setLevel(getattr(logging, level.upper()))
    
    def _get_all_backends(self) -> dict[str, Backend]:
        """Get all backends including parent's backends."""
        if self.parent:
            # Parent backends + our backends (ours override if same name)
            backends = dict(self.parent._get_all_backends())
            backends.update(self.backends)
            return backends
        return dict(self.backends)
    
    def _get_merged_context(self) -> dict[str, object]:
        """Get context merged with parent's context."""
        if self.parent:
            # Parent context + our context (ours override if same key)
            return {**self.parent._get_merged_context(), **self.global_context}
        return dict(self.global_context)
    
    #endregion - Child Logger Factory
    
    #region - Backend Management
    
    def get_python_backend(
        self, 
        name: str = "root",
        level: str = "INFO",
        to_file: Optional[str] = None,
        verbose: bool = False
    ) -> 'PythonBackend':
        """Get a Python logging backend (uses global logging.getLogger()).
        
        The backend uses Python's global logger via logging.getLogger(name).
        Backend configures itself (handlers, formatters, levels).
        
        Args:
            name: Logger name for logging.getLogger(name)
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            to_file: Optional file path for rotating file handler
            verbose: Enable verbose output
            
        Returns:
            The created PythonBackend instance
        """
        from .python_backend import PythonBackend
        backend = PythonBackend(name=name, level=level, to_file=to_file, verbose=verbose)
        self.backends[backend.name] = backend
        return backend
    
    def get_rclpy_backend(self, node, name: str = "rclpy") -> 'RclpyBackend':
        """Get a ROS2 rclpy logging backend (uses node.get_logger()).
        
        The backend uses the node's logger via node.get_logger().
        Backend publishes to /rosout and integrates with ROS 2 logging.
        
        Args:
            node: ROS2 node instance
            name: Backend name
            
        Returns:
            The created RclpyBackend instance
        """
        from .rclpy_backend import RclpyBackend
        backend = RclpyBackend(node=node, name=name)
        self.backends[backend.name] = backend
        return backend
    
    def get_foxglove_backend(
        self,
        name: str = "foxglove",
        server_enabled: bool = True,
        mcap_file: Optional[str] = None,
        server_port: int = 8765,
        server_host: str = "127.0.0.1",
        verbose: bool = False
    ):
        """Get a Foxglove logging backend for live visualization and MCAP recording.
        
        Requires foxglove-sdk: pip install foxglove-sdk
        
        Args:
            name: Backend name
            server_enabled: Enable Foxglove WebSocket server for live visualization
            mcap_file: Optional MCAP file path for recording
            server_port: WebSocket server port
            server_host: WebSocket server host
            verbose: Enable verbose output
            
        Returns:
            The created FoxgloveBackend instance
            
        Raises:
            ImportError: If foxglove-sdk is not installed
        """
        from .foxglove_backend import FoxgloveBackend
        backend = FoxgloveBackend(
            name=name,
            server_enabled=server_enabled,
            mcap_file=mcap_file,
            server_port=server_port,
            server_host=server_host,
            verbose=verbose
        )
        self.backends[backend.name] = backend
        return backend
    
    def get_artist_backend(
        self,
        artist = None,
        name: str = "artist",
        convert_strings_to_text: bool = False,
        verbose: bool = False
    ):
        """Get an Artist logging backend for visual object rendering.
        
        Requires bam_artist and visual_objects packages.
        
        Args:
            artist: Artist instance (or create new one if None)
            name: Backend name
            convert_strings_to_text: Convert string messages to 3D text annotations
            verbose: Enable verbose output
            
        Returns:
            The created ArtistBackend instance
            
        Raises:
            ImportError: If bam_artist is not installed
        """
        from .artist_backend import ArtistBackend
        backend = ArtistBackend(
            artist=artist,
            name=name,
            convert_strings_to_text=convert_strings_to_text,
            verbose=verbose
        )
        self.backends[backend.name] = backend
        return backend
    
    def clear_backends(self) -> None:
        """Clear all backends from this logger.
        
        Useful for disabling logging by removing all output destinations.
        Child loggers created after clearing will inherit no backends.
        
        Example:
            logger = get_logger(name="calibration", robot_id="BAM-01")
            logger.get_python_backend()
            logger.clear_backends()  # Now silent
            
            # Pass to class - child logger will also be silent
            calibrator = Calibrator(..., logger=logger)
        """
        self.backends.clear()
    
    #endregion - Backend Management
    
    #region - Core Logging API
    
    def _should_log(
        self,
        call_site: str,
        throttle_duration_sec: Optional[float] = None,
        skip_first: bool = False,
        once: bool = False
    ) -> bool:
        """Check if message should be logged based on throttle/once/skip_first.
        
        Args:
            call_site: Unique identifier for this log call location
            throttle_duration_sec: Minimum time between messages (seconds)
            skip_first: Skip the first occurrence
            once: Only log once
            
        Returns:
            True if message should be logged
        """
        # Once: only log the first time
        if once:
            if call_site in self._once_state:
                return False
            self._once_state.add(call_site)
            return True
        
        # Skip first: skip the first occurrence
        if skip_first:
            if call_site not in self._skip_first_state:
                self._skip_first_state.add(call_site)
                return False
        
        # Throttle: minimum time between messages
        if throttle_duration_sec is not None:
            now = time.monotonic()
            last_time = self._throttle_state.get(call_site, 0.0)
            if now - last_time < throttle_duration_sec:
                return False
            self._throttle_state[call_site] = now
        
        return True
    
    def _log(
        self,
        level: str,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        throttle_duration_sec: Optional[float] = None,
        skip_first: bool = False,
        once: bool = False,
        **context
    ) -> None:
        """Internal logging method that handles all levels.
        
        Args:
            level: Log level (debug, info, warning, error, critical)
            msg: The log message (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm/state")
            log_time: Optional timestamp for the message (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        
        # Get call site from actual caller (skip _log and public method)
        frame = inspect.currentframe().f_back.f_back  # Skip _log() and debug/info/etc
        call_site = f"{frame.f_code.co_filename}:{frame.f_lineno}"
        
        # Check if should log
        if not self._should_log(call_site, throttle_duration_sec, skip_first, once):
            return
        
        # Merge context (includes parent context if child logger)
        base_context = self._get_merged_context()
        merged_context = {**base_context, **context}
        
        # Add call site to context if enabled
        if self.include_call_site:
            # Extract filename and line number from call_site
            parts = call_site.rsplit(':', 1)
            if len(parts) == 2:
                file_path, line_num = parts
                # Convert to absolute path for introspection
                abs_path = os.path.abspath(file_path)
                # Store absolute path and line number for introspection
                merged_context['file_abs'] = abs_path
                merged_context['line'] = line_num
                # Store shortened display version
                merged_context['file'] = _format_path_for_display(abs_path)
        
        # Dispatch to all backends (includes parent's backends if child logger)
        all_backends = self._get_all_backends()
        for backend in all_backends.values():
            getattr(backend, level)(msg, topic=topic, log_time=log_time, **merged_context)
    
    def debug(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, throttle_duration_sec: Optional[float] = None, skip_first: bool = False, once: bool = False, **context) -> None:
        """Log debug message.
        
        Args:
            msg: Message to log (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm")
            log_time: Optional timestamp (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        self._log("debug", msg, topic, log_time, throttle_duration_sec, skip_first, once, **context)
    
    def info(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, throttle_duration_sec: Optional[float] = None, skip_first: bool = False, once: bool = False, **context) -> None:
        """Log info message.
        
        Args:
            msg: Message to log (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm")
            log_time: Optional timestamp (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        self._log("info", msg, topic, log_time, throttle_duration_sec, skip_first, once, **context)
    
    def warning(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, throttle_duration_sec: Optional[float] = None, skip_first: bool = False, once: bool = False, **context) -> None:
        """Log warning message.
        
        Args:
            msg: Message to log (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm")
            log_time: Optional timestamp (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        self._log("warning", msg, topic, log_time, throttle_duration_sec, skip_first, once, **context)
    
    def error(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, throttle_duration_sec: Optional[float] = None, skip_first: bool = False, once: bool = False, **context) -> None:
        """Log error message.
        
        Args:
            msg: Message to log (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm")
            log_time: Optional timestamp (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        self._log("error", msg, topic, log_time, throttle_duration_sec, skip_first, once, **context)
    
    def critical(self, msg: Any, topic: Optional[str] = None, log_time: Optional[float] = None, throttle_duration_sec: Optional[float] = None, skip_first: bool = False, once: bool = False, **context) -> None:
        """Log critical message.
        
        Args:
            msg: Message to log (any type - str, dict, dataclass, etc.)
            topic: Optional topic for message routing (e.g., "/robot/arm")
            log_time: Optional timestamp (seconds since epoch)
            throttle_duration_sec: Minimum seconds between messages
            skip_first: Skip the first occurrence
            once: Only log once
            **context: Additional context key-value pairs
        """
        self._log("critical", msg, topic, log_time, throttle_duration_sec, skip_first, once, **context)
    
    #endregion - Core Logging API
    
    #region - Context Management
    
    def update_context(self, **context) -> None:
        """Update global context with new key-value pairs.
        
        Args:
            **context: Context items to add/update
        """
        self.global_context.update(context)
    
    def clear_context(self) -> None:
        """Clear all global context."""
        self.global_context.clear()
    
    def get_context(self) -> dict[str, object]:
        """Get a copy of the current global context.
        
        Returns:
            Copy of global context dict
        """
        return self.global_context.copy()
    
    #endregion - Context Management


# Module-level functions for global logger access

_root_logger: Optional[Logger] = None
_logger_registry: dict[str, Logger] = {}  # name -> Logger instance


def get_root() -> Logger:
    """Get the global root logger, creating it if needed.
    
    Returns:
        Global root Logger instance
    """
    global _root_logger
    if _root_logger is None:
        _root_logger = Logger()
    return _root_logger


def get_logger(name: str, **context) -> Logger:
    """Get a logger from the global root (no need to create root yourself).
    
    Like Python's logging.getLogger() - same name returns same instance.
    
    Args:
        name: Logger name (hierarchical with dots)
        **context: Logger context (only used on first creation)
        
    Returns:
        Child logger from global root (cached by name)
        
    Example:
        # Get loggers anywhere
        from bam_logger import get_logger
        
        logger1 = get_logger(name="arm")
        logger2 = get_logger(name="arm")  # Same instance!
        
        # Hierarchical
        arm = get_logger(name="robot.arm")
        joints = arm.get_logger(name="joints")  # "robot.arm.joints"
    """
    return get_root().get_logger(name=name, **context)


def get_logger_registry() -> dict[str, Logger]:
    """Get the global logger registry.
    
    Returns:
        Dictionary of logger name -> Logger instance
        
    Example:
        from bam_logger import get_logger_registry
        
        registry = get_logger_registry()
        for name, logger in registry.items():
            print(f"Logger: {name}")
    """
    return dict(_logger_registry)


def set_logger_level(name: Optional[str] = None, level: str = "INFO") -> None:
    """Set log level for a logger or all loggers.
    
    Args:
        name: Logger name (None = set all loggers)
        level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        
    Example:
        from bam_logger import set_logger_level
        
        # Set specific logger
        set_logger_level(name="robot.arm", level="DEBUG")
        
        # Set all loggers
        set_logger_level(level="WARNING")
    """
    if name is None:
        # Set all loggers
        for logger in _logger_registry.values():
            logger.set_logger_level(level)
        # Also set root
        get_root().set_logger_level(level)
    else:
        # Set specific logger
        if name in _logger_registry:
            _logger_registry[name].set_logger_level(level)


def configure(**kwargs) -> Logger:
    """Configure the global root logger.
    
    Call this once at application startup to set global context and backends.
    
    Args:
        py: If True, attach Python backend (default: False)
        py_level: Python backend log level (default: "INFO")
        py_file: Python backend log file (default: None)
        rclpy: If True, attach ROS 2 backend (default: False)
        node: ROS 2 node for rclpy backend
        include_call_site: Include file/line in context (default: True)
        **kwargs: Additional global context
        
    Returns:
        The configured root logger
        
    Example:
        # In main.py
        from bam_logger import configure, get_logger
        configure(py=True, robot_id="BAM-01")
        
        # In any module
        logger = get_logger(component="arm")
        logger.info("Ready")
    """
    global _root_logger
    
    # Extract backend configuration
    py = kwargs.pop('py', False)
    py_level = kwargs.pop('py_level', 'INFO')
    py_file = kwargs.pop('py_file', None)
    rclpy_enabled = kwargs.pop('rclpy', False)
    node = kwargs.pop('node', None)
    include_call_site = kwargs.pop('include_call_site', True)
    
    # Create root logger with remaining kwargs as context
    _root_logger = Logger(include_call_site=include_call_site, **kwargs)
    
    # Attach backends
    if py:
        _root_logger.get_python_backend(level=py_level, to_file=py_file)
    
    if rclpy_enabled and node is not None:
        _root_logger.get_rclpy_backend(node)
    
    return _root_logger


if __name__ == "__main__":
    # Test the global pattern
    print("Testing global logger pattern...\n")
    
    # Configure once
    configure(py=True, robot_id="BAM-01", run_id="test_123")
    
    # Get loggers anywhere (no need to pass root around)
    logger1 = get_logger(component="test1")
    logger1.info("From logger1")
    
    logger2 = get_logger(component="test2")
    logger2.info("From logger2", value=42)
    
    # Nested child
    child = logger1.get_logger(subsystem="joints")
    child.info("Nested logger")
    
    print("\n✓ Global logger pattern works!")

