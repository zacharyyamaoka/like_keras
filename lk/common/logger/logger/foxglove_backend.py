#!/usr/bin/env python3

"""
Foxglove Backend - Logging backend using Foxglove SDK.

Routes messages to Foxglove for live visualization and MCAP recording.
Supports Foxglove schemas, BAM/ROS messages, and plain strings.

Features:
- Live visualization via Foxglove WebSocket server
- MCAP file recording for later playback
- Automatic type conversion from BAM/ROS messages to Foxglove schemas
- Topic-based routing
"""

# PYTHON
import time
from typing import Any, Optional

try:
    import foxglove
    from foxglove.schemas import Log, LogLevel, Timestamp

    FOXGLOVE_AVAILABLE = True
except ImportError:
    FOXGLOVE_AVAILABLE = False
    Log = None
    LogLevel = None
    Timestamp = None


class FoxgloveBackend:
    """Logging backend using Foxglove SDK.

    Routes messages to Foxglove server and/or MCAP file.
    Handles type conversion from various message formats.
    """

    def __init__(
        self,
        name: str = "foxglove",
        server_enabled: bool = True,
        mcap_file: Optional[str] = None,
        server_port: int = 8765,
        server_host: str = "127.0.0.1",
        verbose: bool = False,
    ):
        """Initialize Foxglove logging backend.

        Args:
            name: Backend name
            server_enabled: Enable Foxglove WebSocket server for live visualization
            mcap_file: Optional MCAP file path for recording
            server_port: WebSocket server port
            server_host: WebSocket server host
            verbose: Enable verbose output
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError(
                "Foxglove SDK not available. Install with: pip install foxglove-sdk"
            )

        self._name = name
        self.verbose = verbose
        self.server_enabled = server_enabled
        self.mcap_file = mcap_file

        # Start Foxglove server if enabled
        self.server = None
        if server_enabled:
            try:
                self.server = foxglove.start_server(
                    port=server_port, host=server_host, name=f"bam_logger_{name}"
                )
                if verbose:
                    print(
                        f"[FoxgloveBackend] Server started on {server_host}:{server_port}"
                    )
            except Exception as e:
                if verbose:
                    print(f"[FoxgloveBackend] Failed to start server: {e}")
                self.server = None

        # Open MCAP file if specified
        self.mcap_writer = None
        if mcap_file:
            try:
                self.mcap_writer = foxglove.open_mcap(mcap_file)
                if verbose:
                    print(f"[FoxgloveBackend] Recording to {mcap_file}")
            except Exception as e:
                if verbose:
                    print(f"[FoxgloveBackend] Failed to open MCAP file: {e}")
                self.mcap_writer = None

        if verbose:
            print(f"[FoxgloveBackend] Initialized: name={name}")

    @property
    def name(self) -> str:
        """Backend name."""
        return f"foxglove_{self._name}"

    def _convert_to_foxglove_log(
        self, msg: Any, level: str, log_time: Optional[float] = None
    ) -> Log:
        """Convert message to Foxglove Log schema.

        Args:
            msg: Message to convert
            level: Log level string (debug, info, warning, error, critical)
            log_time: Optional timestamp (seconds since epoch)

        Returns:
            Foxglove Log message
        """
        # Map log levels to Foxglove LogLevel enum
        level_map = {
            "debug": LogLevel.Debug,
            "info": LogLevel.Info,
            "warning": LogLevel.Warning,
            "error": LogLevel.Error,
            "critical": LogLevel.Fatal,
        }
        foxglove_level = level_map.get(level.lower(), LogLevel.Info)

        # Convert message to string
        if isinstance(msg, str):
            msg_str = msg
        elif hasattr(msg, "__dataclass_fields__"):
            msg_str = f"{msg.__class__.__name__}: {msg}"
        else:
            msg_str = str(msg)

        # Create timestamp
        if log_time is not None:
            timestamp = Timestamp.from_secs(log_time)
        else:
            timestamp = Timestamp.now()

        return Log(
            timestamp=timestamp,
            level=foxglove_level,
            message=msg_str,
        )

    def _log_to_foxglove(
        self,
        msg: Any,
        level: str,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Internal method to log message to Foxglove.

        Args:
            msg: Message to log
            level: Log level
            topic: Optional topic for routing
            log_time: Optional timestamp
            **context: Additional context (added to log message)
        """
        # Use default topic if not provided
        if topic is None:
            topic = "/log"

        # If message is already a Foxglove schema, log it directly
        if hasattr(msg, "__class__") and hasattr(msg.__class__, "__module__"):
            if "foxglove.schemas" in msg.__class__.__module__:
                try:
                    foxglove.log(topic, msg)
                    return
                except Exception as e:
                    if self.verbose:
                        print(f"[FoxgloveBackend] Error logging Foxglove schema: {e}")
                    return

        # Convert to Foxglove Log schema
        log_msg = self._convert_to_foxglove_log(msg, level, log_time)

        # Add context to message if provided
        if context:
            context_str = " | ".join(f"{k}={v}" for k, v in context.items())
            log_msg.message = f"{log_msg.message} | {context_str}"

        # Log to Foxglove
        try:
            foxglove.log(topic, log_msg)
        except Exception as e:
            if self.verbose:
                print(f"[FoxgloveBackend] Error logging message: {e}")

    def debug(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log debug message."""
        self._log_to_foxglove(msg, "debug", topic, log_time, **context)

    def info(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log info message."""
        self._log_to_foxglove(msg, "info", topic, log_time, **context)

    def warning(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log warning message."""
        self._log_to_foxglove(msg, "warning", topic, log_time, **context)

    def error(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log error message."""
        self._log_to_foxglove(msg, "error", topic, log_time, **context)

    def critical(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log critical message."""
        self._log_to_foxglove(msg, "critical", topic, log_time, **context)

    def close(self) -> None:
        """Close the backend and cleanup resources."""
        if self.mcap_writer:
            try:
                self.mcap_writer.close()
                if self.verbose:
                    print(f"[FoxgloveBackend] Closed MCAP file")
            except Exception as e:
                if self.verbose:
                    print(f"[FoxgloveBackend] Error closing MCAP: {e}")

        if self.server:
            try:
                self.server.stop()
                if self.verbose:
                    print(f"[FoxgloveBackend] Stopped server")
            except Exception as e:
                if self.verbose:
                    print(f"[FoxgloveBackend] Error stopping server: {e}")


if __name__ == "__main__":
    # Test the backend
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Install with: pip install foxglove-sdk")
        exit(1)

    print("Testing FoxgloveBackend...")
    print("Open Foxglove app and connect to ws://localhost:8765")

    backend = FoxgloveBackend(
        name="test",
        server_enabled=True,
        mcap_file="/tmp/test_bam_logger.mcap",
        verbose=True,
    )

    # Test different message types
    backend.info("Simple string message", topic="/test/string")
    backend.debug("Debug message", run_id="test_123", component="main")
    backend.warning("Warning with context", latency_ms=150, status="degraded")
    backend.error("Error message", error_code="TIMEOUT", retry=3)

    # Test with Foxglove schema directly
    from foxglove.schemas import Log, LogLevel, Timestamp

    log_msg = Log(
        timestamp=Timestamp.now(),
        level=LogLevel.Info,
        message="Direct Foxglove Log schema",
    )
    backend.info(log_msg, topic="/test/foxglove_log")

    # Keep running for a bit to see messages in Foxglove
    print("\nLogging messages for 5 seconds...")
    for i in range(5):
        backend.info(f"Periodic message {i}", topic="/test/periodic", count=i)
        time.sleep(1)

    print("\nClosing backend...")
    backend.close()
    print("✓ Test complete!")
