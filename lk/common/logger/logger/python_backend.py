#!/usr/bin/env python3

"""
Python Backend - Logging backend using Python's stdlib logging.

Wraps Python's logging module with custom formatting for context injection.
Supports file output, configurable levels, and structured context display.
"""

# PYTHON
import logging
from logging.handlers import RotatingFileHandler
from typing import Optional, Any


class ContextFormatter(logging.Formatter):
    """Custom formatter that pretty-prints context dict."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record with context appended."""
        # Get base formatted message
        base_msg = super().format(record)

        # Extract context from record (added by LoggerAdapter)
        context = getattr(record, "context", {})

        if context:
            # Format context as key=value pairs
            context_str = " | ".join(f"{k}={v}" for k, v in context.items())
            return f"{base_msg} | {context_str}"

        return base_msg


class ContextAdapter(logging.LoggerAdapter):
    """LoggerAdapter that injects context into log records."""

    def process(self, msg: str, kwargs: dict) -> tuple[str, dict]:
        """Process log call to inject context as extra fields.

        Args:
            msg: Log message
            kwargs: Keyword arguments from logging call

        Returns:
            Tuple of (message, updated kwargs with context in extra)
        """
        # Get context from kwargs, merge with adapter's extra
        context = kwargs.pop("context", {})
        merged_context = {**self.extra, **context}

        # Inject into extra field for formatter
        extra = kwargs.get("extra", {})
        extra["context"] = merged_context
        kwargs["extra"] = extra

        return msg, kwargs


class PythonBackend:
    """Logging backend using Python's stdlib logging module."""

    def __init__(
        self,
        name: str = "root",
        level: str = "INFO",
        to_file: Optional[str] = None,
        verbose: bool = False,
        fmt: str = "%(asctime)s | %(levelname)-7s | %(name)s | %(message)s",
        datefmt: str = "%H:%M:%S",
    ):
        """Initialize Python logging backend.

        Gets logger from global logging.getLogger() and configures handlers.

        Args:
            name: Logger name for logging.getLogger(name)
            level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            to_file: Optional file path for rotating file handler
            verbose: Enable verbose output
            fmt: Log format string
            datefmt: Date format string
        """
        self._name = name
        self.verbose = verbose

        # Get logger from global logging system
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))

        # Clear any existing handlers to avoid duplicates
        self.logger.handlers.clear()

        # Create formatter
        formatter = ContextFormatter(fmt=fmt, datefmt=datefmt)

        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)

        # Optional file handler
        if to_file:
            file_handler = RotatingFileHandler(
                to_file, maxBytes=5_000_000, backupCount=3  # 5MB
            )
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

        if verbose:
            print(
                f"[PythonBackend] Initialized: name={name}, level={level}, file={to_file}"
            )

    @property
    def name(self) -> str:
        """Backend name."""
        return f"python_{self._name}"

    def _format_msg(self, msg: Any, topic: Optional[str] = None, **context) -> str:
        """Convert any message type to string for Python logging.

        Args:
            msg: Message of any type (str, dict, dataclass, etc.)
            topic: Optional topic for the message
            **context: Additional context

        Returns:
            String representation of the message
        """
        # Add topic to context if provided
        if topic:
            context["topic"] = topic

        # Handle string messages (most common case)
        if isinstance(msg, str):
            return msg

        # Handle dict messages
        if isinstance(msg, dict):
            return str(msg)

        # Handle dataclass messages (BAM/ROS messages)
        if hasattr(msg, "__dataclass_fields__"):
            return f"{msg.__class__.__name__}: {msg}"

        # Default: convert to string
        return str(msg)

    def debug(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log debug message with context."""
        formatted_msg = self._format_msg(msg, topic, **context)
        self.logger.debug(formatted_msg, extra={"context": context})

    def info(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log info message with context."""
        formatted_msg = self._format_msg(msg, topic, **context)
        self.logger.info(formatted_msg, extra={"context": context})

    def warning(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log warning message with context."""
        formatted_msg = self._format_msg(msg, topic, **context)
        self.logger.warning(formatted_msg, extra={"context": context})

    def error(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log error message with context."""
        formatted_msg = self._format_msg(msg, topic, **context)
        self.logger.error(formatted_msg, extra={"context": context})

    def critical(
        self,
        msg: Any,
        topic: Optional[str] = None,
        log_time: Optional[float] = None,
        **context,
    ) -> None:
        """Log critical message with context."""
        formatted_msg = self._format_msg(msg, topic, **context)
        self.logger.critical(formatted_msg, extra={"context": context})


if __name__ == "__main__":
    # Test the backend directly
    backend = PythonBackend(name="test", level="DEBUG", verbose=True)

    backend.info("Test message", run_id="test_123", component="main")
    backend.debug("Debug message", value=42, status="ok")
    backend.warning("Warning message", latency_ms=150)
    backend.error("Error message", error_code="TIMEOUT", retry=3)
    backend.critical("Critical message", emergency=True)
