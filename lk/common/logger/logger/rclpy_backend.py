#!/usr/bin/env python3

"""
    ROS2 RclPy Backend - Logging backend using rclpy's logger (STUB).
    
    TODO: Full implementation to integrate with ROS2 logging system.
    Will format messages with context and publish to /rosout topic.
"""

# PYTHON
from typing import Any


class RclpyBackend:
    """Logging backend using ROS2 rclpy (stub implementation)."""
    
    def __init__(self, node: Any, name: str = "rclpy"):
        """Initialize rclpy logging backend.
        
        Args:
            node: ROS2 node instance (should have get_logger() method)
            name: Backend name
        """
        self._name = name
        self.node = node
        
        # TODO: Get node's logger
        # self.rclpy_logger = node.get_logger()
        
        print(f"[RclpyBackend] STUB: Initialized with node (not yet functional)")
    
    @property
    def name(self) -> str:
        """Backend name."""
        return self._name
    
    def _format_msg(self, msg: Any, topic: str | None = None, context: dict[str, Any] | None = None) -> str:
        """Format message with context for rclpy output.
        
        Args:
            msg: Log message (any type)
            topic: Optional topic for the message
            context: Context dict
            
        Returns:
            Formatted message string
        """
        # Add topic to context if provided
        if context is None:
            context = {}
        if topic:
            context['topic'] = topic
        
        # Convert message to string
        if isinstance(msg, str):
            msg_str = msg
        elif hasattr(msg, '__dataclass_fields__'):
            msg_str = f"{msg.__class__.__name__}: {msg}"
        else:
            msg_str = str(msg)
        
        # Add context
        if context:
            context_str = " | ".join(f"{k}={v}" for k, v in context.items())
            return f"{msg_str} | {context_str}"
        return msg_str
    
    def debug(self, msg: Any, topic: str | None = None, log_time: float | None = None, **context) -> None:
        """Log debug message with context."""
        # TODO: formatted_msg = self._format_msg(msg, topic, context)
        # TODO: self.rclpy_logger.debug(formatted_msg)
        pass
    
    def info(self, msg: Any, topic: str | None = None, log_time: float | None = None, **context) -> None:
        """Log info message with context."""
        # TODO: formatted_msg = self._format_msg(msg, topic, context)
        # TODO: self.rclpy_logger.info(formatted_msg)
        pass
    
    def warning(self, msg: Any, topic: str | None = None, log_time: float | None = None, **context) -> None:
        """Log warning message with context."""
        # TODO: formatted_msg = self._format_msg(msg, topic, context)
        # TODO: self.rclpy_logger.warning(formatted_msg)
        pass
    
    def error(self, msg: Any, topic: str | None = None, log_time: float | None = None, **context) -> None:
        """Log error message with context."""
        # TODO: formatted_msg = self._format_msg(msg, topic, context)
        # TODO: self.rclpy_logger.error(formatted_msg)
        pass
    
    def critical(self, msg: Any, topic: str | None = None, log_time: float | None = None, **context) -> None:
        """Log critical message with context."""
        # TODO: formatted_msg = self._format_msg(msg, topic, context)
        # TODO: self.rclpy_logger.fatal(formatted_msg)
        pass


if __name__ == "__main__":
    # Minimal test with mock node
    class MockNode:
        pass
    
    node = MockNode()
    backend = RclpyBackend(node=node)
    
    print(f"Backend name: {backend.name}")
    print("Note: This is a stub implementation - not yet functional")

