#!/usr/bin/env python3

"""
Node system for component execution contexts.

Nodes manage how components execute - single-threaded, multi-threaded, etc.
Components belong to nodes, and nodes coordinate their execution.
"""

# BAM
from lk.common.component import Component

# PYTHON
from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import threading
import queue


class ExecutionMode(Enum):
    """Execution modes for nodes."""

    SEQUENTIAL = "sequential"  # Single-threaded, sequential execution
    THREADED = "threaded"  # Multi-threaded execution
    ASYNC = "async"  # Async/await execution (future)


@dataclass
class NodeConfig:
    """
    Configuration for node execution.
    """

    execution_mode: ExecutionMode = ExecutionMode.SEQUENTIAL
    thread_priority: int = 0
    cpu_affinity: Optional[List[int]] = None

    # For threaded mode
    max_queue_size: int = 100

    # General settings
    rate_hz: Optional[float] = None  # Target execution rate
    name: str = "default_node"


class Node:
    """
    Execution context for components.

    Nodes manage component execution - single-threaded loops,
    multi-threaded message passing, etc.

    Can optionally store component configurations for convenient setup:

    node = Node(
        name="my_node",
        component_configs={
            "agent": Agent.Config(...),
            "env": Env.Config(...)
        }
    )
    """

    def __init__(
        self,
        name: str,
        config: Optional[NodeConfig] = None,
        component_configs: Optional[Dict[str, Any]] = None,
    ):
        """
        Initialize node.

        Args:
            name: Node identifier
            config: Node configuration
            component_configs: Dict mapping component IDs to their configs
        """
        self.name = name
        self.config = config or NodeConfig(name=name)
        self.component_configs = component_configs or {}
        self._components: List[Component] = []

        # Threading support
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._message_queue: queue.Queue = queue.Queue(
            maxsize=self.config.max_queue_size
        )

    def get_component_config(self, component_id: str) -> Optional[Any]:
        """
        Get config for a component by ID.

        Args:
            component_id: Component identifier

        Returns:
            Component config if found, None otherwise
        """
        return self.component_configs.get(component_id)

    def add_component(self, component: Component):
        """
        Add a component to this node.

        Args:
            component: Component to add
        """
        if component not in self._components:
            self._components.append(component)
            component.node = self

    def remove_component(self, component: Component):
        """
        Remove a component from this node.

        Args:
            component: Component to remove
        """
        if component in self._components:
            self._components.remove(component)
            component.node = None

    @property
    def components(self) -> List[Component]:
        """Get all components in this node."""
        return self._components.copy()

    def start(self):
        """
        Start node execution.

        Behavior depends on execution mode:
        - SEQUENTIAL: Does nothing (execution managed externally)
        - THREADED: Starts background thread
        - ASYNC: Sets up async event loop
        """
        if self.config.execution_mode == ExecutionMode.THREADED:
            if self._thread is None or not self._thread.is_alive():
                self._running = True
                self._thread = threading.Thread(
                    target=self._run_threaded, name=f"node_{self.name}", daemon=True
                )
                self._thread.start()
        elif self.config.execution_mode == ExecutionMode.SEQUENTIAL:
            self._running = True

    def stop(self):
        """Stop node execution."""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)

    def _run_threaded(self):
        """
        Background thread execution loop.

        Processes messages from queue and executes components.
        """
        import time

        rate_limiter = None
        if self.config.rate_hz:
            dt = 1.0 / self.config.rate_hz
            rate_limiter = lambda: time.sleep(dt)

        while self._running:
            try:
                # Process messages from queue
                try:
                    msg = self._message_queue.get(timeout=0.1)
                    # Handle message (to be implemented based on system needs)
                except queue.Empty:
                    pass

                # Rate limiting
                if rate_limiter:
                    rate_limiter()

            except Exception as e:
                print(f"Error in node {self.name}: {e}")
                break

    def execute_sequential(self):
        """
        Execute one iteration in sequential mode.

        This is called externally by the System in a loop.
        """
        # Sequential execution handled by System
        # Individual component execution depends on the execution graph
        pass

    def __repr__(self) -> str:
        return f"Node(name={self.name}, mode={self.config.execution_mode.value}, components={len(self._components)})"
