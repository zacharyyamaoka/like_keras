#!/usr/bin/env python3

"""
    Mixin to add default state and diagnostics ports to components.
    
    All components automatically get state and diagnostics output ports
    that can be connected to monitoring systems like the web server.
"""

# BAM
from lk.common.component import Component
from lk.common.port import OutputPort
from lk.msgs.diagnostics import ComponentStateMsg, DiagnosticStatus, ComponentState, DiagnosticLevel
from lk.common.component import LifecycleState

# PYTHON
from typing import Optional
import time


class DiagnosticsMixin:
    """
        Mixin that adds state and diagnostics ports to components.
        
        Components using this mixin automatically get:
        - state_port: Outputs ComponentStateMsg
        - diagnostics_port: Outputs DiagnosticStatus
        
        These ports are auto-connected to the web server by default.
    """
    
    def __init__(self, *args, **kwargs):
        """Initialize diagnostics ports."""
        super().__init__(*args, **kwargs)
        
        # Create default state and diagnostics ports
        self.state_port = OutputPort(
            name="state",
            msg_type=ComponentStateMsg,
            owner=self
        )
        self.diagnostics_port = OutputPort(
            name="diagnostics",
            msg_type=DiagnosticStatus,
            owner=self
        )
        
        # Track execution metrics
        self._execution_times: list[float] = []
        self._iteration_count = 0
        self._last_update_time = time.time()
        
        # Initialize state message
        self._update_state()
        self._update_diagnostics()
    
    def _update_state(self):
        """Update component state message."""
        state = ComponentState.UNCONFIGURED
        
        # Map lifecycle state if component has it
        if hasattr(self, 'lifecycle_state'):
            lifecycle_map = {
                LifecycleState.UNCONFIGURED: ComponentState.UNCONFIGURED,
                LifecycleState.INACTIVE: ComponentState.INACTIVE,
                LifecycleState.ACTIVE: ComponentState.ACTIVE,
                LifecycleState.FINALIZED: ComponentState.FINALIZED,
            }
            state = lifecycle_map.get(self.lifecycle_state, ComponentState.UNCONFIGURED)
        
        # Calculate average execution time
        avg_time_ms = 0.0
        max_time_ms = 0.0
        if self._execution_times:
            avg_time_ms = sum(self._execution_times) / len(self._execution_times) * 1000
            max_time_ms = max(self._execution_times) * 1000
        
        state_msg = ComponentStateMsg(
            component_name=self.name,
            state=state,
            is_running=state == ComponentState.ACTIVE,
            iteration_count=self._iteration_count,
            last_update_time=time.time(),
            avg_execution_time_ms=avg_time_ms,
            max_execution_time_ms=max_time_ms
        )
        
        self.state_port.write(state_msg)
        self._last_update_time = time.time()
    
    def _update_diagnostics(self, level: DiagnosticLevel = DiagnosticLevel.OK, 
                          message: str = "", hardware_id: Optional[str] = None):
        """Update diagnostic status."""
        diag_msg = DiagnosticStatus(
            level=level,
            message=message,
            hardware_id=hardware_id,
            timestamp=time.time(),
            source_component=self.name
        )
        self.diagnostics_port.write(diag_msg)
    
    def _record_execution_time(self, duration: float):
        """Record execution time for metrics."""
        self._execution_times.append(duration)
        # Keep only last 100 measurements
        if len(self._execution_times) > 100:
            self._execution_times.pop(0)
        self._iteration_count += 1
        self._update_state()
    
    def set_diagnostic_error(self, message: str, hardware_id: Optional[str] = None):
        """Set diagnostic to error level."""
        self._update_diagnostics(
            level=DiagnosticLevel.ERROR,
            message=message,
            hardware_id=hardware_id
        )
    
    def set_diagnostic_warn(self, message: str, hardware_id: Optional[str] = None):
        """Set diagnostic to warning level."""
        self._update_diagnostics(
            level=DiagnosticLevel.WARN,
            message=message,
            hardware_id=hardware_id
        )
    
    def set_diagnostic_ok(self, message: str = ""):
        """Set diagnostic to OK level."""
        self._update_diagnostics(
            level=DiagnosticLevel.OK,
            message=message
        )


class DiagnosticsComponent(DiagnosticsMixin, Component):
    """
        Component with built-in diagnostics support.
        
        Convenience class that combines Component with DiagnosticsMixin.
        Use this instead of Component to get automatic state/diagnostics ports.
    """
    pass

