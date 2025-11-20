#!/usr/bin/env python3

"""
    Diagnostic and state messages for component monitoring.
    
    Provides standardized messages for component state, diagnostics,
    and health monitoring that can be visualized in the web UI.
"""

# PYTHON
from dataclasses import dataclass, field
from typing import Dict, Optional, List, Any
from enum import Enum
from datetime import datetime

# BAM
from lk.msgs.msg import Msg


class DiagnosticLevel(Enum):
    """Diagnostic severity levels."""
    OK = "ok"
    WARN = "warn"
    ERROR = "error"
    STALE = "stale"  # Data hasn't updated recently


class ComponentState(Enum):
    """Component lifecycle state."""
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"
    ERROR = "error"


@dataclass
class DiagnosticStatus(Msg):
    """
        Diagnostic status for a component.
        
        Tracks health, errors, and warnings that can propagate
        through the system graph.
    """
    level: DiagnosticLevel = DiagnosticLevel.OK
    message: str = ""
    hardware_id: Optional[str] = None  # For URDF link association
    timestamp: float = field(default_factory=lambda: datetime.now().timestamp())
    
    # Propagation tracking
    source_component: str = ""  # Component that generated this diagnostic
    affected_components: List[str] = field(default_factory=list)  # Downstream components affected
    
    # Additional metadata
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ComponentStateMsg(Msg):
    """
        Component state information.
        
        Tracks lifecycle state, execution status, and basic metrics.
    """
    component_name: str = ""
    state: ComponentState = ComponentState.UNCONFIGURED
    is_running: bool = False
    iteration_count: int = 0
    last_update_time: float = field(default_factory=lambda: datetime.now().timestamp())
    
    # Performance metrics
    avg_execution_time_ms: float = 0.0
    max_execution_time_ms: float = 0.0
    
    # Resource usage (optional)
    cpu_percent: Optional[float] = None
    memory_mb: Optional[float] = None


@dataclass
class SystemDiagnostics(Msg):
    """
        Aggregated system-wide diagnostics.
        
        Contains all component states and diagnostics for visualization.
    """
    system_name: str = ""
    timestamp: float = field(default_factory=lambda: datetime.now().timestamp())
    
    # Component information
    component_states: Dict[str, ComponentStateMsg] = field(default_factory=dict)
    component_diagnostics: Dict[str, DiagnosticStatus] = field(default_factory=dict)
    
    # Propagation graph (which components affect which)
    diagnostic_propagation: Dict[str, List[str]] = field(default_factory=dict)
    
    # Overall system health
    system_health: DiagnosticLevel = DiagnosticLevel.OK
    critical_errors: List[str] = field(default_factory=list)  # Component names with errors

