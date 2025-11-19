#!/usr/bin/env python3

"""
    MoteusActuatorTimeline - Interactive timeline visualization for MoteusActuator(s)
    
    Displays actuator timelines with:
    - Actual position (ground truth)
    - Measured position (encoder reading)
    - Velocity
    - Commands
    - Hardstop limits
    
    Supports interactive scrubbing, animation, and hover details.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class ActuatorTimelineData:
    """
    Timeline data for a single actuator
    
    Attributes:
        name: Actuator/joint name
        times: Timestamps for each recorded point
        actual_positions: Actual position (ground truth) at each time
        measured_positions: Measured position (encoder) at each time
        velocities: Velocity at each time
        commands: Command names/descriptions at each time
        min_pos: Minimum position limit (hardstop)
        max_pos: Maximum position limit (hardstop)
    """
    name: str
    times: list[float]
    actual_positions: list[float]
    measured_positions: list[float]
    velocities: list[float]
    commands: list[str]
    min_pos: float
    max_pos: float
    
    def __post_init__(self):
        """Validate that all arrays have same length"""
        n = len(self.times)
        assert len(self.actual_positions) == n, "actual_positions length mismatch"
        assert len(self.measured_positions) == n, "measured_positions length mismatch"
        assert len(self.velocities) == n, "velocities length mismatch"
        assert len(self.commands) == n, "commands length mismatch"


@dataclass
class MoteusActuatorTimeline(VisualObject):
    """
    Visual object for interactive MoteusActuator timeline visualization.
    
    Displays position, velocity, and command timelines for one or more actuators.
    Supports interactive scrubbing with slider and play/pause animation.
    
    Attributes:
        actuators: List of actuator timeline data
        title: Optional title for the visualization
        height_per_actuator: Height in pixels for each actuator row (default: 250)
        show_legend: Whether to show legend (default: True)
        show_hardstops: Whether to show hardstop limit lines (default: True)
        colormap: Color scheme for plots (default: 'default')
    """
    actuators: list[ActuatorTimelineData] = field(default_factory=list)
    title: Optional[str] = None
    height_per_actuator: int = 250
    show_legend: bool = True
    show_hardstops: bool = True
    colormap: str = "default"
    marker_text_angle: float = 45.0  # Angle for command marker labels (degrees)
    
    @property
    def n_actuators(self) -> int:
        """Number of actuators in timeline"""
        return len(self.actuators)
    
    @property
    def max_time(self) -> float:
        """Maximum time across all actuators"""
        if not self.actuators:
            return 0.0
        return max(max(act.times) if act.times else 0.0 for act in self.actuators)
    
    @property
    def total_height(self) -> int:
        """Total height of visualization"""
        return self.height_per_actuator * self.n_actuators
    
    def validate(self) -> bool:
        """Validate timeline data"""
        if not self.actuators:
            return False
        
        for actuator in self.actuators:
            if not actuator.times:
                return False
            if actuator.min_pos >= actuator.max_pos:
                raise ValueError(f"Actuator {actuator.name}: min_pos must be < max_pos")
        
        return True
    
    @classmethod
    def from_mock_actuator_group(cls, mock_group, name: str = "actuator_timeline", title: Optional[str] = None):
        """
        Create timeline visualization from MockMoteusActuatorGroup
        
        Args:
            mock_group: MockMoteusActuatorGroup instance with recorded timeline
            name: Name for the visual object
            title: Optional title for visualization
            
        Returns:
            MoteusActuatorTimeline visual object
        """
        actuators = []
        
        for mock_actuator in mock_group.actuators:
            if not mock_actuator.timeline:
                continue
            
            times = []
            actual_positions = []
            measured_positions = []
            velocities = []
            commands = []
            
            for record in mock_actuator.timeline:
                times.append(record.relative_time)
                actual_positions.append(record.actual_position)
                measured_positions.append(record.measured_position)
                velocities.append(record.velocity)
                
                # Format command string
                cmd = record.command
                cmd_str = cmd.__class__.__name__
                if hasattr(cmd, 'data'):
                    if hasattr(cmd.data, 'velocity'):
                        cmd_str = f"{cmd_str}(v={cmd.data.velocity:.2f})"
                commands.append(cmd_str)
            
            actuators.append(ActuatorTimelineData(
                name=mock_actuator.name,
                times=times,
                actual_positions=actual_positions,
                measured_positions=measured_positions,
                velocities=velocities,
                commands=commands,
                min_pos=mock_actuator.min_pos,
                max_pos=mock_actuator.max_pos
            ))
        
        return cls(
            name=name,
            actuators=actuators,
            title=title or "Moteus Actuator Timeline"
        )
    
    @classmethod
    def from_mock_actuator(cls, mock_actuator, name: str = "actuator_timeline", title: Optional[str] = None):
        """
        Create timeline visualization from single MockMoteusActuator
        
        Args:
            mock_actuator: MockMoteusActuator instance with recorded timeline
            name: Name for the visual object
            title: Optional title for visualization
            
        Returns:
            MoteusActuatorTimeline visual object
        """
        if not mock_actuator.timeline:
            raise ValueError("Actuator has no timeline data")
        
        times = []
        actual_positions = []
        measured_positions = []
        velocities = []
        commands = []
        
        for record in mock_actuator.timeline:
            times.append(record.relative_time)
            actual_positions.append(record.actual_position)
            measured_positions.append(record.measured_position)
            velocities.append(record.velocity)
            
            # Format command string
            cmd = record.command
            cmd_str = cmd.__class__.__name__
            if hasattr(cmd, 'data'):
                if hasattr(cmd.data, 'velocity'):
                    cmd_str = f"{cmd_str}(v={cmd.data.velocity:.2f})"
            commands.append(cmd_str)
        
        actuator_data = ActuatorTimelineData(
            name=mock_actuator.name,
            times=times,
            actual_positions=actual_positions,
            measured_positions=measured_positions,
            velocities=velocities,
            commands=commands,
            min_pos=mock_actuator.min_pos,
            max_pos=mock_actuator.max_pos
        )
        
        return cls(
            name=name,
            actuators=[actuator_data],
            title=title or f"{mock_actuator.name} Timeline"
        )

