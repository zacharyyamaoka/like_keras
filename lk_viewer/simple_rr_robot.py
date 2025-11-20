#!/usr/bin/env python3
"""
    Simple RR (Revolute-Revolute) Robot Description
    
    Minimal example for testing lk_viewer reactive visualization.
"""

# BAM
from lk.msgs import Pose, Point
from lk.common.robot_description import RobotDescription

# PYTHON
from dataclasses import dataclass, field
import numpy as np


@dataclass
class SimpleRRRobot(RobotDescription):
    """Simple 2-DOF revolute-revolute robot."""
    
    # Configurable parameters
    link1_length: float = 0.5  # meters
    link2_length: float = 0.3  # meters
    
    base_radius: float = 0.1
    link_radius: float = 0.05
    
    joint1_angle: float = 0.0  # radians
    joint2_angle: float = 0.0  # radians
    
    def to_dict(self) -> dict:
        """Serialize to dict for viewer."""
        return {
            "name": "SimpleRR",
            "type": "robot",
            "links": [
                {
                    "name": "base_link",
                    "geometry": {
                        "type": "cylinder",
                        "radius": self.base_radius,
                        "height": 0.05,
                    },
                    "color": "#888888",
                    "position": [0, 0, 0.025],
                },
                {
                    "name": "link1",
                    "geometry": {
                        "type": "cylinder",
                        "radius": self.link_radius,
                        "height": self.link1_length,
                    },
                    "color": "#FF6B6B",
                    "position": [0, 0, self.link1_length / 2],
                    "parent": "joint1",
                },
                {
                    "name": "link2",
                    "geometry": {
                        "type": "cylinder",
                        "radius": self.link_radius,
                        "height": self.link2_length,
                    },
                    "color": "#4ECDC4",
                    "position": [0, 0, self.link2_length / 2],
                    "parent": "joint2",
                },
            ],
            "joints": [
                {
                    "name": "joint1",
                    "type": "revolute",
                    "parent": "base_link",
                    "child": "link1",
                    "origin": {
                        "position": [0, 0, 0.05],
                        "rotation": [0, 0, 0],
                    },
                    "axis": [0, 0, 1],
                    "angle": self.joint1_angle,
                },
                {
                    "name": "joint2",
                    "type": "revolute",
                    "parent": "link1",
                    "child": "link2",
                    "origin": {
                        "position": [0, 0, self.link1_length],
                        "rotation": [0, 0, 0],
                    },
                    "axis": [0, 0, 1],
                    "angle": self.joint2_angle,
                },
            ],
        }


if __name__ == "__main__":
    """Quick test to see the serialized output."""
    robot = SimpleRRRobot(link1_length=0.6, link2_length=0.4)
    
    import json
    print(json.dumps(robot.to_dict(), indent=2))



