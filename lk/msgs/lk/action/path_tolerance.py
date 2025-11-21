"""
Path and goal tolerance parameters.

Defines tolerance thresholds for path following and goal achievement.
Used in trajectory generation and execution.
"""

# PYTHON
from dataclasses import dataclass


@dataclass
class PathTolerance:
    goal_tol: float = None
    path_tol: float = None
