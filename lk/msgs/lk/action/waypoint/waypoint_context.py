"""
Waypoint context with both Cartesian and joint-space constraints.

Supports multiple planning paradigms:
- Cartesian-space: Use cartesian_limits and cartesian_impedance
- Joint-space: Use arm_limits and hand_limits
- Hybrid: Use both (planner chooses appropriate constraints)
"""

# BAM
from ..cartesian_limits import CartesianLimits
from ..cartesian_impedance import CartesianImpedance
from ..joint_limits import JointLimits

# PYTHON
from dataclasses import dataclass, field


@dataclass
class WaypointContext:
    """Context information for waypoint trajectory execution."""

    sync_target_idx: int = 0
    target_link: str = "tcp_world"
    world_frame: str = "world"

    # Cartesian space motion constraints (for task-space planners)
    cartesian_limits: CartesianLimits | None = field(default_factory=CartesianLimits)

    # Cartesian impedance control parameters (optional, for compliant motion)
    cartesian_impedance: CartesianImpedance | None = None

    # Joint space motion constraints (for joint-space planners)
    arm_limits: JointLimits | None = None
    hand_limits: JointLimits | None = None
