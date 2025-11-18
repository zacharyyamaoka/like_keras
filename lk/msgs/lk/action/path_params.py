"""
    Cartesian path generation parameters for trajectories.
    
    Controls both interpolation (adding intermediate points on straight segments)
    and blending (circular filleting at corners) for smooth trajectory generation.
"""

# PYTHON
from dataclasses import dataclass

@dataclass
class PathParams:
    """
        Parameters controlling cartesian path generation between waypoints.
        
        Includes:
        - Interpolation: Adding intermediate points along straight segments
        - Blending: Circular filleting at corners to avoid sharp direction changes
        
        Pilz planner uses joint-space blending, but this implementation uses
        pure cartesian space blending which is more intuitive for shaping motion.
        Ruckig can be used for optimal joint-space smoothing separately.
    """
    
    # Interpolation parameters
    n_intermediate_points: int = 0  # Number of points to add between waypoints (straight line)
    
    # Blending parameters (user-specified)
    blend_radius: float = 0.0  # Fillet radius in meters (must be 0 on final waypoint)
    n_intermediate_blend_points: int = 0  # Number of points along the arc (excluding tangent points)
    
    # Flags set during path generation
    start_blend_point: bool = False  # True if this is the start tangent point of a blend
    end_blend_point: bool = False    # True if this is the end tangent point of a blend
    blend_distance: float = 0.0      # Distance from corner to this point (for interpolation offset)
    
    # Interpolation offset for n_intermediate_points
    # Most params can be lerped directly, but n_intermediate_points requires
    # tracking this offset more explicitly to handle blended segments correctly
    blend_lerp_offset: float = 0.0  # Fraction of n_intermediate_points to offset

