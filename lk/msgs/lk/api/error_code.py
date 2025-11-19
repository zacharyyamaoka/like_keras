"""
    ErrorCode enum for standardized error handling across BAM system.
    
    ROS-free version that can be used in any Python context.
"""

# PYTHON
from enum import IntEnum


class ErrorCode(IntEnum):
    """Standard error codes for BAM API responses."""
    
    # Overall behavior
    UNDEFINED = 0
    SUCCESS = 1
    FAILURE = 2
    
    # Sensor errors
    UNABLE_TO_ACQUIRE_SENSOR_DATA = 5
    
    # Client-side errors (10-19)
    CLIENT_ERROR = 10  # Generic client error
    CLIENT_BUSY = 11  # Client is busy with another request
    CLIENT_SERVER_UNAVAILABLE = 12  # Server is unavailable
    CLIENT_REQUEST_TIMED_OUT = 13  # Request timed out
    CLIENT_REQUEST_ERROR = 14  # Error sending request
    
    # Server-side errors (20-29)
    SERVER_ERROR = 20  # Generic server error
    SERVER_UNCALIBRATED = 21  # Server not calibrated
    SERVER_PREPROCESS_ERROR = 22  # Error in preprocessing
    SERVER_BUSY = 23  # Server processing another request
    SERVER_EXCEPTION = 24  # Unhandled exception on server
    
    # Planning errors (30-39)
    PLANNING_FAILED = 30
    INVALID_MOTION_PLAN = 31
    NO_IK_SOLUTION = 32
    START_STATE_IN_COLLISION = 33
    GOAL_IN_COLLISION = 34
    TRAJECTORY_GENERATION_FAILED = 35
    
    # Kinematics errors (40-49)
    IK_FAILED = 40  # Generic IK failure
    FK_FAILED = 41  # Generic FK failure
    IK_NO_SOLUTION = 42  # No IK solution found
    IK_COLLISION = 43  # IK solution results in collision
    IK_OUT_OF_BOUNDS = 44  # Target pose outside workspace bounds
    IK_TIME_LIMIT = 45  # IK solver time limit exceeded
    IK_SINGULARITY = 46  # IK failed due to singularity
    IK_INVALID_INPUT = 47  # Invalid input to IK solver
    FK_COLLISION = 48  # FK configuration results in collision
    FK_INVALID_INPUT = 49  # Invalid input to FK solver
    
    # Trajectory generation errors (50-59)
    RUCKIG_FAILED = 50
    TOPPRA_FAILED = 51
    DYNAMICS_COMPUTATION_FAILED = 52
    
    # Execution errors (60-69)
    CONTROL_FAILED = 60
    MOTION_PLAN_INVALIDATED = 61
    TIMED_OUT = 62
    PREEMPTED = 63
    
    def to_str(self) -> str:
        """Convert error code to string name with value."""
        return f"{self.name}[{self.value}]"

