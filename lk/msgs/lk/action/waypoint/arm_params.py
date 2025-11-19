"""
    Per-waypoint arm motion parameters.
    
    Contains motion control parameters for a single waypoint, including
    optional per-waypoint kinematic limits that override global context limits.
"""

# BAM
from bam.msgs.ros_msgs import Vector3
from bam.msgs.msg_utils import lerp_list, lerp_value
from ..joint_impedance import JointImpedance
from ..joint_limits import JointLimits
from ..path_params import PathParams
from ..path_tolerance import PathTolerance

# PYTHON
from dataclasses import dataclass, field

import copy
import random

@dataclass
class ArmParams:    

    call_function: str = "" #close_hand, open_hand, will call getattr(self, run_action)
    function_blocking_sec: float = 0.0 # by default 0 and non blocking, if not zero, then will block until timeout or action complete.
    #TODO make it a list so you potetailly call multiple functions...
    #TODO potetially add sync_vector and velocity?
    sleep_sec: float = 0.0 # sleep for this many seconds after the waypoint is reached.

    sync_dir: Vector3 = field(default_factory=Vector3) # normalized direction vector for moving target synchronization
    sync_speed: float = 0.0 # speed (m/s) for moving target synchronization

    # Inspired by Delta X, setting the start and inital velocities
    # https://docs.deltaxrobot.com/reference/gcodes/gc_xs_v5/
    # Instead of defining segments, these velocities, or leave it open...
    # Direction can be taken from the segment direction. You cannot define the velocity for two segments though as not tangent.
    start_vel: float = None
    travel_vel: float = None
    end_vel: float = None


    # MOTEUS #TODO POTETIALLY APPLY STIFFNESS IN DIFFERENT DIRECTIONS...
    joint_impedance: JointImpedance = field(default_factory=JointImpedance)
    max_vel: float = None
    max_effort: float = None
    vel_scale: float = 0.0
    accel_scale: float = 0.0

    vertical_angle_scale: float = 0.0 # 0 (Approach/Retreat completely along grasp z_axis) to 1 (Approach/Retreat offsets should be completely vertical to gravity/table surface)
    duration: float = 0.0 # seconds

    # MOVEIT
    joint_angles: list[float] = field(default_factory=list)

    # Optional per-waypoint joint limits (overrides WaypointContext arm_limits for this waypoint)
    # Order of joints should correspond to arm_ik_sol joint order
    joint_limits: JointLimits = field(default_factory=JointLimits)  # Use typing string to avoid circular import

    # PATH GENERATION (interpolation & blending)
    path: PathParams = field(default_factory=PathParams)

    # TOLERANCE
    tol: PathTolerance = field(default_factory=PathTolerance)


    target_link: str = "" # TCP, IK_TIP, etc. what is the target link?
    planner: str = "" # avaliable are "ptp", "lin"


    def lerp(self, target: 'ArmParams', fraction: float) -> 'ArmParams':

        new_params = copy.deepcopy(self)
        
        # Interpolate kp_scale and kd_scale per joint using lerp_list helper
        new_params.joint_impedance.kp_scale = lerp_list(
            self.joint_impedance.kp_scale, 
            target.joint_impedance.kp_scale, 
            fraction
        )
        new_params.joint_impedance.kd_scale = lerp_list(
            self.joint_impedance.kd_scale, 
            target.joint_impedance.kd_scale, 
            fraction
        )

        # NOTE: Lots of ways what is just simple and straight forward? 
        # new_params.n_intermediate_points = target.n_intermediate_points
        # new_params.lerp_fraction = fraction
    
        # if you blend, you should still be able to interpolate between the new start/end points
        # new_params.n_intermediate_points = int(lerp_value(self.n_intermediate_points, target.n_intermediate_points, fraction) + 0.5)  # Round: <0.5 down, >=0.5 up
        # BUG: Don't account for the current n_intermediate_points. The goal is just to create a new point with fewer than the target_n_intermediate_points as your moving towards it...
        # new_params.n_intermediate_points = int(lerp_value(0, target.n_intermediate_points, fraction) + 0.5)  # Round: <0.5 down, >=0.5 up
        return new_params

    def difference(self, target: 'ArmParams') -> dict[str, float]:
        diff = {}
        
        # Compute average difference across all joints
        if self.joint_impedance.kp_scale and target.joint_impedance.kp_scale:
            n_joints = min(len(self.joint_impedance.kp_scale), len(target.joint_impedance.kp_scale))
            kp_diffs = [abs(self.joint_impedance.kp_scale[i] - target.joint_impedance.kp_scale[i]) for i in range(n_joints)]
            diff['kp_scale'] = sum(kp_diffs) / len(kp_diffs) if kp_diffs else 0.0
        else:
            diff['kp_scale'] = 0.0
            
        if self.joint_impedance.kd_scale and target.joint_impedance.kd_scale:
            n_joints = min(len(self.joint_impedance.kd_scale), len(target.joint_impedance.kd_scale))
            kd_diffs = [abs(self.joint_impedance.kd_scale[i] - target.joint_impedance.kd_scale[i]) for i in range(n_joints)]
            diff['kd_scale'] = sum(kd_diffs) / len(kd_diffs) if kd_diffs else 0.0
        else:
            diff['kd_scale'] = 0.0
            
        return diff

    @classmethod
    def make_random(cls, n_joints: int = 6) -> 'ArmParams':
        arm_params = cls()
        arm_params.joint_impedance.kp_scale = [random.uniform(0.0, 1.0) for _ in range(n_joints)]
        arm_params.joint_impedance.kd_scale = [random.uniform(0.0, 1.0) for _ in range(n_joints)]
        return arm_params