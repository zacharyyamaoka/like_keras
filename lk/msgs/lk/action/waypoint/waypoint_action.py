from bam.msgs.ros_msgs import JointState
from bam.msgs.ros_msgs import PoseStamped
from .waypoint_params import WaypointParams
from .waypoint_context import WaypointContext
from ..joint_limits import JointLimits
from ..mdp_action import MdpAction
import numpy as np

from dataclasses import dataclass, field
from typing import Optional

"""
Q: Should the waypoints have the current pose? 

A: Yes totally. Ref: Moveit, you always plan from current pose, Ruckig as well.

How to deal with fact that current pose is in joint space? What if you have intial velocity etc at the starting state?

To date I had assumed I would just have a unified approach, and just add it as a waypoint. I do now think it would be better to have a JointState or similar as the starting state...

https://docs.ros.org/en/noetic/api/moveit_msgs/html/msg/MotionPlanRequest.html

In my moveit_client, I had:

def get_motion_sequence_request(self, start_state: JointState, goal_list: List[TransformStamped], params_list: List[WaypointParams]) -> MotionSequenceRequest:

def get_motion_plan_request(self, start_state: JointState, goal: Union[TransformStamped, JointState], params: WaypointParams, stamp=None) -> MotionPlanRequest:

v1: I just passed in the start_state JointState and no TransformStamped. This matches moveit, and while clean, its frusterating that I cannot do path
planning without calling the IK function...

v2: Make TransformStamped a first class citizen, joint state is now just an additional optional struct to add info regarding starting velocity, and acceleration.

v3: Ok I decided I want to go back to pose stamped. but in the waypoint context I need to add a child_frame_id. to avoid the crazy house I think, all points
should be for the same frame ID no?


It makes sense that this holds cartesian and joint information.

Ultimately we will ne to turn the cartesian into joint info at each waypoint for ruckig. you don't put it in intially, then we will need to calculate it after!
I like how I can collect all the information in this one data struct, the cartesian and joint info
"""

@dataclass
class WaypointAction(MdpAction):

    start_state: JointState = field(default_factory=JointState) # Optional - add more information regarding starting state
    context: WaypointContext = field(default_factory=WaypointContext) # Global parameters not associated with a particular waypoint. helpful to avoid repeating parameters in each waypoint.
    waypoints: list[PoseStamped] = field(default_factory=list) # List of waypoints to reach
    params: list[WaypointParams] = field(default_factory=list) # List of parameters associated with each waypoint


    def is_sync_required(self) -> bool:
        for p in self.params:
            # sync_magnitude = np.linalg.norm(p.arm.sync_velocity.to_numpy())
            if p.arm.sync_speed > 0:
                return True
        return False

    def get_waypoint_joint_angles(self, use_hand: bool = True) -> list[list[float]]:
        """Extract joint angles (arm + hand) for all waypoints."""

        # BUG make sure both are lists! are they won't concate properly

        assert isinstance(self.params[0].arm.joint_angles, list)
        assert isinstance(self.params[0].hand.joint_angles, list)

        # for param in self.params:
        #     print(f"joint_angles: {param.arm.joint_angles} {type(param.arm.joint_angles)} + {param.hand.joint_angles} {type(param.hand.joint_angles)}")

        return [
            wp.arm.joint_angles.extend(wp.hand.joint_angles) or wp.arm.joint_angles if use_hand else wp.arm.joint_angles
            for wp in self.params
        ]

    def get_waypoint_joint_limits(self, use_hand: bool = True, skip_0: bool = True) -> list[JointLimits]:
        """Extract kinematic limits for each section between waypoints.
        
        Returns a list of JointLimits, one for each waypoint (combined arm + hand if use_hand=True).
        If no limits are specified in waypoints, returns empty list.
        """
        section_limits = []
        
        for i, wp in enumerate(self.params):
            if skip_0 and i == 0:
                continue
            
            arm, hand = wp.arm, wp.hand
            
            # Use JointLimits.extend() helper to combine arm and hand limits
            combined_limits = arm.joint_limits.extend(hand.joint_limits) if use_hand else arm.joint_limits
            section_limits.append(combined_limits)
        
        return section_limits
