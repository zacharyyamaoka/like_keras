#!/usr/bin/env python3

"""
    Arm Hand Robot Description

    Creates a reusable RobotDescription for combining an arm and hand.
    This acts like a Python xacro macro - combines arm and hand descriptions
    with a joint connecting the arm_to_hand_mount and hand_to_arm_mount links.
"""

# BAM
from bam.descriptions import (
    RobotDescription,
    JointDescription, RobotInfo
)
from bam.msgs.ros_msgs import TransformStamped
from typing import Optional
import os

class ArmHand(RobotDescription):
    """RobotDescription for combining arm and hand with connecting joint."""

    @classmethod
    def make(
        cls,
        arm: RobotDescription,
        hand: RobotDescription,
        T_arm_to_hand: Optional[TransformStamped] = None,
        robot_info: Optional[RobotInfo] = None,
    ) -> 'ArmHand':
        """Create an ArmHand RobotDescription by combining arm and hand.
        
        Args:
            arm: RobotDescription for the arm (must have arm_to_hand_mount link)
            hand: RobotDescription for the hand (must have hand_to_arm_mount link)
            T_arm_to_hand: Transform from arm_to_hand_mount to hand_to_arm_mount 
                          (optional, defaults to identity)
            robot_info: Optional RobotInfo. If None, creates a new RobotInfo.
        
        Returns:
            ArmHand containing combined arm, hand, and connecting joint
        """
        # Get mount links
        arm_mount = arm.get_arm_to_hand_mount()
        hand_mount = hand.get_hand_to_arm_mount()
        
        # Create arm_to_hand joint
        if T_arm_to_hand is None:
            T_arm_to_hand = TransformStamped.from_frames(
                frame_id=arm_mount.name,
                child_frame_id=hand_mount.name,
            )
        
        arm_to_hand_joint = JointDescription(
            name="arm_to_hand",
            type="fixed",
            transform=T_arm_to_hand,
        )
        
        # Handle robot_info
        if robot_info is None:
            # Use names from arm and hand
            arm_name = arm.info.name if arm.info.name else "arm"
            hand_name = hand.info.name if hand.info.name else "hand"
            robot_info = RobotInfo(name=f"{arm_name}_{hand_name}")
        
        # Set save_dir if not already set
        if not robot_info.save_dir:
            robot_info.save_dir = os.path.join(os.path.dirname(__file__), f"{os.path.splitext(os.path.basename(__file__))[0]}_configs")
        
        # Combine arm, hand, and the connecting joint
        return cls.combine(
            [arm, hand],
            entities_to_add=[arm_to_hand_joint],
            robot_info=robot_info,
        )


if __name__ == "__main__":
    # Example usage would require actual arm and hand descriptions
    pass

