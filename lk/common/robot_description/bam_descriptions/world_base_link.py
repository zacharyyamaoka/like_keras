#!/usr/bin/env python3

"""
World Base Link Robot Description

Creates a reusable RobotDescription for mounting robots to the world frame.
This acts like a Python xacro macro - generates world_link, base_link, and
the joints connecting them to a base_mount link.
"""

# BAM
from bam.descriptions import (
    RobotDescription,
    Links,
    Joints,
    LinkDescription,
    JointDescription,
    RobotInfo,
)
from bam.msgs.ros_msgs import TransformStamped
from typing import Optional
import os


class WorldBaseLink(RobotDescription):
    """RobotDescription for world_link, base_link, and connecting joints."""

    @classmethod
    def make(
        cls,
        base_mount: str,
        T_world_to_base_link: Optional[TransformStamped] = None,
        T_base_link_to_base_mount: Optional[TransformStamped] = None,
        prefix: Optional[str] = None,
        robot_info: Optional[RobotInfo] = None,
    ) -> "WorldBaseLink":
        """Create a WorldBaseLink RobotDescription.

        Args:
            base_mount: Name of the base mount link (required, first parameter)
            T_world_to_base_link: Transform from world to base_link (optional, defaults to identity)
            T_base_link_to_base_mount: Transform from base_link to base_mount (optional, defaults to identity)
            prefix: Optional prefix string. If provided and robot_info is None, sets prefix on new RobotInfo.
            robot_info: Optional RobotInfo. If None, creates a new RobotInfo.

        Returns:
            WorldBaseLink containing world_link, base_link, and connecting joints
        """
        world_link = LinkDescription(
            name="world",
            is_frame=True,
        )

        base_link = LinkDescription(
            name="base_link",
            is_frame=True,
        )

        # Create world_to_base_link joint
        if T_world_to_base_link is None:
            T_world_to_base_link = TransformStamped.from_frames(
                frame_id=world_link.name,
                child_frame_id=base_link.name,
            )

        world_joint = JointDescription(
            name="world_to_base_link",
            type="fixed",
            transform=T_world_to_base_link,
        )

        # Create base_link_to_base_mount joint
        if T_base_link_to_base_mount is None:
            T_base_link_to_base_mount = TransformStamped.from_frames(
                frame_id=base_link.name,
                child_frame_id=base_mount,
            )

        base_link_to_base_mount_joint = JointDescription(
            name="base_link_to_base_mount",
            type="fixed",
            transform=T_base_link_to_base_mount,
        )

        entities = [
            world_link,
            base_link,
            world_joint,
            base_link_to_base_mount_joint,
        ]

        # Handle robot_info and prefix logic
        if robot_info is None:
            robot_info = RobotInfo(name="world_base_link")

        if prefix is not None:
            robot_info.prefix = prefix

        return (
            cls.from_entities(entities, robot_info=robot_info)
            .set_flags(generate_py_xml=True)
            .init_joint_positions()
        )


if __name__ == "__main__":
    world_base_link = WorldBaseLink.make(base_mount="base_mount", prefix="robot_1_")

    print(world_base_link.to_urdf_xml())
    world_base_link.dump_to_file(verbose=True)
