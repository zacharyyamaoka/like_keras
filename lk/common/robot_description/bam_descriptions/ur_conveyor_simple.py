from bam.descriptions import (
    RobotDescription,
    LinkDescription,
    JointDescription,
    RobotInfo,
)
from bam.msgs.ros_msgs import TransformStamped
import os
from typing import Optional

from .arms import UR
from .rows import ConveyorDescription


def make_ur_conveyor_simple(T_conveyor_to_ur: Optional[TransformStamped] = None):

    world_link = LinkDescription(
        name="world",
        is_frame=True,
    )

    base_link = LinkDescription(
        name="base_link",
        is_frame=True,
    )

    world_joint = JointDescription(
        name="world_to_base_link",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=world_link.name,
            child_frame_id=base_link.name,
        ),
    )

    ur = UR.make_UR5e(prefix="ur_")

    conveyor = ConveyorDescription.make(
        width=0.4,
        length=1.5,
        height=0.05,
        railing_width=0.02,
        railing_height_above=0.05,
        max_speed=0.8,
    )

    base_link_to_conveyor_joint = JointDescription(
        name="base_link_to_conveyor",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=base_link.name,
            child_frame_id=conveyor.links.surface_center.name,
        ),
    )

    if T_conveyor_to_ur is None:
        T_conveyor_to_ur = TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0.5],
            rpy=[0, 0, 0],
            frame_id=conveyor.links.surface_center.name,
            child_frame_id=ur.get_base_mount().name,
        )

    conveyor_to_ur_joint = JointDescription(
        name="conveyor_to_ur", type="fixed", transform=T_conveyor_to_ur
    )

    new_entities = [
        world_link,
        base_link,
        world_joint,
        base_link_to_conveyor_joint,
        conveyor_to_ur_joint,
    ]

    file_name = os.path.splitext(os.path.basename(__file__))[0]
    robot_info = RobotInfo(
        name="ur_conveyor_simple",
        type="ur_conveyor_simple",
        sku="v1",
        version="0.0.0",
        save_dir=os.path.join(os.path.dirname(__file__), f"{file_name}_configs"),
    )

    return RobotDescription.combine(
        [ur, conveyor],
        entities_to_add=new_entities,
        robot_info=robot_info,
    )
