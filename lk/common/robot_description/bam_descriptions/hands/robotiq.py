#!/usr/bin/env python3

"""
Shared Robotiq hand definitions and convenience factories.

Provides the generic Robotiq metadata, URDF configuration, and the
convenience factories used across Robotiq variants.
"""

from __future__ import annotations

# BAM
from bam.descriptions import (
    UrdfInfo,
    RobotInfo,
    BAM_DESCRIPTIONS_PATH,
    ROBOTIQ_DESCRIPTION_PATH,
    ROBOTIQ_MESH_PACKAGE_PATH,
    TAGS,
    HandArgs,
    RobotDescription,
)


# PYTHON
from dataclasses import dataclass, field
from typing import Any, Optional

import os


@dataclass
class RobotiqUrdfInfo(UrdfInfo):
    """URDF information for Robotiq grippers."""

    macro_path: str = ""
    xacro_path: str = ""
    abs_package_dirs: dict[str, str] = field(
        default_factory=lambda: {
            "robotiq_description": ROBOTIQ_MESH_PACKAGE_PATH,
        }
    )


@dataclass
class RobotiqInfo(RobotInfo):
    """Metadata that identifies the Robotiq gripper variant."""

    name: str = "robotiq_gripper"
    type: str = "Robotiq"
    sku: str = ""
    version: str = "0.0.0"
    save_dir: str = os.path.join(
        os.path.dirname(__file__),
        f"{os.path.splitext(os.path.basename(__file__))[0]}_configs",
    )


@dataclass
class RobotiqArgs(HandArgs):
    """Shared runtime parameters for Robotiq grippers."""

    sim_gazebo: bool = False
    sim_isaac: bool = False
    isaac_joint_commands: str = "/isaac_joint_commands"
    isaac_joint_states: str = "/isaac_joint_states"
    use_fake_hardware: bool = False
    mock_sensor_commands: bool = False
    include_ros2_control: bool = True
    com_port: str = "/dev/ttyUSB0"

    gripper_speed_multiplier: float = 1.0
    gripper_force_multiplier: float = 0.5
    gripper_max_speed: float = 0.150
    gripper_max_force: float = 235.0
    gripper_closed_position: float = 0.7929
    gripper_max_width: float = 0.085
    parent_link: str = ""


@dataclass
class Robotiq(RobotDescription):
    """Common implementation shared by the Robotiq gripper variants."""

    args: RobotiqArgs = field(default_factory=RobotiqArgs)
    urdf: RobotiqUrdfInfo = field(default_factory=RobotiqUrdfInfo)
    info: RobotiqInfo = field(default_factory=RobotiqInfo)

    def __post_init__(self) -> None:
        super().__post_init__()

    @staticmethod
    def make_2F140(
        prefix: Optional[str] = None,
        **kwargs: Any,
    ) -> Robotiq:
        from .robotiq_2f140 import Robotiq2F140, Robotiq2F140Args, Robotiq2F140Info

        return Robotiq2F140(
            info=Robotiq2F140Info(prefix=prefix),
            args=Robotiq2F140Args(),
            **kwargs,
        )

    @staticmethod
    def make_2F85(
        prefix: Optional[str] = None,
        **kwargs: Any,
    ) -> Robotiq:
        from .robotiq_2f85 import Robotiq2F85, Robotiq2F85Args, Robotiq2F85Info

        return Robotiq2F85(
            info=Robotiq2F85Info(prefix=prefix),
            args=Robotiq2F85Args(),
            **kwargs,
        )
