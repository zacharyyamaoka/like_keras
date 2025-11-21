#!/usr/bin/env python3

"""
BAM Descriptions Package

Provides specific robot descriptions built on top of the base description types.
"""

# BAM
from bam.descriptions import (
    BAM_DESCRIPTIONS_PATH,
    BAM_MESH_PACKAGE_PATH,
    ROBOTIQ_MESH_PACKAGE_PATH,
    UR_MESH_PACKAGE_PATH,
    UrdfInfo,
)

# PYTHON
from dataclasses import dataclass, field


@dataclass
class BamUrdfInfo(UrdfInfo):
    xacro_imports: list[str] = field(
        default_factory=lambda: [f"{BAM_DESCRIPTIONS_PATH}/urdf/common.xacro"]
    )
    abs_package_dirs: dict[str, str] = field(
        default_factory=lambda: {
            "ur_description": UR_MESH_PACKAGE_PATH,
            "robotiq_description": ROBOTIQ_MESH_PACKAGE_PATH,
            "bam_descriptions": BAM_MESH_PACKAGE_PATH,
        }
    )


from .rows import ConveyorDescription
from .arms import UR, DummyDH
from .hands import Robotiq, Robotiq2F85, Robotiq2F140
from .ur_conveyor_simple import make_ur_conveyor_simple
from .world_base_link import WorldBaseLink
from .arm_hand import ArmHand
