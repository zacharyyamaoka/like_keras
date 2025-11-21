# BAM
from bam.descriptions.robot_description import DescriptionArgs

# PYTHON
import os
from dataclasses import dataclass, field, asdict


"""
Here we extend the RobotDescription to add information relevant to arms.
"""


@dataclass
class HandArgs(DescriptionArgs):

    max_width: float = 100 / 1000
    z_into_table: bool = False
