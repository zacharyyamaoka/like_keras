# BAM
from bam.descriptions.robot_description import DescriptionArgs

# PYTHON
import os
from dataclasses import dataclass, field, asdict


"""
Here we extend the RobotDescription to add information relevant to arms.
"""


@dataclass
class ArmArgs(DescriptionArgs):

    ...

    def __post_init__(self):
        super().__post_init__()
