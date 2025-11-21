"""
Drop/bin point of interest description with visualization helpers.
"""

# BAM
from bam.msgs.ros_msgs import PoseStamped
from ..links.rgba import RGBA

# PYTHON
from dataclasses import dataclass, field

from .point_of_interest import PointOfInterest


@dataclass
class DropPoint(PointOfInterest):
    """
    Point of interest representing a drop or bin location.
    """

    bin_opening_pose: PoseStamped = field(default_factory=PoseStamped)
    bin_size: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    bin_color: RGBA = field(default_factory=lambda: RGBA.grey())
    bin_center_pose: PoseStamped = field(default_factory=PoseStamped)

    def __post_init__(self) -> None:
        super().__post_init__()
        _, _, depth = self.bin_size
        self.bin_center_pose = self.bin_opening_pose.offset(
            xyz=[0.0, 0.0, -depth / 2.0],
            local=True,
        )


if __name__ == "__main__":
    drop_point = DropPoint(name="example_drop_point")
    print(drop_point.bin_center_pose)
