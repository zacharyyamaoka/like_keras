"""
Point of interest description for scene interaction targets.
"""

# BAM
from bam.msgs.ros_msgs import TransformStamped

# PYTHON
from dataclasses import dataclass, field


@dataclass
class PointOfInterest:
    name: str = ""
    transform: TransformStamped = field(default_factory=TransformStamped)

    def __post_init__(self) -> None:
        """Customize in subclasses as needed."""
        return None


if __name__ == "__main__":
    poi = PointOfInterest(name="example_poi")
    print(poi)
