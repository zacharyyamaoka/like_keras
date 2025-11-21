# BAM
from ..VisualObject import VisualObject
from ..RGBA import RGBA
from bam.msgs import PoseStamped

# PYTHON
from dataclasses import dataclass, field
import numpy as np


# https://docs.foxglove.dev/docs/visualization/panels/3d#ros-markers
# https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg


@dataclass
class Marker(VisualObject):
    pose: PoseStamped = field(default_factory=lambda: PoseStamped())
    scale: list[float, float, float] = field(default_factory=lambda: [1.0, 1.0, 1.0])
    color: RGBA = field(default_factory=lambda: RGBA.grey())

    @classmethod
    def random(
        cls, scale_range: tuple[float, float] = (0.1, 1.0), name: str = ""
    ) -> "Marker":
        return cls(
            pose=PoseStamped.random(),
            scale=np.random.uniform(scale_range[0], scale_range[1], 3).tolist(),
            color=RGBA.random(),
            name=name,
        )


if __name__ == "__main__":
    marker = Marker()
    print(marker)
