# https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Transform.msg
from .VisualObject import VisualObject
from .RGBA import RGBA
from bam.msgs import TransformStamped
from dataclasses import dataclass, field


@dataclass
class Frame(VisualObject):
    """
    Internally this has rules which deal with None values. So its ok to pass in None values.
    Control visibility with the show_axis/show_origin, and the values will auto compute form the axis_length
    """

    transform: TransformStamped = field(default_factory=lambda: TransformStamped())

    show_axis: bool = True
    show_origin: bool = True
    axis_length: float = 0.1
    axis_radius: float = None
    origin_radius: float = None
    origin_color: RGBA = field(default_factory=lambda: RGBA.origin_gold())

    def __post_init__(self):
        super().__post_init__()

        # first check here, before overriding the defaults
        if self.axis_length <= 0:
            self.show_axis = False

        if self.show_axis is False and self.origin_radius is None:
            self.show_origin = False

        if self.axis_radius is None:
            self.axis_radius = self.axis_length * 0.025

        if self.origin_radius is None:
            self.origin_radius = self.axis_radius * 2.0

        # final check here at the end
        if self.origin_radius <= 0:
            self.show_origin = False

        # its helpful if the Frame can internally deal with None values, as it simplifies the construction process outside!
        if self.origin_color is None:
            self.origin_color = RGBA.origin_gold()


if __name__ == "__main__":
    frame = Frame()
    print(frame)
