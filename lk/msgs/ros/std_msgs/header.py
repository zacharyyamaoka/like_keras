from dataclasses import dataclass, field

from ..builtin_interfaces.time import Time
from ..ros_msg import RosMsg

# https://docs.ros2.org/foxy/api/std_msgs/msg/Header.html


@dataclass
class Header(RosMsg):
    stamp: Time = field(default_factory=Time)
    frame_id: str = ""

    def to_dict(self):
        return {
            "stamp": self.stamp.to_dict(),
            "frame_id": self.frame_id,
        }

    @classmethod
    def from_dict(cls, d: dict):
        stamp = Time.from_dict(d.get("stamp", {}))  # empty dict otherwise
        frame_id = d.get("frame_id", "")
        return cls(stamp=stamp, frame_id=frame_id)

    def lerp(self, target: "Header", fraction: float) -> "Header":
        stamp = self.stamp.lerp(target.stamp, fraction)
        return Header(stamp=stamp, frame_id=self.frame_id)


if __name__ == "__main__":
    header = Header()
    header2 = Header()
    header2.stamp = Time(1, 2)
    header2.frame_id = "test_frame"
    header_lerp = header.lerp(header2, 0.5)
    # print(header_lerp.to_dict())
    # print(header_lerp.stamp.to_dict())
    print(header_lerp)

    header2.stamp = Time(2, 2)
    header2.frame_id = "test_frame"
    header_lerp = header.lerp(header2, 0.5)
    print(header_lerp)
