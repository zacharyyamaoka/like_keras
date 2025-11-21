from dataclasses import dataclass

from .time import Time
from ..ros_msg import RosMsg

# https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Duration.html


@dataclass
class Duration(RosMsg):
    sec: int = 0
    nanosec: int = 0

    def to_float(self) -> float:
        return self.sec + self.nanosec / 1e9

    @classmethod
    def from_float(cls, value: float) -> "Duration":
        sec = int(value)
        nanosec = int((value - sec) * 1e9)
        return cls(sec=sec, nanosec=nanosec)

    def __add__(self, other: "Duration") -> "Duration":
        return Duration(self.sec + other.sec, self.nanosec + other.nanosec)

    def __sub__(self, other: "Duration") -> "Duration":
        return Duration(self.sec - other.sec, self.nanosec - other.nanosec)

    # def __str__(self) -> str:
    #     return f"Duration(sec={self.sec}, nanosec={self.nanosec})"

    @classmethod
    def from_time(cls, time: Time) -> "Duration":
        return cls(time.sec, time.nanosec)

    def to_time(self) -> Time:
        return Time(self.sec, self.nanosec)
