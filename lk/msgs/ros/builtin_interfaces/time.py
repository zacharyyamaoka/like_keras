# BAM
from ..ros_msg import RosMsg

# PYTHON
from dataclasses import dataclass
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from .duration import Duration

# https://docs.ros2.org/foxy/api/builtin_interfaces/msg/Time.html
@dataclass
class Time(RosMsg):
    sec: int = 0
    nanosec: int = 0

    def to_float(self):
        return self.sec + self.nanosec / 1e9

    @classmethod
    def from_float(cls, value: float):
        sec = int(value)
        nanosec = int((value - sec) * 1e9)
        return cls(sec, nanosec)

    def lerp(self, target: 'Time', fraction: float) -> 'Time':
        # Convert both times to float, interpolate, then create new Time from float
        t0 = self.to_float()
        t1 = target.to_float()
        t_lerp = t0 + (t1 - t0) * fraction
        return Time.from_float(t_lerp)

    def __add__(self, other: Union['Time', 'Duration']) -> 'Time':
        return Time(self.sec + other.sec, self.nanosec + other.nanosec)

    def __sub__(self, other: Union['Time', 'Duration']) -> 'Time':
        return Time(self.sec - other.sec, self.nanosec - other.nanosec)

    def __lt__(self, other: Union['Time', 'Duration']) -> bool:
        return self.to_float() < other.to_float()

    def __le__(self, other: Union['Time', 'Duration']) -> bool:
        return self.to_float() <= other.to_float()

    def __gt__(self, other: Union['Time', 'Duration']) -> bool:
        return self.to_float() > other.to_float()

    def __ge__(self, other: Union['Time', 'Duration']) -> bool:
        return self.to_float() >= other.to_float()