from bam.msgs.ros_msgs import Time


# https://docs.ros.org/en/rolling/p/control_msgs/msg/JointTolerance.html
class JointTolerancePoint:
    """for joint tolerance point data."""

    def __init__(self):
        self.position: list[float] = []
        self.velocity: list[float] = []
        self.acceleration: list[float] = []
        self.effort: list[float] = []
        self.time_from_start = Time()
