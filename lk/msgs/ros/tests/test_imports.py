"""
Idea is to test importing from expected locations

To help keep organized the pattern is to import to the msg namesace (geometry_msgs, std_msgs, builtin_interfaces, etc.)

"""

import pytest

from bam.msgs.ros_msgs import (
    BoundingBox2D,
    CameraInfo,
    CompressedImage,
    Detection2D,
    Duration,
    Header,
    Image,
    JointState,
    JointTrajectory,
    JointTrajectoryPoint,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point,
    Point2D,
    Polygon,
    Pose,
    Pose2D,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    RegionOfInterest,
    Time,
    Transform,
    TransformStamped,
    Vector3,
)


def test_std_msgs():
    Header()


def test_geometry_msgs():
    Pose()
    PoseStamped()
    TransformStamped()
    Transform()
    Vector3()
    Point()
    Quaternion()
    Polygon()
    PoseWithCovariance()


def test_builtin_interfaces():
    Time()
    Duration()


def test_sensor_msgs():
    Image()
    CompressedImage()
    CameraInfo()
    JointState()
    RegionOfInterest()


def test_trajectory_msgs():
    JointTrajectory()
    JointTrajectoryPoint()


def test_vision_msgs():
    Point2D()
    Pose2D()
    BoundingBox2D()
    ObjectHypothesis()
    ObjectHypothesisWithPose()
    Detection2D()


if __name__ == "__main__":
    pytest.main(["-v", __file__])
