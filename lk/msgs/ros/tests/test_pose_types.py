"""
We are expecting standard accesses


"""

from bam.msgs.ros_msgs import (
    PoseType,
    Pose,
    PoseStamped,
    TransformStamped,
    Transform,
)

import numpy as np
import pytest

pose_types = [Pose, PoseStamped, TransformStamped, Transform]


def test_xyzrpy_accessors():
    pose_type: PoseType
    for pose_type in pose_types:
        pose = pose_type.random()

        assert isinstance(pose.xyz, np.ndarray)
        assert isinstance(pose.rpy, np.ndarray)
        assert isinstance(pose.wxyz, np.ndarray)
        assert isinstance(pose.xyzw, np.ndarray)

        # assert pose.xyz.tolist() == pose.to_list()[:3]
        # assert pose.rpy.tolist() == pose.to_list()[3:]
        # assert pose.wxyz.tolist() == pose.to_list()[3:]
        # assert pose.xyzw.tolist() == pose.to_list()[:3]
        # assert pose.xyz.tolist() == pose.to_list()[:3]
        # assert pose.rpy.tolist() == pose.to_list()[3:]
        # assert pose.wxyz.tolist() == pose.to_list()[3:]
        # assert pose.xyzw.tolist() == pose.to_list()[:3]


def test_round_trip_list():
    xyz = [0.3, -0.7, 1.2]
    rpy = [0.2, -1.1, 0.9]

    pose_type: PoseType
    for pose_type in pose_types:
        pose = pose_type.from_xyzrpy(xyz, rpy)
        pose_list = pose.to_list()
        reconstructed = pose_type.from_list(pose_list)

        np.testing.assert_allclose(
            reconstructed.to_list(),
            pose_list,
            atol=1e-8,
            rtol=1e-8,
        )


def test_round_trip_matrix():
    xyz = [0.3, -0.7, 1.2]
    rpy = [0.2, -1.1, 0.9]

    pose_type: PoseType
    for pose_type in pose_types:
        pose = pose_type.from_xyzrpy(xyz, rpy)
        matrix = pose.to_matrix()
        reconstructed = pose_type.from_matrix(matrix)

        np.testing.assert_allclose(
            reconstructed.to_matrix(),
            matrix,
            atol=1e-8,
            rtol=1e-8,
        )


def test_round_trip_xyzrpy():
    xyz = [0.3, -0.7, 1.2]
    rpy = [0.2, -1.1, 0.9]

    pose_type: PoseType
    for pose_type in pose_types:
        pose = pose_type.from_xyzrpy(xyz, rpy)
        xyz_out, rpy_out = pose.to_xyzrpy()
        reconstructed = pose_type.from_xyzrpy(xyz_out, rpy_out)
        xyz_reconstructed, rpy_reconstructed = reconstructed.to_xyzrpy()

        np.testing.assert_allclose(
            xyz_reconstructed,
            xyz_out,
            atol=1e-8,
            rtol=1e-8,
        )
        np.testing.assert_allclose(
            rpy_reconstructed,
            rpy_out,
            atol=1e-8,
            rtol=1e-8,
        )


if __name__ == "__main__":
    pytest.main([__file__])
