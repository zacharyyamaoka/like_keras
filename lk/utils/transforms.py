#!/usr/bin/env python3

"""
    Backwards-compatible wrappers for tf_transformations matrix helpers.
"""

# ROS

# BAM
from tf_transformations import (
    apply_transform_matrix,
    CartesianPathMetrics,
    cartesian_path_analysis,
    matrix_cartesian_distance,
    matrix_from_points,
    matrix_is_close,
    matrix_to_xyzrpy,
    quaternion_axangle_diff,
    quaternion_euler_diff,
    quaternion_to_axangle,
    rotation_is_close,
    rotate_matrix,
    rpy_to_R,
    translate_matrix,
    xyzrpy_offset,
    xyzrpy_to_matrix,
)

# PYTHON
from typing import Iterable

import numpy as np


axangle_diff = quaternion_axangle_diff
euler_diff = quaternion_euler_diff


def matrix_from_point(
    p1: Iterable[float],
    p2: Iterable[float],
    p3: Iterable[float],
) -> np.ndarray:
    return matrix_from_points(p1, p2, p3)


__all__ = [
    "quaternion_to_axangle",
    "rpy_to_R",
    "xyzrpy_to_matrix",
    "matrix_to_xyzrpy",
    "rotation_is_close",
    "matrix_is_close",
    "matrix_cartesian_distance",
    "euler_diff",
    "axangle_diff",
    "matrix_from_point",
    "apply_transform_matrix",
    "xyzrpy_offset",
    "translate_matrix",
    "rotate_matrix",
    "CartesianPathMetrics",
    "cartesian_path_analysis",
]

