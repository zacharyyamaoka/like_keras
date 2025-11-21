#!/usr/bin/env python3

import numpy as np
from enum import IntEnum


class MaskEnum(IntEnum):
    UNDEFINED = 0
    OBJ = 1  # Object
    BG = 2  # Background
    BG_OBJ = 3  # Background Object


"""

    Inspired by:
    - https://github.com/andyzeng/arc-robot-vision/tree/master


    Class for finding grasps in RGB + Depth + Segmentation images

    main function is find_grasp()

    Inputs:

    - color (req)
    - depth (req)
    - mask: (W, H, 1) uint8 image. See MaskEnum for Values

    - x
    - Y
    - Z
    - Rx
    - Ry
    - Rz
    - Grasp Width

    - w_

    
    Main Use Cases:

    find_grasp(color, depth)
    - Generate PC
    - Use Base Line algorithim to select prompt point
    - Use SAM to segment
    - Detect Bumps
    - Select 

"""


class GraspFinder:
    def __init__(self, camera_info=None, model="baseline"):

        # Make model. Do not dynamically make it at run time
        pass

    def find_grasp_pose(
        self,
        depth: np.ndarray,
        seg: np.ndarray,
        x: float,
        y: float,
        z: float = None,
    ):
        """
        Inputs:
        - Color, depth, seg images
        - x, y, z, rx, ry, rz floats in camera frame of reference

        Returns:
        - Pose in camera frame of reference
        """

    def find_grasp(
        self,
        depth: np.ndarray,
        seg: np.ndarray,
        x: int,
        y: int,
        z: int = None,
    ):
        """
        Factory Function that Routes to correct function based on what is passed in
            Inputs:
            - Color, depth, seg images
            - x, y, z, rx, ry, rz are discrete variables. # of bins is determine in __init__()

            Returns:
            - Pose in camera frame of reference
        """

    def calc_heatmap(self, color: np.ndarray, depth: np.ndarray) -> np.ndarray:
        pass

        heatmap = None
        return heatmap
