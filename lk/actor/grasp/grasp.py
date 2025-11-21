#!/usr/bin/env python3

import numpy as np
from py_utils.math.pointcloud import align_xyz, unalign_xyz
from py_utils.math.transforms import xyzrpy_to_matrix
from bam_utils.python.o3d_helper import parallel_gripper_model
import copy

"""
Instead of having refine_w, etc. you can just manually call the functions.

Use zero as default value instead of None

[June 26 2025] I want to take a step back and provide a unifiying datastructure, to make passing around data easier.

I have been very inspired by working with the Graspnet API library, and the GraspGroup/Grasp classes. (https://github.com/graspnet/graspnetAPI/blob/master/graspnetAPI/grasp.py)

Grasp
    - Provide a way to turn an array into a grasp class
    - getters/setters for values
    - to_o3d helper
GraspGroup
    - Can hold multiple grasps, representing them as an array to allow vectorized operations
    - __len__, __repr__, __getitem__ helpers
    - from_npy, save_npy
    - add/remove
    - to_o3d helper which recursively builds
    - sort by sore, randomly sample, near max supression

- While they represent the grasps with only 17 values [score, width, height, depth, rotation_matrix(9), translation(3), object_id]

- I think it would be useful to also add information regarding the finger height, width, thickness, color, etc.. 
perhaps its bad design to pass around these redundant values, but it easier if everything is bundled together! Simplicity > Performance

- I don't want to change the graspnet representation, as its used for collision checking, etc. and is already tested!

All the algorithims, wether sample based or heatmap, will at the end produce a bunch of grasps and confidence scores.
(The exception is regression, but even that produces one grasp with a score)

Therefore any grasping algorithim can be displayed as a heatmap, (if sampled based then you would need to align to grid first)

- The algorthims will keep on changing! you will need to try a millon things to see what actually works in practice, this is a huge amount of work
similar to what Brendan does with kaggle. Right now I am trying to make the dataset :) 
- I should implement the best off the shelf algorithims as that becomes the baseline for the learned system. See RL at scale, scripted exploration,
there is zero cost to using it, so you can generate alot of data to distill it into the new algorithim. (beutiful idea from Hinton)
- bruteforce -> rgbd_matters -> ?

helper functions for cameras:

downsample_mask()
downsample_camera_info()
downsample_color()
downsample_depth()
downsample_rgbd(): uses functions above

helper functions:
generate_hemisphere(): useful to checking between transforms of grasps TCP, as a hemisphere checks position and rotaiton,
and is easy to visually see if correct, option to align inital rotation point to middle like a pumkin, or outside, or to point in tangent velocity direction (cw, cww).

Helper functions for grasping problem:

view_dataset([grasps], [rewards], [color], [depth], [camera_info], [mask], [bg_color], [bg_depth], cmap):
    - switch between different scenes in data set and display with view_label()

view_label(grasps, rewards, color, depth, camera_info, mask, bg_color, bg_depth, cmap): 
    - view grasps above a certain reward threshold (Green is 1, Red is 0 or heatmap [0 to 1])
    - Potetially functionality to go through on x,y (can be done in contious space and discrete)

discretize_grasps(grasps, grid_params):
    - grid_params is a list of N x 3, where the 3 are (n_bucket, min, max) for each dimension, for normal grasps is 7 x 3
    - generates_grasp_grid, and then aligns grasps to that grid see: https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/gen_label.py
    - It should deal with additional variables, in case I want to pass along color, finger_height, etc.
    - discretize grasps into a heatmap, where each grasp is represented by a tuple of (n_y, n_x, n_z, n_rx, n_ry, n_rz, n_width)
    - helpful to align grasps in contious space to a grid
    - define a rule to combine grasps that are close together (max, min, average)
    - You may want to hold onto the orginal countious grasp... for execution later, etc
    - returns a heatmap with confidence scores for

discretize_grasps(grasps, n_y, y_max, y_min, n_x, x_max, x_min n_z, n_rx, n_ry, n_rz, n_width):

generate_grasp_grid():
    - calls generate_grid with default params for grasps
    - 
generate_grid(grid_params):
    - generate a grid

    
The GraspGroup should have:
    - minimal/full: represent grasp by 17 params or all 17 + (color, finger height, width, thickness, etc.)
    - to_graspgroup(): converts basically anything into a graspgroup
    - graspgroup_dict: helpful to avoid sparisty of heatmap/list or if using continous poses, that don't align to grid
    - list_i_to_heatmap_key: Dict which from position in list to heatmap key (n_y, n_x, n_z, n_rx, n_ry, n_rz, n_width)
    - list_i_to_graspgroup_key(): func which directly maps from position in list to heatmap key (n_y, n_x, n_z, n_rx, n_ry, n_rz, n_width)
    - graspgroup_list_key: (-1, 1) used to track key of graspgroup_list as it is sorted, sampled, etc.
    - graspgroup_list: (-1, n_grasp_params)
    - graspgroup_heatmap: (n_y, n_x, n_z, n_rx, n_ry, n_rz, n_width, n_grasp_params): if you slice n_grasp_params[0] you get the heatmap!
        n_grasp_params = [score, width, height, rotation_matrix(9), translation(3), object_id, etc]
    - heatmap: (n_y, n_x, n_z, n_rx, n_ry, n_rz, n_width) array with a confidence score [0, 1] for each discrete grasp
    - heatmap_max: (n_y, n_x)
    - heatmap_img: (color_height, color_width, 3) with heatmap_max upscaled and overlaid
    - sort_by_score(reverse=False): 
    - group_by_score(step_size): return a list with the index that can be used to slice the graspgroup_list to keep grasps above a certain score
    - sample(n_samples=100)
    - score_threshold(min=0, max=1): return a list of indices that have a score above the threshold
    - top_k(k=50):
    - top_k_region(k=50, bbox): can use this to implement quadrant sampling
    - transform(T): transform_matrix @ T for all grasps, useful to convert to different TCP
    - to_graspnet: Calls transform() with the T_tcp_graspnet, small unit test to verify it works, and hemisphere test
    - to_o3d(cmap="none","grey","heat","greenred")
    - transform_matrix: return T for current pose. 
    - FYI. make sure use graspnetAPI.graspgroup imports to tell these apart
    - ability to set all the scores quickly with a single slice once you get it back from the gym
    - The individual grasp classes don't exist, you only construct as needed using the latest data: https://github.com/graspnet/graspnetAPI/blob/master/graspnetAPI/grasp.py#L468



Vectorized hlpers:

- to_patch(graspgroup, window_size, stride, padding, color, depth, camera_info, mask, bg_color, bg_depth):
    returns (N, patch_height, patch_width, 3+1+3+1), patch_centers
    where N is total number of patches, and 3+1+3+1 are optionally different imgs, masks, etc you can add to the patch like a pancake
    patch centers helps you position the patch afterwards.
    This basically does what a conv net does
    todo fix what you pass in.... not sure you actually need the grasps here...

- collision_checking(graspgroup, color, depth, camera_info, mask, bg_color, bg_depth):
    - turns downsample and turn patches internally to decrease memory? TBD
    - vectorized collision checking.
    - Examples:
        https://github.com/GouMinghao/rgb_matters/blob/main/rgbd_graspnet/data/utils/collision.py
        https://github.com/graspnet/graspnetAPI/blob/master/graspnetAPI/utils/eval_utils.py#L185C1-L186C1


"""


class Grasp:
    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, width=0):

        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.width = width

        # For ranking grasps
        self.points_inside = 0  # In colission
        self.points_between = 0  # between fingers, it is good
        self.points_above = 0

        self.grasp_depth = 0
        self.point_count = 0
        self.score = 0.0
        self.reward = 0.0

        self.class_id = 0
        # TODO right now matching these to the collision values used in Collision checker
        self.finger_height = 60 / 1000
        self.finger_width = 20 / 1000  # side to side
        # self.finger_thickness = 7/1000 # front to back
        self.finger_thickness = 10 / 1000  # front to back

        self.palm_thickness = 10 / 1000

        self.parent_grasp: Grasp = None

        self.xyz_inside = None
        self.xyz_collision = None
        self.xyz_outside = None
        self.xyz_counted = None
        self.xyz_not_counted = None
        self.z_local_offset = 0

    @property
    def pose_matrix(self):
        """Return pose matrix with latest values of x, y, z, rx, ry, rz"""
        return xyzrpy_to_matrix([self.x, self.y, self.z], [self.rx, self.ry, self.rz])

    def to_array(self):
        """
        Returns the grasp as a numpy array.
        The order is: [x, y, z, rx, ry, rz, width]
        """
        return np.array([self.x, self.y, self.z, self.rx, self.ry, self.rz, self.width])

    def to_dict(self, frame_id=""):
        """
        Returns the grasp as a dictionary formatted to match the gym action space dict.
        """
        return {
            "pose": {
                "header": {
                    "frame_id": frame_id,
                },
                "pose": {
                    "position": {
                        "x": self.x,
                        "y": self.y,
                        "z": self.z,
                    },
                    "orientation": {
                        "x": self.rx,
                        "y": self.ry,
                        "z": self.rz,
                    },
                },
            },
            "grasp_width": self.width,
        }

    def align_xyz(self, xyz_global: np.ndarray) -> np.ndarray:

        return align_xyz(xyz_global, self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def unalign_xyz(self, xyz_local: np.ndarray) -> np.ndarray:

        return unalign_xyz(xyz_local, self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def xy_grasp_mask(self, xyz: np.ndarray, grasp_width: float = None) -> None:
        # Keep this simple so it can be used for different xyz pointclouds.

        if grasp_width is None:
            grasp_width = self.width

        y_inside = np.abs(xyz[:, 1]) < self.finger_width / 2

        x_inside = np.abs(xyz[:, 0]) < grasp_width / 2
        x_not_inside = np.abs(xyz[:, 0]) > grasp_width / 2
        x_collision = np.abs(xyz[:, 0]) < grasp_width / 2 + self.finger_thickness

        # points are either inside the gripper, in collision with the gripper, or outside the gripper
        xy_inside_mask = y_inside & x_inside
        xy_collision_mask = y_inside & x_not_inside & x_collision
        xy_outside_mask = ~(xy_inside_mask | xy_collision_mask)

        return xy_inside_mask, xy_collision_mask, xy_outside_mask

    def z_grasp_mask(self, xyz: np.ndarray, grasp_z: float = None) -> None:
        if grasp_z is None:
            grasp_z = self.z

        z_inside_mask = np.abs(xyz[:, 2]) < grasp_z
        return z_inside_mask

    def autocomplete(self, pc, refine_w=True):

        if self.z == 0 and self.width == 0:
            self.find_z_w(pc)

        elif self.z == 0 and self.width != 0:
            self.find_z(pc)

        elif self.z != 0 and self.width == 0:
            self.find_w(pc)

    def find_z(
        self, xyz_aligned: np.ndarray, grasp_width: float = 0.0, copy_debug_points=True
    ):
        """
        - This returns a new grasp, doesn't mutate itself.
        - Designed for 4 dof grasping. Assumes no points behind gripper, so you can push forward and directly find the first contact point.
        for top down grasping on a table, this is a good assumption. but if you start tilting the gripper, you might have to change this.
        """

        xy_inside_mask, xy_collision_mask, xy_outside_mask = self.xy_grasp_mask(
            xyz_aligned, grasp_width
        )

        self.xyz_inside = xyz_aligned[xy_inside_mask]
        self.xyz_collision = xyz_aligned[xy_collision_mask]
        self.xyz_outside = xyz_aligned[xy_outside_mask]

        # Push grasp forward until first contact with object or background
        # Local frame starts at 0. As you move in +z direction, you are pushing forwards
        # To find closest colission point you want to take the min(collision_xyz)
        # to calculate grasp depth You want to find the max(inside_point) or min(collision_point) [deepest] - min(inside_point) [shallowest]

        # We want to calculate the z_local_offset, which is the offset from the current TCP (z=0) which is at a point on the cloud
        z_local = 0  # its at tcp
        z_local_offset = (
            1  # start with very large forward push, and then iterate to bring it closer
        )
        z_first_contact = 1
        # print("xy_collision_mask count: ", np.sum(xy_collision_mask))
        # z_local_offset = min(np.min(self.xyz_collision[:, 2]), z_local_offset)
        if np.any(xy_collision_mask):
            z_first_contact = np.min(self.xyz_collision[:, 2])

        # check that top of palm doesn't hit
        # You could nest this inside the first `if np.any(xy_collision_mask)` but
        # there could be a case where the fingers have no collision, but the palm does.
        # this will also look bad if the patch is not big enough! then you will get grasps into flat surfaces..
        if np.any(xy_inside_mask):
            z_min_inside = np.min(self.xyz_inside[:, 2])
            z_first_contact = min(z_min_inside + self.finger_height, z_first_contact)

        z_local_offset = min(z_first_contact, z_local_offset)

        if z_local_offset == 1:
            # Very unlikely, but in case no collision found, we can use the current z
            z_local_offset = z_local
        # print("z_local_offset: ", z_local_offset)
        z_inside_mask = self.z_grasp_mask(xyz_aligned, z_local_offset)
        xyz_inside_mask = xy_inside_mask & z_inside_mask
        # print("xyz_inside_mask count: ", np.sum(xyz_inside_mask))
        self.xyz_counted = xyz_aligned[xyz_inside_mask]
        self.xyz_not_counted = xyz_aligned[~xyz_inside_mask]
        if np.any(xyz_inside_mask):
            inside_points = xyz_aligned[xyz_inside_mask][:, 2]
            point_count = inside_points.shape[0]
            grasp_depth = z_local_offset - np.min(
                inside_points
            )  # measure from deepest point in grasp to most shallow (close to palm)

        else:
            point_count = 0
            grasp_depth = 0

        grasp_z = (
            self.z + z_local_offset
        )  # Adjust grasp z to the current position + local offset
        new_grasp = Grasp(
            self.x, self.y, grasp_z, self.rx, self.ry, self.rz, grasp_width
        )
        new_grasp.point_count = point_count
        new_grasp.grasp_depth = grasp_depth
        new_grasp.z_local_offset = z_local_offset
        new_grasp.parent_grasp = copy.deepcopy(
            self
        )  # if you don't copy, then as you modify the parent it will modify the child too

        return new_grasp

    def _find_z(self, pc):
        pass

    def find_w(self, pc):
        self._find_w(pc)
        return self

    def _find_w(self, pc):
        pass

    def find_z_w(self, pc):
        return self

    def find_width_at_z_offset(
        self, xyz_aligned: np.ndarray, min_w: float, max_w: float, z_offset: float
    ):
        """
        Try to find a suitable grasp width at the given z offset.
        Returns (success: bool, found_grasp: Grasp)
        """
        # Shift the points by the z_offset
        xyz_shifted = xyz_aligned.copy()
        xyz_shifted[:, 2] += z_offset

        # Try widths from min_w to max_w in small steps
        step = 0.002  # 2mm step, can be parameterized
        widths = np.arange(min_w, max_w + step, step)
        for w in widths:
            self.width = w
            score = self.score(xyz_shifted)
            # You can define a threshold for a valid grasp, e.g., enough points inside
            if self.xyz_inside is not None and len(self.xyz_inside) > 0:
                found_grasp = Grasp(
                    self.x, self.y, self.z + z_offset, self.rx, self.ry, self.rz, w
                )
                return True, found_grasp
        return False, None

    def get_o3d_model(self, local=False, z_offset=0, color=[1.0, 0.0, 0.0]):
        """
        args:
            local: if True, the model is centered at origin (0,0,0 + z_offset), which is helpful to vizualise align_xyz pointclouds

        """

        grasp_width = self.width if self.width > 0 else 0.001  # avoid 0 width

        # Grasp base pose
        grasp_pose = self.pose_matrix

        if local:
            grasp_pose = xyzrpy_to_matrix([0, 0, z_offset], [0, 0, 0])

        geometries = parallel_gripper_model(
            grasp_pose,
            grasp_width,
            self.finger_thickness,
            self.finger_height,
            self.finger_width,
            self.palm_thickness,
            color,
        )

        return geometries

    def __str__(self):
        return (
            f"Grasp:\n"
            f"\tx: {self.x:.4f}\n"
            f"\ty: {self.y:.4f}\n"
            f"\tz: {self.z:.4f}\n"
            f"\trx: {self.rx:.4f}\n"
            f"\try: {self.ry:.4f}\n"
            f"\trz: {self.rz:.4f}\n"
            f"\twidth: {self.width:.4f}\n"
            f"\tgrasp_depth: {self.grasp_depth}\n"
            f"\tpoints_inside: {self.points_inside}\n"
            f"\tpoints_between: {self.points_between}\n"
            f"\tpoints_above: {self.points_above}\n"
            f"\tclass_id: {self.class_id}"
        )


class GraspGroup:
    def __init__(self, *args):
        """
        **Input:**

        - args can be (1) nothing (2) numpy array of grasp group array (3) str of the npy file.
        """
        if len(args) == 0:
            self.grasp_group_array = np.zeros((0, GRASP_ARRAY_LEN), dtype=np.float64)
        elif len(args) == 1:
            if isinstance(args[0], np.ndarray):
                self.grasp_group_array = args[0]
            elif isinstance(args[0], str):
                self.grasp_group_array = np.load(args[0])
            else:
                raise ValueError("args must be nothing, numpy array or string.")
        else:
            raise ValueError("args must be nothing, numpy array or string.")

    def __len__(self):
        """
        **Output:**

        - int of the length.
        """
        return len(self.grasp_group_array)

    def __repr__(self):
        repr = "----------\nGrasp Group, Number={}:\n".format(self.__len__())
        if self.__len__() <= 6:
            for grasp_array in self.grasp_group_array:
                repr += Grasp(grasp_array).__repr__() + "\n"
        else:
            for i in range(3):
                repr += Grasp(self.grasp_group_array[i]).__repr__() + "\n"
            repr += "......\n"
            for i in range(3):
                repr += Grasp(self.grasp_group_array[-(3 - i)]).__repr__() + "\n"
        return repr + "----------"

    def __getitem__(self, index):
        """
        **Input:**

        - index: int, slice, list or np.ndarray.

        **Output:**

        - if index is int, return Grasp instance.

        - if index is slice, np.ndarray or list, return GraspGroup instance.
        """
        if type(index) == int:
            return Grasp(self.grasp_group_array[index])
        elif type(index) == slice:
            graspgroup = GraspGroup()
            graspgroup.grasp_group_array = copy.deepcopy(self.grasp_group_array[index])
            return graspgroup
        elif type(index) == np.ndarray:
            return GraspGroup(self.grasp_group_array[index])
        elif type(index) == list:
            return GraspGroup(self.grasp_group_array[index])
        else:
            raise TypeError(
                'unknown type "{}" for calling __getitem__ for GraspGroup'.format(
                    type(index)
                )
            )

    @property
    def scores(self):
        """
        **Output:**

        - numpy array of shape (-1, ) of the scores.
        """
        return self.grasp_group_array[:, 0]

    @scores.setter
    def scores(self, scores):
        """
        **Input:**

        - scores: numpy array of shape (-1, ) of the scores.
        """
        assert scores.size == len(self)
        self.grasp_group_array[:, 0] = copy.deepcopy(scores)

    @property
    def widths(self):
        """
        **Output:**

        - numpy array of shape (-1, ) of the widths.
        """
        return self.grasp_group_array[:, 1]

    @widths.setter
    def widths(self, widths):
        """
        **Input:**

        - widths: numpy array of shape (-1, ) of the widths.
        """
        assert widths.size == len(self)
        self.grasp_group_array[:, 1] = copy.deepcopy(widths)

    @property
    def heights(self):
        """
        **Output:**

        - numpy array of shape (-1, ) of the heights.
        """
        return self.grasp_group_array[:, 2]

    @heights.setter
    def heights(self, heights):
        """
        **Input:**

        - heights: numpy array of shape (-1, ) of the heights.
        """
        assert heights.size == len(self)
        self.grasp_group_array[:, 2] = copy.deepcopy(heights)

    @property
    def depths(self):
        """
        **Output:**

        - numpy array of shape (-1, ) of the depths.
        """
        return self.grasp_group_array[:, 3]

    @depths.setter
    def depths(self, depths):
        """
        **Input:**

        - depths: numpy array of shape (-1, ) of the depths.
        """
        assert depths.size == len(self)
        self.grasp_group_array[:, 3] = copy.deepcopy(depths)

    @property
    def rotation_matrices(self):
        """
        **Output:**

        - np.array of shape (-1, 3, 3) of the rotation matrices.
        """
        return self.grasp_group_array[:, 4:13].reshape((-1, 3, 3))

    @rotation_matrices.setter
    def rotation_matrices(self, rotation_matrices):
        """
        **Input:**

        - rotation_matrices: numpy array of shape (-1, 3, 3) of the rotation_matrices.
        """
        assert rotation_matrices.shape == (len(self), 3, 3)
        self.grasp_group_array[:, 4:13] = copy.deepcopy(
            rotation_matrices.reshape((-1, 9))
        )

    @property
    def translations(self):
        """
        **Output:**

        - np.array of shape (-1, 3) of the translations.
        """
        return self.grasp_group_array[:, 13:16]

    @translations.setter
    def translations(self, translations):
        """
        **Input:**

        - translations: numpy array of shape (-1, 3) of the translations.
        """
        assert translations.shape == (len(self), 3)
        self.grasp_group_array[:, 13:16] = copy.deepcopy(translations)

    @property
    def object_ids(self):
        """
        **Output:**

        - numpy array of shape (-1, ) of the object ids.
        """
        return self.grasp_group_array[:, 16]

    @object_ids.setter
    def object_ids(self, object_ids):
        """
        **Input:**

        - object_ids: numpy array of shape (-1, ) of the object_ids.
        """
        assert object_ids.size == len(self)
        self.grasp_group_array[:, 16] = copy.deepcopy(object_ids)

    def transform(self, T):
        """
        **Input:**

        - T: np.array of shape (4, 4)

        **Output:**

        - GraspGroup instance after transformation, the original GraspGroup will also be changed.
        """
        rotation = T[:3, :3]
        translation = T[:3, 3]
        self.translations = (
            np.dot(rotation, self.translations.T).T + translation
        )  # (-1, 3)
        self.rotation_matrices = np.matmul(rotation, self.rotation_matrices).reshape(
            (-1, 3, 3)
        )  # (-1, 9)
        return self

    def add(self, element):
        """
        **Input:**

        - element: Grasp instance or GraspGroup instance.
        """
        if isinstance(element, Grasp):
            self.grasp_group_array = np.concatenate(
                (
                    self.grasp_group_array,
                    element.grasp_array.reshape((-1, GRASP_ARRAY_LEN)),
                )
            )
        elif isinstance(element, GraspGroup):
            self.grasp_group_array = np.concatenate(
                (self.grasp_group_array, element.grasp_group_array)
            )
        else:
            raise TypeError("Unknown type:{}".format(element))
        return self

    def remove(self, index):
        """
        **Input:**

        - index: list of the index of grasp
        """
        self.grasp_group_array = np.delete(self.grasp_group_array, index, axis=0)
        return self

    def from_npy(self, npy_file_path):
        """
        **Input:**

        - npy_file_path: string of the file path.
        """
        self.grasp_group_array = np.load(npy_file_path)
        return self

    def save_npy(self, npy_file_path):
        """
        **Input:**

        - npy_file_path: string of the file path.
        """
        np.save(npy_file_path, self.grasp_group_array)

    def to_open3d_geometry_list(
        self, color_list=None, use_defaults=True, use_collision=False, show_frame=False
    ):
        """
        **Output:**

        - list of open3d.geometry.Geometry of the grippers.
        """
        if color_list is not None:
            assert len(color_list) == len(
                self.grasp_group_array
            ), "color_list must have the same length as the grasp group array"

        geometry = []
        for i in range(len(self.grasp_group_array)):
            g = Grasp(self.grasp_group_array[i])
            color = None
            if color_list is not None:
                color = color_list[i]
            geometry.extend(
                g.to_open3d_geometry(
                    color,
                    use_defaults=use_defaults,
                    use_collision=use_collision,
                    show_frame=show_frame,
                )
            )

        return geometry

    def sort_by_score(self, reverse=False):
        """
        **Input:**

        - reverse: bool of order, if False, from high to low, if True, from low to high.

        **Output:**

        - no output but sort the grasp group.
        """
        score = self.grasp_group_array[:, 0]
        index = np.argsort(score)
        if not reverse:
            index = index[::-1]
        self.grasp_group_array = self.grasp_group_array[index]
        return self

    def random_sample(self, numGrasp=20):
        """
        **Input:**

        - numGrasp: int of the number of sampled grasps.

        **Output:**

        - GraspGroup instance of sample grasps.
        """
        if numGrasp > self.__len__():
            raise ValueError(
                "Number of sampled grasp should be no more than the total number of grasps in the group"
            )
        shuffled_grasp_group_array = copy.deepcopy(self.grasp_group_array)
        np.random.shuffle(shuffled_grasp_group_array)
        shuffled_grasp_group = GraspGroup()
        shuffled_grasp_group.grasp_group_array = copy.deepcopy(
            shuffled_grasp_group_array[:numGrasp]
        )
        return shuffled_grasp_group

    def to_rect_grasp_group(self, camera):
        """
        **Input:**

        - camera: string of type of camera, 'realsense' or 'kinect'.

        **Output:**

        - RectGraspGroup instance or None.
        """
        tranlations = self.translations
        rotations = self.rotation_matrices
        depths = self.depths
        scores = self.scores
        widths = self.widths
        object_ids = self.object_ids

        mask = rotations[:, 2, 0] > 0.99
        tranlations = tranlations[mask]
        depths = depths[mask]
        widths = widths[mask]
        scores = scores[mask]
        rotations = rotations[mask]
        object_ids = object_ids[mask]

        if tranlations.shape[0] == 0:
            return None

        k_points = get_batch_key_points(tranlations, rotations, widths)
        k_points = k_points.reshape([-1, 3])
        k_points = k_points.reshape([-1, 4, 3])
        rect_grasp_group_array = batch_key_points_2_tuple(
            k_points, scores, object_ids, camera
        )
        rect_grasp_group = RectGraspGroup()
        rect_grasp_group.rect_grasp_group_array = rect_grasp_group_array
        return rect_grasp_group

    def nms(self, translation_thresh=0.03, rotation_thresh=30.0 / 180.0 * np.pi):
        """
        **Input:**

        - translation_thresh: float of the translation threshold.

        - rotation_thresh: float of the rotation threshold.

        **Output:**

        - GraspGroup instance after nms.
        """
        from grasp_nms import nms_grasp

        return GraspGroup(
            nms_grasp(self.grasp_group_array, translation_thresh, rotation_thresh)
        )
