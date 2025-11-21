#!/usr/bin/env python3

"""

V3 NOTES:

This is good but there is way to much comments, etc... I want to focus in on what matters!

Inspired by:

https://github.com/andyzeng/arc-robot-vision/tree/master?tab=readme-ov-file#baseline-algorithm-1
https://github.com/andyzeng/arc-robot-vision/blob/master/parallel-jaw-grasping/baseline/predict.m
- Taking patches, looking for bumps, check for 2cm depth and no collisions

    Our baseline algorithm detects anti-podal parallel-jaw grasps by detecting "hill-like" geometric features
    (through brute-force sliding window search) from the 3D point cloud of an input heightmap (no color).
    These geometric features should satisfy two constraints: (1) gripper fingers fit within the concavities
    along the sides of the hill, and (2) top of the hill should be at least 2cm above the lowest points of
    the concavities. A valid grasp is ranked by an affordance score, which is computed by the percentage of
    3D surface points between the gripper fingers that are above the lowest points of the concavities.

Classifying and sorting cluttered piles of unknown objects with robots:a learning approach: https://arxiv.org/pdf/1609.01044
- Idea of bump detection, circular conveyor, coloring objects, drop zone verification

RGB Matters: Learning 7-DoF Grasp Poses on Monocular RGBD Images: https://arxiv.org/pdf/2103.02184
- Idea of fast analytical searching, search of z and width, remove collision and zero points, select on with largest distance and smallest width

    As shown in Figure 5, for our network, the input RGB
    image size is 384×288 and the output AVH size is 96×72.
    We uniformly sample V = 60 views in the upper hemisphere
    and A = 6 angles, generating 360 heatmaps in total. In FAS,
    the widths are sampled from 1cm to 10cm with the step of
    1cm. Depths from the place 2cm below the origin point to the
    place 2cm above the point with the step of 1cm are explored.


A key idea is we are looking for bumps (convex surfaces), take the starting point as the top of the hill,
This doesn't work if object, flat or concave, but thoose are bad cases anyways!


- I should do the 4 DOF version to start with. hmm they are all variants of each other basically...

- Simplest is just x, y and rz, and then calculate the rest

- This is a conceptually important algorithim, its shows the tabular case of the MDP


Success Criteria:

Constraints:
(1) Grasp should be at least 2cm below top of hill
(2) Grasp should be not be in collision

Optimize:
(1) Maximise Number of points between fingers
(2) Maximise Depth of grasp
(3) Minise width of grasp

1. Rotate points into local frame of the grasp
2. Offset grasp by 2cm
3. Calc the minimum grasp width with no collisions. (anything bigger is just in open space)
4. Check that there is a mimum number of points between fingers (changing width will not change this)
5. If it passes then add it as a succesful grasp
6. Step down 1cm and repeat until you stop getting succesful grasps!

We prefer long narrow grasps vs wide short grasps.

Look at the ratio of depth to width.
- First select the best ratio
- The select the highest number of points

That doesn't work! it works for many cases... but not if there is a double bump..

- Ok I can directly calculate the min depth though pushing forward the grasp


Options:

W and Z are connected variables, so you can loop through them in various ways, set one and cacluate the other.

1. Loop through w and z
    cons: takes longer
2. Loop through z and calculate w
    the issue is that you cannot directly calculate w, as you have the case of a double bump.
    you need to check various widths along that z line, to check if it fits
3. Loop through w and calculate z
    The works. You can calculate the minimum point underneath the finger tips, and then push the grasp to that z depth (+ an offset)

A deeper grasp is always prefered, will have a better ratio, and will have more points between fingers.
No need to check these floating grasps that could be pushed down deeper!

1. Rotate points into local frame of the grasp
2. Check a bunch of widths (if using teleop, you can just start with the seed width)
3. Push z down to the minimum point under the finger tips
4. Keep grasp if its is deep enough, has the right ratio, has enough points.


----

I should decouple the point cloud downsampling, and the stride length and window size.

----

Ok I want to to behave more like a conv net

You define the input image size.. if it doesn't match then it should throw an error!

Yes you may want to have a different z and x stride. Unlikely I would not that in conv though...

So lets assume I will be using the same stride, so aspect ratio of action space matches imagespace

---

Ok, almost there. the problem is now holes...
If you start at them then your starting point will be at 0,0,0 which screws up everything!
Solution?

Idea 1 is to but a background place behind the point cloud so even at angles you cannot sneak through the holes...
Idea 2 is to just patch the holes by average around...

Ok in the past I have used a background pointcloud to fill in the hole. another solution is to average from around...
For speed I think I will just average from around for now.... I can do the point cloud filling later

---

Ok working well.. I think there like 100 things I could do to improve it.. but its not about making this amazing... just something that
provides reasonable first guesses, and runs fast!

Takes in
- RGBD
- Mask

Outputs

- Exhaustive Search

- Discretize Space

"""

import numpy as np
import open3d as o3d

from bam_grasp.grasp import Grasp

from py_utils.math.pointcloud import (
    depth_2_xyz,
    downsample_rgbd,
    xyz_2_pc,
    rgbxyz_2_pc,
    xyz_2_patch,
    px_2_point_safe,
)

from bam_utils.python.o3d_helper import o3d_viewer, O3DViewer


class BruteForce:
    def __init__(
        self,
        img_height=144,  # This will check internally that the raw_img that is passed matches this aspect ratio
        img_width=256,
        window_size=35,  # size of the sliding window (pixels)
        stride=5,
        n_rx=1,
        n_ry=1,
        n_rz=8,
        n_w=10,
        min_width=0.0,  # min width of the gripper (meters)
        max_width=0.1,  # max width of the gripper (meters)
        min_ratio=0.5,  # min raito of z_offset/gripper_width, this is to avoid very wide grasps that are not deep enough
        min_grasp_depth=0.02,  # min depth of the grasp (meters), this is to avoid very shallow grasps
        min_points_inside=5,  # this is to avoid empty grasps
        verbose=True,
    ):
        self.verbose = verbose
        if self.verbose:
            self.viewer = O3DViewer(cam_pos=[0, 0, 0], lookat=[0, 0, 0.4])

        assert (
            window_size % 2 == 1
        ), "window_size should be odd to ensure centered patches"

        self.img_height = img_height
        self.img_width = img_width
        self.window_size = window_size
        self.stride = stride
        self.verbose = verbose

        # Compute valid patch centers like CNN
        assert (
            self.window_size % 2 == 1
        ), "window_size should be odd to ensure centered patches"
        self.half_window = self.window_size // 2

        # Use "valid" padding (no padding, full window must fit)
        self.n_y = (self.img_height - self.window_size) // self.stride + 1
        self.n_x = (self.img_width - self.window_size) // self.stride + 1

        self.y_range = np.arange(
            self.half_window, self.half_window + self.n_y * self.stride, self.stride
        )
        self.x_range = np.arange(
            self.half_window, self.half_window + self.n_x * self.stride, self.stride
        )

        print(f"[UNCONFIGURED] BruteForce")
        print(f"Intialize grid to overlay on downsampled image...")
        print(f"    Image size: {self.img_height} rows × {self.img_width} cols")
        print(
            f"    Window size: ",
            self.window_size,
            "pixels  (half: ",
            self.half_window,
            "pixels)",
        )
        print(f"    Grid size: {self.n_y} rows × {self.n_x} cols")

        x_min = self.x_range[0] - self.half_window
        x_max = self.x_range[-1] + self.half_window
        y_min = self.y_range[0] - self.half_window
        y_max = self.y_range[-1] + self.half_window

        print(
            f"    y_range ({len(self.y_range)}): {self.y_range} px - bounds: [{y_min}, {y_max}]"
        )
        print(
            f"    x_range ({len(self.x_range)}): {self.x_range} px - bounds: [{x_min}, {x_max}]"
        )

        self.n_z = 1  # Set w and calculate z by pushing forward until contact
        self.n_rx = int(n_rx)
        self.n_ry = int(n_ry)
        self.n_rz = int(n_rz)
        self.n_w = int(n_w)

        # don't include endpoint beacuse it is a duplicate of the first angle!
        if self.n_rx == 1:
            self.rx_range = np.array([0.0])
        else:
            self.rx_range = np.linspace(0, np.pi, self.n_rx, endpoint=False) - np.pi / 2

        if self.n_ry == 1:
            self.ry_range = np.array([0.0])
        else:
            self.ry_range = np.linspace(0, np.pi, self.n_ry, endpoint=False) - np.pi / 2

        if self.n_rz == 1:
            self.rz_range = np.array([0.0])
        else:
            self.rz_range = np.linspace(0, np.pi, self.n_rz, endpoint=False) - np.pi / 2

        print(
            f"    rx_range ({len(self.rx_range)}): {np.round(np.rad2deg(self.rx_range))} deg"
        )  # print in mm for better readability
        print(
            f"    ry_range ({len(self.ry_range)}): {np.round(np.rad2deg(self.ry_range))} deg"
        )  # print in mm for better readability
        print(
            f"    rz_range ({len(self.rz_range)}): {np.round(np.rad2deg(self.rz_range))} deg"
        )  # print in mm for better readability

        self.min_width = min_width
        self.max_width = max_width
        self.min_ratio = min_ratio  # min ratio of grasp_depth/gripper_width, this is to avoid very wide grasps that are not deep enough
        self.min_grasp_depth = min_grasp_depth
        self.min_points_inside = min_points_inside
        self.width_list = np.linspace(self.min_width, self.max_width, self.n_w)
        print(
            f"    width_range ({len(self.width_list)}): {np.round(self.width_list*1000)} mm"
        )  # print in mm for better readability

        self.C_dim = self.n_rx * self.n_ry * self.n_rz * self.n_z * self.n_w  # naive
        self.action_dim = self.n_y * self.n_x * self.C_dim
        print(
            f"    Action Space Dim ({self.action_dim:,}): {(self.n_y, self.n_x, self.n_z, self.n_rx, self.n_ry, self.n_rz, self.n_w)}"
        )

        print(f"[READY] BruteForce")

    def print_v(self, msg):
        if self.verbose:
            print(msg)

    def heatmap(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        camera_info,
        mask: np.ndarray = None,
        verbose_lvl=0,
    ):
        """

        Verbose Levels:
        0 - No verbose, just run the algorithm and return the heatmap
        1 - Show each patch and grasp in the viewer, but don't do any computation
        2 - Show each patch and grasp in the viewer, and do computation, but don't

        It doesn't need to be perfect right now! I am just trying to tie it all together before optimising the system...

        - It actually may make more sense to have 3D heatmap, instead of having this very high dimensional one... what will the kernals look like? not possible...

        Inputs:
        - color: (img_height, img_width, 3)
        - depth: (img_height, img_width, 1)
        - camera_info: flat list of 9 elements (ros2-style intrinsics)

        Note that color and depth need to be rezied ahead of time to be the same size!

        make fingers thicker than they need to be, if using lower resolution point cloud... and a bit longer....

        #TODO edit it to consider the background table to avoid the gripper falling through the floor...
        """

        if verbose_lvl > 0:
            assert self.verbose

        xyz = depth_2_xyz(depth, camera_info)
        xyz[:, :, 2] += 0.01  # meters offset to viz in o3d

        color_ds, depth_ds, camera_info_ds = downsample_rgbd(
            color,
            depth,
            camera_info,
            new_height=self.img_height,
            new_width=self.img_width,
        )
        xyz_ds = depth_2_xyz(depth_ds, camera_info_ds)
        # pc_ds = rgbxyz_2_pc(color_ds, xyz_ds)
        pc = rgbxyz_2_pc(color, xyz)

        if verbose_lvl > 0:
            self.viewer.add_geometries([pc])

        # if self.verbose:
        #     o3d_viewer_origin([pc, pc_ds], cam_pos=[0, 0, 0.5])

        # Iterate through all possibilties

        # Create empty heatmap to hold all values
        # define heat map in image style with (y, x)
        heatmap = np.zeros(
            (self.n_y, self.n_x, self.n_z, self.n_rx, self.n_ry, self.n_rz, self.n_w),
            dtype=np.float32,
        )
        grasp_dict = dict()
        print("Heatmap Shape: ", heatmap.shape)
        # heatmap = np.zeros((self.n_x, self.n_y, self.C_dim), dtype=np.float32)

        # slide a window across the image
        count = 0
        use_mask = isinstance(mask, np.ndarray)

        # For dense striding:
        # Downsample (ds) ratio for image to action
        # self.img_2_action_ds_ratio = n_x/self.img_width
        # aspect_img = img_width / img_height
        # aspect_heat = n_x / n_y
        # assert np.isclose(aspect_img, aspect_heat, rtol=1e-2), "Aspect Ratios don't match, distortion will occur"
        # x_range = range(half_window, self.n_x - half_window)
        # y_range = range(half_window, self.n_y - half_window)

        # We always want full sized window patch so don't start on edge, but start at half window size,
        # this means there will be value in heat map that are empty, thats ok for now
        for i_y, px_y in enumerate(self.y_range):
            for i_x, px_x in enumerate(self.x_range):

                # Set mask to control which pixels to consider
                # ex: just consider pixels that are objects
                # if use_mask:
                #     if mask[px_y, px_x] == 0:
                #         if verbose_lvl not in [1]: # don't skip you want to see the empty patches
                #             continue

                # Get x, y, z in pc frame
                (x_val, y_val, z_val) = px_2_point_safe(
                    px_x, px_y, depth_ds, camera_info_ds
                )
                # Accessing from raw point cloud may mean you select a point in hole with depth 0
                # x_val = xyz_ds[px_y, px_x, 0]
                # y_val = xyz_ds[px_y, px_x, 1]
                # z_val = xyz_ds[px_y, px_x, 2]

                # rgb_patch = rgb_2_patch(color_ds, x, y, window_size)
                if True:
                    print(
                        f"Patch ({i_y}/{self.n_y}, {i_x}/{self.n_x})",
                    )
                xyz_patch = xyz_2_patch(
                    xyz_ds, px_x, px_y, self.window_size, verbose=True
                )

                # Loop through all patches, and visualize them. Don't do any computation
                if verbose_lvl == 1:
                    # create black point cloud of xyz_patch
                    patch_pc = xyz_2_pc(xyz_patch, [0, 0, 0])
                    self.viewer.update([pc, patch_pc])
                    self.viewer.run(duration=0.05)
                    # o3d_viewer([pc, patch_pc], cam_pos=[0, 0, 0], lookat=[0, 0, 0.4])
                    continue

                # print(f"Patch Shape: {rgb_patch.shape}, {xyz_patch.shape}")
                for i_rx, rx_val in enumerate(self.rx_range):
                    for i_ry, ry_val in enumerate(self.ry_range):
                        for i_rz, rz_val in enumerate(self.rz_range):

                            # xyz_ds_pc = rgbxyz_2_pc(rgb_patch, xyz_patch, viz=True)
                            grasp = Grasp(
                                x_val, y_val, z_val, rx_val, ry_val, rz_val, 0
                            )

                            xyz_patch_aligned = grasp.align_xyz(
                                xyz_patch.reshape(-1, 3)
                            )

                            # precompute z offset list

                            for i_w, w_val in enumerate(self.width_list):
                                key = (i_y, i_x, 0, i_rx, i_ry, i_rz, i_w)
                                self.print_v(
                                    f"Grasp # {count}/{self.action_dim} - index: {key}"
                                )
                                # self.print_v(grasp)
                                count += 1

                                # if self.verbose:
                                #     grasp.width = w_val
                                #     patch_pc = xyz_2_pc(xyz_patch, [0,0,0])
                                #     o3d_viewer([pc, patch_pc] + grasp.get_o3d_model(), cam_pos=[0, 0, 0], lookat=[0, 0, 0.4])
                                #     continue

                                grasp_width = w_val
                                found_grasp, point_count, grasp_depth = grasp.find_z(
                                    xyz_patch_aligned, grasp_width
                                )

                                """
                                Its better to not skip grasps, as we want to continually practice, even if they are bad!
                                It would be undesirable for items to be passing underneath and for it to not try.

                                Filtering bad grasps can help remove noise though... so the better grasps are more likely to be selected.
                                """
                                # FYI: Its possible to get a much larger grasp depth, if there is a higher point between fingers that is not the inital point
                                # Success! Point Count: 83, Grasp Depth: 0.046999999999999986
                                # Grasp Z diff 0.477 - 0.501 = -0.02400000000000002

                                # if grasp_depth < self.min_grasp_depth:
                                #     # self.print_v(f"grasp_depth [{grasp_depth}] < self.min_grasp_depth [{self.min_grasp_depth}], skipping")
                                #     continue
                                """
                                Examples:
                                grasp_depth = -2cm, grasp_width = 1cm, ratio = 2
                                grasp_depth = -2cm, grasp_width = 4cm, ratio = 0.5
                                grasp_depth = -2cm, grasp_width = 8cm, ratio = 0.25
                                """
                                ratio = grasp_depth / grasp_width
                                # if ratio < self.min_ratio:
                                #     # self.print_v(f"ratio [{ratio}] < self.min_ratio [{self.min_ratio}], skipping")
                                #     continue

                                # if point_count < self.min_points_inside:
                                #     self.print_v(f"point_count [{point_count}] < self.min_points_inside [{self.min_points_inside}], skipping")
                                #     continue

                                # index = 0 for n = 1

                                """
                                You can use m or mm, doesn't matter its a constant scale factor
                                Examples (ratio 1:1:1)
                                count = 10, ratio = 1, grasp_depth = 5mm -> 50
                                count = 10, ratio = 1, grasp_depth = 10mm -> 100
                                count = 20, ratio = 1, grasp_depth = 5mm -> 100
                                count = 10, ratio = 2, grasp_depth = 5mm -> 100

                                """
                                score = (
                                    (1 * point_count) * (1 * ratio) * (1 * grasp_depth)
                                )
                                heatmap[key] = max(
                                    0, score
                                )  # in case grasp_depth is negative,
                                grasp_dict[key] = found_grasp

                                if verbose_lvl > 0:
                                    gripper_global = found_grasp.get_o3d_model(
                                        local=False
                                    )
                                    self.viewer.update([pc] + gripper_global)
                                    self.viewer.run(duration=0)  # no need to block

                                if verbose_lvl > 5:
                                    print(grasp)
                                    z_offset = found_grasp.z - grasp.z
                                    print(
                                        f"Success! Point Count: {point_count}, Grasp Depth: {grasp_depth}"
                                    )
                                    print(
                                        f"Grasp Z diff {found_grasp.z} - {grasp.z} = {z_offset}"
                                    )
                                    print(found_grasp)

                                    pc_local_inside = xyz_2_pc(
                                        grasp.xyz_inside, [0, 1, 0]
                                    )
                                    pc_local_collision = xyz_2_pc(
                                        grasp.xyz_collision, [1, 0, 0]
                                    )
                                    pc_local_outside = xyz_2_pc(
                                        grasp.xyz_outside, [0, 0, 0]
                                    )
                                    pc_local_counted = xyz_2_pc(
                                        grasp.xyz_counted, [0, 1, 0]
                                    )
                                    pc_local_not_counted = xyz_2_pc(
                                        grasp.xyz_not_counted, [0, 0, 0]
                                    )
                                    gripper_local = found_grasp.get_o3d_model(
                                        local=True, z_offset=z_offset
                                    )
                                    frame_local = o3d.geometry.TriangleMesh.create_coordinate_frame(
                                        size=0.01
                                    )
                                    # local_geometry = [pc_local_inside, pc_local_collision, pc_local_outside, frame_local] #+ gripper_local
                                    local_geometry = [
                                        pc_local_counted,
                                        pc_local_not_counted,
                                    ] + gripper_local

                                    pc_global_inside = xyz_2_pc(
                                        grasp.unalign_xyz(grasp.xyz_inside), [0, 1, 0]
                                    )
                                    pc_global_collision = xyz_2_pc(
                                        grasp.unalign_xyz(grasp.xyz_collision),
                                        [1, 0, 0],
                                    )
                                    pc_global_outside = xyz_2_pc(
                                        grasp.unalign_xyz(grasp.xyz_outside), [0, 0, 0]
                                    )
                                    gripper_global = found_grasp.get_o3d_model(
                                        local=False
                                    )
                                    frame_global = o3d.geometry.TriangleMesh.create_coordinate_frame(
                                        size=0.01
                                    )
                                    frame_global.transform(found_grasp.pose_matrix)
                                    global_geometry = [
                                        pc,
                                        pc_global_inside,
                                        pc_global_collision,
                                        pc_global_outside,
                                        frame_global,
                                    ] + gripper_global

                                    # grasp_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01)
                                    # grasp_frame.translate((0, 0, grasp_z))

                                    # o3d_viewer(global_geometry+local_geometry, cam_pos=[0, 0, -0.4], lookat=[0, 0, 0])
                                    o3d_viewer(
                                        global_geometry,
                                        cam_pos=[0, 0, 0],
                                        lookat=[0, 0, 0.5],
                                    )

                                    # if count > 10:
                                    #     return

        return heatmap
