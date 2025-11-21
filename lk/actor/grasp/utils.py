#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def draw_patches_on_blank_image(img_height, img_width, x_range, y_range, window_size):
    """
    Draws sliding window patches on a blank image canvas.

    Args:
        img_height (int): height of the image.
        img_width (int): width of the image.
        x_range (np.ndarray): x center positions of patches.
        y_range (np.ndarray): y center positions of patches.
        window_size (int): patch size in pixels (square assumed).
    """
    # Create a blank white image
    blank_image = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

    fig, ax = plt.subplots(1, figsize=(10, 6))
    ax.imshow(blank_image)

    half = window_size // 2

    for j, y in enumerate(y_range):
        for i, x in enumerate(x_range):
            top_left = (x - half, y - half)
            rect = patches.Rectangle(
                top_left,
                window_size,
                window_size,
                linewidth=1,
                edgecolor="red",
                facecolor="none",
            )
            ax.add_patch(rect)

            # Optional: add index label
            ax.text(
                x, y, f"{j},{i}", color="blue", ha="center", va="center", fontsize=6
            )

    ax.set_title(f"{len(y_range)} × {len(x_range)} Sliding Patches")
    ax.set_xlim(0, img_width)
    ax.set_ylim(img_height, 0)  # invert Y axis to match image coords
    ax.set_aspect("equal")
    ax.axis("off")
    plt.tight_layout()
    plt.show()


# --- Example usage
# img_height = 144
# img_width = 256
# window_size = 36
# stride = 5

# half = window_size // 2
# n_y = (img_height - window_size) // stride + 1
# n_x = (img_width - window_size) // stride + 1

# y_range = np.arange(half, half + n_y * stride, stride)
# x_range = np.arange(half, half + n_x * stride, stride)

# draw_patches_on_blank_image(img_height, img_width, x_range, y_range, window_size)


class Grasp:
    def __init__(self, x, y, z=0, rx=0, ry=0, rz=0, w=0):

        self.pose = (x, y, z, rx, ry, rz)  # pass euler angles in standard order xyz
        self.width = w
        self.rx = rx
        self.ry = ry
        self.rz = rz

        # For ranking grasps
        self.grasp_depth = 0
        self.points_inside = 0  # In colission
        self.points_between = 0  # between fingers, it is good
        self.points_above = 0

        self.class_id = 0

        self.finger_height = 60 / 1000
        self.finger_width = 18 / 1000  # side to side
        self.finger_thickness = 7 / 1000  # front to back


def get_gripper_model(grasp, frame_id=None, stamp=None, color=[1.0, 0.0, 0.0]):

    thickness = grasp.finger_thickness
    width = grasp.finger_width
    height = grasp.finger_height

    grasp_width = grasp.width
    if grasp_width <= 0:
        grasp_width = 0.01  # avoid errors when drawing boxes

    def create_centered_box(pose_matrix, size):
        """
        Creates a box centered at the origin with a given size and color.
        The standard o3d box is from the edge
        """
        box = o3d.geometry.TriangleMesh.create_box(
            width=size[0], height=size[1], depth=size[2]
        )
        box.paint_uniform_color(color)
        box.compute_vertex_normals()

        # Shift the box to center it around the origin
        transformation = np.eye(4)
        transformation[:3, 3] = [-size[0] / 2, -size[1] / 2, -size[2] / 2]
        box.transform(transformation)

        box.transform(pose_matrix)

        return box
