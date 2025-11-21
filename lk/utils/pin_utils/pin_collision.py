#!/usr/bin/env python3

# BAM
from ..tempfile_utils import temp_srdf_file
from .pin_robot_model import PinRobotModel

# PYTHON
import pinocchio as pin
import hppfcl as fcl
from typing import Optional, TYPE_CHECKING
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import subprocess
import os

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription

"""
Multi Robot Collision Detection: https://github.com/stack-of-tasks/pinocchio/discussions/1841
"""


class PinCollision:

    @classmethod
    def from_robot_description(
        cls, robot_description: "RobotDescription", verbose=False
    ):
        robot_model = PinRobotModel.from_robot_description(robot_description)
        return cls(
            robot_model, srdf_xml=robot_description.get_srdf_xml(), verbose=verbose
        )

    def __init__(
        self,
        robot_model: PinRobotModel,
        srdf_path: Optional[str] = None,
        srdf_xml: Optional[str] = None,
        verbose=False,
    ):
        """
        DescriptionArgs:
            robot_model (PinModel): PinModel containing model and collision_model
            srdf_path: You basically always need an srdf, otherwise links next to each other will always be in collision!
            srdf_xml: SRDF as XML string (alternative to srdf_path)
            verbose (bool): Whether to print verbose output
        """
        self.robot_model = robot_model
        self.model = robot_model.model
        self.collision_model = robot_model.collision_model
        self.verbose = verbose

        if self.collision_model is None:
            raise ValueError(
                "PinModel must have collision_model loaded for collision checking"
            )

        # Add all collision pairs
        self.collision_model.addAllCollisionPairs()

        # Map geometry object names to indices for quick lookup
        self.geometry_name_to_id = {
            geom.name: idx
            for idx, geom in enumerate(self.collision_model.geometryObjects)
        }

        # Save full list BEFORE removal for inspection
        all_pairs = [
            (
                pair.first,
                pair.second,
                self.collision_model.geometryObjects[pair.first].name,
                self.collision_model.geometryObjects[pair.second].name,
            )
            for pair in self.collision_model.collisionPairs
        ]

        def remove_collision_pairs(srdf_path: str):
            if not os.path.exists(srdf_path):
                raise FileNotFoundError(f"SRDF path does not exist: {srdf_path}")

            if verbose:
                print(f"[Collision] Removing collision pairs using SRDF: {srdf_path}")
            pin.removeCollisionPairs(self.model, self.collision_model, srdf_path)

        # Remove collision pairs that are next to each other, never in contact, etc.
        # If srdf is provided as a string, create a temporary file for it and use its path
        if srdf_xml:
            with temp_srdf_file(srdf_xml) as srdf_path:
                remove_collision_pairs(srdf_path)

        elif srdf_path:
            remove_collision_pairs(srdf_path)

        else:
            if verbose:
                print("[Collision] No SRDF provided, using all collision pairs")

        # Create these objects after removing collision pairs or you will have seg faults!
        self.data = self.model.createData()
        self.collision_data = pin.GeometryData(self.collision_model)

        # Track custom obstacles (name -> geometry_id)
        self.obstacles = {}

        # Store all active name pairs after SRDF filtering
        active_name_pairs = set(
            (
                self.collision_model.geometryObjects[pair.first].name,
                self.collision_model.geometryObjects[pair.second].name,
            )
            for pair in self.collision_model.collisionPairs
        )

        # Store pair counts
        self.n_pairs = len(all_pairs)
        self.n_active_pairs = len(active_name_pairs)

        if self.verbose:
            print(
                f"[Collision] Pairs after SRDF filtering: {len(active_name_pairs)} active of {len(all_pairs)} total"
            )
            print("[Collision] Collision Pairs (with SRDF tag):")

            for i, (first_idx, second_idx, name1, name2) in enumerate(all_pairs):
                tag = (
                    "ACTIVE"
                    if (name1, name2) in active_name_pairs
                    or (name2, name1) in active_name_pairs
                    else "REMOVED"
                )
                print(f"  {i+1:03d}. {name1} <-> {name2}   [{tag}]")

        else:
            print(
                f"[Collision] Pairs after SRDF filtering: {len(active_name_pairs)} active of {len(all_pairs)} total"
            )

    def get_collision_pairs(
        self, filter_links: Optional[list[str]] = None
    ) -> list[tuple[str, str]]:
        """Get list of active collision pairs as (name1, name2) tuples.

        DescriptionArgs:
            filter_links: Optional list of link names to filter on. If provided, only returns
                         pairs where at least one link name matches a name in the filter list.

        Returns:
            List of collision pair tuples, where each tuple contains the names
            of two geometry objects that are checked for collision.

        Example:
            >>> pairs = pin_collision.get_collision_pairs()
            >>> print(f"Checking {len(pairs)} collision pairs")
            >>> for name1, name2 in pairs[:5]:
            >>>     print(f"  {name1} <-> {name2}")
            >>>
            >>> # Filter for specific links
            >>> filtered_pairs = pin_collision.get_collision_pairs(filter_links=["base_link", "link1"])
        """
        all_pairs = [
            (
                self.collision_model.geometryObjects[pair.first].name,
                self.collision_model.geometryObjects[pair.second].name,
            )
            for pair in self.collision_model.collisionPairs
        ]

        if filter_links is None:
            return all_pairs

        # Filter pairs where at least one link name matches
        filtered_pairs = [
            (name1, name2)
            for name1, name2 in all_pairs
            if name1 in filter_links or name2 in filter_links
        ]

        print(
            f"Total Collision Pairs: {len(all_pairs)}, Matching Pairs: {len(filtered_pairs)}/{len(all_pairs)}"
        )

        return filtered_pairs

    def enable_collision_pair(self, name_a: str, name_b: str) -> bool:
        """Force-enable collision checking between two geometry objects by name."""
        idx_a = self.geometry_name_to_id.get(name_a)
        idx_b = self.geometry_name_to_id.get(name_b)
        if idx_a is None or idx_b is None:
            raise KeyError(f"Unknown collision geometry name(s): {name_a}, {name_b}")

        existing_pairs = {
            (pair.first, pair.second) for pair in self.collision_model.collisionPairs
        }
        if (idx_a, idx_b) in existing_pairs or (idx_b, idx_a) in existing_pairs:
            return False

        self.collision_model.addCollisionPair(pin.CollisionPair(idx_a, idx_b))
        self.n_active_pairs += 1
        return True

    def plot_collision_matrix(
        self,
        title: Optional[str] = None,
        robot_links: Optional[list[str]] = None,
        ax: Optional[plt.Axes] = None,
        show_labels: bool = True,
        figsize: tuple[float, float] = (12, 10),
    ) -> plt.Axes:
        """Plot collision matrix showing which link pairs can collide.

        Creates a visual matrix where green cells indicate collision checking is enabled
        between two links, and white cells indicate no collision checking.

        DescriptionArgs:
            title: Title for the plot (defaults to "Collision Matrix")
            robot_links: Optional list of robot link names to display (filters out non-robot links)
            ax: Optional matplotlib axes to plot on (creates new figure if None)
            show_labels: Whether to show link name labels (only for ≤30 links)
            figsize: Figure size as (width, height) in inches

        Returns:
            Matplotlib axes with the plot

        Example:
            >>> import matplotlib.pyplot as plt
            >>> pin_collision.plot_collision_matrix(title="UR5e Collision Pairs")
            >>> plt.savefig("collision_matrix.png")
            >>> plt.show()
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=figsize)

        # Get collision pairs
        collision_pairs = self.get_collision_pairs()

        # Get all unique link names from collision pairs
        all_links = set()
        for pair in collision_pairs:
            all_links.add(pair[0])
            all_links.add(pair[1])

        # Filter to robot links if specified
        if robot_links is not None:
            all_links = {
                link
                for link in all_links
                if any(rlink in link for rlink in robot_links)
            }

        # Sort links for consistent ordering
        link_names = sorted(list(all_links))
        n_links = len(link_names)

        # Create collision matrix
        collision_matrix = np.zeros((n_links, n_links), dtype=bool)

        # Fill in collision pairs
        for pair in collision_pairs:
            if pair[0] in link_names and pair[1] in link_names:
                i = link_names.index(pair[0])
                j = link_names.index(pair[1])
                collision_matrix[i, j] = True
                collision_matrix[j, i] = True  # Symmetric

        # Plot matrix (green = collision enabled, white = no collision)
        ax.imshow(
            collision_matrix, cmap="Greens", interpolation="nearest", aspect="auto"
        )

        # Add gridlines between each cell (link pair)
        # Minor ticks at cell boundaries: -0.5, 0.5, 1.5, ..., n_links-0.5
        ax.set_xticks(np.arange(n_links + 1) - 0.5, minor=True)
        ax.set_yticks(np.arange(n_links + 1) - 0.5, minor=True)
        ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.5, alpha=0.5)

        # Major ticks at cell centers for labels
        ax.set_xticks(np.arange(n_links))
        ax.set_yticks(np.arange(n_links))

        # Add labels if requested
        if show_labels:
            ax.set_xticklabels(link_names, rotation=90, ha="right", fontsize=8)
            ax.set_yticklabels(link_names, fontsize=8)
        else:
            ax.set_xticklabels([])
            ax.set_yticklabels([])

        # Add title and statistics
        n_pairs = np.sum(collision_matrix) // 2  # Divide by 2 since matrix is symmetric
        n_possible = n_links * (n_links - 1) // 2

        if title is None:
            title = "Collision Matrix"

        percentage = 100 * n_pairs / n_possible if n_possible > 0 else 0
        ax.set_title(
            f"{title}\n{n_links} links, {n_pairs}/{n_possible} collision pairs ({percentage:.1f}%)",
            fontsize=12,
            fontweight="bold",
        )

        # Add legend
        green_patch = mpatches.Patch(color="green", label="Collision enabled")
        white_patch = mpatches.Patch(
            color="white", label="No collision", edgecolor="gray"
        )
        ax.legend(handles=[green_patch, white_patch], loc="upper right", fontsize=10)

        return ax

    def is_collision(self, q: np.ndarray) -> bool:
        """
        is_collision for collision at a given configuration.

        DescriptionArgs:
            q (np.ndarray): Joint configuration

        Returns:
            bool: True if collision is detected, False otherwise
        """
        is_collision, _ = self.get_collision_info(q, break_on_collision=True)
        return is_collision

    def get_collision_info(
        self, q: np.ndarray, break_on_collision: bool = False
    ) -> tuple[bool, list[tuple[str, str]]]:
        """
        Get detailed collision information at a given configuration.

        DescriptionArgs:
            q (np.ndarray): Joint configuration
            break_on_collision (bool): If True, return immediately after first collision found

        Returns:
            tuple: (is_collision, collision_pairs) where:
                - is_collision (bool): True if any collision detected
                - collision_pairs (list[tuple[str, str]]): List of (geom1_name, geom2_name) tuples

        Example:
            >>> is_collision, pairs = collision_checker.get_collision_info(q)
            >>> if any('left_finger' in pair for pair in pairs):
            >>>     print("Left finger is colliding")
            >>> if ('left_finger', 'target_outer') in pairs:
            >>>     print("Exact pair match")
        """
        # Using deprecated API because new API has compatibility issues with dynamic obstacles

        pin.computeCollisions(
            self.model, self.data, self.collision_model, self.collision_data, q, False
        )

        collision_pairs = []
        for k, pair in enumerate(self.collision_model.collisionPairs):
            result = self.collision_data.collisionResults[k]
            if result.isCollision():
                geom1_name = self.collision_model.geometryObjects[pair.first].name
                geom2_name = self.collision_model.geometryObjects[pair.second].name

                collision_pairs.append((geom1_name, geom2_name))

                if self.verbose:
                    print(
                        f"[Collision] Collision detected: {geom1_name} <-> {geom2_name}"
                    )

                if break_on_collision:
                    return True, collision_pairs

        is_collision = len(collision_pairs) > 0
        return is_collision, collision_pairs

    def _add_collision_pairs(
        self,
        obstacle_id: int,
        obstacle_name: str,
        check_link_names: Optional[list[str]] = None,
    ):
        """
        Helper method to add collision pairs between obstacle and robot links.

        DescriptionArgs:
            obstacle_id: Geometry ID of the obstacle
            obstacle_name: Name of the obstacle
            check_link_names: Robot links to check collision with (None = all robot links)
        """
        if check_link_names is None:
            # Check against all existing robot geometries (not other obstacles)
            for geom_obj in self.collision_model.geometryObjects:
                if (
                    geom_obj.name != obstacle_name
                    and geom_obj.name not in self.obstacles
                ):
                    link_id = self.collision_model.getGeometryId(geom_obj.name)
                    pair = pin.CollisionPair(obstacle_id, link_id)
                    self.collision_model.addCollisionPair(pair)
        else:
            # Check against specific links only
            # Note: Geometry names in collision model may have suffixes (e.g., "link_0")
            # so we need to find geometries whose names start with the requested link_name
            for link_name in check_link_names:
                matched = False
                for geom_obj in self.collision_model.geometryObjects:
                    # Match exact name or name with suffix pattern (e.g., "link" or "link_0")
                    if geom_obj.name == link_name or geom_obj.name.startswith(
                        f"{link_name}_"
                    ):
                        try:
                            link_id = self.collision_model.getGeometryId(geom_obj.name)
                            pair = pin.CollisionPair(obstacle_id, link_id)
                            self.collision_model.addCollisionPair(pair)
                            matched = True
                            if self.verbose:
                                print(
                                    f"[Collision] Added collision pair: {obstacle_name} <-> {geom_obj.name}"
                                )
                        except Exception as e:
                            if self.verbose:
                                print(
                                    f"[Collision] Warning: Could not add pair for {geom_obj.name}: {e}"
                                )

                if not matched and self.verbose:
                    print(
                        f"[Collision] Warning: No geometry found matching '{link_name}'"
                    )

        # Recreate collision_data after adding new pairs
        self.collision_data = pin.GeometryData(self.collision_model)

    def add_box_obstacle(
        self,
        name: str,
        size: float | tuple[float, float, float],
        pose: np.ndarray | pin.SE3,
        check_link_names: Optional[list[str]] = None,
        color: Optional[np.ndarray] = None,
    ) -> int:
        """
        Add a box obstacle to the collision model.

        DescriptionArgs:
            name: Name for the box geometry object
            size: Side length (single value for cube) or (x, y, z) for rectangular box
            pose: Either a 4x4 homogeneous matrix or pin.SE3 object
            check_link_names: Robot links to check collision with (None = all robot links)
            color: RGBA color array (defaults to red)

        Returns:
            Geometry ID of the added box
        """
        if name in self.obstacles:
            raise ValueError(
                f"Obstacle '{name}' already exists. Use update_obstacle_pose() to move it."
            )

        # Create box shape
        if isinstance(size, (int, float)):
            box_shape = fcl.Box(size, size, size)
        else:
            box_shape = fcl.Box(size[0], size[1], size[2])

        # Convert pose to SE3 if needed
        if isinstance(pose, np.ndarray):
            box_placement = pin.SE3(pose)
        else:
            box_placement = pose

        # Create GeometryObject (parent_joint=0 means world frame)
        box_geom = pin.GeometryObject(name, 0, box_placement, box_shape)

        # Set color
        if color is None:
            color = np.array([1.0, 0.0, 0.0, 1.0])  # Red by default
        box_geom.meshColor = color

        # Add to collision model
        # Using deprecated API for compatibility with all model types

        box_id = self.collision_model.addGeometryObject(box_geom)

        # Add collision pairs
        self._add_collision_pairs(box_id, name, check_link_names)

        # Track this obstacle
        self.obstacles[name] = box_id

        if self.verbose:
            print(
                f"[Collision] Added box obstacle '{name}' (ID: {box_id}) at position {box_placement.translation}"
            )

        return box_id

    def add_cylinder_obstacle(
        self,
        name: str,
        radius: float,
        height: float,
        pose: np.ndarray | pin.SE3,
        check_link_names: Optional[list[str]] = None,
        color: Optional[np.ndarray] = None,
    ) -> int:
        """
        Add a cylinder obstacle to the collision model.

        DescriptionArgs:
            name: Name for the cylinder geometry object
            radius: Radius of the cylinder
            height: Height of the cylinder (along z-axis)
            pose: Either a 4x4 homogeneous matrix or pin.SE3 object
            check_link_names: Robot links to check collision with (None = all robot links)
            color: RGBA color array (defaults to blue)

        Returns:
            Geometry ID of the added cylinder
        """
        if name in self.obstacles:
            raise ValueError(
                f"Obstacle '{name}' already exists. Use update_obstacle_pose() to move it."
            )

        # Create cylinder shape
        cylinder_shape = fcl.Cylinder(radius, height)

        # Convert pose to SE3 if needed
        if isinstance(pose, np.ndarray):
            cylinder_placement = pin.SE3(pose)
        else:
            cylinder_placement = pose

        # Create GeometryObject (parent_joint=0 means world frame)
        cylinder_geom = pin.GeometryObject(name, 0, cylinder_placement, cylinder_shape)

        # Set color
        if color is None:
            color = np.array([0.0, 0.0, 1.0, 1.0])  # Blue by default
        cylinder_geom.meshColor = color

        # Add to collision model
        # Using deprecated API for compatibility with all model types

        cylinder_id = self.collision_model.addGeometryObject(cylinder_geom)

        # Add collision pairs
        self._add_collision_pairs(cylinder_id, name, check_link_names)

        # Track this obstacle
        self.obstacles[name] = cylinder_id

        if self.verbose:
            print(
                f"[Collision] Added cylinder obstacle '{name}' (ID: {cylinder_id}) at position {cylinder_placement.translation}"
            )

        return cylinder_id

    def add_mesh_obstacle(
        self,
        name: str,
        mesh_path: str,
        pose: np.ndarray | pin.SE3,
        scale: tuple[float, float, float] = (1.0, 1.0, 1.0),
        check_link_names: Optional[list[str]] = None,
        color: Optional[np.ndarray] = None,
    ) -> int:
        """
        Add a mesh obstacle to the collision model.

        DescriptionArgs:
            name: Name for the mesh geometry object
            mesh_path: Path to the mesh file (.obj, .stl, .dae, etc.)
            pose: Either a 4x4 homogeneous matrix or pin.SE3 object
            scale: Scale factors for (x, y, z) dimensions
            check_link_names: Robot links to check collision with (None = all robot links)
            color: RGBA color array (defaults to green)

        Returns:
            Geometry ID of the added mesh
        """
        if name in self.obstacles:
            raise ValueError(
                f"Obstacle '{name}' already exists. Use update_obstacle_pose() to move it."
            )

        # Load mesh using hppfcl
        mesh_loader = fcl.MeshLoader()
        mesh_shape = mesh_loader.load(mesh_path, np.array(scale))

        # Convert pose to SE3 if needed
        if isinstance(pose, np.ndarray):
            mesh_placement = pin.SE3(pose)
        else:
            mesh_placement = pose

        # Create GeometryObject (parent_joint=0 means world frame)
        mesh_geom = pin.GeometryObject(name, 0, mesh_placement, mesh_shape)
        mesh_geom.meshPath = mesh_path
        mesh_geom.meshScale = np.array(scale)

        # Set color
        if color is None:
            color = np.array([0.0, 1.0, 0.0, 1.0])  # Green by default
        mesh_geom.meshColor = color

        # Add to collision model
        # Using deprecated API for compatibility with all model types

        mesh_id = self.collision_model.addGeometryObject(mesh_geom)

        # Add collision pairs
        self._add_collision_pairs(mesh_id, name, check_link_names)

        # Track this obstacle
        self.obstacles[name] = mesh_id

        if self.verbose:
            print(
                f"[Collision] Added mesh obstacle '{name}' (ID: {mesh_id}) at position {mesh_placement.translation}"
            )

        return mesh_id

    def update_obstacle_pose(self, name: str, pose: np.ndarray | pin.SE3):
        """
        Update the pose of an existing obstacle.

        DescriptionArgs:
            name: Name of the obstacle to update
            pose: Either a 4x4 homogeneous matrix or pin.SE3 object
        """
        if name not in self.obstacles:
            raise ValueError(
                f"Obstacle '{name}' not found. Use add_box_obstacle(), add_cylinder_obstacle(), or add_mesh_obstacle() to create it first."
            )

        geom_id = self.obstacles[name]

        # Convert pose to SE3 if needed
        if isinstance(pose, np.ndarray):
            new_placement = pin.SE3(pose)
        else:
            new_placement = pose

        # Update the geometry object's placement
        self.collision_model.geometryObjects[geom_id].placement = new_placement

        # Update the collision data's placement
        self.collision_data.oMg[geom_id] = new_placement

        if self.verbose:
            print(
                f"[Collision] Updated obstacle '{name}' to position {new_placement.translation}"
            )
