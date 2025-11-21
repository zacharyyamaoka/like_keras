#!/usr/bin/env python3

"""
GraspPath - Compound visual object for visualizing paths with gripper states.

Shows waypoints with frames, connecting lines, and gripper configurations.
Useful for visualizing pick-and-place trajectories, blended paths, and grasp planning.

GraspPath = Path + grippers

Example:
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist
    from bam.msgs import PoseStamped

    # Create path with parallel gripper
    path = viz.Path(pose_list=[pose1, pose2, pose3])
    gripper = viz.ParallelGripper(grasp_width=0.05)

    grasp_path = viz.GraspPath(
        name="pick_place",
        path=path,
        gripper_opening=[0.08, 0.02, 0.08],
        gripper_props=gripper
    )

    artist.draw(grasp_path)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from .Path import Path

# PYTHON
from dataclasses import dataclass, field
import copy
from typing import Optional, Union, TYPE_CHECKING

if TYPE_CHECKING:
    from visual_objects.compound_objects.ParallelGripper import ParallelGripper
    from visual_objects.compound_objects.ClawGripper import ClawGripper

GripperType = Union["ParallelGripper", "ClawGripper"]


@dataclass
class GraspPath(CompoundVisualObject):
    """Path visualization with gripper states for pick-and-place trajectories.

    Essentially: Path + grippers

    GraspPath = Path + gripper_opening + gripper_props
    - Displays path with frames and lines (via Path)
    - Adds gripper models at each waypoint with varying widths/angles
    """

    # The actual path (holds poses, frames, lines, visual properties)
    path: Path = field(default_factory=Path)

    # Gripper data
    gripper_opening: list[float] = field(
        default_factory=list
    )  # Opening width (m) or angle (rad)

    # Gripper configuration
    gripper_props: Optional[GripperType] = None

    def __post_init__(self):
        super().__post_init__()
        self.build_model()

    def build_model(self):
        """Build the grasp path model geometry based on current parameters."""
        super().build_model()

        if len(self.path.pose_list) == 0:
            return

        # Validate gripper_opening matches pose_list
        assert len(self.gripper_opening) == len(
            self.path.pose_list
        ), f"gripper_opening length ({len(self.gripper_opening)}) must match pose_list length ({len(self.path.pose_list)})"

        # Add path as child (handles frames and lines)
        if self.path is not None:
            # Prefix path name with GraspPath's visual_id (strip leading slash if present)
            path_name = self.path.name.lstrip("/")
            self.path.name = f"{self.visual_id}/{path_name}"

            # Prefix all path children names with GraspPath's visual_id to maintain hierarchy
            for child in self.path.children:
                child_name = child.name.lstrip("/")
                child.name = f"{self.visual_id}/{child_name}"

            self.add(self.path)

        # Add gripper models at each waypoint
        if self.gripper_props is not None:
            self._add_grippers()

    def _add_grippers(self):
        """Create gripper models at each waypoint with corresponding widths."""
        # Import here to avoid circular imports
        from visual_objects.compound_objects.ParallelGripper import ParallelGripper
        from visual_objects.compound_objects.ClawGripper import ClawGripper

        for i, (pose, width) in enumerate(
            zip(self.path.pose_list, self.gripper_opening)
        ):
            # Deep copy the gripper template
            gripper = copy.deepcopy(self.gripper_props)

            # Update the gripper's name and pose
            gripper.name = f"{self.visual_id}/grippers/gripper_{i}"
            gripper.T_world_to_tcp = pose

            # Set the width/angle based on gripper type
            if isinstance(gripper, ParallelGripper):
                gripper.grasp_width = width
            elif isinstance(gripper, ClawGripper):
                gripper.opening_angle = width

            # Rebuild the model with new parameters
            gripper.build_model()

            # Add gripper to children
            self.add(gripper)

    @classmethod
    def from_waypoint_action(
        cls,
        name: str,
        action: "WaypointAction",
        gripper_props: Optional[GripperType] = None,
        path_props: Optional[Path] = None,
    ):
        """Create a Grasppath_props from a WaypointAction.


        Careful the data is passed in as the WaypointAction, but we need to switch it to be held in the path...

        Args:
            name: Name for the visual object
            action: WaypointAction containing waypoints and params
            gripper_props: Optional gripper visual properties (ParallelGripper or ClawGripper)
            path_props: Optional path_props with custom frame/line properties (if None, creates default path_props)
        """
        # Import here to avoid circular dependency
        from bam.msgs import WaypointAction

        # Extract gripper widths from each waypoint's params
        gripper_opening = [param.hand.width for param in action.params]

        # Create path_props from waypoints if not provided
        if path_props is not None:
            path = copy.deepcopy(path_props)
            path.pose_list = (
                action.waypoints
            )  # Keep the visual properties, just add the data
            path.build_model()
        else:
            path = Path(pose_list=action.waypoints)

        return cls(
            name=name,
            path=path,
            gripper_opening=gripper_opening,
            gripper_props=gripper_props,
        )


if __name__ == "__main__":
    from bam.msgs import PoseStamped
    from visual_objects.compound_objects.ParallelGripper import ParallelGripper
    from visual_objects.compound_objects.ClawGripper import ClawGripper
    from visual_objects.markers import Box
    from visual_objects.Frame import Frame
    from visual_objects.LineSegments import LineSegments
    from visual_objects.RGBA import RGBA
    import numpy as np

    # Example 1: Pick-and-place path with parallel gripper
    print("Example 1: Pick-and-place with ParallelGripper")
    poses = [
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.2], [0, 0, 0]),  # Approach
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.05], [0, 0, 0]),  # Pre-grasp
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.05], [0, 0, 0]),  # Grasp
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.2], [0, 0, 0]),  # Lift
        PoseStamped.from_xyzrpy([0.3, 0.0, 0.2], [0, 0, 0]),  # Move
        PoseStamped.from_xyzrpy([0.3, 0.0, 0.05], [0, 0, 0]),  # Place
        PoseStamped.from_xyzrpy([0.3, 0.0, 0.2], [0, 0, 0]),  # Retreat
    ]

    gripper_opening = [
        0.08,
        0.08,
        0.02,
        0.02,
        0.02,
        0.08,
        0.08,
    ]  # Open -> Close -> Open

    # Create path with custom frame properties
    path = Path(
        pose_list=poses,
        frame_props=Frame(axis_length=0.03),
        line_props=LineSegments(colors=[RGBA.grey()], line_width=2.0),
    )

    # Create a template parallel gripper
    gripper = ParallelGripper(
        grasp_width=0.05, box_props=Box(color=RGBA.red()), frame_scale=0.02
    )

    grasp_path = GraspPath(
        name="pick_place",
        path=path,
        gripper_opening=gripper_opening,
        gripper_props=gripper,
    )

    print(f"  Created with {len(grasp_path.children)} children")
    print(f"  GraspPath name: {grasp_path.name}")
    print(f"  GraspPath visual_id: {grasp_path.visual_id}")
    print(f"\n  GraspPath children (all):")
    for child in grasp_path.children:
        print(f"    {child.name}")
    print(f"\n  Path name: {path.name}")
    print(f"  Path's internal children (frames, lines):")
    for child in path.children:
        print(f"    {child.name}")

    # Example 2: Claw gripper path
    print("\nExample 2: Path with ClawGripper")
    claw_gripper = ClawGripper(
        opening_angle=np.pi / 4, box_props=Box(color=RGBA.green()), show_frames=False
    )

    claw_poses = [
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.1], [0, 0, 0]),
        PoseStamped.from_xyzrpy([0.1, 0.0, 0.1], [0, 0, 0]),
        PoseStamped.from_xyzrpy([0.2, 0.0, 0.1], [0, 0, 0]),
    ]

    claw_angles = [np.pi / 3, np.pi / 6, np.pi / 3]  # Open -> Close -> Open

    # Create simple path
    claw_path_obj = Path(pose_list=claw_poses)

    claw_path = GraspPath(
        name="claw_path",
        path=claw_path_obj,
        gripper_opening=claw_angles,
        gripper_props=claw_gripper,
    )

    print(f"  Created with {len(claw_path.children)} children")
    print(f"  GraspPath name: {claw_path.name}")
    print(f"\n  GraspPath children (all):")
    for child in claw_path.children:
        print(f"    {child.name}")
    print(f"\n  Path name: {claw_path_obj.name}")
    print(f"  Path's internal children:")
    for child in claw_path_obj.children:
        print(f"    {child.name}")

    # Example 3: Create GraspPath from WaypointAction
    print("\nExample 3: GraspPath from WaypointAction")
    from bam.msgs import WaypointAction, WaypointParams, HandParams

    # Create waypoint action with waypoints and gripper widths
    action = WaypointAction()
    action.name = "test_action"
    action.waypoints = [
        PoseStamped.from_xyzrpy([0.0, 0.0, 0.1], [0, 0, 0]),
        PoseStamped.from_xyzrpy([0.1, 0.0, 0.1], [0, 0, 0]),
        PoseStamped.from_xyzrpy([0.2, 0.0, 0.1], [0, 0, 0]),
    ]
    action.params = [
        WaypointParams(hand=HandParams(width=0.08)),
        WaypointParams(hand=HandParams(width=0.02)),
        WaypointParams(hand=HandParams(width=0.08)),
    ]

    # Create gripper template
    test_gripper = ParallelGripper(
        grasp_width=0.05, box_props=Box(color=RGBA.blue()), frame_scale=0.015
    )

    # Convert to GraspPath
    action_path = GraspPath.from_waypoint_action(
        name="action_path", action=action, gripper_props=test_gripper
    )

    print(f"  Created from WaypointAction with {len(action_path.children)} children")
    print(f"  GraspPath name: {action_path.name}")
    print(f"  Pose list length: {len(action_path.path.pose_list)}")
    print(f"  Gripper opening: {action_path.gripper_opening}")
    print(f"\n  GraspPath children (all):")
    for child in action_path.children:
        print(f"    {child.name}")
    print(f"\n  Path name: {action_path.path.name}")
    print(f"  Path's internal children:")
    for child in action_path.path.children:
        print(f"    {child.name}")

    print("\n✓ All examples created successfully!")
    print("\nKey Design:")
    print("  - GraspPath = Path + grippers")
    print("  - No code duplication - reuses Path for frames/lines")
    print("  - from_waypoint_action() factory method for easy creation")
