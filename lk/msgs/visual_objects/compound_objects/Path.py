#!/usr/bin/env python3

"""
Path - Compound visual object for visualizing sequences of poses.

Automatically creates Frame objects at each pose and LineSegments connecting them.
This demonstrates the visual object pattern: data + visual properties → rendered primitives.

Example:
    import bam.msgs.visual_objects as viz
    from bam_artist import Artist
    
    # Create path from poses
    path = viz.Path(
        name="robot_path",
        pose_list=[pose1, pose2, pose3],
        frame_props=viz.Frame(axis_length=0.05),
        line_props=viz.LineSegments(colors=[viz.RGBA.blue()], line_width=2.0)
    )
    
    # Draw it
    artist.draw(path)
    
Factory Methods (Future):
    - Path.from_WaypointAction(action) - create from waypoint action data
    - Path.from_Trajectory(traj) - create from trajectory samples
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..Frame import Frame
from ..LineSegments import LineSegments
from ..RGBA import RGBA
from bam.msgs import TransformStamped, Pose, PoseStamped, Transform, PoseType

# PYTHON
from dataclasses import dataclass, field
import copy

@dataclass
class Path(CompoundVisualObject):
    """Visual representation of a sequence of poses as frames + connecting lines."""

    # Data
    pose_list: list[PoseType] = field(default_factory=list)

    # Viz properties. We use the classes as a container for the properties.
    frame_props: Frame = field(default_factory=Frame)
    line_props: LineSegments = field(default_factory=LineSegments)

    def __post_init__(self):
        super().__post_init__()
        self.build_model()
    
    def build_model(self):
        """Build the path model geometry based on current parameters."""
        super().build_model()
        
        # We automatically populate the children of the CompoundVisualObject based on the data + viz properties.
        
        # Add frames for each pose
        if self.frame_props is not None:
            for i, pose in enumerate(self.pose_list):
                frame = copy.deepcopy(self.frame_props)
                frame.name = f"{self.visual_id}/frame_{i}"
                frame.transform = pose
                self.add(frame)

        # Add line segments connecting the self.pose_list
        if self.line_props is not None and len(self.pose_list) >= 2:
            # Extract positions from self.pose_list
            points = []
            for i in range(len(self.pose_list) - 1):
                p1 = self.pose_list[i]
                p2 = self.pose_list[i + 1]
                points.extend([p1.xyz.tolist(), p2.xyz.tolist()]) # this will work on any type that supports this,

            # num. line segments = (len(self.pose_list) - 1)
            # len(points) == (len(self.pose_list) - 1) * 2 

            n_colors = len(self.line_props.colors)

            if n_colors == 0:
                colors = [RGBA.grey()] * len(points)
            elif n_colors == 1:
                colors = self.line_props.colors * len(points)
            else:
                assert n_colors == len(points), "Number of colors must be equal to the number of points"
                colors = self.line_props.colors
    
            line_segments = copy.deepcopy(self.line_props)
            line_segments.name = f"{self.visual_id}/lines"
            line_segments.points = points
            line_segments.colors = colors

            self.add(line_segments)


if __name__ == "__main__":
    from bam.msgs import Transform, Vector3, Quaternion

    # Example usage
    pose_list = []
    for i in range(2):
        tf = TransformStamped()
        tf.transform.translation = Vector3(x=i * 0.1, y=i * 0.05, z=0.0)
        tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_list.append(tf)

    path = Path(
        name="test_path",
        pose_list=pose_list,
        frame_props=Frame(axis_length=0.05),
        line_props=LineSegments(colors=[RGBA.red()], line_width=3.0),
        visible=False
    )

    print(path)
    # print(path.visual_id)
    # path.dump_to_yaml()
