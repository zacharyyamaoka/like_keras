#!/usr/bin/env python3

"""
    ParallelGripper - Compound visual object for parallel jaw grippers.
    
    Creates a parallel gripper model with configurable geometry and visual properties.
    Useful for visualizing grasps, gripper states, and pick-and-place operations.
    
    Example:
        import bam.msgs.visual_objects as viz
        from bam_artist import Artist
        from bam.msgs import PoseStamped
        
        # Create basic parallel gripper
        gripper = viz.ParallelGripper(
            name="gripper",
            T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0, 0.1], [0, 0, 0]),
            grasp_width=0.05,
            box_props=viz.Box(color=viz.RGBA.red())
        )
        
        artist.draw(gripper)
        
        # Create skeleton gripper (thinner)
        skeleton = viz.ParallelGripper.make_skeleton(
            name="skeleton_gripper",
            T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0, 0.1], [0, 0, 0]),
            grasp_width=0.03,
            thickness_scale=1.0
        )
        
        artist.draw(skeleton)
"""

# BAM
from ..CompoundVisualObject import CompoundVisualObject
from ..markers import Box
from ..Frame import Frame
from ..RGBA import RGBA
from ..Material import Material
from bam.msgs import PoseStamped, PoseType

# PYTHON
from dataclasses import dataclass, field
import copy
import numpy as np
from typing import Literal, Optional
from tf_transformations import xyzrpy_to_matrix, xyzrpy_offset


@dataclass
class ParallelGripper(CompoundVisualObject):
    """Parallel jaw gripper visualization with configurable geometry.
    
    Creates a gripper model with:
    - Two parallel fingers (left and right)
    - Palm connecting the fingers
    - Wrist/handle extending from palm
    - Optional coordinate frame
    
    The gripper opens along the configured axis (default X).
    
    Note: name and T_world_to_tcp are optional - useful for creating templates
    that will be cloned and positioned later.
    """
    
    # Data
    T_world_to_tcp: PoseType = field(default_factory=PoseStamped)
    grasp_width: float = 0.05  # Distance between finger tips in meters
    
    # Geometry configuration
    thickness: float = 0.01
    finger_height: float = 0.06
    finger_width: float = 0.02
    palm_thickness: float = 0.01
    
    # Visual properties
    box_props: Box = field(default_factory=lambda: Box(color=RGBA.red()))
    
    # Gripper options
    frame_scale: float = 0.025  # Size of coordinate frame (0 to disable)
    z_into_table: bool = False  # If True, fingers point down
    opening_axis: Literal['x', 'y'] = 'x'  # Axis along which gripper opens
    
    def __post_init__(self):
        super().__post_init__()
        self.build_model()
    
    def build_model(self):
        """Build the gripper model geometry based on current parameters."""
        super().build_model()
        
        # Handle edge case
        if self.grasp_width == 0:
            self.grasp_width = 0.00001
        
        # Get z direction
        z_dir = -1 if self.z_into_table else 1
        
        # Get base transform (with optional rotation for opening axis)
        T_world_to_tcp = self.T_world_to_tcp.to_matrix()
        if self.opening_axis == 'y':
            T_world_to_tcp = xyzrpy_offset(T_world_to_tcp, rpy=(0, 0, np.pi/2), local=True)
        
        # Create right finger
        T_gripper_finger_r = xyzrpy_to_matrix(
            [self.grasp_width/2 + self.thickness/2, 0, z_dir*self.finger_height/2],
            [0, 0, 0]
        )
        T_world_finger_r = T_world_to_tcp @ T_gripper_finger_r
        finger_r = self._create_box(
            f"{self.visual_id}/finger_right",
            T_world_finger_r,
            [self.thickness, self.finger_width, self.finger_height]
        )
        self.add(finger_r)
        
        # Create left finger
        T_gripper_finger_l = xyzrpy_to_matrix(
            [-self.grasp_width/2 - self.thickness/2, 0, z_dir*self.finger_height/2],
            [0, 0, 0]
        )
        T_world_finger_l = T_world_to_tcp @ T_gripper_finger_l
        finger_l = self._create_box(
            f"{self.visual_id}/finger_left",
            T_world_finger_l,
            [self.thickness, self.finger_width, self.finger_height]
        )
        self.add(finger_l)
        
        # Create palm
        T_gripper_palm = xyzrpy_to_matrix(
            [0, 0, z_dir*(self.finger_height + self.palm_thickness/2)],
            [0, 0, 0]
        )
        T_world_palm = T_world_to_tcp @ T_gripper_palm
        palm = self._create_box(
            f"{self.visual_id}/palm",
            T_world_palm,
            [self.grasp_width, self.finger_width, self.palm_thickness]
        )
        self.add(palm)
        
        # Create wrist/handle
        wrist_height = self.finger_height * 0.2
        T_gripper_wrist = xyzrpy_to_matrix(
            [0, 0, z_dir*(self.finger_height + self.palm_thickness + wrist_height/2)],
            [0, 0, 0]
        )
        T_world_wrist = T_world_to_tcp @ T_gripper_wrist
        wrist = self._create_box(
            f"{self.visual_id}/wrist",
            T_world_wrist,
            [self.finger_width, self.finger_width, wrist_height]
        )
        self.add(wrist)
        
        # Add coordinate frame if enabled
        if self.frame_scale > 0.0:
            frame = Frame(
                name=f"{self.visual_id}/frame",
                transform=self.T_world_to_tcp,
                axis_length=self.frame_scale
            )
            self.add(frame)
    
    def _create_box(self, name: str, transform: np.ndarray, scale: list[float]) -> Box:
        """Create a box with the specified visual properties."""
        box = copy.deepcopy(self.box_props)
        box.name = name
        box.pose = PoseStamped.from_matrix(transform)
        box.scale = scale
        return box
    
    @classmethod
    def make_skeleton(
        cls,
        name: str = "",
        T_world_to_tcp: Optional[PoseType] = None,
        grasp_width: float = 0.03,
        thickness_scale: float = 1.0,
        box_props: Optional[Box] = None,
        frame_scale: float = 0.025,
        z_into_table: bool = False,
        opening_axis: Literal['x', 'y'] = 'x'
    ) -> 'ParallelGripper':
        """Create a thin skeleton gripper for lightweight visualization.
        
        Args:
            name: Name for the gripper (auto-generated if empty)
            T_world_to_tcp: Transform from world to gripper TCP (defaults to origin if None)
            grasp_width: Opening width between fingers
            thickness_scale: Scale factor for thickness (default 1.0)
            box_props: Optional Box template for visual properties
            frame_scale: Size of coordinate frame (0 to disable)
            z_into_table: If True, fingers point down
            opening_axis: Axis along which gripper opens
            
        Returns:
            ParallelGripper with thin skeleton geometry
        """
        # Use provided template or create default
        if box_props is None:
            box_props = Box(color=RGBA.red())
        
        if T_world_to_tcp is None:
            T_world_to_tcp = PoseStamped()
        
        return cls(
            name=name,
            T_world_to_tcp=T_world_to_tcp,
            grasp_width=grasp_width,
            thickness=thickness_scale * 0.0025,
            finger_height=thickness_scale * 0.075,
            finger_width=thickness_scale * 0.0025,
            palm_thickness=thickness_scale * 0.0025,
            box_props=box_props,
            frame_scale=frame_scale,
            z_into_table=z_into_table,
            opening_axis=opening_axis
        )


if __name__ == "__main__":
    from bam.msgs import PoseStamped
    
    # Example 1: Standard parallel gripper
    print("Example 1: Standard parallel gripper")
    gripper1 = ParallelGripper(
        name="standard_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0, 0.1], [0, 0, 0]),
        grasp_width=0.05,
        box_props=Box(color=RGBA.red())
    )
    print(f"  Created with {len(gripper1.children)} children")
    
    # Example 2: Skeleton gripper
    print("\nExample 2: Skeleton gripper (thin)")
    gripper2 = ParallelGripper.make_skeleton(
        name="skeleton_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0, 0.1], [0, 0, 0]),
        grasp_width=0.03,
        thickness_scale=1.0,
        box_props=Box(color=RGBA.green())
    )
    print(f"  Created with {len(gripper2.children)} children")
    
    # Example 3: Gripper with different opening width
    print("\nExample 3: Wide open gripper")
    gripper3 = ParallelGripper(
        name="wide_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, 0.2, 0.1], [0, 0, 0]),
        grasp_width=0.08,
        box_props=Box(color=RGBA.blue())
    )
    print(f"  Created with grasp width: {gripper3.grasp_width}m")
    
    # Example 4: Gripper with Y-axis opening
    print("\nExample 4: Y-axis opening gripper")
    gripper4 = ParallelGripper(
        name="y_axis_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0, -0.2, 0.1], [0, 0, 0]),
        grasp_width=0.04,
        opening_axis='y',
        box_props=Box(color=RGBA.cyan())
    )
    print(f"  Created with opening_axis: {gripper4.opening_axis}")
    
    # Example 5: Gripper pointing down
    print("\nExample 5: Gripper pointing down (z_into_table=True)")
    gripper5 = ParallelGripper(
        name="down_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, 0.2, 0.1], [0, 0, 0]),
        grasp_width=0.03,
        z_into_table=True,
        box_props=Box(color=RGBA.magenta())
    )
    print(f"  Created with z_into_table: {gripper5.z_into_table}")
    
    # Example 6: Custom geometry
    print("\nExample 6: Custom geometry")
    gripper6 = ParallelGripper(
        name="custom_gripper",
        T_world_to_tcp=PoseStamped.from_xyzrpy([0.2, -0.2, 0.1], [0, 0, 0]),
        grasp_width=0.06,
        thickness=0.015,
        finger_height=0.08,
        finger_width=0.03,
        palm_thickness=0.015,
        box_props=Box(color=RGBA.yellow())
    )
    print(f"  Created with custom geometry: thickness={gripper6.thickness}m")
    
    print("\n✓ All examples created successfully!")

