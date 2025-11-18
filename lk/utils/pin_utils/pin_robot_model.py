

# BAM

from ..xacro_utils import xacrodoc_from_file, xml_from_xacro
from ..tempfile_utils import temp_urdf_file

# PYTHON
import pinocchio as pin
import numpy as np
from xacrodoc import XacroDoc
import itertools
from typing import Optional, TYPE_CHECKING
import copy

from pinocchio.robot_wrapper import RobotWrapper

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription

# all the cool things you can do with a Pin URDF Model...

"""
Refactored from a long list of random functions.

Workflow is to load in a pin model from a variety of sources, then you can do a bunch of things with it...

The issue is that the pin.Model is not really that pythonic, so this is a wrapper

Ok This is just a light weight wrapper that provides basic URDF functions...
"""

def transpose_q_by_name(new_name_to_joint_index: dict[str, int], new_q: np.ndarray, name_to_joint_index_old: dict[str, int], old_q: np.ndarray) -> np.ndarray:
    for name, index in new_name_to_joint_index.items():
        if name in name_to_joint_index_old:
            new_q[index] = old_q[name_to_joint_index_old[name]]
    return new_q


class PinRobotModel():

    # CONSTRUCTORS

    @classmethod
    def from_robot_description(cls, description: 'RobotDescription', mimic=True, verbose=False):
        rd = description
        # Config file is automatically dumped when needed via get_config_file() in to_urdf_xml()

        # print("xacro_path: ", rd.urdf.xacro_path)
        # print("mesh_dirs: ", rd.urdf.mesh_dirs)
        # print("to_xacro_args: ")

        # for key, value in rd.to_xacro_args().items():
        #     print(f"{key}: {value}")

        with rd.get_temp_urdf_path() as urdf_path:
            klass = cls.from_urdf(urdf_path, rd.urdf.abs_package_dirs, mimic=mimic, verbose=verbose)

        # if rd.lock_joints:
        #     return klass.robot_wrapper.buildReducedRobot(rd.get_lock_joint_names(), np.array(rd.q_locked))
        
        return klass

    @classmethod
    def from_urdf_xml(cls, urdf_xml: str, mimic=True):
        model = pin.buildModelFromXML(urdf_xml)
        robot_wrapper = RobotWrapper(model)
        return cls(robot_wrapper)

    @classmethod
    def from_xacro(cls, xacro_path: str, xacro_args: dict, abs_package_dirs: dict[str, str], mimic=True, resolve_packages=False, verbose=False):

        urdf_xml = xml_from_xacro(xacro_path, xacro_args, resolve_packages, abs_package_dirs)

        with temp_urdf_file(urdf_xml) as urdf_path:
            return cls.from_urdf(urdf_path, abs_package_dirs, mimic=mimic, verbose=verbose)
  

    @classmethod
    def from_urdf(cls, urdf_path: str, abs_package_dirs: dict[str, str], mimic=True, verbose=False):

        package_dirs = list(abs_package_dirs.values())
        model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, package_dirs, mimic=mimic, verbose=verbose)
        robot_wrapper = RobotWrapper(model, collision_model, visual_model)
        return cls(robot_wrapper)


    def __init__(self, robot_wrapper: RobotWrapper):
        self.robot_wrapper = robot_wrapper

        self.model = self.robot_wrapper.model
        self.collision_model = self.robot_wrapper.collision_model
        self.visual_model = self.robot_wrapper.visual_model

        self.data = self.robot_wrapper.data

        self.name_to_joint_index = self.get_name_to_joint_index()

        self.n_dof = self.robot_wrapper.nq

    @property
    def upper_limits(self) -> np.ndarray:
        return self.model.upperPositionLimit

    @property
    def lower_limits(self) -> np.ndarray:
        return self.model.lowerPositionLimit

    @property
    def root_link(self) -> str:
        return self.model.frames[1].name

    @property
    def frame_names(self) -> list[str]:
        return [frame.name for frame in self.model.frames]

    def get_fixed_joint_names(self) -> list[str]:
        """Get list of all fixed joint names (FIXED_JOINT type)."""
        fixed_joint_names = []
        for frame in self.model.frames:
            if frame.type == pin.FrameType.FIXED_JOINT:
                fixed_joint_names.append(frame.name)
        return fixed_joint_names
    
    def get_rotating_joint_names(self) -> list[str]:
        """Get list of all rotating joint names (JOINT type - revolute, continuous, prismatic)."""
        rotating_joint_names = []
        for frame in self.model.frames:
            if frame.type == pin.FrameType.JOINT:
                rotating_joint_names.append(frame.name)
        return rotating_joint_names
    
    def get_all_joint_names(self) -> list[str]:
        """Get list of all joint names (both rotating and fixed joints)."""
        return self.get_rotating_joint_names() + self.get_fixed_joint_names()
    
    def get_joint_frame_names(self) -> list[str]:
        """Get list of all joint frame names (both JOINT and FIXED_JOINT types).
        
        Alias for get_all_joint_names() for backwards compatibility.
        """
        return self.get_all_joint_names()

    def get_link_frame_names(self) -> list[str]:
        """Get list of all link/body frame names (BODY type)."""
        link_frames = []
        for frame in self.model.frames:
            if frame.type == pin.FrameType.BODY:
                link_frames.append(frame.name)
        return link_frames
    
    def get_all_link_names(self) -> list[str]:
        """Get list of all link names (BODY type).
        
        Alias for get_link_frame_names() for consistency with get_all_joint_names().
        """
        return self.get_link_frame_names()

    def get_operational_frame_names(self) -> list[str]:
        """Get list of all operational frame names (OP_FRAME type, e.g., end-effector frames)."""
        op_frames = []
        for frame in self.model.frames:
            if frame.type == pin.FrameType.OP_FRAME:
                op_frames.append(frame.name)
        return op_frames


    # FRAMES
    def frames_exists(self, frame_names: str | list[str]) -> bool:

        if isinstance(frame_names, str):
            frame_names = [frame_names]
        
        found_results = []
        for frame_name in frame_names:
            if self.model.existFrame(frame_name):
                print(f"Frame {frame_name} found in model")
                found_results.append(True)
            else:
                found_results.append(False)

        return all(found_results)
    
    def update_frames(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def get_frame_pose(self, frame_name: str, q: np.ndarray) -> np.ndarray:
        return self.get_transform_between_frames(self.root_link, frame_name, q)

    def get_rotating_joint_placement(self, joint_name: str) -> np.ndarray:
        """
        Get the static placement transform for a rotating joint.
        
        This returns the fixed transform from parent link to child link as defined 
        in the URDF for rotating joints (revolute, continuous, prismatic).
        
        Args:
            joint_name: Name of the rotating joint
            
        Returns:
            4x4 homogeneous transformation matrix for the joint placement
        """
        if not self.model.existJointName(joint_name):
            raise ValueError(f"Joint '{joint_name}' does not exist in the joint model")
        
        joint_id = self.model.getJointId(joint_name)
        return self.model.jointPlacements[joint_id].homogeneous
    
    def get_transform_between_frames(self, frame_a: str, frame_b: str, q_update: np.ndarray = None) -> np.ndarray:
        """
            Compute transform from frame_a to frame_b.
            
            DescriptionArgs:
                frame_a: Source frame name
                frame_b: Target frame name  
                q_update: Optional joint configuration. If provided, updates frames before computing transform.
                         If None, assumes frames are already updated (use update_frames() separately).
            
            Returns:
                4x4 homogeneous transformation matrix from frame_a to frame_b
        """
        
        # Update frames if configuration provided
        if q_update is not None:
            self.update_frames(q_update)

        # Check if frames exist
        if not self.model.existFrame(frame_a):
            raise ValueError(f"Frame '{frame_a}' not found in model")
        if not self.model.existFrame(frame_b):
            raise ValueError(f"Frame '{frame_b}' not found in model")

        # Get frame indices
        idx_a = self.model.getFrameId(frame_a)
        idx_b = self.model.getFrameId(frame_b)

        # Get transforms
        T_world_a = self.data.oMf[idx_a]
        T_world_b = self.data.oMf[idx_b]

        # Transform from a to b
        T_a_b = T_world_a.inverse() * T_world_b

        T = np.eye(4)
        T[:3, :3] = T_a_b.rotation
        T[:3, 3] = T_a_b.translation

        return T


    # JOINTS
    def get_mimic_joint_indicies(self) -> set[int]:
        """Get set of mimic joint indices."""
        mimic_joint_indices = set()
        for joint_id in self.model.mimicking_joints:
            mimic_joint_indices.add(joint_id)
        return mimic_joint_indices

    def get_name_to_joint_index(self, del_universe=True, actuated=True, verbose=False) -> dict[str, int]:

        joint_names = [name for name in self.model.names]
        
        # Filter out mimic joints if actuated=True
        if actuated:
            mimic_joint_indices = self.get_mimic_joint_indicies()
            
            joint_names = [name for idx, name in enumerate(joint_names) if idx not in mimic_joint_indices]

 

        name_to_joint_index = {name: idx-1 for idx, name in enumerate(joint_names)}
        
        if del_universe:
            name_to_joint_index.pop('universe', None)
        
        if verbose:
            for name, index in name_to_joint_index.items():
                print(f"{name}: {index}")

        return name_to_joint_index

    def get_joint_names(self, actuated: bool = True) -> list[str]:
        
        return list(self.get_name_to_joint_index(actuated=actuated).keys())

    def order_q_by_name(self, joint_names: list[str], q: np.ndarray) -> np.ndarray:

        q_ordered = np.full(self.n_dof, np.nan)


        for i, name in enumerate(joint_names):

            if name not in self.name_to_joint_index:
                raise ValueError(f"Joint {name} not found in name_to_joint_index")

            idx = self.name_to_joint_index[name]
            q_ordered[idx] = q[i]

        return q_ordered


    def get_q_limits(self, margin: float = 1e-3) -> tuple[np.ndarray, np.ndarray]:

        lower = self.model.lowerPositionLimit.copy()
        upper = self.model.upperPositionLimit.copy()

        lower += margin
        upper -= margin

        return lower, upper

    def get_q_neutral(self) -> np.ndarray:
        return pin.neutral(self.model)  # or set manually, e.g. np.array([...])

    def get_q_random(self) -> np.ndarray:
        lower, upper = self.get_q_limits()

        # Sample within the reduced bounds
        q = pin.randomConfiguration(self.model, lower, upper)
        # BUG: If you wrap you can go outside the limits!!!
        # q_wrapped = (q + np.pi) % (2 * np.pi) - np.pi # [-pi, pi]
        return q

    def get_q_grid(self, step_size_deg=5.) -> list[np.ndarray]:
        
        lower, upper = self.get_q_limits()

        print("Generating q grid")
        print("Lower limits: ", lower)
        print("Upper limits: ", upper)

        q_grid = []

        joint_ranges = []
        for i in range(self.model.nq):
            step_size_rad = np.deg2rad(step_size_deg)
            joint_ranges.append(np.arange(lower[i], upper[i]+step_size_rad, step_size_rad)) # inclusive of upper limit
            print(f"[{i}] Joint range: {joint_ranges[-1]}")

        q_grid = [np.array(combo) for combo in itertools.product(*joint_ranges)]

        print("len(q_grid): ", len(q_grid))

        return q_grid

    # MISC.

    def bbox_bounds(self, ik_tip:str, n_samples=1000, scale: float = 1.1) -> tuple[np.ndarray, np.ndarray]:
        data = self.model.createData()
        
        # Sample 1000 random joint configurations
        positions = []

        q_grid = self.get_q_grid(step_size_deg=90.0)
        
        frame_id = self.model.getFrameId(ik_tip)

        for q in q_grid:
            pin.forwardKinematics(self.model, data, q)
            pin.updateFramePlacements(self.model, data)
            
            # Collect all frame positions
            positions.append(copy.deepcopy(data.oMf[frame_id].translation))
        
        positions = np.array(positions)
        
        # Find min/max for x, y, z
        x_min, y_min, z_min = positions.min(axis=0)
        x_max, y_max, z_max = positions.max(axis=0)

        # print("Positions: ", positions)
        print("Min positions: ", x_min, y_min, z_min)
        print("Max positions: ", x_max, y_max, z_max)
        
        # Calculate center
        x_center = (x_min + x_max) / 2.0
        y_center = (y_min + y_max) / 2.0
        z_center = (z_min + z_max) / 2.0
        center = np.array([x_center, y_center, z_center])
        
        # Calculate extents (half-widths)
        x_extent = (x_max - x_min) * scale
        y_extent = (y_max - y_min) * scale
        z_extent = (z_max - z_min) * scale
        extents = np.array([x_extent, y_extent, z_extent])
        
        return center, extents

    # INSPECT

    def inspect(self, print_model=True, print_joints=True, print_frames=True, print_inertias=True):
        if print_model:
            self.print_model_info()

        if print_joints:
            self.print_joint_info()

        if print_frames:
            self.print_frame_info()

        if print_inertias:
            self.print_interia_info()

    def print_joint_info(self):
        model = self.model

        print(f"\nJoints (n = {model.njoints}):")
        for i, joint in enumerate(model.joints):
            joint_name = model.names[i]
            parent_id = model.parents[i]
            print(f"  [{i}] {joint_name}, type: {joint.shortname()}, parent id: {parent_id}")

        data = model.createData()
        q = pin.neutral(model) 
        print("Joint Positions: ", q)
        pin.forwardKinematics(model, data, q) # set starting position
        print(("[i] {:<24} :  Position xyz".format( "Joint Name")))
        for i in range(model.njoints):
            print(("[{}] {:<24} : {: .4f} {: .4f} {: .4f}".format( i, model.names[i], *data.oMi[i].translation.T.flat )))


    def print_frame_info(self):
        model = self.model
        print(f"\nFrames (n = {model.nframes}):")
        for i, frame in enumerate(model.frames):
            print(f"  Frame {i}: {frame.name}, type: {frame.type}, parent: {frame.parentJoint}")
            
    def print_interia_info(self):
        model = self.model
        print("\n Joint (Inertias):")
        for i in range(0, len(model.inertias)):  # skip root (0)
            link_name = model.names[i]
            # link_name = model.frames[i].name  # Get link name from frames instead of links

            inertia = model.inertias[i]
            mass_g = inertia.mass * 1000  # kg → g
            com_mm = inertia.lever * 1000  # m → mm

            # Format mass and com
            mass_str = f"{mass_g:.2f}"

            com_vals = [f"{val:.2f}" for val in com_mm]
            com_str = "[" + ", ".join(com_vals) + "]"

            # Format inertia matrix manually to control spacing
            inertia_matrix_str = ""
            for row in inertia.inertia:
                row_str = "\n    [" + ", ".join(f"{val:.6f}" for val in row) + "]"
                inertia_matrix_str += row_str

            print(f"\n  Joint {i} ({link_name}) inertia:")
            print(f"    mass (g)        = {mass_str}")
            print(f"    com (mm)        = {com_str}")
            print(f"    inertia (kg·m²) = \n{inertia_matrix_str}")

    def print_model_info(self):
        model = self.model
        data = model.createData()

        print("Model Name      :", model.name)
        print("Gravity         :", model.gravity.linear)

        print("\nWeight (kg):")
        total_weight_kg = 0 
        for i in range(0, len(model.inertias)): 
            inertia = model.inertias[i]
            body_weight = inertia.mass  # Mass in kg
            total_weight_kg += body_weight  # Sum mass for total weight
            link_name = model.names[i]  # Get the body name (link name)
            print(f"  {link_name}: {body_weight:.2f}")  # Print each body's weight
        print(f"  --\n  Total: {total_weight_kg:.2f}")  # Print the total weight

        print("\nCounts:")
        print(f"  Joints         : {model.njoints}")
        print(f"  Bodies         : {model.nbodies}")
        print(f"  Frames         : {model.nframes}")
        print(f"  Position (dof) : {model.nq}")
        print(f"  Velocity (dof) : {model.nv}")
        print(f"  Effort (dof)   : {data.tau.shape[0]}") # access via data as self.tau may not be set yet or have correct dof


