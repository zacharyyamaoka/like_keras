#! /usr/bin/env python3


# BAM
# PYTHON
import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from bam.msgs.ros_msgs import TransformStamped
from bam.utils import (
    ConfigMixin,
    generate_srdf_from_urdf,
    temp_srdf_file,
    temp_urdf_file,
)

from .composite_description_helper import generate_composite_xml_body
from .description_types import (
    JointDescription,
    LinkDescription,
    PointOfInterest,
    RobotInfo,
    SrdfInfo,
    UrdfInfo,
)
from .joint_positions import JointPositions

# from pin import inspect


@dataclass
class DescriptionArgs:
    reflect: int = 1
    plugin: str = "none"
    world_xyz: tuple[float, float, float] = (0, 0, 0)
    world_rpy: tuple[float, float, float] = (0, 0, 0)
    base_xyz: tuple[float, float, float] = (0, 0, 0)
    base_rpy: tuple[float, float, float] = (0, 0, 0)

    def __post_init__(self): ...


"""
We want a very light weight class that can support both explicit field construction and also maintain a seperate dict.
Each of thoose NamedEntities
"""
DescriptionEntity = JointDescription | LinkDescription


@dataclass
class NamedEntities:
    entities: dict[str, DescriptionEntity] = field(default_factory=dict)

    # Subclasses must set this as a class variable (not a dataclass field)
    def __post_init__(self) -> None:
        # If you added any fields
        for field_name, field_value in self.__dict__.items():
            if hasattr(field_value, "name") and hasattr(field_value, "unprefixed_name"):
                field_value.unprefixed_name = field_name
                field_value.name = field_name
                self.entities[field_value.name] = field_value

    def prefix(self, prefix: str) -> None:
        prefixed_entities = {}
        for entity in self.entities.values():
            entity.prefixes.append(prefix)
            entity.name = "".join(entity.prefixes) + entity.unprefixed_name
            prefixed_entities[entity.name] = entity
        self.entities = prefixed_entities

    def __iter__(self):
        return iter(self.entities.values())

    @property
    def names(self) -> list[str]:
        return [entity.name for entity in self]

    @staticmethod
    def combine(named_entities: list["NamedEntities"]) -> "NamedEntities":
        """
        Combine multiple Links instances, validating name uniqueness.

        Can be called as static method:
            combined = NamedEntities.combine(entities1, entities2, entities3)
            combined = NamedEntities.combine([entities1, entities2, entities3])
        """
        first_instance_type = type(named_entities[0])
        combined_dict = {}
        for instance in named_entities:
            for entity in instance.entities.values():
                if entity.name in combined_dict:
                    raise ValueError(
                        f"Entity name {entity.name} already exists in combined dictionary"
                    )
                combined_dict[entity.name] = entity

        return first_instance_type(entities=combined_dict)

    def extend(self, named_entities: list["NamedEntities"]) -> "NamedEntities":
        """
        Combine this Links instance with others, validating name uniqueness.

        Can be called as instance method:
            combined = entities1.combine(entities2, entities3)
        """
        return NamedEntities.combine(self, *named_entities)

    def add(self, entity: DescriptionEntity) -> None:
        if entity.name in self.entities:
            raise ValueError(f"Entity name {entity.name} already exists")
        self.entities[entity.name] = entity


@dataclass
class DescriptionTags:
    # Generic
    root_link: str = "root_link"
    base_link: str = "base_link"

    # Arm
    base_mount: str = "base_mount_link"
    ik_base: str = "ik_base_link"
    ik_tip: str = "ik_tip"
    ik_tip_flip: str = "ik_tip_flip"
    arm_to_hand_mount: str = "arm_to_hand_mount"

    # Hand
    tcp_tool: str = (
        "tcp_tool"  # similar to tool0 frame, in that Z pointing away from palm
    )
    tcp_world: str = "tcp_world"  # Z pointing upwards, into palm. Matches orientation of object you want to pick up
    tcp_world_floating: str = "tcp_world_floating"  # Optional frame that moves with opening of hand, in case you want to change arm alignment
    hand_to_arm_mount: str = "hand_to_arm_mount"


TAGS = DescriptionTags()


@dataclass
class Links(NamedEntities):
    entities: dict[str, LinkDescription] = field(default_factory=dict)

    def find_by_tag(
        self, tag: str, unique: bool = True
    ) -> LinkDescription | list[LinkDescription]:
        matching_links = [link for link in self if tag in link.tags]

        if unique:
            if len(matching_links) > 1:
                raise ValueError(
                    f"Multiple links: {matching_links} found with tag {tag}"
                )
            return matching_links[0]
        else:
            return matching_links

    def get_display_link_names(self) -> list[str]:
        """Get names of all links that have display=True.

        Returns:
            list[str]: List of link names with display property set to True
        """
        return [link.name for link in self if link.display]

    def set_simple_collision(self) -> "Links":
        link: LinkDescription
        for link in self:
            link.set_simple_collision()
        return self


@dataclass
class Joints(NamedEntities):
    entities: dict[str, JointDescription] = field(default_factory=dict)

    def get_actuated_joints(self) -> list[JointDescription]:
        return [joint for joint in self if joint.is_actuated]

    def get_actuated_joint_names(self, prefix=True) -> list[str]:
        return [
            joint.name if prefix else joint.unprefixed_name
            for joint in self.get_actuated_joints()
        ]

    def get_specified_joints(self) -> list[JointDescription]:
        """
        Get all joints that have explicitly defined transforms.

        Returns a list of JointDescription objects for joints where the transform
        has been set (both parent and child frame_ids are defined).
        """
        specified_joints = []
        for joint in self:
            # Check if transform exists and has been explicitly set
            if joint.transform is None:
                continue

            # A valid transform needs both parent frame_id and child_frame_id
            if joint.transform.header.frame_id and joint.transform.child_frame_id:
                specified_joints.append(joint)

        return specified_joints

    def get_joint_transforms(self) -> dict[str, TransformStamped]:
        """
        Get all joints that have explicitly defined transforms.

        Returns dict mapping joint name to TransformStamped for all joints
        where the transform has been set (not None/default).

        This is useful for verification to check that Python-specified transforms
        match what ends up in the generated URDF.
        """
        joint_transforms = {}
        for joint in self:
            # Check if transform exists and has been explicitly set
            if joint.transform is None:
                continue

            # A valid transform needs both parent frame_id and child_frame_id
            if joint.transform.header.frame_id and joint.transform.child_frame_id:
                joint_transforms[joint.name] = joint.transform

        return joint_transforms


# Design is to only have a single RobotDescription at a time, which represents a single piece of hardware (could be quite large)


def xacro_template(name: str, body: str) -> str:
    return f"""
    <robot name="{name}">
        {body}
    </robot>
    """


@dataclass
class RobotDescription(ConfigMixin):
    info: RobotInfo = field(default_factory=RobotInfo)

    joints: Joints = field(default_factory=Joints)
    links: Links = field(default_factory=Links)

    joint_positions: JointPositions = field(default_factory=JointPositions)
    points_of_interest: list[PointOfInterest] = field(
        default_factory=list
    )  # TODO make this easier to look up?

    urdf: UrdfInfo = field(default_factory=UrdfInfo)
    srdf: SrdfInfo = field(default_factory=SrdfInfo)
    config_file: str = (
        ""  # Path to config file, set via info.get_file_path("_config.yaml")
    )

    args: DescriptionArgs = field(
        default_factory=DescriptionArgs
    )  # Unique args for each robot

    children: list["RobotDescription"] = field(default_factory=list)

    # Builds Flags
    hash_file_name: bool = False
    generate_py_xml: bool = (
        False  # If True, generate XML programmatically from links/joints
    )

    @classmethod
    def from_entities(
        cls,
        entities: list[LinkDescription | JointDescription],
        robot_info: RobotInfo | None = None,
    ) -> "RobotDescription":
        links = Links()
        joints = Joints()

        for entity in entities:
            if isinstance(entity, LinkDescription):
                links.add(entity)
            elif isinstance(entity, JointDescription):
                joints.add(entity)
            else:
                raise ValueError(f"Invalid entity type: {type(entity)}")

        if robot_info is None:
            robot_info = RobotInfo()

        return cls(
            info=robot_info,
            links=links,
            joints=joints,
            generate_py_xml=True,  # Descriptions from entities are Python-generated
        )

    @classmethod
    def from_urdf_xml(
        cls, urdf_xml: str, robot_name: str | None = None
    ) -> "RobotDescription":
        """Create RobotDescription from URDF XML string.

        Args:
            urdf_xml: URDF XML string
            robot_name: Optional name override for robot

        Returns:
            RobotDescription instance
        """
        from .urdf_converter import from_urdf_string

        links_list, joints_list, robot_info = from_urdf_string(urdf_xml, robot_name)

        # Create Links and Joints containers
        links = Links()
        for link in links_list:
            links.add(link)

        joints = Joints()
        for joint in joints_list:
            joints.add(joint)

        return cls(
            info=robot_info,
            links=links,
            joints=joints,
            generate_py_xml=True,  # URDF imports generate Python representation
        )

    @classmethod
    def from_urdf_file(
        cls, urdf_path: str | Path, robot_name: str | None = None
    ) -> "RobotDescription":
        """Create RobotDescription from URDF file.

        Args:
            urdf_path: Path to URDF file
            robot_name: Optional name override for robot

        Returns:
            RobotDescription instance
        """
        from .urdf_converter import from_urdf_file

        links_list, joints_list, robot_info = from_urdf_file(urdf_path, robot_name)

        # Create Links and Joints containers
        links = Links()
        for link in links_list:
            links.add(link)

        joints = Joints()
        for joint in joints_list:
            joints.add(joint)

        return cls(
            info=robot_info,
            links=links,
            joints=joints,
            generate_py_xml=True,  # URDF imports generate Python representation
        )

    @staticmethod
    def combine(
        descriptions: list["RobotDescription"],
        entities_to_add: list[LinkDescription | JointDescription] | None = None,
        robot_info: RobotInfo | None = None,
    ) -> "RobotDescription":
        """Combine multiple RobotDescriptions into one.

        If entities_to_add is provided, creates a RobotDescription from those entities
        and appends it to the descriptions list. This ensures new links/joints go at
        the end in case they reference other ones.

        Note: This only combines the data structures. To generate URDF XML, call
        to_urdf_xml() or get_body_xml() explicitly.
        """
        # If entities_to_add is provided, create a RobotDescription from them and append to descriptions
        if entities_to_add:
            entity_description = RobotDescription.from_entities(entities_to_add)
            descriptions = list(descriptions) + [entity_description]

        if robot_info is None:
            robot_info = RobotInfo(name=f"{descriptions[0].info.name}_combined")

        links = Links.combine([description.links for description in descriptions])
        joints = Joints.combine([description.joints for description in descriptions])
        joint_positions = JointPositions.combine(
            [description.joint_positions for description in descriptions]
        )
        points_of_interest = [
            point
            for description in descriptions
            for point in description.points_of_interest
        ]
        srdf = SrdfInfo.combine([description.srdf for description in descriptions])
        urdf = UrdfInfo.combine([description.urdf for description in descriptions])
        # args = DescriptionArgs.combine(*[description.args for description in descriptions])
        args = None

        description = RobotDescription(
            links=links,
            joints=joints,
            joint_positions=joint_positions,
            points_of_interest=points_of_interest,
            urdf=urdf,
            srdf=srdf,
            args=args,
            children=descriptions,
        )

        return description

    def extend(
        self,
        descriptions: list["RobotDescription"],
        entities_to_add: list[LinkDescription | JointDescription] | None = None,
        robot_info: RobotInfo | None = None,
    ) -> "RobotDescription":
        """Extend this RobotDescription with additional descriptions."""
        return RobotDescription.combine(
            [self] + descriptions,
            entities_to_add=entities_to_add,
            robot_info=robot_info,
        )

    def __str__(self):
        return json.dumps(asdict(self), indent=2)

    def __post_init__(self):
        # No auto-init - everything is explicit now
        pass

    def inspect(self, indent: int = 0) -> None:
        """Inspect robot description and its children, printing formatted information.

        Recursively prints robot info and calls inspect() on all children.

        Args:
            indent: Current indentation level for nested output
        """
        indent_str = "  " * indent

        print(f"{indent_str}RobotDescription:")
        print(f"{indent_str}  info:")
        # Print RobotInfo with additional indentation
        info_lines = str(self.info).split("\n")
        for line in info_lines:
            if line:
                print(f"{indent_str}    {line}")
            else:
                print()

        print(f"{indent_str}  generate_py_xml: {self.generate_py_xml}")
        print(f"{indent_str}  links: {len(self.links.entities)}")
        print(f"{indent_str}  joints: {len(self.joints.entities)}")
        print(
            f"{indent_str}  joint_positions: {len(self.joint_positions.joint_names)} joints"
        )
        print(f"{indent_str}  children: {len(self.children)}")

        if self.children:
            print(f"{indent_str}  children:")
            for i, child in enumerate(self.children):
                print(f"{indent_str}    [{i}]")
                child.inspect(indent=indent + 2)

    def set_flags(self, generate_py_xml: bool | None = None) -> "RobotDescription":
        """Set flags on the robot description.

        Args:
            generate_py_xml: If True, generate XML programmatically from links/joints.
                           If None, leaves current value unchanged.

        Returns:
            self for method chaining
        """
        if generate_py_xml is not None:
            self.generate_py_xml = generate_py_xml
        return self

    def get_body_xml(self) -> str:
        """Generate XML body for the robot description.

        This method handles all body XML generation logic:
        - If generate_py_xml is True: Generate XML directly from links and joints (all children already merged)
        - If has children AND generate_py_xml is False: Recursively call get_body_xml() on each child and concatenate
        - If no children: Get XML body from macro path

        Returns:
            XML body string (without header/footer)
        """
        if self.generate_py_xml:
            # Generate XML programmatically from links and joints
            return generate_composite_xml_body(self)
        elif self.children:
            # Has children: recursively get body XML from each child
            body_xmls = []
            for child in self.children:
                body_xmls.append(child.get_body_xml())
            return "\n".join(body_xmls)
        else:
            # No children: get XML body from macro path
            return self.urdf.get_xml_body(
                self.get_config_file(), resolve_packages=False
            )

    def to_urdf_xml(self) -> str:
        """Generate complete URDF XML for the robot description.

        Returns:
            Complete URDF XML string with header and footer
        """
        # If no children AND not generate_py_xml: use urdf.get_xml() directly
        if not self.children and not self.generate_py_xml:
            return self.urdf.get_xml(
                xacro_args=self.to_xacro_args(), resolve_packages=False
            )
        elif self.generate_py_xml:
            # Generate URDF directly from Python description
            from .urdf_converter import to_urdf_string

            return to_urdf_string(
                list(self.links.entities.values()),
                list(self.joints.entities.values()),
                self.info.name,
            )
        else:
            return self.urdf.get_combined_xml(self.get_body_xml())

    def get_config_file(self, dump_to_file: bool = True) -> str:
        """Get config file path, always regenerating and optionally dumping file.

        This method always regenerates file paths to ensure the config file path
        reflects the most up-to-date information (prefixes, etc.). By default,
        it also dumps the config file to disk to ensure it reflects the current state.

        Args:
            dump_to_file: If True, dump config file to disk. Set to False to avoid
                         recursion when called from dump_to_file().

        Returns:
            Path to config file
        """
        if not self.info.save_dir:
            raise ValueError(f"RobotInfo.save_dir for {self.info.name} must be set")

        if self.hash_file_name:
            self.info.urdf_hash = self.get_hash()[:12]

        self.config_file = self.info.get_file_path("_config.yaml")

        # Dump config file to ensure it reflects current state (unless disabled to avoid recursion)
        if dump_to_file:
            self.dump_to_file(verbose=np.True_)

        return self.config_file

    def get_hash(self) -> str:
        # BUG: file name is required for config file name, which is needed for urdf, but urdf is needed for hash. Hash xacro args instead
        # Hash the xacro args (what defines the robot for URDF generation)
        # This is available immediately and avoids circular dependency
        xacro_args_dict = self.to_xacro_args(set_config_file=False)
        # Remove fields that depend on the hash itself, as would create a loop.
        xacro_args_dict.pop("config_file", None)
        xacro_args_dict.pop("config_file_path", None)
        xacro_args_str = json.dumps(xacro_args_dict, sort_keys=True)
        config_hash = hashlib.md5(xacro_args_str.encode()).hexdigest()
        return config_hash

    def prefix(self, prefix: str, prefix_info: bool = True) -> "RobotDescription":
        """Apply prefix to all joint and link names, recursively prefixing children.

        This method iterates through all joints and links and sets their .name
        attribute based on prefix history. It also recursively calls prefix() on
        all child descriptions to ensure prefixes are applied throughout the tree.

        Every time prefix() is called, it applies the prefix to joints, links, and
        joint_positions. If prefix_info is True, it will also be added to self.info
        via add_prefix(), allowing you to call prefix() multiple times to build up
        a prefix chain.

        Args:
            prefix: Prefix string to apply. Always applied to joints, links, and joint_positions.
            prefix_info: If True, adds prefix to self.info. Set to False for internal calls
                        to avoid double prefixing.

        Returns:
            self for method chaining

        The field names in the Joints/Links dataclasses always hold the original
        unprefixed names, which is helpful for lookups and combinations.
        """
        if prefix == "":
            return self

        if prefix_info:
            self.info.add_prefix(prefix)

        # Always prefix joints, links, and joint_positions with the provided prefix
        self.joints.prefix(prefix)
        self.links.prefix(prefix)
        if self.joint_positions is not None:
            self.joint_positions.prefix(prefix)

        # Recursively prefix all children with the same prefix
        # This applies the prefix as a new layer on top of whatever prefixes children already have
        for child in self.children:
            child.prefix(prefix, prefix_info=True)

        return self

    def set_simple_collision(self) -> "RobotDescription":
        self.links.set_simple_collision()
        for child in self.children:
            child.set_simple_collision()
        return self

    def init_joint_positions(self) -> "RobotDescription":
        """Initialize JointPositions if not already set.

        Returns:
            self for method chaining
        """
        # Auto create joinpositionsf they are not already creater
        if len(self.joint_positions.joint_names) == 0:
            self.joint_positions.joint_names = self.joints.get_actuated_joint_names(
                prefix=True
            )
        if len(self.joint_positions.unprefixed_joint_names) == 0:
            self.joint_positions.unprefixed_joint_names = (
                self.joints.get_actuated_joint_names(prefix=False)
            )

        # Add default positions if not already defined
        actuated_joints = self.joints.get_actuated_joints()

        if "initial" not in self.joint_positions.positions:
            self.joint_positions.positions["initial"] = [
                joint.initial_position for joint in actuated_joints
            ]

        if "zeros" not in self.joint_positions.positions:
            # Get lower limit, defaulting to -pi if min_position is None
            self.joint_positions.positions["zeros"] = [
                max(
                    0.0,
                    (
                        joint.limits.min_position
                        if joint.limits.min_position is not None
                        else -np.pi
                    ),
                )
                for joint in actuated_joints
            ]

        if "middle" not in self.joint_positions.positions:
            # Calculate middle position between limits
            self.joint_positions.positions["middle"] = [
                (
                    (
                        joint.limits.min_position
                        if joint.limits.min_position is not None
                        else -np.pi
                    )
                    + joint.limits.max_position
                )
                / 2.0
                for joint in actuated_joints
            ]

        for key, value in self.joint_positions.positions.items():
            print(f"key: {key}, value: {value}")

        # Validate that each non-fixed joint is in positions.joint_names
        for joint_name in self.joints.get_actuated_joint_names(prefix=False):
            if joint_name not in self.joint_positions.get_joint_names(prefix=False):
                raise ValueError(
                    f"'{joint_name}' should be in joint_positions.joint_names: {self.joint_positions.joint_names}"
                )

        return self

    @property
    def q_initial(self) -> np.ndarray:
        """Return the initial joint configuration."""
        self.init_joint_positions()
        positions = self.joint_positions.get("initial")
        if positions is None:
            raise ValueError("JointPositions does not contain an 'initial' entry.")
        return np.array(positions, dtype=float)

    @property
    def q_initial_active(self) -> np.ndarray:
        """Return initial configuration restricted to actuated joints."""
        return self.q_initial.copy()

    def to_xacro_args(self, set_config_file: bool = True) -> dict[str, str]:
        from .format_for_xacro import (
            format_for_xacro,
        )  # We want to check for the robot description, careful for circular

        # Ensure config_file is set if it hasn't been already (unless disabled to avoid circular dependency)
        if set_config_file:
            self.config_file = self.get_config_file()

        return format_for_xacro(self)

    def dump_to_file(self, xacro_format=True, verbose=True):
        """Dump robot description to config file.

        This method ensures the config file path is set, then writes the data.
        Uses get_config_file(dump_to_file=False) to avoid recursion.

        Args:
            xacro_format: If True, use xacro format, otherwise use JSON
            verbose: If True, print verbose output
        """
        # Get config file path without auto-dumping to avoid recursion
        self.config_file_path = self.get_config_file(dump_to_file=False)

        if xacro_format:
            data = self.to_xacro_args(set_config_file=False)
        else:
            data = self.to_json()

        self.save_yaml_file(self.config_file_path, data, verbose=verbose)

    def to_rviz_cli(self) -> str:
        """

        - One option could be to just pass the configuration...
        - This is more generic though, you can pass any number of xacro args...

        """

        # args_str = " ".join([f"{k}:={v}" for k, v in xacro_args_dict.items()]) if xacro_args_dict else ""
        xacro_args_dict = {"config_file_path": self.get_config_file()}

        args_str = (
            " ".join([f"{k}:={v}" for k, v in xacro_args_dict.items()])
            if xacro_args_dict
            else ""
        )

        if args_str:
            return f"ros2 launch bam.descriptions display_xacro.launch.py description.file_path:={self.urdf.xacro_path} xacro_args:='{args_str}'"
        else:
            return f"ros2 launch bam.descriptions display_xacro.launch.py description.file_path:={self.urdf.xacro_path}"

    # region - URDF & SRDF

    def get_srdf_cache_path(self) -> Path:
        """Get SRDF cache path based on config_file location."""
        return Path(self.info.get_file_path(".srdf", subdir="srdf_cache"))

    def load_srdf_path_from_cache(self) -> bool:
        cached_path = self.get_srdf_cache_path()

        if cached_path.exists():
            self.srdf.path = str(cached_path)
            self.srdf.loaded_from_cache = True
            self.srdf.newly_generated = False
            return True

        return False

    # Reduce from 100,000 to 10,000 for dev so faster, but in reality likey want to use more!
    def generate_new_srdf(self, num_samples: int | None = None, verbose: bool = True):
        """Generate new SRDF file from URDF.

        Note: Config file is automatically dumped via get_config_file() when needed.
        If num_samples is None, uses self.srdf.num_samples (default 10,000).
        """
        # Use srdf.num_samples if not explicitly provided
        if num_samples is None:
            num_samples = self.srdf.num_samples

        srdf_save_path = self.get_srdf_cache_path()

        # Ensure config file exists (get_config_file() will dump it automatically)
        self.get_config_file()

        with self.get_temp_urdf_path() as urdf_path:
            generate_srdf_from_urdf(
                urdf_path=urdf_path,
                mesh_package_dirs=self.urdf.abs_package_dirs,
                srdf_save_path=str(srdf_save_path),
                num_samples=num_samples,
                verbose=verbose,
            )

        self.srdf.path = str(srdf_save_path)
        self.srdf.newly_generated = True
        self.srdf.loaded_from_cache = False

    def get_srdf_xml(self, resolve_packages: bool = False) -> str:
        """Get SRDF XML string, generating/loading it if needed.

        This method handles SRDF generation and loading lazily:
        - If force_regenerate is True: Generate new SRDF
        - If SRDF path already exists: Use existing SRDF
        - If cached SRDF exists: Load from cache
        - Otherwise: Generate new SRDF

        Returns:
            SRDF XML string
        """
        # Ensure SRDF is loaded/generated before getting XML
        if self.srdf.force_regenerate:
            self.generate_new_srdf()
        elif self.srdf.srdf_paths_exist():
            pass  # SRDF already specified, do nothing
        elif self.load_srdf_path_from_cache():
            pass  # Found cached SRDF, path assigned
        else:  # Generate new SRDF
            self.generate_new_srdf()

        if not self.srdf.srdf_paths_exist():
            raise RuntimeError("SRDF path is empty after generation attempt")

        return self.srdf.get_xml(
            self.to_xacro_args(), resolve_packages=resolve_packages
        )

    def get_srdf_path(self) -> str:
        return self.srdf.get_path()

    def get_temp_srdf_path(self):
        return temp_srdf_file(self.get_srdf_xml())

    # Put these helpers here so you can easily inject in the xacro args.

    def get_urdf_xml(self, resolve_packages: bool = False) -> str:
        """Get URDF XML string, handling both Python-generated and xacro-based descriptions.

        This is an alias for to_urdf_xml() for backward compatibility.
        """
        return self.to_urdf_xml()

    def get_urdf_path(self) -> str:
        return self.urdf.get_path()

    def get_temp_urdf_path(self):
        """Get the context of a temporary URDF file path. Example:

        with rd.temp_urdf_path() as path:
            pin_collision = PinCollision.from_xacro(path, ...)
        """
        return temp_urdf_file(self.get_urdf_xml())

    # region - Helpers to get common links
    def get_root_link(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.root_link)

    def get_base_link(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.base_link)

    def get_base_mount(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.base_mount)

    def get_ik_base(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.ik_base)

    def get_ik_tip(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.ik_tip)

    def get_ik_tip_flip(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.ik_tip_flip)

    def get_arm_to_hand_mount(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.arm_to_hand_mount)

    def get_hand_to_arm_mount(self) -> LinkDescription:
        return self.links.find_by_tag(TAGS.hand_to_arm_mount)

    # endregion
