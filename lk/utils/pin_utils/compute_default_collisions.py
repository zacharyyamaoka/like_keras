#!/usr/bin/env python3

"""
Compute default collision pairs for SRDF generation.

This mimics MoveIt2's setup assistant functionality to automatically determine
which link pairs should be disabled for collision checking. The algorithm samples
random robot configurations to classify collision pairs as:
- Adjacent: Links connected by a joint (always disabled)
- Default: Links in collision at zero configuration
- Always: Links that collide in all sampled configurations
- Never: Links that never collide in any sampled configuration

Reference:
    https://moveit.picknik.ai/main/api/html/namespacemoveit__setup_1_1srdf__setup.html


Based on mplib: https://motion-planning-lib.readthedocs.io/latest/_modules/mplib/urdf_utils.html#generate_srdf

"""

# BAM
from .pin_robot_model import PinRobotModel

# PYTHON
import numpy as np
import warnings
import xml.etree.ElementTree as ET
from xml.dom import minidom
from dataclasses import dataclass
from tqdm import tqdm


@dataclass
class CollisionPair:
    """
    Represents a collision pair that should be disabled.
    """

    link1: str
    link2: str
    reason: str  # "Adjacent", "Default", "Always", "Never"


def collision_pairs_to_srdf(
    collision_pairs: list[CollisionPair], robot_name: str
) -> str:
    """
    Generate SRDF XML string from a list of collision pairs.

    DescriptionArgs:
        collision_pairs: List of CollisionPair objects to disable
        robot_name: Name of the robot for the SRDF root element

    Returns:
        SRDF content as formatted XML string

    Example:
        >>> pairs = [
        >>>     CollisionPair("link1", "link2", "Adjacent"),
        >>>     CollisionPair("link3", "link4", "Never"),
        >>> ]
        >>> srdf_xml = collision_pairs_to_srdf(pairs, "my_robot")
    """
    root = ET.Element("robot", {"name": robot_name})

    # Deduplicate pairs to ensure no duplicate entries in SRDF
    # Use dict to preserve order and keep first reason for each pair
    unique_pairs = {}
    for pair in collision_pairs:
        # Create canonical key (sorted tuple) for deduplication
        pair_key = tuple(sorted([pair.link1, pair.link2]))
        if pair_key not in unique_pairs:
            unique_pairs[pair_key] = pair

    # Add unique pairs to XML
    for pair in unique_pairs.values():
        ET.SubElement(
            root,
            "disable_collisions",
            attrib={"link1": pair.link1, "link2": pair.link2, "reason": pair.reason},
        )

    # Generate formatted XML string
    srdf_xml = minidom.parseString(ET.tostring(root)).toprettyxml(indent="  ")
    return srdf_xml


def compute_default_collisions(
    robot_model: PinRobotModel, num_samples: int = 100000, verbose: bool = False
) -> tuple[str, list[CollisionPair]]:
    """
    Compute default collision pairs for generating SRDF.

    This function mimics MoveIt2's moveit_setup::srdf_setup::computeDefaultCollisions()
    by sampling random configurations to determine which link pairs should be disabled
    for collision checking.

    DescriptionArgs:
        robot_model: PinRobotModel with collision_model loaded
        num_samples: Number of random configurations to sample
        verbose: Print debug information during computation

    Returns:
        Tuple of (srdf_xml_string, list_of_collision_pairs)
        - srdf_xml_string: SRDF content as formatted XML string
        - list_of_collision_pairs: List of CollisionPair objects with reasons

    Example:
        >>> from pin_utils import PinRobotModel, compute_default_collisions
        >>> from bam.descriptions import UR
        >>>
        >>> ur5e = UR.make_UR5e()
        >>> robot_model = PinRobotModel.from_robot_description(ur5e)
        >>> srdf_xml, pairs = compute_default_collisions(robot_model, verbose=True)
        >>>
        >>> print(f"Found {len(pairs)} collision pairs to disable")
        >>> for pair in pairs[:5]:
        >>>     print(f"  {pair.link1} <-> {pair.link2}: {pair.reason}")
    """
    import pinocchio as pin

    if robot_model.collision_model is None:
        raise ValueError(
            "PinRobotModel must have collision_model loaded for collision detection"
        )

    if verbose:
        print("Generating SRDF with default collision pairs.")
        print(f"This will sample {num_samples:,} random configurations...")

    model = robot_model.model
    collision_model = robot_model.collision_model

    # Create fresh data structures
    data = model.createData()
    collision_data = pin.GeometryData(collision_model)

    # Add all possible collision pairs (before any filtering)
    collision_model_full = collision_model.copy()
    collision_model_full.addAllCollisionPairs()
    collision_data_full = pin.GeometryData(collision_model_full)

    # Get all geometry object names and create lookup
    geom_names = [geom.name for geom in collision_model_full.geometryObjects]

    # Extract parent link names from geometry objects
    # Geometry objects are named like "link_name_collision" or similar
    def get_link_name(geom_name: str) -> str:
        # Try to extract link name by removing common suffixes
        for suffix in ["_collision", "_visual", "_0", "_1", "_2"]:
            if geom_name.endswith(suffix):
                return geom_name[: -len(suffix)]
        return geom_name

    # Group geometries by their parent link
    link_to_geom_indices = {}
    for idx, geom in enumerate(collision_model_full.geometryObjects):
        link_name = model.frames[geom.parentFrame].name
        if link_name not in link_to_geom_indices:
            link_to_geom_indices[link_name] = []
        link_to_geom_indices[link_name].append(idx)

    link_names = sorted(link_to_geom_indices.keys())
    n_links = len(link_names)
    link_name_to_idx = {name: idx for idx, name in enumerate(link_names)}

    if verbose:
        print(f"\nFound {n_links} links with collision geometries")
        print(
            f"Total collision pairs to check: {len(collision_model_full.collisionPairs)}"
        )

    # Initialize structures
    collision_pairs_to_disable = []
    disabled_link_pairs = set()  # Track which link pairs are already disabled

    # 1. Disable adjacent link pairs
    if verbose:
        print("\n1. Finding adjacent links...")

    for joint_id in range(1, model.njoints):  # Skip universe joint (id=0)
        joint = model.joints[joint_id]
        parent_joint_id = model.frames[joint.id].parentJoint
        child_frame_id = joint.id

        if parent_joint_id > 0 and child_frame_id > 0:
            parent_link = model.frames[parent_joint_id].name
            child_link = model.frames[child_frame_id].name

            # Only process if both links have collision geometries
            if (
                parent_link in link_to_geom_indices
                and child_link in link_to_geom_indices
            ):
                pair_key = tuple(sorted([parent_link, child_link]))
                if pair_key not in disabled_link_pairs:
                    disabled_link_pairs.add(pair_key)
                    link1, link2 = pair_key

                    collision_pairs_to_disable.append(
                        CollisionPair(link1, link2, "Adjacent")
                    )

                    if verbose:
                        print(f"  Disable: {link1} <-> {link2} (Adjacent)")

    # 2. Disable all-zeros qpos (default) collision
    if verbose:
        print("\n2. Checking default configuration (all zeros)...")

    q_zero = pin.neutral(model)
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", message="This function has been marked as deprecated"
        )
        pin.computeCollisions(
            model, data, collision_model_full, collision_data_full, q_zero, False
        )

    for pair_idx, pair in enumerate(collision_model_full.collisionPairs):
        result = collision_data_full.collisionResults[pair_idx]
        if result.isCollision():
            geom1 = collision_model_full.geometryObjects[pair.first]
            geom2 = collision_model_full.geometryObjects[pair.second]
            link1 = model.frames[geom1.parentFrame].name
            link2 = model.frames[geom2.parentFrame].name

            pair_key = tuple(sorted([link1, link2]))
            if pair_key not in disabled_link_pairs:
                disabled_link_pairs.add(pair_key)
                link1, link2 = pair_key

                collision_pairs_to_disable.append(
                    CollisionPair(link1, link2, "Default")
                )

                if verbose:
                    print(f"  Disable: {link1} <-> {link2} (Default)")

    # 3. Sample random configurations to find Always/Never collision pairs
    if verbose:
        print(f"\n3. Sampling {num_samples:,} random configurations...")

    collision_count = np.zeros((n_links, n_links), dtype=int)

    # Sample configurations with progress bar
    for sample_idx in tqdm(
        range(num_samples),
        desc="Sampling configurations",
        unit="sample",
        disable=not verbose,
        dynamic_ncols=True,
    ):
        # Generate random configuration
        q_random = pin.randomConfiguration(model)

        # Check collisions
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore", message="This function has been marked as deprecated"
            )
            pin.computeCollisions(
                model, data, collision_model_full, collision_data_full, q_random, False
            )

        # Count collisions for each link pair
        for pair_idx, pair in enumerate(collision_model_full.collisionPairs):
            result = collision_data_full.collisionResults[pair_idx]
            if result.isCollision():
                geom1 = collision_model_full.geometryObjects[pair.first]
                geom2 = collision_model_full.geometryObjects[pair.second]
                link1 = model.frames[geom1.parentFrame].name
                link2 = model.frames[geom2.parentFrame].name

                if link1 in link_name_to_idx and link2 in link_name_to_idx:
                    idx1 = link_name_to_idx[link1]
                    idx2 = link_name_to_idx[link2]
                    collision_count[idx1, idx2] += 1

    if verbose:
        print("\n4. Analyzing collision statistics...")

    # Build set of link pairs that actually exist in collision model
    existing_link_pairs = set()
    for pair in collision_model_full.collisionPairs:
        geom1 = collision_model_full.geometryObjects[pair.first]
        geom2 = collision_model_full.geometryObjects[pair.second]
        link1 = model.frames[geom1.parentFrame].name
        link2 = model.frames[geom2.parentFrame].name
        pair_key = tuple(sorted([link1, link2]))
        existing_link_pairs.add(pair_key)

    # Analyze results and disable Always/Never collision pairs
    always_count = 0
    never_count = 0

    for i, link1 in enumerate(link_names):
        for j in range(i + 1, n_links):
            link2 = link_names[j]

            pair_key = tuple(sorted([link1, link2]))
            if pair_key in disabled_link_pairs:
                continue  # Already disabled

            # Only consider pairs that actually exist in the collision model
            if pair_key not in existing_link_pairs:
                continue

            collision_cnt = collision_count[i, j] + collision_count[j, i]

            if collision_cnt == num_samples:
                # Always collides
                disabled_link_pairs.add(pair_key)
                link1_sorted, link2_sorted = pair_key

                collision_pairs_to_disable.append(
                    CollisionPair(link1_sorted, link2_sorted, "Always")
                )
                always_count += 1

                if verbose:
                    print(f"  Disable: {link1_sorted} <-> {link2_sorted} (Always)")

            elif collision_cnt == 0:
                # Never collides
                disabled_link_pairs.add(pair_key)
                link1_sorted, link2_sorted = pair_key

                collision_pairs_to_disable.append(
                    CollisionPair(link1_sorted, link2_sorted, "Never")
                )
                never_count += 1

                if verbose:
                    print(f"  Disable: {link1_sorted} <-> {link2_sorted} (Never)")

    # Generate SRDF XML string from collision pairs
    srdf_xml = collision_pairs_to_srdf(collision_pairs_to_disable, model.name)

    if verbose:
        print(f"\n{'='*60}")
        print("SUMMARY:")
        print(
            f"  Adjacent pairs  : {len([p for p in collision_pairs_to_disable if p.reason == 'Adjacent'])}"
        )
        print(
            f"  Default pairs   : {len([p for p in collision_pairs_to_disable if p.reason == 'Default'])}"
        )
        print(f"  Always pairs    : {always_count}")
        print(f"  Never pairs     : {never_count}")
        print(f"  Total disabled  : {len(collision_pairs_to_disable)}")
        print(f"{'='*60}")

    return srdf_xml, collision_pairs_to_disable


def generate_srdf_from_urdf(
    urdf_path: str,
    mesh_package_dirs: str | list[str],
    srdf_save_path: str,
    num_samples: int = 100000,
    verbose: bool = False,
) -> str:
    """Generate and save SRDF from URDF file.

    Simplified wrapper that loads URDF, computes collisions, and saves SRDF.

    DescriptionArgs:
        urdf_path: Path to URDF file
        srdf_save_path: Path where SRDF should be saved
        num_samples: Number of random configurations to sample
        verbose: Print progress information

    Returns:
        SRDF XML string
    """
    # Load URDF into PinRobotModel
    robot_model = PinRobotModel.from_urdf(urdf_path, mesh_package_dirs)

    # Compute collision pairs and generate SRDF
    srdf_xml, _ = compute_default_collisions(robot_model, num_samples, verbose)

    # Save to file
    with open(srdf_save_path, "w") as f:
        f.write(srdf_xml)

    if verbose:
        print(f"Saved SRDF to: {srdf_save_path}")

    return srdf_xml
