#!/usr/bin/env python3

"""
Verification utilities for RobotDescription correctness.

Purpose
-------
Provides programmatic validation of robot descriptions to ensure correctness
without requiring visual inspection. This enables AI agents and automated
systems to validate robot configurations reliably.

Verification Tests
------------------
1. **URDF Parsing**: Validates that the description generates valid URDF by
   successfully creating a PinRobotModel. This catches XML syntax errors,
   missing links/joints, invalid kinematic chains, etc.

2. **Name Consistency**: Verifies that all joints and links defined in the
   RobotDescription (with prefixes) exist in the generated URDF. This catches
   prefix mismatches and ensures Python representation matches XML output.

3. **Named Positions**: Tests that all named positions (e.g., "home", "ready")
   can be applied without errors. Validates joint name mappings and position
   dimensionality.

4. **Transform Verification**: For joints with explicitly defined transforms,
   verifies the Python-specified transform matches what Pinocchio extracts
   from the URDF. This ensures Python XML generation produces correct geometry.

5. **Tag Validation** (optional): Checks that required semantic tags exist
   (e.g., "base_mount", "ik_tip") for arm descriptions.

6. **Stability Testing** (optional): Regression testing that saves joint/link
   names and transforms to detect unintended changes over time.

Usage
-----
Basic validation:
```python
from bam.descriptions import verify_robot_description

# Validate description is correct
verify_robot_description(robot_description)

# Validate with required tags
verify_robot_description(
    robot_description,
    expected_tags=["base_mount", "ik_base", "ik_tip"]
)

# Enable stability regression testing
verify_robot_description(
    robot_description,
    verify_stable=True,
    stability_name="ur5e_robotiq"
)
```

Notes
-----
- Validation will raise descriptive exceptions on failure
- For composite descriptions, validates the combined result
- Transform verification only checks joints with non-identity transforms
- Stability files are saved as pickle in the same directory
"""

# BAM
from bam.utils.pin_utils import PinRobotModel

# PYTHON
import pickle
import numpy as np
from pathlib import Path
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .robot_description import RobotDescription


class VerificationError(Exception):
    """
    Raised when robot description verification fails.
    """

    pass


def verify_robot_description(
    robot_description: "RobotDescription",
    expected_tags: Optional[list[str]] = None,
    verify_stable: bool = False,
    save_stability: bool = False,
    stable_transform_tol: float = 1e-6,
    verbose: bool = False,
) -> None:
    """
    Verify robot description correctness through comprehensive checks.

    Args:
        robot_description: The RobotDescription instance to verify
        expected_tags: List of required tags to check for (e.g., ["base_mount", "ik_tip"])
        verify_stable: If True, compare against saved reference data for regression testing
        save_stability: If True, save current state as new reference
        stable_transform_tol: Numerical tolerance for transform comparisons
        verbose: If True, print detailed verification progress

    Raises:
        VerificationError: If any verification check fails
    """

    def print_v(msg: str) -> None:
        """Print message only if verbose mode is enabled."""
        if verbose:
            print(msg)

    print("\n" + "=" * 80)
    print("VERIFYING ROBOT DESCRIPTION")
    print("=" * 80)

    # 1. Verify URDF parsing
    print("\n[1/5] Testing URDF parsing...")
    try:
        robot_model = PinRobotModel.from_robot_description(robot_description)
        print(f"  ✓ Successfully created PinRobotModel")
        print_v(f"    - DOF: {robot_model.n_dof}")
        print_v(f"    - Root link: {robot_model.root_link}")
    except Exception as e:
        raise VerificationError(f"Failed to create PinRobotModel: {e}")

    # 2. Verify joint and link names exist in URDF
    print("\n[2/5] Verifying joint and link names...")
    _verify_names(robot_description, robot_model, verbose)

    # 3. Verify named positions
    print("\n[3/5] Verifying named positions...")
    _verify_named_positions(robot_description, robot_model, verbose)

    # 4. Verify joint transforms
    print("\n[4/5] Verifying joint transforms...")
    _verify_joint_transforms(
        robot_description, robot_model, stable_transform_tol, verbose
    )

    # 5. Verify tags if requested
    if expected_tags:
        print("\n[5/5] Verifying expected tags...")
        _verify_tags(robot_description, expected_tags, verbose)
    else:
        print("\n[5/5] Skipping tag verification (no tags specified)")

    # 6. Verify stability if requested
    if verify_stable or save_stability:
        print("\n[OPTIONAL] Running stability verification...")
        _verify_stability(
            robot_description,
            robot_model,
            save=save_stability,
            tolerance=stable_transform_tol,
            verbose=verbose,
        )

    print("\n" + "=" * 80)
    print("✓ ALL VERIFICATION CHECKS PASSED")
    print("=" * 80 + "\n")


def _verify_names(
    robot_description: "RobotDescription", robot_model: PinRobotModel, verbose: bool
) -> None:
    """
    Verify all joints and links in description exist in the URDF.
    """

    def print_v(msg: str) -> None:
        if verbose:
            print(msg)

    # Get all joint and link names from Pin model
    all_pin_joints = set(robot_model.get_all_joint_names())
    all_pin_links = set(robot_model.get_all_link_names())

    # Check all joints
    description_joints = robot_description.joints.names
    missing_joints = []
    for joint_name in description_joints:
        if joint_name not in all_pin_joints:
            missing_joints.append(joint_name)

    if missing_joints:
        raise VerificationError(
            f"Missing joints: These joints are in the Python description but not found in URDF:\n"
            f"  Missing (in Py not Pin): {missing_joints}\n"
            f"  All Pin joints: {sorted(all_pin_joints)}\n"
            f"This typically indicates a prefix mismatch."
        )

    print(f"  ✓ All {len(description_joints)} joints found in URDF")
    for joint_name in description_joints:
        print_v(f"    - {joint_name}")

    # Check all links
    description_links = robot_description.links.names
    missing_links = []
    for link_name in description_links:
        if link_name not in all_pin_links:
            missing_links.append(link_name)

    if missing_links:
        raise VerificationError(
            f"Missing links: These links are in the Python description but not found in URDF:\n"
            f"  Missing (in Py not Pin): {missing_links}\n"
            f"  All Pin links: {sorted(all_pin_links)}\n"
            f"This typically indicates a prefix mismatch."
        )

    print(f"  ✓ All {len(description_links)} links found in URDF")
    for link_name in description_links:
        print_v(f"    - {link_name}")


def _verify_named_positions(
    robot_description: "RobotDescription", robot_model: PinRobotModel, verbose: bool
) -> None:
    """
    Verify all named positions can be applied without errors.
    """

    def print_v(msg: str) -> None:
        if verbose:
            print(msg)

    joint_positions = robot_description.joint_positions
    position_names = joint_positions.keys()

    if not position_names:
        print("  ⚠ No named positions to verify")
        return

    # Get actuated joint names from the description
    actuated_joint_names = robot_description.joints.get_actuated_joint_names()

    if not actuated_joint_names:
        print("  ⚠ No actuated joints, skipping position verification")
        return

    errors = []
    for pos_name in position_names:
        try:
            # Get the position from joint_positions
            q_values = joint_positions.get(pos_name)

            if q_values is None:
                errors.append(f"Position '{pos_name}' returned None")
                continue

            # Verify dimensions match
            if len(q_values) != len(actuated_joint_names):
                errors.append(
                    f"Position '{pos_name}' has {len(q_values)} values but "
                    f"{len(actuated_joint_names)} actuated joints"
                )
                continue

            # Try to order the configuration for the robot model
            q_ordered = robot_model.order_q_by_name(
                actuated_joint_names, np.array(q_values)
            )

            # Verify no NaN values
            if np.any(np.isnan(q_ordered)):
                errors.append(
                    f"Position '{pos_name}' resulted in NaN values after ordering"
                )
                continue

            # Try to update frames (catches invalid joint limits, etc.)
            robot_model.update_frames(q_ordered)
            print_v(f"    - '{pos_name}': OK")

        except Exception as e:
            errors.append(f"Position '{pos_name}': {str(e)}")

    if errors:
        raise VerificationError(
            f"Named position verification failed:\n  " + "\n  ".join(errors)
        )

    print(
        f"  ✓ All {len(position_names)} named positions verified: {list(position_names)}"
    )


def _verify_joint_transforms(
    robot_description: "RobotDescription",
    robot_model: PinRobotModel,
    tolerance: float,
    verbose: bool,
) -> None:
    """
    Verify joint transforms match between description and URDF.

    Only checks joints that have explicitly defined transforms (not None).
    """

    def print_v(msg: str) -> None:
        if verbose:
            print(msg)

    # Get joints with defined transforms using helper method
    specified_joints = robot_description.joints.get_specified_joints()

    if not specified_joints:
        print("  ⚠ No joints with defined transforms to verify")
        return

    errors = []
    q_neutral = robot_model.get_q_neutral()
    robot_model.update_frames(q_neutral)

    rotating_count = 0
    fixed_count = 0

    for joint in specified_joints:
        try:
            expected_transform = joint.transform.to_matrix()

            # Get actual transform based on joint type
            if joint.is_fixed:
                # For fixed joints, get transform between parent and child frames
                parent_frame = joint.transform.header.frame_id
                child_frame = joint.transform.child_frame_id
                actual_transform = robot_model.get_transform_between_frames(
                    parent_frame, child_frame
                )
                joint_label = f"{joint.name} (fixed)"
                fixed_count += 1
            else:
                actual_transform = robot_model.get_rotating_joint_placement(joint.name)
                joint_label = joint.name
                rotating_count += 1

            # Compare transforms
            diff = np.abs(expected_transform - actual_transform)
            max_diff = np.max(diff)

            if max_diff > tolerance:
                errors.append(
                    f"Joint '{joint.name}' transform mismatch:\n"
                    f"    Max difference: {max_diff:.6e}\n"
                    f"    Expected:\n{expected_transform}\n"
                    f"    Actual:\n{actual_transform}"
                )
            else:
                print_v(f"    - {joint_label}: OK (max diff: {max_diff:.6e})")
        except Exception as e:
            errors.append(f"Joint '{joint.name}': {str(e)}")

    if errors:
        raise VerificationError(
            f"Joint transform verification failed:\n" + "\n".join(errors)
        )

    print(
        f"  ✓ All {rotating_count + fixed_count} joint transforms verified ({rotating_count} rotating, {fixed_count} fixed)"
    )


def _verify_tags(
    robot_description: "RobotDescription", expected_tags: list[str], verbose: bool
) -> None:
    """
    Verify that expected semantic tags exist in the description.
    """

    def print_v(msg: str) -> None:
        if verbose:
            print(msg)

    # Collect all tags from all links
    all_tags = set()
    for link in robot_description.links:
        all_tags.update(link.tags)

    missing_tags = []
    for tag in expected_tags:
        if tag not in all_tags:
            missing_tags.append(tag)
        else:
            print_v(f"    - {tag}: found")

    if missing_tags:
        raise VerificationError(
            f"Required tags not found in description:\n"
            f"  Missing: {missing_tags}\n"
            f"  Available: {sorted(all_tags)}"
        )

    print(f"  ✓ All {len(expected_tags)} required tags found: {expected_tags}")


def _verify_stability(
    robot_description: "RobotDescription",
    robot_model: PinRobotModel,
    save: bool,
    tolerance: float,
    verbose: bool,
) -> None:
    """
    Regression testing to ensure joint/link names and transforms don't change.

    Similar to test_rd_transforms.py approach - saves state and compares on future runs.
    """

    def print_v(msg: str) -> None:
        if verbose:
            print(msg)

    # Get save directory from robot info, create stable subfolder
    info_save_dir = robot_description.info.save_dir
    if not info_save_dir:
        raise VerificationError(
            "robot_description.info.save_dir is not set. "
            "Cannot perform stability verification without a save directory."
        )

    stability_name = robot_description.info.name or "robot"
    save_dir = Path(info_save_dir) / "stable"
    save_dir.mkdir(exist_ok=True, parents=True)
    save_file = save_dir / f"{stability_name}.pkl"

    print_v(f"  Stability file: {save_file}")

    # Collect current state
    current_state = {
        "joint_names": robot_description.joints.names,
        "link_names": robot_description.links.names,
        "actuated_joint_names": robot_description.joints.get_actuated_joint_names(),
        "position_names": list(robot_description.joint_positions.keys()),
        "frame_names": robot_model.frame_names,
        "n_dof": robot_model.n_dof,
    }

    # Add joint transforms for joints with defined transforms
    joint_transform_dict = robot_description.joints.get_joint_transforms()
    joint_transforms = {}
    q_neutral = robot_model.get_q_neutral()
    robot_model.update_frames(q_neutral)

    for joint_name in joint_transform_dict.keys():
        try:
            # Get the joint description to check if it's fixed
            joint = robot_description.joints.entities.get(joint_name)
            if joint is None:
                print(f"  ⚠ Joint '{joint_name}' not found in description")
                continue

            if joint.is_fixed:
                # Fixed joints - get transform between parent and child frames
                parent_frame = joint.transform.header.frame_id
                child_frame = joint.transform.child_frame_id
                joint_transforms[joint_name] = robot_model.get_transform_between_frames(
                    parent_frame, child_frame
                )
                print_v(f"    - Saved transform for {joint_name} (fixed)")
            else:
                # Rotating joints - use joint placement
                joint_transforms[joint_name] = robot_model.get_rotating_joint_placement(
                    joint_name
                )
                print_v(f"    - Saved transform for {joint_name}")
        except Exception as e:
            print(f"  ⚠ Could not get transform for joint '{joint_name}': {e}")

    current_state["joint_transforms"] = joint_transforms

    if save:
        # Save current state
        with open(save_file, "wb") as f:
            pickle.dump(current_state, f)
        print(f"  ✓ Saved stability reference to {save_file}")
    else:
        # Compare against saved state
        if not save_file.exists():
            raise VerificationError(
                f"Stability reference file not found: {save_file}\n"
                f"Run with save_stability=True to create reference."
            )

        with open(save_file, "rb") as f:
            saved_state = pickle.load(f)

        errors = []

        # Check joint names
        if current_state["joint_names"] != saved_state["joint_names"]:
            errors.append("Joint names changed")

        # Check link names
        if current_state["link_names"] != saved_state["link_names"]:
            errors.append("Link names changed")

        # Check DOF
        if current_state["n_dof"] != saved_state["n_dof"]:
            errors.append(
                f"DOF changed from {saved_state['n_dof']} to {current_state['n_dof']}"
            )

        # Check joint transforms
        for joint_name, current_transform in current_state["joint_transforms"].items():
            if joint_name not in saved_state["joint_transforms"]:
                errors.append(f"New joint transform: {joint_name}")
                continue

            saved_transform = saved_state["joint_transforms"][joint_name]
            diff = np.abs(current_transform - saved_transform)
            max_diff = np.max(diff)

            if max_diff > tolerance:
                errors.append(
                    f"Joint '{joint_name}' transform changed (max diff: {max_diff:.6e})"
                )

        # Check for removed transforms
        for joint_name in saved_state["joint_transforms"]:
            if joint_name not in current_state["joint_transforms"]:
                errors.append(f"Joint transform removed: {joint_name}")

        if errors:
            raise VerificationError(
                f"Stability verification failed (regression detected):\n  "
                + "\n  ".join(errors)
            )

        print(f"  ✓ Stability verified against {save_file}")
        print(f"    - {len(current_state['joint_names'])} joints")
        print(f"    - {len(current_state['link_names'])} links")
        print(f"    - {len(current_state['joint_transforms'])} joint transforms")


if __name__ == "__main__":
    """
    Example usage demonstrating verification of a robot description.
    """
    # BAM
    from bam.configs import ur_robotiq_conveyor_dev

    print("\n" + "=" * 80)
    print("ROBOT DESCRIPTION VERIFICATION - Quick Example")
    print("=" * 80 + "\n")

    print("Loading robot configuration...")
    agent_config = ur_robotiq_conveyor_dev.generate_agent_config()
    description = agent_config.description

    print(f"  Robot: {description.info.name}")
    print(f"  Joints: {len(description.joints.names)}")
    print(f"  Links: {len(description.links.names)}")

    # Basic verification
    print("\nRunning basic verification...")
    try:
        verify_robot_description(description)
        print("\n✓ SUCCESS: Description is valid!")
    except VerificationError as e:
        print(f"\n✗ FAILED: {e}")
        exit(1)

    print("\n" + "=" * 80)
    print("See examples/ex_verify_description.py for more usage examples")
    print("=" * 80 + "\n")
