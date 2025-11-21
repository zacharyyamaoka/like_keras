#!/usr/bin/env python3

"""
Progressive Composite Build Example

Demonstrates building a complex multi-stack MRF system from simple components.
Shows how to:
1. Start with simple arm and hand
2. Combine them into arm_hand
3. Create prefixed/reflected variations
4. Build racks with multiple arms
5. Add conveyors to make rows
6. Stack rows to make levels
7. Duplicate along y to make columns
8. Duplicate along x to make multiple stacks

Final hierarchy:
- stack1, stack2 (duplicated along x)
  - col1, col2, col3 (duplicated along y, front to back)
    - lvl1, lvl2, lvl3 (duplicated along z, bottom to top)
      - rack1, rack2, rack3 (along the conveyor)
        - front_left_arm, front_right_arm, back_left_arm, back_right_arm

Example naming: stack2_col3_lvl2_rack3_front_right_arm

At some point we want to do this but nested within system configs...
When you combine system configs... well perhaps turn into a system and then combine at that level? hmm n
"""

# BAM
from bam.common.artist import RobotArtist
from bam.descriptions import RobotDescription, UR, Robotiq
from bam.descriptions.bam_descriptions import ArmHand, ConveyorDescription
from bam.msgs import JointDescription, TransformStamped, LinkDescription, RobotInfo

# PYTHON


def generate_robot_description() -> RobotDescription:

    # ============================================================================
    # STEP 1: Simple Arm
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 1: Create simple UR5e arm")
    print("=" * 80)

    arm = UR.make_UR5e()
    verify_robot_description(arm, "arm")

    arm_artist = RobotArtist.from_robot_description(arm).block_till_input()

    # ============================================================================
    # STEP 2: Simple Hand
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 2: Create Robotiq 2F85 hand")
    print("=" * 80)

    hand = Robotiq.make_2F85()
    verify_robot_description(hand, "hand")

    hand_artist = RobotArtist.from_robot_description(hand).block_till_input()

    # ============================================================================
    # STEP 3: Combine Arm + Hand
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 3: Combine arm and hand")
    print("=" * 80)

    # Create joint connecting arm to hand
    arm_to_hand_joint = JointDescription(
        name="arm_to_hand",
        type="fixed",
        transform=TransformStamped.from_frames(
            frame_id=arm.get_arm_to_hand_mount().name,
            child_frame_id=hand.get_hand_to_arm_mount().name,
        ),
    )

    arm_hand = RobotDescription.combine(
        [arm, hand], entities_to_add=[arm_to_hand_joint]
    )

    verify_robot_description(arm_hand, "arm_hand")

    arm_hand_artist = RobotArtist.from_robot_description(arm_hand).block_till_input()

    # ============================================================================
    # STEP 4: Create Four Arms for Rack (with prefix and reflect)
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 4: Create four arms using prefix and reflect")
    print("=" * 80)

    # Use ArmHand.make for cleaner syntax
    arm_hand_clean = ArmHand.make(arm=UR.make_UR5e(), hand=Robotiq.make_2F85())

    # Create four variations with prefix and reflection
    front_left = arm_hand_clean.prefix("front_left_").reflect(-1)
    front_right = arm_hand_clean.prefix("front_right_").reflect(1)
    back_left = arm_hand_clean.prefix("back_left_").reflect(-1)
    back_right = arm_hand_clean.prefix("back_right_").reflect(1)

    verify_robot_description(front_left, "front_left")
    verify_robot_description(front_right, "front_right")
    verify_robot_description(back_left, "back_left")
    verify_robot_description(back_right, "back_right")

    # Visualize one of them
    front_left_artist = RobotArtist.from_robot_description(
        front_left
    ).block_till_input()

    # ============================================================================
    # STEP 5: Build Quad Arm Rack
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 5: Build quad arm rack with proper joints")
    print("=" * 80)

    # Create base links for rack
    world_link = LinkDescription(
        name="world",
        is_frame=True,
    )

    rack_base_link = LinkDescription(
        name="rack_base_link",
        is_frame=True,
    )

    # Create world joint
    world_joint = JointDescription(
        name="world_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=world_link.name,
            child_frame_id=rack_base_link.name,
        ),
    )

    # Define arm positions on rack
    arm_spacing = 0.3  # spacing between arms

    # Front left joint
    front_left_joint = JointDescription(
        name="front_left_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[-arm_spacing, arm_spacing, 0],
            rpy=[0, 0, 0],
            frame_id=rack_base_link.name,
            child_frame_id=front_left.get_base_mount().name,
        ),
    )

    # Front right joint
    front_right_joint = JointDescription(
        name="front_right_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[arm_spacing, arm_spacing, 0],
            rpy=[0, 0, 0],
            frame_id=rack_base_link.name,
            child_frame_id=front_right.get_base_mount().name,
        ),
    )

    # Back left joint
    back_left_joint = JointDescription(
        name="back_left_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[-arm_spacing, -arm_spacing, 0],
            rpy=[0, 0, 0],
            frame_id=rack_base_link.name,
            child_frame_id=back_left.get_base_mount().name,
        ),
    )

    # Back right joint
    back_right_joint = JointDescription(
        name="back_right_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[arm_spacing, -arm_spacing, 0],
            rpy=[0, 0, 0],
            frame_id=rack_base_link.name,
            child_frame_id=back_right.get_base_mount().name,
        ),
    )

    # Combine into rack
    rack = RobotDescription.combine(
        [front_left, front_right, back_left, back_right],
        entities_to_add=[
            world_joint,
            front_left_joint,
            front_right_joint,
            back_left_joint,
            back_right_joint,
        ],
        links_to_add=[world_link, rack_base_link],
        robot_info=RobotInfo(
            name="quad_arm_rack", type="rack", sku="v1", version="0.0.1"
        ),
    )

    verify_robot_description(rack, "rack")

    rack_artist = RobotArtist.from_robot_description(rack).block_till_input()

    # ============================================================================
    # STEP 6: Make Multiple Racks
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 6: Create three racks with prefixes")
    print("=" * 80)

    rack1 = rack.prefix("rack1_")
    rack2 = rack.prefix("rack2_")
    rack3 = rack.prefix("rack3_")

    verify_robot_description(rack1, "rack1")
    verify_robot_description(rack2, "rack2")
    verify_robot_description(rack3, "rack3")

    # ============================================================================
    # STEP 7: Add Conveyor to Make a Row (Level)
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 7: Add conveyor belt and attach 3 racks to make a row")
    print("=" * 80)

    rack_spacing = 0.5  # spacing between racks along conveyor
    rack_height = 0.3  # height of racks above conveyor
    conveyor_length = rack_spacing * 4

    conveyor = ConveyorDescription.make(
        width=0.4,
        length=conveyor_length,
        height=0.05,
        railing_width=0.02,
        railing_height_above=0.05,
        max_speed=0.8,
    )

    # Create joints between racks and conveyor
    # rack2 is in the middle (y=0)
    conveyor_to_rack2_joint = JointDescription(
        name="conveyor_to_rack2",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, rack_height],
            rpy=[0, 0, 0],
            frame_id=conveyor.links.surface_center.name,
            child_frame_id=rack2.get_base_mount().name,
        ),
    )

    # rack1 is offset in positive y
    conveyor_to_rack1_joint = JointDescription(
        name="conveyor_to_rack1",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, rack_spacing, rack_height],
            rpy=[0, 0, 0],
            frame_id=conveyor.links.surface_center.name,
            child_frame_id=rack1.get_base_mount().name,
        ),
    )

    # rack3 is offset in negative y
    conveyor_to_rack3_joint = JointDescription(
        name="conveyor_to_rack3",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, -rack_spacing, rack_height],
            rpy=[0, 0, 0],
            frame_id=conveyor.links.surface_center.name,
            child_frame_id=rack3.get_base_mount().name,
        ),
    )

    # Combine into a row (which is actually one level)
    row = RobotDescription.combine(
        [conveyor, rack1, rack2, rack3],
        entities_to_add=[
            conveyor_to_rack1_joint,
            conveyor_to_rack2_joint,
            conveyor_to_rack3_joint,
        ],
        robot_info=RobotInfo(name="rack_row", type="row", sku="v1", version="0.0.1"),
    )

    verify_robot_description(row, "row")

    row_artist = RobotArtist.from_robot_description(row).block_till_input()

    # ============================================================================
    # STEP 8: Stack Rows to Make Levels (duplicate along z)
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 8: Stack 3 rows vertically to make 3 levels")
    print("=" * 80)

    level_height = 1.5  # vertical spacing between levels

    # Create prefixed levels
    lvl1 = row.prefix("lvl1_")
    lvl2 = row.prefix("lvl2_")
    lvl3 = row.prefix("lvl3_")

    # Create base link for column
    col_base_link = LinkDescription(
        name="col_base_link",
        is_frame=True,
    )

    col_world_link = LinkDescription(
        name="col_world",
        is_frame=True,
    )

    col_world_joint = JointDescription(
        name="col_world_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=col_world_link.name,
            child_frame_id=col_base_link.name,
        ),
    )

    # Create joints for each level
    base_to_lvl1_joint = JointDescription(
        name="base_to_lvl1",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],  # lvl1 at base height
            rpy=[0, 0, 0],
            frame_id=col_base_link.name,
            child_frame_id=lvl1.get_base_mount().name,
        ),
    )

    base_to_lvl2_joint = JointDescription(
        name="base_to_lvl2",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, level_height],  # lvl2 one level up
            rpy=[0, 0, 0],
            frame_id=col_base_link.name,
            child_frame_id=lvl2.get_base_mount().name,
        ),
    )

    base_to_lvl3_joint = JointDescription(
        name="base_to_lvl3",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, level_height * 2],  # lvl3 two levels up
            rpy=[0, 0, 0],
            frame_id=col_base_link.name,
            child_frame_id=lvl3.get_base_mount().name,
        ),
    )

    # Combine into column
    column = RobotDescription.combine(
        [lvl1, lvl2, lvl3],
        entities_to_add=[
            col_world_joint,
            base_to_lvl1_joint,
            base_to_lvl2_joint,
            base_to_lvl3_joint,
        ],
        links_to_add=[col_world_link, col_base_link],
        robot_info=RobotInfo(name="column", type="column", sku="v1", version="0.0.1"),
    )

    verify_robot_description(column, "column")

    column_artist = RobotArtist.from_robot_description(column).block_till_input()

    # ============================================================================
    # STEP 9: Duplicate Columns Along Y (stack conveyors end-to-end)
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 9: Create 3 columns stacked along y (conveyor direction)")
    print("=" * 80)

    column_spacing = conveyor_length  # Stack conveyors end to end

    # Create prefixed columns
    col1 = column.prefix("col1_")  # front
    col2 = column.prefix("col2_")  # middle
    col3 = column.prefix("col3_")  # back

    # Create base link for stack
    stack_base_link = LinkDescription(
        name="stack_base_link",
        is_frame=True,
    )

    stack_world_link = LinkDescription(
        name="stack_world",
        is_frame=True,
    )

    stack_world_joint = JointDescription(
        name="stack_world_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=stack_world_link.name,
            child_frame_id=stack_base_link.name,
        ),
    )

    # Create joints for each column
    base_to_col1_joint = JointDescription(
        name="base_to_col1",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, column_spacing, 0],  # col1 at front
            rpy=[0, 0, 0],
            frame_id=stack_base_link.name,
            child_frame_id=col1.get_base_mount().name,
        ),
    )

    base_to_col2_joint = JointDescription(
        name="base_to_col2",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],  # col2 in middle
            rpy=[0, 0, 0],
            frame_id=stack_base_link.name,
            child_frame_id=col2.get_base_mount().name,
        ),
    )

    base_to_col3_joint = JointDescription(
        name="base_to_col3",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, -column_spacing, 0],  # col3 at back
            rpy=[0, 0, 0],
            frame_id=stack_base_link.name,
            child_frame_id=col3.get_base_mount().name,
        ),
    )

    # Combine into stack
    stack = RobotDescription.combine(
        [col1, col2, col3],
        entities_to_add=[
            stack_world_joint,
            base_to_col1_joint,
            base_to_col2_joint,
            base_to_col3_joint,
        ],
        links_to_add=[stack_world_link, stack_base_link],
        robot_info=RobotInfo(name="stack", type="stack", sku="v1", version="0.0.1"),
    )

    verify_robot_description(stack, "stack")

    stack_artist = RobotArtist.from_robot_description(stack).block_till_input()

    # ============================================================================
    # STEP 10: Create Multiple Stacks Along X
    # ============================================================================
    print("\n" + "=" * 80)
    print("STEP 10: Create 2 stacks offset along x")
    print("=" * 80)

    stack_spacing = 3.0  # lateral spacing between stacks

    # Create prefixed stacks
    stack1 = stack.prefix("stack1_")
    stack2 = stack.prefix("stack2_")

    # Create base link for facility
    facility_base_link = LinkDescription(
        name="facility_base_link",
        is_frame=True,
    )

    facility_world_link = LinkDescription(
        name="facility_world",
        is_frame=True,
    )

    facility_world_joint = JointDescription(
        name="facility_world_joint",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[0, 0, 0],
            rpy=[0, 0, 0],
            frame_id=facility_world_link.name,
            child_frame_id=facility_base_link.name,
        ),
    )

    # Create joints for each stack
    base_to_stack1_joint = JointDescription(
        name="base_to_stack1",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[-stack_spacing / 2, 0, 0],  # stack1 on left
            rpy=[0, 0, 0],
            frame_id=facility_base_link.name,
            child_frame_id=stack1.get_base_mount().name,
        ),
    )

    base_to_stack2_joint = JointDescription(
        name="base_to_stack2",
        type="fixed",
        transform=TransformStamped.from_xyzrpy(
            xyz=[stack_spacing / 2, 0, 0],  # stack2 on right
            rpy=[0, 0, 0],
            frame_id=facility_base_link.name,
            child_frame_id=stack2.get_base_mount().name,
        ),
    )

    # Combine into facility
    facility = RobotDescription.combine(
        [stack1, stack2],
        entities_to_add=[
            facility_world_joint,
            base_to_stack1_joint,
            base_to_stack2_joint,
        ],
        links_to_add=[facility_world_link, facility_base_link],
        robot_info=RobotInfo(
            name="mrf_facility", type="facility", sku="v1", version="0.0.1"
        ),
    )

    verify_robot_description(facility, "facility")

    print("\n" + "=" * 80)
    print("FINAL RESULT: Complete MRF Facility")
    print("=" * 80)
    print(f"Total links: {len(facility.links)}")
    print(f"Total joints: {len(facility.joints)}")
    print("\nExample naming hierarchy:")
    print("  - stack1_col1_lvl1_rack1_front_left_arm")
    print("  - stack2_col3_lvl2_rack3_front_right_arm")
    print("=" * 80)

    facility_artist = RobotArtist.from_robot_description(facility).block_till_input()

    print(
        "\n✓ Complete! Successfully built complex MRF facility from simple components."
    )


if __name__ == "__main__":

    robot_description = generate_robot_description()
    verify_robot_description(robot_description)
