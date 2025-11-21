#!/usr/bin/env python3

"""
Tests for FrameTransformer class to ensure frame transformations work correctly.
"""

# BAM
from bam.utils.pin_utils import FrameTransformer, PinRobotModel
from bam.descriptions import UR, RobotDescription
from tf_transformations import xyzrpy_to_matrix

# PYTHON
import numpy as np
import pytest


@pytest.fixture(scope="module")
def arm() -> RobotDescription:
    return UR.make_UR5e()


f"""
UR FRAMES

Frames (n = 28):
  Frame 0: universe, type: FIXED_JOINT, parent: 0
  Frame 1: world, type: BODY, parent: 0
  Frame 2: base_joint, type: FIXED_JOINT, parent: 0
  Frame 3: base_link, type: BODY, parent: 0
  Frame 4: base_link-base_fixed_joint, type: FIXED_JOINT, parent: 0
  Frame 5: base, type: BODY, parent: 0
  Frame 6: base_link-base_link_inertia, type: FIXED_JOINT, parent: 0
  Frame 7: base_link_inertia, type: BODY, parent: 0
  Frame 8: shoulder_pan_joint, type: JOINT, parent: 1
  Frame 9: shoulder_link, type: BODY, parent: 1
  Frame 10: shoulder_lift_joint, type: JOINT, parent: 2
  Frame 11: upper_arm_link, type: BODY, parent: 2
  Frame 12: elbow_joint, type: JOINT, parent: 3
  Frame 13: forearm_link, type: BODY, parent: 3
  Frame 14: wrist_1_joint, type: JOINT, parent: 4
  Frame 15: wrist_1_link, type: BODY, parent: 4
  Frame 16: wrist_2_joint, type: JOINT, parent: 5
  Frame 17: wrist_2_link, type: BODY, parent: 5
  Frame 18: wrist_3_joint, type: JOINT, parent: 6
  Frame 19: wrist_3_link, type: BODY, parent: 6
  Frame 20: ee_fixed_joint, type: FIXED_JOINT, parent: 6
  Frame 21: ee_link, type: BODY, parent: 6
  Frame 22: wrist_3-flange, type: FIXED_JOINT, parent: 6
  Frame 23: flange, type: BODY, parent: 6
  Frame 24: flange-tool0, type: FIXED_JOINT, parent: 6
  Frame 25: tool0, type: BODY, parent: 6
  Frame 26: wrist_3_link-ft_frame, type: FIXED_JOINT, parent: 6
  Frame 27: ft_frame, type: BODY, parent: 6

---

What do I want to test?

- 4 cases of linke
    1. base_link to ik_tip
    2. base_link to diff_ik_tip
    3. diff_base_link to ik_tip
    4. diff_base_link to diff_ik_tip
- at a variety of poses
- The different functions should all give the same result (consistent)
- The should match the manual calculations (accurate)
- They shouldn't change over time (stable) [Not high risk not going to test]
 

"""

BASE_LINK = "base_link"
IK_TIP = "tool0"
# DIFF_BASE_LINK = "world" # This is actually the same as base_link
DIFF_BASE_LINK = "base"  # this is in same location but different orientation
# DIFF_IK_TIP = "wrist_3_link" # This is actually the same as tool0
DIFF_IK_TIP = "tcp_world_test"  # this is in same location but different orientation


# Make a new one to clear its cache...
@pytest.fixture(scope="module")
def frame_transformer_shared(arm: RobotDescription):
    """Create FrameTransformer instance for UR5e robot (shared across tests)."""
    return FrameTransformer.from_robot_description(
        arm,
        cache_transforms={DIFF_IK_TIP: IK_TIP, DIFF_BASE_LINK: BASE_LINK},
    )


@pytest.fixture
def frame_transformer(frame_transformer_shared: FrameTransformer):
    """Clear cache for each test to ensure test isolation."""
    frame_transformer_shared.transform_cache.clear()
    return frame_transformer_shared


@pytest.fixture()
def sample_matrix():
    """Create a deterministic sample transformation matrix for testing."""
    return xyzrpy_to_matrix([0.3, 0.2, 0.1], [np.pi / 6, 0, np.pi / 4])


@pytest.fixture(scope="module")
def T_target_link_to_ik_tip(arm: RobotDescription):
    pin_model = PinRobotModel.from_robot_description(arm)
    T_target_link_to_ik_tip = pin_model.get_transform_between_frames(
        DIFF_IK_TIP, IK_TIP, q_update=pin_model.get_q_neutral()
    )
    print(f"T_target_link_to_ik_tip: \n {np.round(T_target_link_to_ik_tip, 3)}")
    return T_target_link_to_ik_tip


@pytest.fixture(scope="module")
def T_frame_id_to_base_link(arm: RobotDescription):
    pin_model = PinRobotModel.from_robot_description(arm)
    T_frame_id_to_base_link = pin_model.get_transform_between_frames(
        DIFF_BASE_LINK, BASE_LINK, q_update=pin_model.get_q_neutral()
    )
    print(f"T_frame_id_to_base_link: \n {np.round(T_frame_id_to_base_link, 3)}")
    return T_frame_id_to_base_link


def test_link_names(frame_transformer: FrameTransformer, arm: RobotDescription):
    """Test that all frames used in tests exist in the robot model."""
    pin_model = PinRobotModel.from_robot_description(arm)
    pin_model.frames_exists([BASE_LINK, IK_TIP, DIFF_BASE_LINK, DIFF_IK_TIP])


@pytest.mark.parametrize(
    "frame_id, target_link",
    [
        (BASE_LINK, IK_TIP),
        (BASE_LINK, DIFF_IK_TIP),
        (DIFF_BASE_LINK, IK_TIP),
        (DIFF_BASE_LINK, DIFF_IK_TIP),
    ],
)
def test_offset_matches_manual_calculation(
    frame_transformer: FrameTransformer,
    frame_id: str,
    target_link: str,
    T_target_link_to_ik_tip: np.ndarray,
    T_frame_id_to_base_link: np.ndarray,
):
    """
    Test that transform calculations match manual calculations for all frame/target combinations.
    Uses the sample pose matrix to test all functions consistently.
    """
    N_TEST_ITER = 10
    np.random.seed(42 + hash(frame_id + target_link) % 1000)
    for i in range(N_TEST_ITER):
        # Generate random transformation matrix
        xyz = np.random.randn(3)
        rpy = np.random.randn(3) * 0.5
        T_frame_id_to_target_link = xyzrpy_to_matrix(xyz, rpy)

        # Step 1: Manually calculate expected result

        # Get transform from target_link to ik_tip (if needed)
        if target_link != IK_TIP:
            T_frame_id_to_ik_tip = T_frame_id_to_target_link @ T_target_link_to_ik_tip
        else:
            T_frame_id_to_ik_tip = T_frame_id_to_target_link

        # Get transform from base_link to frame_id (if needed)
        if frame_id != BASE_LINK:
            T_base_link_to_ik_tip = T_frame_id_to_base_link @ T_frame_id_to_ik_tip
        else:
            T_base_link_to_ik_tip = T_frame_id_to_ik_tip

        # Step 2: Test transform() with matrix
        result_matrix = frame_transformer.transform(
            T_frame_id_to_target_link,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )

        np.testing.assert_array_almost_equal(
            result_matrix,
            T_base_link_to_ik_tip,
            err_msg=f"transform() failed for frame_id={frame_id}, target_link={target_link}",
        )

        # Step 3: Test transform_vectorized() with batch of matrices (duplicate the same matrix)
        T_matrices = np.tile(T_frame_id_to_target_link, (3, 1, 1))
        result_matrices = frame_transformer.transform_vectorized(
            T_matrices,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )

        for i, result_matrix in enumerate(result_matrices):
            np.testing.assert_array_almost_equal(
                result_matrix,
                T_base_link_to_ik_tip,
                err_msg=f"transform_vectorized() matrix {i} failed for frame_id={frame_id}, target_link={target_link}",
            )

        # Step 4: Test transform_vectorized() (duplicate the same matrix)
        T_matrices = np.tile(T_frame_id_to_target_link, (3, 1, 1))
        result_matrices = frame_transformer.transform_vectorized(
            T_matrices,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )

        for i, result_matrix in enumerate(result_matrices):
            np.testing.assert_array_almost_equal(
                result_matrix,
                T_base_link_to_ik_tip,
                err_msg=f"transform_vectorized() matrix {i} failed for frame_id={frame_id}, target_link={target_link}",
            )

        # Step 5: Test transform_vectorized_split() (duplicate translation and rotation)
        t_array = np.tile(T_frame_id_to_target_link[:3, 3], (3, 1))
        R_array = np.tile(T_frame_id_to_target_link[:3, :3], (3, 1, 1))

        t_result, R_result = frame_transformer.transform_vectorized_split(
            t_array,
            R_array,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )

        for i in range(3):
            np.testing.assert_array_almost_equal(
                t_result[i],
                T_base_link_to_ik_tip[:3, 3],
                err_msg=f"transform_vectorized_split() translation {i} failed for frame_id={frame_id}, target_link={target_link}",
            )
            np.testing.assert_array_almost_equal(
                R_result[i],
                T_base_link_to_ik_tip[:3, :3],
                err_msg=f"transform_vectorized_split() rotation {i} failed for frame_id={frame_id}, target_link={target_link}",
            )


@pytest.mark.parametrize(
    "frame_id, target_link, should_be_equal",
    [
        (BASE_LINK, IK_TIP, True),
        (BASE_LINK, DIFF_IK_TIP, False),
        (DIFF_BASE_LINK, IK_TIP, False),
        (DIFF_BASE_LINK, DIFF_IK_TIP, False),
    ],
)
def test_offset_different_frame_id_and_target_link(
    frame_transformer: FrameTransformer,
    frame_id: str,
    target_link: str,
    should_be_equal: bool,
):
    """Test transform with various frame_id and target_link combinations."""
    # Generate random transformation matrix
    np.random.seed(42)
    xyz = np.random.randn(3)
    rpy = np.random.randn(3) * 0.5
    T_input = xyzrpy_to_matrix(xyz, rpy)

    result = frame_transformer.transform(
        T_input,
        input_frame=frame_id,
        input_link=target_link,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )

    # Always check output shape
    assert result.shape == (4, 4)

    # Check whether output should be equal or not
    if should_be_equal:
        np.testing.assert_array_almost_equal(result, T_input)
    else:
        assert not np.allclose(
            T_input, result
        ), f"Transform should modify the matrix for frame_id: {frame_id} and target_link: {target_link}"


@pytest.mark.parametrize(
    "frame_id, target_link",
    [
        (BASE_LINK, IK_TIP),
        (BASE_LINK, DIFF_IK_TIP),
        (DIFF_BASE_LINK, IK_TIP),
        (DIFF_BASE_LINK, DIFF_IK_TIP),
    ],
)
def test_offset_vectorized_consistency(
    frame_transformer: FrameTransformer,
    frame_id: str,
    target_link: str,
):
    """Test that transform_vectorized produces same results as repeated transform calls."""
    # Create list of random transformation matrices
    np.random.seed(42)
    N = 5
    T_list = []
    for _ in range(N):
        xyz = np.random.randn(3)
        rpy = np.random.randn(3) * 0.5
        T = xyzrpy_to_matrix(xyz, rpy)
        T_list.append(T)

    # Get results using single transform
    single_results = [
        frame_transformer.transform(
            T,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )
        for T in T_list
    ]

    # Get results using vectorized transform
    T_array = np.stack(T_list, axis=0)
    vectorized_results = frame_transformer.transform_vectorized(
        T_array,
        input_frame=frame_id,
        input_link=target_link,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )

    # Check that the single and vectorized results are the same, regardless of the frame_id and target_link
    assert len(single_results) == len(vectorized_results)

    for single, vectorized in zip(single_results, vectorized_results):
        np.testing.assert_array_almost_equal(single, vectorized)


def test_offset_matrix_vectorized_identity(frame_transformer: FrameTransformer):
    """Test that identity transforms remain identity when no offset needed."""
    N = 5
    T_matrices = np.tile(np.eye(4), (N, 1, 1))

    result = frame_transformer.transform_vectorized(
        T_matrices,
        input_frame=BASE_LINK,
        input_link=IK_TIP,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )

    assert result.shape == (N, 4, 4)

    # Should remain identity when frame and link match
    for i in range(N):
        np.testing.assert_array_almost_equal(result[i], np.eye(4))


@pytest.mark.parametrize(
    "frame_id, target_link, should_be_equal",
    [
        (BASE_LINK, IK_TIP, True),
        (BASE_LINK, DIFF_IK_TIP, False),
        (DIFF_BASE_LINK, IK_TIP, False),
        (DIFF_BASE_LINK, DIFF_IK_TIP, False),
    ],
)
def test_offset_matrix_vectorized_split_consistency(
    frame_transformer: FrameTransformer,
    frame_id: str,
    target_link: str,
    should_be_equal: bool,
):
    """Test that split version produces same results as matrix version."""
    N = 2
    np.random.seed(42)  # For reproducibility
    T_list = []
    for _ in range(N):
        xyz = np.random.randn(3)
        rpy = np.random.randn(3) * 0.5
        T_list.append(xyzrpy_to_matrix(xyz, rpy))
    T_array = np.stack(T_list, axis=0)
    t_array = T_array[:, :3, 3]
    R_array = T_array[:, :3, :3]

    # Get results from matrix version
    T_result = frame_transformer.transform_vectorized(
        T_array,
        input_frame=frame_id,
        input_link=target_link,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )

    # Get results from split version
    t_result, R_result = frame_transformer.transform_vectorized_split(
        t_array,
        R_array,
        input_frame=frame_id,
        input_link=target_link,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )

    print(f"T_result: {T_result}")
    print(f"t_result: {t_result}")
    print(f"R_result: {R_result}")

    # Compare results
    np.testing.assert_array_almost_equal(T_result[:, :3, 3], t_result)
    np.testing.assert_array_almost_equal(T_result[:, :3, :3], R_result)


def test_transform_cache(frame_transformer: FrameTransformer, sample_matrix):
    """Test that transform caching works correctly."""
    initial_cache_size = len(frame_transformer.transform_cache)

    # First call should add to cache
    frame_transformer.transform(
        sample_matrix,
        input_frame=BASE_LINK,
        input_link=IK_TIP,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )
    cache_size_after_first = len(frame_transformer.transform_cache)

    # Second call should reuse cache
    frame_transformer.transform(
        sample_matrix,
        input_frame=BASE_LINK,
        input_link=IK_TIP,
        output_frame=BASE_LINK,
        output_link=IK_TIP,
    )
    cache_size_after_second = len(frame_transformer.transform_cache)

    # Cache should grow on first call but not on second
    assert cache_size_after_first >= initial_cache_size
    assert cache_size_after_second == cache_size_after_first


def test_offset_invalid_target_link(frame_transformer: FrameTransformer, sample_matrix):
    """Test that invalid target link raises appropriate error."""
    with pytest.raises(Exception):
        frame_transformer.transform(
            sample_matrix,
            input_frame=BASE_LINK,
            input_link="not_a_real_link",
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )


def test_offset_invalid_frame_id(frame_transformer: FrameTransformer, sample_matrix):
    """Test that invalid frame_id raises appropriate error."""
    with pytest.raises(Exception):
        frame_transformer.transform(
            sample_matrix,
            input_frame="not_a_real_frame",
            input_link=IK_TIP,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )


def test_offset_matrix_vectorized_wrong_shape(frame_transformer: FrameTransformer):
    """Test that wrong input shape raises appropriate error."""
    # Wrong shape: (3, 3) instead of (N, 4, 4)
    T_wrong = np.eye(3)

    with pytest.raises(Exception):
        frame_transformer.transform_vectorized(
            T_wrong,
            input_frame=BASE_LINK,
            input_link=IK_TIP,
            output_frame=BASE_LINK,
            output_link=IK_TIP,
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
