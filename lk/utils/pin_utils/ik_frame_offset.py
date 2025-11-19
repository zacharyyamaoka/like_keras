#!/usr/bin/env python3

"""
    Backward compatible wrapper for FrameTransformer.
    
    IkFrameOffset provides the old API with hardcoded base_link/ik_tip frames.
    New code should use FrameTransformer directly for more flexibility.
"""

# BAM
from .frame_transformer import FrameTransformer, MockFrameTransformer
from .pin_robot_model import PinRobotModel

# PYTHON
import numpy as np
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription, RobotDescription


class IkFrameOffset(FrameTransformer):
    """
    Backward compatible wrapper for FrameTransformer with hardcoded base_link/ik_tip.
    
    Old API: Always transforms to base_link frame and ik_tip link.
    New code should use FrameTransformer directly for more flexibility.
    """
    
    @classmethod
    def from_robot_description(
        cls,
        robot_description: Union['RobotDescription', 'RobotDescription'],
        cache_transforms: dict[str, str] = None,
        base_link: str = None,
        ik_tip: str = None
    ):
        pin_model = PinRobotModel.from_robot_description(robot_description)
        base_link = base_link if base_link is not None else robot_description.base_link
        ik_tip = ik_tip if ik_tip is not None else robot_description.ik_tip
        return cls(pin_model, base_link, ik_tip, cache_transforms)
    
    def __init__(
        self,
        pin_model: PinRobotModel,
        base_link: str,
        ik_tip: str,
        cache_transforms: dict[str, str] = None
    ):
        # Build cache_transforms dict for parent class
        _cache_transforms = {}
        if cache_transforms:
            for key, value in cache_transforms.items():
                # The only useful transforms are ones related to ik_tip or base_link
                assert key == ik_tip or key == base_link or \
                       value == ik_tip or value == base_link, \
                       f"Not useful transform: {key} -> {value}"
                _cache_transforms[key] = value
        
        # Always cache the base transform
        _cache_transforms[base_link] = ik_tip
        
        # Initialize parent with cache AND base_link/ik_tip
        super().__init__(pin_model, _cache_transforms, base_link, ik_tip)
        
        # Keep references for compatibility
        self.model = pin_model.model
        self.data = self.model.createData()

    def offset_matrix_vectorized(
        self,
        T_frame_id_to_target_link: np.ndarray,
        frame_id: str,
        target_link: str
    ) -> np.ndarray:
        """
        Vectorized version of offset that transforms multiple poses at once using 4x4 matrices.
        
        DescriptionArgs:
            T_frame_id_to_target_link: (N, 4, 4) array of transformation matrices in frame_id
            frame_id: The frame these poses are expressed in
            target_link: The target link to offset to
            
        Returns:
            (N, 4, 4) array of transformation matrices T_base_link_to_ik_tip expressed in base_link frame
        """
        # Use new generic API with hardcoded output_frame and output_link
        return self.transform_vectorized(
            T_frame_id_to_target_link,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=self.base_link,
            output_link=self.ik_tip
        )


    def offset_matrix_vectorized_split(
        self,
        t_frame_id_to_target_link: np.ndarray,
        R_frame_id_to_target_link: np.ndarray | list[np.ndarray],
        frame_id: str,
        target_link: str
    ):
        """
        Vectorized version of offset that transforms multiple poses at once.
        
        DescriptionArgs:
            t_frame_id_to_target_link: (N, 3) array of translations in frame_id
            R_frame_id_to_target_link: (N, 3, 3) array of rotation matrices in frame_id
            frame_id: The frame these poses are expressed in
            target_link: The target link to offset to
            
        Returns:
            Tuple of (t_base_link_to_ik_tip, R_base_link_to_ik_tip) expressed in base_link frame
        """
        # Use new generic API with hardcoded output_frame and output_link
        return self.transform_vectorized_split(
            t_frame_id_to_target_link,
            R_frame_id_to_target_link,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=self.base_link,
            output_link=self.ik_tip
        )

    def offset(
        self,
        T_frame_id_to_target_link: np.ndarray,
        frame_id: str,
        target_link: str
    ) -> np.ndarray:
        """
        Simple offset that transforms a single pose using a 4x4 matrix.
        
        DescriptionArgs:
            T_frame_id_to_target_link: (4, 4) transformation matrix in frame_id
            frame_id: The frame this pose is expressed in
            target_link: The target link to offset to
            
        Returns:
            (4, 4) transformation matrix T_base_link_to_ik_tip expressed in base_link frame
        """
        # Use new generic API with hardcoded output_frame and output_link
        return self.transform(
            T_frame_id_to_target_link,
            input_frame=frame_id,
            input_link=target_link,
            output_frame=self.base_link,
            output_link=self.ik_tip
        )


class MockIkFrameOffset(MockFrameTransformer):
    """Mock that passes through values unchanged - useful for testing."""
    
    def __init__(self, pin_model: PinRobotModel | None = None, base_link: str = "base_link", ik_tip: str = "tool0", cache_transforms: dict[str, str] = None):
        super().__init__(pin_model, cache_transforms)
        self.base_link = base_link
        self.ik_tip = ik_tip
        self.model = None
        self.data = None
    
    @classmethod
    def from_robot_description(
        cls,
        robot_description: Union['RobotDescription', 'RobotDescription'],
        cache_transforms: dict[str, str] = None,
        base_link: str = None,
        ik_tip: str = None
    ):
        base_link = base_link if base_link is not None else robot_description.base_link
        ik_tip = ik_tip if ik_tip is not None else robot_description.ik_tip
        return cls(None, base_link, ik_tip, cache_transforms)
    
    def offset_matrix_vectorized(self, T_frame_id_to_target_link: np.ndarray, frame_id: str, target_link: str) -> np.ndarray:
        assert T_frame_id_to_target_link.shape[1] == 4 and T_frame_id_to_target_link.shape[2] == 4
        return T_frame_id_to_target_link

    def offset_matrix_vectorized_split(self, t_frame_id_to_target_link: np.ndarray, R_frame_id_to_target_link: np.ndarray | list[np.ndarray], frame_id: str, target_link: str):
        R_frame_id_to_target_link = np.array(R_frame_id_to_target_link)
        assert R_frame_id_to_target_link.ndim == 3 and R_frame_id_to_target_link.shape[1] == 3 and R_frame_id_to_target_link.shape[2] == 3
        return t_frame_id_to_target_link, R_frame_id_to_target_link

    def offset(self, T_frame_id_to_target_link: np.ndarray, frame_id: str, target_link: str) -> np.ndarray:
        assert T_frame_id_to_target_link.shape == (4, 4)
        return T_frame_id_to_target_link


if __name__ == "__main__":

    # BAM
    from bam.descriptions import UR
    from bam.msgs.ros_msgs.geometry_msgs import PoseStamped
    
    # PYTHON
    import numpy as np
    
    print("="*80)
    print("IkFrameOffset Demo - UR5e Robot")
    print("="*80)
    
    rd = UR.make_UR5e()
    
    # Method 1: From robot description (convenience)
    ik_frame_offset = IkFrameOffset.from_robot_description(rd)
    
    # Method 2: From PinModel directly (if you already have one)
    # pin_model = PinRobotModel.from_robot_description(rd)
    # ik_frame_offset = IkFrameOffset(pin_model, rd.ik_tip, rd.base_link)
    
    print(f"\nIK Tip: {ik_frame_offset.ik_tip}")
    print(f"Base Link: {ik_frame_offset.base_link}")
    print(f"Cached transforms: {list(ik_frame_offset.transform_cache.keys())}")
    
    print("\n" + "="*80)
    print("Test 1: Offset from universe to tool0 (matrix API)")
    print("="*80)
    target = PoseStamped.random()
    T_universe_to_tool0 = target.to_matrix().reshape(1, 4, 4)
    T_result = ik_frame_offset.offset_matrix_vectorized(T_universe_to_tool0, "universe", "tool0")
    print(f"✅ Success: universe -> {ik_frame_offset.base_link}")
    print(f"   Input shape: {T_universe_to_tool0.shape}, Output shape: {T_result.shape}")
    
    print("\n" + "="*80)
    print("Test 2: Offset from base_link to ik_tip (split API)")
    print("="*80)
    t_array = np.random.randn(5, 3)  # 5 positions
    R_array = np.tile(np.eye(3), (3, 1, 1))  # 3 orientations
    # Note: Different # of positions and orientations only works when target_link == ik_tip
    t_result, R_result = ik_frame_offset.offset_matrix_vectorized_split(
        t_array, R_array, "base_link", rd.ik_tip
    )
    print(f"✅ Success: base_link -> {ik_frame_offset.base_link}")
    print(f"   Input: {t_array.shape} positions, {R_array.shape} rotations")
    print(f"   Output: {t_result.shape} positions, {R_result.shape} rotations")

    print("\n" + "="*80)
    print("Test 3: Invalid target link (should fail)")
    print("="*80)
    try:
        T_test = np.eye(4).reshape(1, 4, 4)
        _ = ik_frame_offset.offset_matrix_vectorized(T_test, "universe", "not_a_real_link")
        print("❌ Should have failed!")
    except Exception as e:
        print(f"✅ Expected error caught: {type(e).__name__}")

    print("\n" + "="*80)
    print("Test 4: Invalid frame_id (should fail)")
    print("="*80)
    try:
        T_test = np.eye(4).reshape(1, 4, 4)
        _ = ik_frame_offset.offset_matrix_vectorized(T_test, "not_a_real_link", "tool0")
        print("❌ Should have failed!")
    except Exception as e:
        print(f"✅ Expected error caught: {type(e).__name__}")
    
    print("\n" + "="*80)
    print("Test 5: MockIkFrameOffset - Pass through")
    print("="*80)
    mock_ik = MockIkFrameOffset(base_link="base_link", ik_tip="tool0")
    T_input = np.random.randn(3, 4, 4)
    T_output = mock_ik.offset_matrix_vectorized(T_input, "frame_a", "frame_b")
    print(f"✅ Pass through works: {np.allclose(T_input, T_output)}")
    
    print("\n" + "="*80)
    print("✅ IkFrameOffset Demo Complete!")
    print("="*80)

