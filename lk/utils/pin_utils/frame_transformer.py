#!/usr/bin/env python3

"""
    Generic frame transformer for robot poses.
    
    Transforms poses between arbitrary reference frames and target links.
    Provides efficient vectorized operations with caching for performance.
"""

# BAM
from .pin_robot_model import PinRobotModel

# PYTHON
import numpy as np
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from bam.descriptions import RobotDescription, RobotDescription


class FrameTransformer:
    """
    Generic frame transformer for robot poses.
    
    Transforms poses between arbitrary reference frames and target links:
        T_input_frame_to_input_link → T_output_frame_to_output_link
    
    Example:
        # Transform from world frame tool0 poses to base_link frame ik_tip poses
        transformer = FrameTransformer.from_robot_description(robot_description)
        T_base_to_ik = transformer.transform(
            T_world_to_tool0,
            input_frame="world",
            input_link="tool0",
            output_frame="base_link",
            output_link="ik_tip"
        )
    """
    
    @classmethod
    def from_robot_description(
        cls,
        robot_description: Union['RobotDescription', 'RobotDescription'],
        cache_transforms: dict[str, str] = None
    ):
        pin_model = PinRobotModel.from_robot_description(robot_description)
        return cls(pin_model, cache_transforms)
    
    def __init__(self, pin_model: PinRobotModel, cache_transforms: dict[str, str] = None):
        self.pin_model = pin_model
        self.model = pin_model.model
        self.data = self.model.createData()
        self.transform_cache = {} 

        #TODO CAREFUL, this just works for static frames... as we cache and just update once...

        # Update frames once to enable transforms
        self.pin_model.update_frames(np.zeros(self.model.nq))
        
        # Pre-cache commonly used transforms
        if cache_transforms:
            for from_link, to_link in cache_transforms.items():
                self._get_transform(from_link, to_link)
    
    def transform(
        self,
        T_input: np.ndarray,
        input_frame: str,
        input_link: str,
        output_frame: str,
        output_link: str,
        cache: bool = True
    ) -> np.ndarray:
        """
        Transform a single pose (4x4 matrix) between frames and links.
        
        DescriptionArgs:
            T_input: (4, 4) transformation matrix T_input_frame_to_input_link
            input_frame: Frame the input pose is expressed in
            input_link: Link the input pose represents
            output_frame: Desired reference frame
            output_link: Desired target link
            cache: If True, use cached transforms. If False, compute fresh each time.
            
        Returns:
            (4, 4) transformation matrix T_output_frame_to_output_link
        """

        # print(f"Transforming from frame: {input_frame} link: {input_link} ---> frame: {output_frame} link: {output_link}")
        assert T_input.shape == (4, 4), f"Expected (4, 4) matrix, got {T_input.shape}"
        
        T_result = T_input.copy()
        
        # Step 1: Transform the target link (what the pose represents)
        # T_input_frame_to_input_link → T_input_frame_to_output_link
        if input_link != output_link:
            T_offset = self._get_transform(input_link, output_link, cache=cache)
            T_result = T_result @ T_offset
        
        # Step 2: Transform the reference frame (what the pose is expressed in)
        # T_input_frame_to_output_link → T_output_frame_to_output_link
        if input_frame != output_frame:
            T_offset = self._get_transform(output_frame, input_frame, cache=cache)
            T_result = T_offset @ T_result
        
        return T_result
    
    def transform_vectorized(
        self,
        T_input: np.ndarray,
        input_frame: str,
        input_link: str,
        output_frame: str,
        output_link: str,
        cache: bool = True
    ) -> np.ndarray:
        """
        Transform multiple poses (N x 4x4 matrices) between frames and links.
        
        DescriptionArgs:
            T_input: (N, 4, 4) transformation matrices T_input_frame_to_input_link
            input_frame: Frame the input poses are expressed in
            input_link: Link the input poses represent
            output_frame: Desired reference frame
            output_link: Desired target link
            cache: If True, use cached transforms. If False, compute fresh each time.
            
        Returns:
            (N, 4, 4) transformation matrices T_output_frame_to_output_link
        """
        assert T_input.ndim == 3 and T_input.shape[1] == 4 and T_input.shape[2] == 4, \
            f"Expected (N, 4, 4) matrices, got {T_input.shape}"
        
        T_result = T_input.copy()
        
        # Step 1: Transform the target link (what the poses represent)
        # T_input_frame_to_input_link → T_input_frame_to_output_link
        if input_link != output_link:
            T_offset = self._get_transform(input_link, output_link, cache=cache)
            # Broadcasting: (N, 4, 4) @ (4, 4) -> (N, 4, 4)
            T_result = T_result @ T_offset
        
        # Step 2: Transform the reference frame (what the poses are expressed in)
        # T_input_frame_to_output_link → T_output_frame_to_output_link
        if input_frame != output_frame:
            T_offset = self._get_transform(output_frame, input_frame, cache=cache)
            # Broadcasting: (4, 4) @ (N, 4, 4) -> (N, 4, 4)
            T_result = T_offset @ T_result
        
        return T_result
    
    def transform_vectorized_split(
        self,
        t_input: np.ndarray,
        R_input: np.ndarray | list[np.ndarray],
        input_frame: str,
        input_link: str,
        output_frame: str,
        output_link: str,
        cache: bool = True
    ):
        """
        Transform multiple poses (split translation/rotation) between frames and links.
        
        DescriptionArgs:
            t_input: (N, 3) array of translations
            R_input: (N, 3, 3) array of rotation matrices
            input_frame: Frame the input poses are expressed in
            input_link: Link the input poses represent
            output_frame: Desired reference frame
            output_link: Desired target link
            cache: If True, use cached transforms. If False, compute fresh each time.
            
        Returns:
            Tuple of (t_output, R_output) expressed in output_frame and representing output_link
        """
        R_input = np.array(R_input)
        assert R_input.ndim == 3 and R_input.shape[1] == 3 and R_input.shape[2] == 3, \
            f"Expected (N, 3, 3) matrices, got {R_input.shape}"
        
        # For link transformations, need same number of positions and orientations
        if input_link != output_link:
            assert t_input.shape[0] == R_input.shape[0], \
                f"Need same number of positions and orientations for link transformation, got {t_input.shape[0]} and {R_input.shape[0]}"
        
        t_result = t_input
        R_result = R_input
        
        # Step 1: Transform the target link
        if input_link != output_link:
            T_offset = self._get_transform(input_link, output_link, cache=cache)
            t_offset = T_offset[:3, 3]
            R_offset = T_offset[:3, :3]
            
            # Transform translations: t_new = t_old + R_old @ t_offset
            t_result = t_result + R_result @ t_offset
            # Transform rotations: R_new = R_old @ R_offset (broadcast over N)
            R_result = R_result @ R_offset
        
        # Step 2: Transform the reference frame
        if input_frame != output_frame:
            T_offset = self._get_transform(output_frame, input_frame, cache=cache)
            t_offset = T_offset[:3, 3]
            R_offset = T_offset[:3, :3]
            
            # Transform translations: t_new = t_offset + t_old @ R_offset.T
            t_result = t_offset + t_result @ R_offset.T
            # Transform rotations: R_new = R_offset @ R_old (broadcast over N)
            R_result = R_offset @ R_result
        
        return t_result, R_result
    
    def _get_transform(self, from_link: str, to_link: str, cache: bool = True) -> np.ndarray:
        """
        Get transform between links, optionally using cache.
        
        Args:
            from_link: Source frame/link
            to_link: Target frame/link
            cache: If True, lookup and store in cache. If False, compute fresh.
        """
        if not cache:
            return self.pin_model.get_transform_between_frames(from_link, to_link)
        
        cache_key = f"T_{from_link}_to_{to_link}"
        if cache_key not in self.transform_cache:
            T = self.pin_model.get_transform_between_frames(from_link, to_link)
            self.transform_cache[cache_key] = T
        return self.transform_cache[cache_key]


class MockFrameTransformer(FrameTransformer):
    """Mock that passes through values unchanged - useful for testing."""
    
    @classmethod
    def from_robot_description(
        cls,
        robot_description: Union['RobotDescription', 'RobotDescription'],
        cache_transforms: dict[str, str] = None
    ):
        return cls(None, cache_transforms)
    
    def __init__(self, pin_model: PinRobotModel | None = None, cache_transforms: dict[str, str] = None):
        # Don't call super().__init__ - we don't need a real pin_model
        self.transform_cache = {}
    
    def transform(self, T_input: np.ndarray, input_frame: str, input_link: str, 
                  output_frame: str, output_link: str, cache: bool = True) -> np.ndarray:
        assert T_input.shape == (4, 4)
        return T_input
    
    def transform_vectorized(self, T_input: np.ndarray, input_frame: str, input_link: str,
                            output_frame: str, output_link: str, cache: bool = True) -> np.ndarray:
        assert T_input.ndim == 3 and T_input.shape[1] == 4 and T_input.shape[2] == 4
        return T_input
    
    def transform_vectorized_split(self, t_input: np.ndarray, R_input: np.ndarray | list[np.ndarray],
                                  input_frame: str, input_link: str, output_frame: str, output_link: str,
                                  cache: bool = True):
        R_input = np.array(R_input)
        assert R_input.ndim == 3 and R_input.shape[1] == 3 and R_input.shape[2] == 3
        return t_input, R_input
    
    def _get_transform(self, from_link: str, to_link: str) -> np.ndarray:
        return np.eye(4)


if __name__ == "__main__":
    
    # BAM
    from bam.descriptions import UR
    from bam.msgs.ros_msgs.geometry_msgs import PoseStamped
    
    print("="*80)
    print("FrameTransformer Demo - UR5e Robot")
    print("="*80)
    
    rd = UR.make_UR5e()
    
    # Create transformer
    transformer = FrameTransformer.from_robot_description(rd)
    
    print(f"\nTransformer created with {len(transformer.transform_cache)} cached transforms")
    print(f"Cached: {list(transformer.transform_cache.keys())}")
    
    print("\n" + "="*80)
    print("Test 1: Transform from universe/tool0 to base_link/tool0")
    print("="*80)
    target = PoseStamped.random()
    T_universe_to_tool0 = target.to_matrix().reshape(1, 4, 4)
    T_result = transformer.transform_vectorized(
        T_universe_to_tool0,
        input_frame="universe",
        input_link="tool0",
        output_frame="base_link",
        output_link="tool0"
    )
    print(f"✅ Success: universe/tool0 -> base_link/tool0")
    print(f"   Input: {T_universe_to_tool0.shape}, Output: {T_result.shape}")
    
    print("\n" + "="*80)
    print("Test 2: Transform from world/tool0 to base_link/wrist_3_link")
    print("="*80)
    N = 3
    T_matrices = np.tile(np.eye(4), (N, 1, 1))
    T_result = transformer.transform_vectorized(
        T_matrices,
        input_frame="world",
        input_link="tool0",
        output_frame="base_link",
        output_link="wrist_3_link"
    )
    print(f"✅ Success: world/tool0 -> base_link/wrist_3_link")
    print(f"   Input: {T_matrices.shape}, Output: {T_result.shape}")
    
    print("\n" + "="*80)
    print("Test 3: Split API - transform translations and rotations")
    print("="*80)
    t_array = np.random.randn(5, 3)
    R_array = np.tile(np.eye(3), (5, 1, 1))
    t_result, R_result = transformer.transform_vectorized_split(
        t_array, R_array,
        input_frame="world",
        input_link="tool0",
        output_frame="base_link",
        output_link="tool0"
    )
    print(f"✅ Success: Split API")
    print(f"   Input: {t_array.shape} + {R_array.shape}")
    print(f"   Output: {t_result.shape} + {R_result.shape}")
    
    print("\n" + "="*80)
    print("Test 4: MockFrameTransformer - Pass through")
    print("="*80)
    mock = MockFrameTransformer()
    T_input = np.random.randn(3, 4, 4)
    T_output = mock.transform_vectorized(
        T_input, "frame_a", "link_a", "frame_b", "link_b"
    )
    print(f"✅ Pass through works: {np.allclose(T_input, T_output)}")
    
    print("\n" + "="*80)
    print("✅ FrameTransformer Demo Complete!")
    print("="*80)

