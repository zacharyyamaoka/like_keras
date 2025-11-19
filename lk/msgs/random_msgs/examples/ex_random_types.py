#!/usr/bin/env python3

"""
    Comprehensive example of all random types in random_msgs.
    
    Demonstrates:
    - RandomPoseStamped: Random poses with frame_id
    - RandomVector3: Random 3D vectors
    - RandomTransform: Random transformations
    - RandomRGBA: Random colors
    - RandomMaterial: Random materials with color and properties
"""

if __name__ == "__main__":
    # BAM
    from ..geometry_msgs import RandomPoseStamped, RandomVector3, RandomTransform
    from ..visual_objects import RandomRGBA, RandomMaterial
    
    # PYTHON
    import numpy as np
    
    print("\n" + "="*70)
    print("RANDOM_PY_MSGS - Complete Random Types Demo")
    print("="*70)
    
    #region - RandomPoseStamped
    print("\n[1] RandomPoseStamped - Random pose with frame")
    print("-" * 50)
    
    # Fixed pose
    rpose = RandomPoseStamped.fixed([0.5, 0.3, 0.1], [0.0, 0.0, 1.57], frame_id="conveyor")
    pose = rpose.sample()
    print(f"Fixed: frame={pose.header.frame_id}, xyz={pose.xyz}, rpy(deg)={np.rad2deg(pose.rpy)}")
    
    # Uniform distribution
    rpose = RandomPoseStamped.uniform(
        xyz_lower=[0.0, 0.0, 0.0],
        xyz_upper=[1.0, 0.5, 0.2],
        rpy_lower=[0.0, 0.0, -3.14],
        rpy_upper=[0.0, 0.0, 3.14],
        frame_id="world"
    ).with_seed(42)
    
    pose = rpose.sample()
    print(f"Random: xyz={pose.xyz}, rpy(deg)={np.rad2deg(pose.rpy)}")
    #endregion - RandomPoseStamped
    
    #region - RandomVector3
    print("\n[2] RandomVector3 - Random 3D vectors")
    print("-" * 50)
    
    # Fixed vector
    rvec = RandomVector3.fixed(1.0, 2.0, 3.0)
    vec = rvec.sample()
    print(f"Fixed: {vec.to_list()}")
    
    # Uniform distribution
    rvec = RandomVector3.uniform([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).with_seed(42)
    vec = rvec.sample()
    print(f"Random: {vec.to_list()}")
    #endregion - RandomVector3
    
    #region - RandomTransform
    print("\n[3] RandomTransform - Random transformations")
    print("-" * 50)
    
    # Fixed transform
    rtf = RandomTransform.fixed([0.5, 0.3, 0.1], [0.0, 0.0, 1.57])
    tf = rtf.sample()
    print(f"Fixed: translation={tf.translation.to_list()}, rpy(deg)={np.rad2deg(tf.rpy)}")
    
    # Uniform distribution
    rtf = RandomTransform.uniform(
        xyz_lower=[0.0, 0.0, 0.0],
        xyz_upper=[1.0, 1.0, 1.0],
        rpy_lower=[0.0, 0.0, -3.14],
        rpy_upper=[0.0, 0.0, 3.14]
    ).with_seed(42)
    
    tf = rtf.sample()
    print(f"Random: translation={tf.translation.to_list()}, rpy(deg)={np.rad2deg(tf.rpy)}")
    #endregion - RandomTransform
    
    #region - RandomRGBA
    print("\n[4] RandomRGBA - Random colors")
    print("-" * 50)
    
    # Fixed colors
    print(f"Red: {RandomRGBA.red().sample().to_tuple()}")
    print(f"Green: {RandomRGBA.green().sample().to_tuple()}")
    print(f"Blue: {RandomRGBA.blue().sample().to_tuple()}")
    print(f"Grey: {RandomRGBA.grey(0.5).sample().to_tuple()}")
    
    # Uniform distribution
    rrgba = RandomRGBA.uniform([0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0, 1.0]).with_seed(42)
    rgba = rrgba.sample()
    print(f"Random: {rgba.to_tuple()}")
    #endregion - RandomRGBA
    
    #region - RandomMaterial
    print("\n[5] RandomMaterial - Random materials")
    print("-" * 50)
    
    # Fixed materials
    mat = RandomMaterial.red(metallic=0.5, roughness=0.2).sample()
    print(f"Red metal: rgba={mat.color.to_tuple()}, metallic={mat.metallic}, roughness={mat.roughness}")
    
    mat = RandomMaterial.blue(roughness=0.1).sample()
    print(f"Blue smooth: rgba={mat.color.to_tuple()}, roughness={mat.roughness}")
    
    # Uniform distribution
    rmat = RandomMaterial.uniform(
        rgba_lower=[0.0, 0.0, 0.0, 1.0],
        rgba_upper=[1.0, 1.0, 1.0, 1.0],
        metallic_lower=0.0,
        metallic_upper=1.0,
        roughness_lower=0.0,
        roughness_upper=1.0
    ).with_seed(42)
    
    mat = rmat.sample()
    print(f"Random: rgba={mat.color.to_tuple()}, metallic={mat.metallic:.2f}, roughness={mat.roughness:.2f}")
    #endregion - RandomMaterial
    
    print("\n" + "="*70)
    print("✓ All random types working correctly!")
    print("="*70)
    print("\nNow use these in your scenarios by importing:")
    print("  from bam.msgs.random_msgs.geometry_msgs import RandomPoseStamped, RandomVector3, RandomTransform")
    print("  from bam.msgs.random_msgs.visual_objects import RandomRGBA, RandomMaterial")
    print("="*70 + "\n")

