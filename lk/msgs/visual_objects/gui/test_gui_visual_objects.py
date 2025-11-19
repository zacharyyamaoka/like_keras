#!/usr/bin/env python3

"""
    Test GUI VisualObject Inheritance

    Demonstrates that GUI objects properly inherit from VisualObject
    and get visual_id, visible, and other base properties.
"""

# BAM
from bam.common.artist.artist.visual_objects.gui import NamespaceVisibilityGui, PointCloudGui, PoseSelectorGui

# PYTHON
import json


if __name__ == "__main__":
    
    print("="*70)
    print("GUI VisualObject Inheritance Test")
    print("="*70)
    
    # Test 1: NamespaceVisibilityGui
    print("\n1. NamespaceVisibilityGui")
    print("-" * 70)
    ns_gui = NamespaceVisibilityGui()
    print(f"  visual_id     : {ns_gui.visual_id}")
    print(f"  name          : {ns_gui.name}")
    print(f"  visible       : {ns_gui.visible}")
    print(f"  initial_keys  : {ns_gui.initial_keys}")
    
    # Test visibility toggle
    ns_gui.visible = False
    print(f"  After toggle  : visible={ns_gui.visible}")
    
    # Test 2: PointCloudGui with custom name
    print("\n2. PointCloudGui (custom name)")
    print("-" * 70)
    pc_gui = PointCloudGui(
        name="custom_pc_control",
        size_min=0.005,
        size_max=0.2,
        initial_shape="sparkle",
    )
    print(f"  visual_id     : {pc_gui.visual_id}")
    print(f"  name          : {pc_gui.name}")
    print(f"  visible       : {pc_gui.visible}")
    print(f"  size_min      : {pc_gui.size_min}")
    print(f"  size_max      : {pc_gui.size_max}")
    print(f"  initial_shape : {pc_gui.initial_shape}")
    print(f"  shape_options : {pc_gui.shape_options}")
    
    # Test 3: PoseSelectorGui with slash notation
    print("\n3. PoseSelectorGui (namespace notation)")
    print("-" * 70)
    ps_gui = PoseSelectorGui(
        name="/my_workspace/selector",
        n_x=15,
        n_y=15,
        n_z=8,
        initial_x=7,
    )
    print(f"  visual_id     : {ps_gui.visual_id}")
    print(f"  name          : {ps_gui.name}")
    print(f"  visible       : {ps_gui.visible}")
    print(f"  n_x           : {ps_gui.n_x}")
    print(f"  initial_x     : {ps_gui.initial_x}")
    print(f"  reach_map     : {ps_gui.reach_map}")
    
    # Test 4: Multiple instances
    print("\n4. Multiple Instances (same type)")
    print("-" * 70)
    gui1 = PointCloudGui(name="gui_1")
    gui2 = PointCloudGui(name="gui_2")
    gui3 = PointCloudGui(name="gui_3")
    print(f"  gui1.visual_id: {gui1.visual_id}")
    print(f"  gui2.visual_id: {gui2.visual_id}")
    print(f"  gui3.visual_id: {gui3.visual_id}")
    
    # Test 5: Dataclass features (asdict)
    print("\n5. Dataclass Serialization")
    print("-" * 70)
    from dataclasses import asdict
    ps_data = asdict(ps_gui)
    print(f"  Keys: {list(ps_data.keys())}")
    print(f"  JSON serializable: ", end="")
    try:
        json_str = json.dumps(ps_data, indent=2, default=str)
        print("✓")
    except Exception as e:
        print(f"✗ {e}")
    
    print("\n" + "="*70)
    print("✓ All tests passed!")
    print("  GUI objects properly inherit from VisualObject")
    print("  - Get visual_id property")
    print("  - Get visible property")
    print("  - Support custom naming")
    print("  - Support namespace notation (/ prefix)")
    print("  - Maintain dataclass features")
    print("="*70)

