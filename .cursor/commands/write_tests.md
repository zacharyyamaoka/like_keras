
# Write Tests


## Test Structure & Organization

### Directory Structure
Tests live in a `tests/` folder at the same level as the library being tested:

```
package_name/
├── package_name/          # Library code
│   ├── __init__.py
│   └── module.py
├── tests/                 # Test files
│   ├── __init__.py
│   ├── test_module.py
│   └── saved_data/        # Regression test data
└── CMakeLists.txt
```

### Test File Naming
- Test files: `test_<module_name>.py`
- Mirror the structure of what you're testing
- Use descriptive names that indicate what's being tested

## Test File Template

```python
#!/usr/bin/env python3

"""
    Test <module/feature name>.
    
    Tests:
    1. <Test case 1 description>
    2. <Test case 2 description>
    3. <Test case 3 description>
"""

# BAM
from package_name import ModuleName
from bam.descriptions import UR  # example

# PYTHON
import pytest
import numpy as np
from pathlib import Path

# Visualization flag - set to False for CI/automated testing
VIZ = False

# Conditional viz imports and setup
if VIZ:
    from bam.common.artist import Artist
    import bam.msgs.visual_objects as viz
    
    artist = Artist()
    artist.attach_viser_viewer()
    artist.draw(viz.Grid())


@pytest.fixture
def example_fixture():
    """Setup common test data or objects."""
    # Setup code
    yield data
    # Teardown code (optional)


def test_basic_functionality():
    """Test basic functionality with detailed output."""
    # Arrange
    input_data = setup_test_data()
    
    # Act
    result = function_under_test(input_data)
    
    # Print for debugging (keep verbose during development)
    print(f"\nInput: {input_data}")
    print(f"Result: {result}")
    print(f"Expected: {expected}")
    
    # Assert
    assert result == expected, f"Expected {expected}, got {result}"
    
    # Visualization (optional)
    if VIZ:
        artist.draw(viz.Frame(name="result_frame"))
        input("Press Enter to continue...")


@pytest.mark.parametrize("input_val, expected", [
    pytest.param(1, 2, id="case_1"),
    pytest.param(2, 4, id="case_2"),
    pytest.param(3, 6, id="case_3"),
])
def test_parametrized(input_val, expected):
    """Test multiple cases efficiently."""
    result = function_under_test(input_val)
    print(f"  {input_val} -> {result} (expected {expected})")
    assert result == expected


if __name__ == "__main__":
    # Run pytest with verbose output
    pytest.main([__file__, "-v"])
```

## Workflow: Making Tests Pass

### Phase 1: Initial Implementation (Verbose & Debugging)

1. **Create test file** with comprehensive print statements
2. **Run tests** and observe failures
3. **Iterate** on implementation/tests until passing
4. **Use lots of prints** to understand what's happening:

```python
def test_transform_calculation():
    """Test coordinate transform calculation."""
    
    # Arrange
    pose_a = PoseStamped.from_xyzrpy([1, 0, 0], [0, 0, 0])
    pose_b = PoseStamped.from_xyzrpy([0, 1, 0], [0, 0, np.pi/2])
    
    print(f"\nPose A:")
    print(f"  xyz: {pose_a.position.to_list()}")
    print(f"  rpy: {pose_a.orientation.to_rpy()}")
    
    print(f"\nPose B:")
    print(f"  xyz: {pose_b.position.to_list()}")
    print(f"  rpy: {pose_b.orientation.to_rpy()}")
    
    # Act
    transform = calculate_transform(pose_a, pose_b)
    
    print(f"\nResulting Transform:")
    print(np.round(transform, 3))
    
    # Assert
    assert np.allclose(transform[0:3, 3], expected_translation)
    print("✓ Translation correct")
    assert np.allclose(transform[0:3, 0:3], expected_rotation)
    print("✓ Rotation correct")
```

### Phase 2: Optimization (Clean & Readable)

Once all tests pass, clean up:

1. **Remove excessive prints** - keep only essential ones
2. **Add clear section comments**
3. **Organize with regions** if file is long
4. **Use fixtures** for repeated setup
5. **Keep helpful print statements** for test status

```python
def test_transform_calculation():
    """Test coordinate transform calculation."""
    pose_a = PoseStamped.from_xyzrpy([1, 0, 0], [0, 0, 0])
    pose_b = PoseStamped.from_xyzrpy([0, 1, 0], [0, 0, np.pi/2])
    
    transform = calculate_transform(pose_a, pose_b)
    
    # Verify translation and rotation
    assert np.allclose(transform[0:3, 3], expected_translation)
    assert np.allclose(transform[0:3, 0:3], expected_rotation)
    print("✓ Transform calculation correct")
```

### Phase 3: Add Visualization (VIZ flag)

After tests are quantitatively passing, add visualization for human verification:

```python
VIZ = False

if VIZ:
    from bam.common.artist import Artist
    import bam.msgs.visual_objects as viz
    
    artist = Artist()
    artist.attach_viser_viewer()
    artist.draw(viz.Grid())


def test_path_planning():
    """Test path planning algorithm."""
    start = PoseStamped.from_xyzrpy([0, 0, 0], [0, 0, 0])
    goal = PoseStamped.from_xyzrpy([1, 1, 0], [0, 0, np.pi/2])
    
    # Run planner
    path = plan_path(start, goal)
    
    # Quantitative checks
    assert len(path) > 0
    assert np.allclose(path[0].position.to_list(), start.position.to_list())
    assert np.allclose(path[-1].position.to_list(), goal.position.to_list())
    print(f"✓ Generated path with {len(path)} waypoints")
    
    # Visual verification
    if VIZ:
        # Draw start and goal
        artist.draw(viz.Frame.from_PoseStamped(start, name="start"))
        artist.draw(viz.Frame.from_PoseStamped(goal, name="goal"))
        
        # Draw planned path
        path_viz = viz.Path.from_pose_list(
            path,
            name="planned_path",
            frame_props=viz.Frame(axis_length=0.05),
            line_props=viz.LineSegments(colors=[viz.RGBA.blue()])
        )
        artist.draw(path_viz)
        
        # Add any obstacles for context
        artist.draw(viz.Box(
            pose=obstacle_pose,
            scale=[0.2, 0.2, 0.3],
            color=viz.RGBA.red(alpha=0.5),
            name="obstacle"
        ))
        
        input("Press Enter to continue...")
        artist.clear_all()
        artist.draw(viz.Grid())
```

## Visualization Patterns with Artist

### Basic Setup Pattern
```python
VIZ = False

if VIZ:
    from bam.common.artist import Artist
    import bam.msgs.visual_objects as viz
    
    artist = Artist()
    artist.attach_viser_viewer()
    artist.draw(viz.Grid())
```

### Robot Visualization Pattern
```python
VIZ = False

if VIZ:
    from bam.common.artist import RobotArtist
    import bam.msgs.visual_objects as viz
    from bam.descriptions import UR
    
    arm = UR.make_UR5e()
    artist = RobotArtist(arm, collision_mesh_mutable=True)
    artist.attach_viser_viewer()
    artist.setup()
```

### In-Test Visualization
```python
def test_collision_detection():
    """Test collision detection between geometries."""
    
    # ... test setup and execution ...
    
    assert is_collision == expected_collision
    print(f"✓ Collision detection: {is_collision}")
    
    # Visualize the scenario
    if VIZ:
        # Draw objects involved
        artist.draw(viz.Box(
            pose=box_a_pose,
            scale=box_a_size,
            color=viz.RGBA.blue(alpha=0.7),
            name="box_a"
        ))
        artist.draw(viz.Box(
            pose=box_b_pose,
            scale=box_b_size,
            color=viz.RGBA.orange(alpha=0.7),
            name="box_b"
        ))
        
        # Show collision points if any
        if is_collision:
            for i, point in enumerate(collision_points):
                artist.draw(viz.Sphere(
                    pose=PoseStamped.from_xyz(point),
                    radius=0.005,
                    color=viz.RGBA.red(),
                    name=f"collision_{i}"
                ))
        
        input("Press Enter to continue...")
        
        # Clean up for next test
        artist.clear_all()
        artist.draw(viz.Grid())
```

## Regression Testing with Saved Data

For complex transforms or calculations, save and verify against reference data:

```python
SAVE = False  # Set True once to generate reference data

@pytest.mark.parametrize("config", [
    pytest.param("ur5e", id="UR5e"),
    pytest.param("ur10", id="UR10"),
])
def test_forward_kinematics(request, config):
    """Test FK matches saved reference values."""
    
    # Load robot
    robot = load_robot(config)
    
    # Calculate FK
    q = get_test_configuration()
    transform = robot.forward_kinematics(q)
    
    # Save or compare
    test_id = request.node.callspec.id
    save_dir = Path(__file__).parent / "saved_transforms"
    save_dir.mkdir(exist_ok=True)
    save_file = save_dir / f"fk_{test_id}.pkl"
    
    if SAVE:
        import pickle
        with open(save_file, 'wb') as f:
            pickle.dump(transform, f)
        print(f"Saved reference to {save_file}")
    else:
        assert save_file.exists(), f"Reference file not found: {save_file}"
        import pickle
        with open(save_file, 'rb') as f:
            reference = pickle.load(f)
        
        assert np.allclose(transform, reference), \
            f"Transform differs from reference (regression detected)"
        print(f"✓ Verified against {save_file}")
```

## Pytest Best Practices

### Use Fixtures for Shared Setup
```python
@pytest.fixture
def robot_arm():
    """Provide a configured robot arm."""
    arm = UR.make_UR5e()
    return arm


@pytest.fixture
def pin_model(robot_arm):
    """Provide PinModel for robot arm."""
    return PinRobotModel.from_robot_description(robot_arm)


def test_with_fixtures(robot_arm, pin_model):
    """Test using fixtures."""
    # robot_arm and pin_model are automatically provided
    q = pin_model.get_q_neutral()
    # ... test logic ...
```

### Parametrize for Multiple Cases
```python
@pytest.mark.parametrize("shape, size, expected_volume", [
    pytest.param("box", (0.1, 0.2, 0.3), 0.006, id="box"),
    pytest.param("cylinder", (0.05, 0.2, 0), np.pi * 0.05**2 * 0.2, id="cylinder"),
    pytest.param("sphere", (0.1, 0, 0), 4/3 * np.pi * 0.1**3, id="sphere"),
])
def test_volume_calculation(shape, size, expected_volume):
    """Test volume calculation for different shapes."""
    volume = calculate_volume(shape, size)
    print(f"  {shape}: {volume:.6f} (expected {expected_volume:.6f})")
    assert np.isclose(volume, expected_volume)
```

### Skip Tests Conditionally
```python
@pytest.mark.skipif(not VIZ, reason="Visualization test only")
def test_visual_quality():
    """Manual test requiring human verification."""
    # ... visual test ...


@pytest.mark.skip(reason="Known issue - tracked in #123")
def test_broken_feature():
    """Test for known broken feature."""
    # ... test code ...
```

## Running Tests

### From Command Line
```bash
# Run all tests in file
pytest test_module.py -v

# Run specific test
pytest test_module.py::test_function_name -v

# Run with output (show prints)
pytest test_module.py -v -s

# Run tests matching pattern
pytest -k "transform" -v
```

### From Within File
```python
if __name__ == "__main__":
    # Run this file's tests
    pytest.main([__file__, "-v"])
    
    # Or run specific tests
    pytest.main([__file__, "-v", "-k", "collision"])
    
    # Or run with output
    pytest.main([__file__, "-v", "-s"])
```

## Key Principles

1. **Quantitative First**: Tests must pass with assertions, not just visual inspection
2. **Visual Second**: Add VIZ flag for human verification after tests pass
3. **Print Liberally**: During development, print everything to understand behavior
4. **Clean Up**: After passing, remove excessive prints but keep helpful status messages
5. **Regression Protection**: Save reference data for complex calculations
6. **Independence**: Each test should run independently
7. **Descriptive**: Use clear names and docstrings explaining what's tested
8. **Artist Preferred**: Use Artist + visual_objects for all visualization needs

## Complete Example

```python
#!/usr/bin/env python3

"""
    Test grasp collision detection.
    
    Tests:
    1. Box geometry grasp progression (clearance -> surface -> compression)
    2. Cylinder geometry grasp progression
    3. Plate geometry grasp progression
"""

# BAM
from bam.env.scenarios import SceneObj
from bam.msgs.bam_msgs.scene import GraspProperties
from bam.utils.pin_utils import PinCollision, PinModel
from bam.msgs import PoseStamped

# PYTHON
import pytest
import numpy as np
import pinocchio as pin

VIZ = False

if VIZ:
    from bam.common.artist import Artist
    import bam.msgs.visual_objects as viz
    
    artist = Artist()
    artist.attach_viser_viewer()
    artist.draw(viz.Grid())


@pytest.fixture
def collision_checker():
    """Provide empty collision checker for obstacle testing."""
    model = pin.Model()
    collision_model = pin.GeometryModel()
    pin_model = PinModel(model, collision_model)
    return PinCollision(pin_model, verbose=False)


@pytest.mark.parametrize("geometry_shape, geometry_size", [
    pytest.param("box", (0.05, 0.03, 0.02), id="box"),
    pytest.param("cylinder", (0.015, 0.05, 0.0), id="cylinder"),
    pytest.param("plate", (0.025, 0.005, 0.0), id="plate"),
])
def test_grasp_geometry_progression(collision_checker, geometry_shape, geometry_size):
    """Test that grasp geometry progression works correctly."""
    
    # Create target object
    target = SceneObj(
        name=f"target_{geometry_shape}",
        geometry_shape=geometry_shape,
        geometry_size=geometry_size,
        grasp=GraspProperties(surface_compression=0.002, surface_clearance=0.010)
    )
    target.pose = PoseStamped()
    
    # Get geometry sizes
    inner = target.get_inner_geometry()
    nominal = target.get_nominal_geometry()
    outer = target.get_outer_geometry()
    
    print(f"\n{geometry_shape}: inner={inner}, nominal={nominal}, outer={outer}")
    
    # Add collision geometries
    pose = target.pose.to_matrix()
    if geometry_shape == "box":
        collision_checker.add_box_obstacle("inner", inner, pose, check_link_names=[])
        collision_checker.add_box_obstacle("nominal", nominal, pose, check_link_names=[])
        collision_checker.add_box_obstacle("outer", outer, pose, check_link_names=[])
    
    # ... rest of test logic ...
    
    if VIZ:
        artist.draw(target.to_grasp_geometry())
        input("Press Enter to continue...")
        artist.clear_all()
        artist.draw(viz.Grid())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

## Summary

**The @mt workflow:**
1. Create test file with verbose prints and VIZ setup
2. Run tests, iterate until all pass quantitatively
3. Clean up prints, optimize for readability
4. Add VIZ blocks for human verification
5. Use Artist + visual_objects for all visualization
6. Keep VIZ=False by default for CI/automated testing
