# ReachMapGui Integration Guide

## Overview

The `ReachMapGui` provides interactive exploration of reachability maps with automatic robot visualization updates.

## Basic Features

### 1. Position & Orientation Selection
- **Position Index Slider**: Navigate through workspace positions (0 to n_positions-1)
- **Orientation Index Slider**: Select orientation at current position (0 to n_orientations-1)

### 2. Score Filtering
- **Score Range Multi-Slider**: Filter visualization by reachability score (min/max)
- Single slider with two handles for intuitive range selection
- Useful for hiding low-quality or unreachable poses

### 3. Robot Visualization
- Automatically updates URDF when sliders change
- Shows target frame at FK solution for verification
- Returns to neutral configuration when pose is invalid

## Usage Patterns

### Pattern 1: Simple Exploration (No ReachMap)

Just creates sliders that print values:

```python
from visual_objects import ReachMapGui

gui = ReachMapGui(name="Simple Explorer")
handles = viewer.draw_reachmap_gui(gui)

# Use handle values programmatically
pos_idx = handles['pos_idx'].value
orient_idx = handles['orient_idx'].value
```

### Pattern 2: Full Integration (With ReachMap)

Automatically updates robot and displays frames:

```python
from visual_objects import ReachMapGui, Urdf
from bam_reach import ReachMap
from bam_descriptions import UR
import numpy as np

# Load reach map
reach_map = ReachMap.load("workspace_map.pkl")

# Setup robot
arm = UR.make_UR5e()
urdf = Urdf.from_robot_description(arm)
viewer.draw_urdf(urdf)

# Create integrated GUI
gui = ReachMapGui(
    name="Workspace Explorer",
    reach_map=reach_map,
    target_urdf_name=urdf.visual_id,
    neutral_q=np.zeros(6),  # Home position
    show_target_frame=True,
    initial_pos_idx=50,  # Start in middle
    initial_orient_idx=0,
    initial_score_min=0.5,
    initial_score_max=1.0,
)

handles = viewer.draw_reachmap_gui(gui)
```

### Pattern 3: With ScoreCloud Visualization

Combine with ScoreCloud for colored reachability visualization:

```python
from visual_objects import ReachMapGui, ScoreCloud
from bam_reach import ReachMap, colorize_reach

# Load and colorize reach map
reach_map = ReachMap.load("workspace_map.pkl")
colors = colorize_reach(reach_map)

# Create score cloud
score_cloud = ScoreCloud.from_reach_map(
    reach_map=reach_map,
    colors=colors,
    colormap="RdYlGn",
)
viewer.draw(score_cloud)

# Add interactive GUI
gui = ReachMapGui(
    reach_map=reach_map,
    target_urdf_name=urdf.visual_id,
    neutral_q=np.zeros(6),
)
handles = viewer.draw_reachmap_gui(gui)
```

## Behavior Details

### Valid Pose Selected
When sliders select a valid pose (ik_success == True):
1. Update URDF to joint configuration `q`
2. Display target frame at FK solution
3. Print confirmation with indices

### Invalid Pose Selected
When sliders select an invalid pose (ik_success == False):
1. Return URDF to `neutral_q` (if provided)
2. Print warning message
3. Don't display target frame

### Handling Missing neutral_q
If `neutral_q` not provided and pose is invalid:
- URDF stays at last valid configuration
- Warning printed to console
- Continue allowing slider movement

## Parameters Reference

```python
@dataclass
class ReachMapGui(VisualObject):
    name: str = "ReachMap Explorer"
    
    # ReachMap integration
    reach_map: Optional[ReachMap] = None          # ReachMap object
    target_urdf_name: Optional[str] = None        # Which URDF to update
    neutral_q: Optional[np.ndarray] = None        # Fallback configuration
    
    # Visualization
    show_target_frame: bool = True                # Show FK frame
    frame_name: str = "reachmap_target"           # Frame visual_id
    
    # Initial values
    initial_pos_idx: int = 0
    initial_orient_idx: int = 0
    initial_score_min: float = 0.0
    initial_score_max: float = 1.0
    
    # Score range
    score_min: float = 0.0
    score_max: float = 1.0
    score_step: float = 0.01
```

## Return Value

```python
handles = viewer.draw_reachmap_gui(gui)

# Dictionary with 3 handles:
handles['pos_idx']      # Position index slider
handles['orient_idx']   # Orientation index slider
handles['score_range']  # Score range multi-slider (returns tuple)

# Access current values:
current_pos = handles['pos_idx'].value
current_orient = handles['orient_idx'].value
score_min, score_max = handles['score_range'].value  # Unpack tuple
```

## Integration with Score Filtering

The score sliders currently print values. For actual filtering:

1. Use `ScoreCloud` instead of `PointCloud`
2. Update `ScoreCloud` when score sliders change
3. Filter points based on score threshold
4. Rebuild point cloud visualization

(This functionality to be implemented - see comments in ex_ur_traj_gen.py)

## Tips

- Start with middle indices for better exploration
- Use `neutral_q = np.zeros(6)` for most robots
- Set score thresholds to hide low-quality poses
- Enable `show_target_frame` to verify IK/FK consistency
- Multiple URDFs? Use dropdown to select target

## See Also

- `ex_reachmap_gui.py`: Basic example without ReachMap
- `ex_ur_traj_gen.py`: Full example with ReachMap integration
- `meshcat_map_viewer.py`: Original inspiration for this GUI
- `ReachMap` class documentation in bam_reach

