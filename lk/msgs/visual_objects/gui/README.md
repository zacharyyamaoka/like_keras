# GUI Visual Objects

Lightweight data structures for GUI controls that inherit from `VisualObject`.

## Overview

GUI objects describe interactive controls to be rendered by visualization backends (like Viser). They are lightweight data structures that only store configuration - the actual rendering logic lives in the viewer.

## Benefits of VisualObject Inheritance

All GUI objects inherit from `VisualObject`, providing:

- **`visual_id`**: Unique identifier with slash notation support
- **`name`**: Human-readable name
- **`visible`**: Visibility toggle
- **Auto-naming**: Automatic unique IDs when name not provided
- **Dataclass features**: Serialization, comparison, etc.

## Available GUI Objects

### 1. NamespaceVisibilityGui

Control visibility of visual objects by namespace.

```python
from visual_objects import NamespaceVisibilityGui

gui = NamespaceVisibilityGui(
    name="Pipeline Control",
    initial_keys=["stage", "path"],
)
viewer.draw_namespace_visibility_gui(gui)
```

**Features:**
- Text input for comma-separated namespace filter keys
- Dropdown to select which namespace to show/hide
- "Show All" and "Hide All" options

### 2. PointCloudGui

Control point cloud visualization properties.

```python
from visual_objects import PointCloudGui

gui = PointCloudGui(
    name="Point Cloud Settings",
    size_min=0.001,
    size_max=0.1,
    size_initial=0.02,
    initial_shape="circle",
)
handles = viewer.draw_point_cloud_gui(gui)
# Returns: {'size': slider_handle, 'shape': dropdown_handle}
```

**Features:**
- Slider to adjust point size of all point clouds
- Dropdown to change point shape of all point clouds
- Available shapes: `"square"`, `"diamond"`, `"circle"`, `"rounded"`, `"sparkle"`
- Configurable min/max/step/initial values

### 3. PoseSelectorGui

Select poses from discretized workspace using sliders.

```python
from visual_objects import PoseSelectorGui

gui = PoseSelectorGui(
    name="Workspace Selector",
    n_x=20, n_y=20, n_z=10,
    n_view_idx=8, n_rot_idx=12,
    initial_x=10, initial_y=10, initial_z=5,
)
viewer.draw_pose_selector_gui(gui)
```

**Features:**
- 5 sliders: x, y, z, view_idx, rot_idx
- Optional ReachMap integration for automatic URDF updates
- Optional target frame display

**Advanced Usage with ReachMap:**
```python
gui = PoseSelectorGui(
    name="Workspace Explorer",
    n_x=20, n_y=20, n_z=10,
    reach_map=reach_map,              # ReachMap object
    target_urdf_name=urdf.visual_id,  # URDF to update
    show_target_frame=True,           # Show verification frame
)
```

## Usage Pattern

All GUI objects follow the same pattern:

1. **Create GUI object** (lightweight, just configuration):
   ```python
   gui = SomeGui(name="my_gui", param1=value1, ...)
   ```

2. **Draw via viewer** (creates actual UI controls):
   ```python
   handles = viewer.draw_some_gui(gui)
   ```

3. **Access properties**:
   ```python
   print(gui.visual_id)  # Inherited from VisualObject
   print(gui.visible)    # Inherited from VisualObject
   print(gui.param1)     # Specific to this GUI
   ```

## Naming Conventions

GUI objects support flexible naming:

```python
# Default name
gui = PointCloudGui()
# visual_id: "/Point Cloud Controls"

# Custom name
gui = PointCloudGui(name="my_control")
# visual_id: "/my_control"

# Namespace notation
gui = PointCloudGui(name="/workspace/controls")
# visual_id: "/workspace/controls"
```

## Testing

Run the test to verify inheritance:
```bash
python3 visual_objects/gui/test_gui_visual_objects.py
```

## Architecture

```
VisualObject (base class)
├── name: str
├── visible: bool
└── visual_id: property

    ↓ inherits

GUI Objects (NamespaceVisibilityGui, PointCloudGui, PoseSelectorGui)
├── All VisualObject properties
└── GUI-specific configuration fields
```

### 4. ReachMapGui

Explore reachability maps with interactive controls.

```python
from visual_objects import ReachMapGui
from bam_reach import ReachMap

# Load reach map
reach_map = ReachMap.load("path/to/reach_map.pkl")

# Create GUI with reach map integration
gui = ReachMapGui(
    name="ReachMap Explorer",
    reach_map=reach_map,
    target_urdf_name=urdf.visual_id,
    neutral_q=np.zeros(6),  # Fallback for invalid poses
    show_target_frame=True,
    initial_score_min=0.5,
    initial_score_max=1.0,
)
handles = viewer.draw_reachmap_gui(gui)
# Returns: {'pos_idx': handle, 'orient_idx': handle, 
#           'score_range': multi_slider_handle}
```

**Features:**
- Position index slider: Select position in workspace
- Orientation index slider: Select orientation at position
- Score range multi-slider: Filter by reachability score (single slider with min/max handles)
- Automatic URDF updates when sliders change
- Target frame display at FK solution
- Graceful handling of invalid poses (returns to neutral)

**Without ReachMap:**
```python
# Just sliders that print values
gui = ReachMapGui(name="Simple Explorer")
handles = viewer.draw_reachmap_gui(gui)
```

## Future GUI Objects

Planned additions:
- `ClickableMeshGui`: Click meshes to command robot via IK
- `TimelineGui`: Scrub through temporal data

See `bam_artist/dev/urdf_gui_proposal.py` for detailed proposals.

