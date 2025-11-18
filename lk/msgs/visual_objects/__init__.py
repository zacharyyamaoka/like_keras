"""
Visual Objects - Data structures for visualization primitives.

This module provides visual objects that describe what to render, independent of
the rendering backend. Objects are organized into:
- Base classes: VisualObject, CompoundVisualObject
- Primitives: Frame, LineSegments, Box, Arrow, Sphere, etc.
- Compound objects: Path, and other complex visuals built from primitives

Usage:
    import bam.msgs.visual_objects as viz
    
    # Create visual objects
    path = viz.Path(pose_list=[...])
    grid = viz.Grid()
    
    # Draw via artist
    artist.draw(path)
"""

# Base classes
from .VisualObject import VisualObject, VisualId
from .CompoundVisualObject import CompoundVisualObject

# Properties
from .RGBA import RGBA
from .Material import Material

# Primitives
from .Frame import Frame
from .UrdfFrame import UrdfFrame
from .LineSegments import LineSegments
from .Grid import Grid
from .PointCloud import PointCloud
from .Urdf import Urdf

# Markers
from .markers import (
    Marker,
    Arrow,
    Box,
    Cylinder,
    Sphere,
    Mesh,
)

# Compound objects
from .compound_objects import (
    Path,
    RList,
    ScoreCloud,
    GraspPath,
    ParallelGripper,
    ClawGripper,
    GraspGeometry,
    Conveyor,
)

# GUI Controls
from .gui import (
    NamespaceVisibilityGui,
    PlaybackGui,
    PointCloudGui,
    PoseSelectorGui,
    ReachMapGui,
    ScoreCloudGui,
    UrdfJointControlGui,
)

# Plotting
from .plotting import (
    LineChart,
    Histogram,
    Series,
    LinePlot,
    VerticalLine,
    HorizontalLine,
    MoteusActuatorTimeline,
    ActuatorTimelineData,
)

# Binning utilities
from bam.utils import BinConfig

