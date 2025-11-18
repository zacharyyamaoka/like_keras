#!/usr/bin/env python3

"""
    PlaybackGui - Interactive playback controls for frame-based animations.
"""

# BAM
from ..VisualObject import VisualObject

# PYTHON
from dataclasses import dataclass
from typing import Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Any

@dataclass
class PlaybackGui(VisualObject):
    """
    Generic GUI control for interactive frame-based playback.
    
    Creates:
    - Time display text showing current time
    - Time slider for scrubbing through frames
    - Play/Pause button
    - Next/Previous frame buttons
    - Speed control slider
    - Speed preset buttons
    
    The frame_callback function should have signature:
        frame_callback(frame_idx: int) -> list[VisualObject]
    
    This callback is invoked whenever the frame changes and should
    return the visual objects to display for that frame. Can be used for 
    trajectories, collision animations, scene animations, or any frame-based content.
    
    Example:
        def get_frame(idx: int) -> list[VisualObject]:
            robot.urdf.q = trajectory[idx]
            return [robot.urdf]
        
        gui = PlaybackGui(
            name="Robot Trajectory",
            t=time_array.tolist(),
            frame_callback=get_frame
        )
        artist.draw(gui)
    """
    name: str = "Playback"
    
    # Time parameters
    t: list[float] | None = None  # Time array (seconds)
    num_frames: int = 0
    initial_frame: int = 0
    
    # Playback settings
    initial_playing: bool = True
    default_speed: float = 1.0  # Playback speed multiplier (1.0 = real-time)
    speed_presets: tuple[str, ...] = ("0.25", "0.5", "1.0", "2.0", "4.0")
    
    # Frame callback - returns list of visual objects for the given frame
    # Signature: frame_callback(frame_idx: int) -> list[VisualObject]
    frame_callback: Callable[[int], list['Any']] | None = None
    
    def __post_init__(self):
        if self.t is None:
            self.t = []
        
        # Validate
        if self.frame_callback is None:
            raise ValueError("frame_callback must be provided")
        
        if self.num_frames == 0 and len(self.t) > 0:
            self.num_frames = len(self.t)
        
        if self.num_frames == 0:
            raise ValueError("num_frames must be > 0")
        
        if self.initial_frame >= self.num_frames:
            self.initial_frame = 0

