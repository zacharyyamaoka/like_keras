"""
    RequestHeader for API requests.
    
    ROS-free version using Python dataclasses.
"""

# PYTHON
from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass
class RequestHeader:
    """Header for API requests containing metadata and control flags."""
    
    # Timestamp when request was created/sent
    stamp: float = field(default_factory=time.time)
    
    # Client identifier for debugging
    client_id: str = ""
    
    # Frame this data is associated with
    frame_id: str = ""
    
    # Request type identifier (for routing/handling)
    request_type: int = 0
    
    # Expected processing duration (seconds)
    expected_duration: float = 0.0
    
    # Priority level (higher = more important)
    priority: int = 0
    
    # Control flags
    force_busy: bool = False  # Process even if busy
    force_uncalibrated: bool = False  # Process even if not calibrated
    force_priority: bool = False  # Override priority rules
    clear_priority: bool = False  # Clear priority queue
    blocking: bool = True  # Client will block waiting for response
    
    @classmethod
    def create(cls, client_id: str = "", frame_id: str = "", **kwargs) -> "RequestHeader":
        """Create a RequestHeader with common defaults."""
        return cls(client_id=client_id, frame_id=frame_id, **kwargs)

