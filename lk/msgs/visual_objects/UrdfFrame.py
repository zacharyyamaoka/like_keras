from .Frame import Frame
from dataclasses import dataclass

@dataclass
class UrdfFrame(Frame):
    """
        Frame that is linked to a URDF link/frame.
        
        The viewer is responsible for looking up the transform from the URDF
        and updating it when the URDF configuration changes.
        
        Args:
            link_name: Name of the URDF link/frame to track
            urdf_id: Visual ID of the URDF this frame belongs to (automatically set based on name)
    """
    link_name: str = ""
    urdf_id: str = ""  # Will be extracted from the name hierarchy
    
    def __post_init__(self):
        super().__post_init__()
        
        # Extract urdf_id from visual_id (format: "/urdf_id/link_name_frame")
        # visual_id always has leading slash, so split gives ['', 'urdf_id', 'link_name_frame']
        if not self.urdf_id:
            parts = self.visual_id.split("/")
            if len(parts) > 1:
                self.urdf_id = f"/{parts[1]}"  # Restore leading slash to match URDF visual_id format


if __name__ == "__main__":
    frame = UrdfFrame(name="my_urdf/base_link_frame", link_name="base_link")
    print(frame)
    print(f"URDF ID: {frame.urdf_id}")

