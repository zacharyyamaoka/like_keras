from dataclasses import dataclass


@dataclass
class Geometry:
    """
        Base class for all geometry types.
        Type is automatically set to the class name (lowercase).
    """
    type: str = ""
    
    def __post_init__(self):
        """Automatically set type to the class name (lowercase)."""
        
        if type(self).__name__ == 'Geometry':
            self.type = "none"
        else:
            self.type = type(self).__name__.lower()
            