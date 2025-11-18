from dataclasses import dataclass

@dataclass
class JointIO:
    """
    Joint comms description.
    """
    can_id: int = 0