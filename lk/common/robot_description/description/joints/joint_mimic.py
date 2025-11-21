from dataclasses import dataclass


@dataclass
class JointMimic:
    """
    Joint mimic description.
    """

    joint: str = ""
    multiplier: float = 1.0
    offset: float = 0.0
    enabled: bool = False

    def is_enabled(self) -> bool:
        return self.enabled
