from dataclasses import dataclass


@dataclass
class JointCalibration:
    """
    Joint calibration description.
    """
    rising_hardstop_position: float = 0.0
    rising_home_position: float = 0.0
    
    falling_hardstop_position: float = 0.0
    falling_home_position: float = 0.0
