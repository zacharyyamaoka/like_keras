from dataclasses import dataclass

"""


Q: Should this build ontop of JointState?
A: Well no, not all actuators will have joint state, for example a light. 

"""

@dataclass
class ActuatorState:    

    ...