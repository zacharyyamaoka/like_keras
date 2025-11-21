from .inertial_properties import InertialProperties
from dataclasses import dataclass, field


@dataclass
class PhysicalProperties:
    inertial: InertialProperties = field(default_factory=InertialProperties)
    friction: float = 0.5  # coefficient of friction
    elasticity: float = 0.5  # coefficient of elasticity
    restitution: float = 0.5  # coefficient of restitution
