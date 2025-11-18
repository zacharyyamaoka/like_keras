#! /usr/bin/env python3

from pin_utils import bbox_bounds, pin_models_from_robot_description
from bam.descriptions import UR

arm = UR.make_UR5e()

center, extents = bbox_bounds(pin_models_from_robot_description(arm)[0], arm.ik_tip)

print(center, extents)