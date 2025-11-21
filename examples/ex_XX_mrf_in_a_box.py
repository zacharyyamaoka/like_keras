# How can we make describe the entiry of the mini MRF is the box system? here with diagonistics and everything!!!
# Would be cool as a fun challenge to do it on a single page. Itslike a huge dash board

from lk import System, Node, Agent, Env
from typing import Optional


def generate_system(config: Optional[System.Config] = None) -> System:

    hand = Hand
    arm = Arm
    arm_hand = ArmHand(arm, hand)

    ...

    return System(config=config)
