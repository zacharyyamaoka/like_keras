from dataclasses import dataclass, field
from .arm_params import ArmParams
from .hand_params import HandParams
import copy


@dataclass
class WaypointParams:
    name: str = (
        ""  # Waypoint identifier: pre_pick_3, pre_pick_2, pre_pick, pick, post_pick, etc.
    )
    arm: ArmParams = field(default_factory=ArmParams)
    hand: HandParams = field(default_factory=HandParams)

    def lerp(self, target: "WaypointParams", fraction: float) -> "WaypointParams":
        new_params = copy.deepcopy(self)
        new_params.arm = self.arm.lerp(target.arm, fraction)
        new_params.hand = self.hand.lerp(target.hand, fraction)
        return new_params

    def difference(self, target: "WaypointParams") -> dict[str, float]:
        diff = {}
        arm_diff = self.arm.difference(target.arm)
        for key, value in arm_diff.items():
            diff[f"arm.{key}"] = value
        hand_diff = self.hand.difference(target.hand)
        for key, value in hand_diff.items():
            diff[f"hand.{key}"] = value
        return diff

    @classmethod
    def make_random(cls) -> "WaypointParams":
        waypoint_params = cls()
        waypoint_params.arm = ArmParams.make_random()
        waypoint_params.hand = HandParams.make_random()
        return waypoint_params


if __name__ == "__main__":
    waypoint_params = WaypointParams()
    print(waypoint_params)
