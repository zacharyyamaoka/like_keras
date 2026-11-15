"""File 1 of the reference consumer (CONTRACT §4.6): the planner declares its ports and nothing else."""

import dataclasses

import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory

from kpi.tasks.arm_cell.plan import PickTarget, plan_pick, starting_at


class PickPlanner(lk.Component):
    target = lk.In(PickTarget, queue=8)  # forward runs once per target
    state = lk.In(ServoState, stale=0.1)  # latest; None if older than 0.1 s
    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        lead_s: float = 0.05

    def forward(self, target, state):
        if state is None:
            self.log.warning("no fresh arm state; target skipped")
            return None
        plan = plan_pick(target, start=state.joint_state())  # seconds of numpy
        return starting_at(plan, self.now() + self.params.lead_s)  # absolute start; returning is sending
