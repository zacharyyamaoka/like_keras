"""Python components of the M0.3 consumer tests (CONTRACT §15 A20, A25; kpi/consumer/test_consumer_friction.py): the Python
twin of period_probe.cpp, and the source of A25's arrivals. Module level, so a deployment can rebuild each one in
another process. A25's Python counter is kpi/consumer/fresh_parts.py, apart: its In(fresh=True) refuses to import
until A25 lands, and must not take A20's test down with it."""

from __future__ import annotations

import dataclasses

import numpy as np

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

REFUSAL = "one bus exchange does not fit in a tick: the period is below min_period_s"


class PeriodProbe(lk.Component):
    """What period() says in activate and in forward (A20); refuses in activate when it is below min_period_s."""

    report = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        min_period_s: float = 0.0

    def activate(self):
        self._in_activate = self.period()
        if self._in_activate < self.params.min_period_s:
            raise RuntimeError(REFUSAL)

    def forward(self):
        return JointState(name=("activate", "forward"), position=[self._in_activate, self.period()],
                          header=Header(stamp=self.now()))


class RatelessPeriodProbe(lk.Component):
    """PeriodProbe in a rate-less group (it steps once per report it receives): period() is 0.0 there (A20)."""

    trigger = lk.In(JointState, queue=4)
    report = lk.Out(JointState)

    def activate(self):
        self._in_activate = self.period()

    def forward(self, trigger):
        return JointState(name=("activate", "forward"), position=[self._in_activate, self.period()],
                          header=Header(stamp=self.now()))


class SameObjectSource(lk.Component):
    """Sends one JointTrajectory object, the same object both times, on its steps 1 and 6 (0.1 s and 0.6 s at
    10 Hz), and nothing on the others (A25: the plan a planner sends again, say after a refusal)."""

    plan = lk.Out(JointTrajectory)

    def activate(self):
        self._steps = 0
        self._plan = JointTrajectory(t=np.array([0.0, 0.5]), joint_names=("j1",), positions=np.array([[0.0], [1.0]]),
                                     header=Header(stamp=0.0))

    def forward(self):
        step, self._steps = self._steps, self._steps + 1
        return self._plan if step in (1, 6) else None



class LogSteps(lk.Component):
    """The Python twin of log_steps.cpp (ruler M0.4; CONTRACT §15 A30, T1): "line 0" (info) at 0.3 s, "line 1"
    (warning) at 1.0 s, "line 2" (error) at 1.7 s on the logical clock, then nothing."""

    AT = (0.3, 1.0, 1.7)

    def forward(self):
        index = getattr(self, "_next", 0)
        # WHY - 1e-9: logical times are sums of ticks; 15 x 0.02 need not be exactly 0.3 in binary
        if index >= len(self.AT) or self.now() < self.AT[index] - 1e-9:
            return None
        (self.log.info, self.log.warning, self.log.error)[index]("line %g", float(index))
        self._next = index + 1
        return None
