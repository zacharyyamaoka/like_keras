"""A25's Python counter (CONTRACT §15 A25; kpi/consumer/test_consumer_friction.py), the twin of arrivals.cpp. A module of
its own: until A25 lands, In(fresh=True) refuses at class creation, and that must fail A25's test only."""

from __future__ import annotations

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory


class Arrivals(lk.Component):
    """Counts arrivals with lk.In(T, fresh=True) (A25): forward gets the plan only on the step after it arrived,
    else None, whatever its identity or content. Each step sends (arrived 0/1, arrivals so far), as arrivals.cpp."""

    plan = lk.In(JointTrajectory, fresh=True)
    count = lk.Out(JointState)

    def forward(self, plan):
        arrived = plan is not None
        self._arrivals = getattr(self, "_arrivals", 0) + arrived
        return JointState(name=("arrived", "arrivals"), position=[float(arrived), float(self._arrivals)],
                          header=Header(stamp=self.now()))
