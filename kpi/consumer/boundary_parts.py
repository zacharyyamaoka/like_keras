"""The like_keras side of A35's tests (CONTRACT §15 A35; consumer-018): a subscriber that keeps every joint state it
receives, and the reference planner with lead 0 that notes when each plan left it.

Module level, so a dora deployment can rebuild each part in a process of its own (CONTRACT §4.1). The raw-dora side
of the A/B is kpi/consumer/raw_arm_node.py; both write the same rows, so one oracle (kpi/consumer/boundary_ab.py)
reads both.
"""

from __future__ import annotations

import dataclasses
import json
import time

import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.trajectory import JointTrajectory

from kpi.consumer.boundary_ab import plan_row
from kpi.tasks.arm_cell.plan import PickTarget
from kpi.tasks.arm_cell.planner import PickPlanner


class SeqLog(lk.Component):
    """A subscriber in a process of its own: every ServoState it receives, its seq and when it arrived (epoch s).
    WHY queue=1000: dora's queue_size is the input's queue (§9.2), so the transport drops nothing for this reader
    over a test; what it misses never left the sender."""

    value = lk.In(ServoState, queue=1000)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, value):
        self._file.write(json.dumps({"seq": int(value.seq), "stamp": float(value.header.stamp), "wall": time.time()})
                         + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class WallPlanner(PickPlanner):
    """The reference planner (§4.6) with lead 0 by default, so a plan runs from the first tick after it arrives (the
    consumer's A/B planner did the same); each plan it returns is noted as plan_row() in `path`. The ports are
    declared again, as kpi/consumer/parts.py's TimedPlanner does."""

    target = lk.In(PickTarget, queue=8)
    state = lk.In(ServoState, stale=0.1)
    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str
        lead_s: float = 0.0

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, target, state):
        plan = super().forward(target, state)
        if plan is None:
            return None
        self._file.write(json.dumps(plan_row(plan, time.time())) + "\n")
        self._file.flush()
        return plan

    def deactivate(self):
        self._file.close()
