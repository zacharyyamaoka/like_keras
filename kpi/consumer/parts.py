"""Probes for the consumer acceptance tests (CONTRACT §10): what the planner side of A1-A3, T2 and T3 needs."""

from __future__ import annotations

import dataclasses
import gc
import json
import time

import like_keras as lk
from robot_msgs.control import ServoState
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory

from kpi.tasks.arm_cell.plan import PickTarget
from kpi.tasks.arm_cell.planner import PickPlanner


class TimedPlanner(PickPlanner):
    """PickPlanner that stamps, into the plan's header.meta, the perf_counter_ns at which forward returns.
    The ports are declared again so the subclass does not depend on how a framework collects inherited ports."""

    target = lk.In(PickTarget, queue=8)
    state = lk.In(ServoState, stale=0.1)
    trajectory = lk.Out(JointTrajectory)

    def forward(self, target, state):
        plan = super().forward(target, state)
        if plan is None:
            return None
        meta = {"returned_ns": str(time.perf_counter_ns())}
        return plan.replace(header=Header(stamp=plan.header.stamp, frame_id=plan.header.frame_id, meta=meta))


class StallingPlanner(PickPlanner):
    """PickPlanner that stalls on its `stall_on`-th target (counted from 0): it plans that target from the arm state
    it read at the target before, as a planner that stalled between reading the state and planning would. Its plan
    then starts where the arm was, not where it is, and the guard sees a runaway (the reference guard's recovery,
    CONTRACT §15 A14). That plan carries header.meta {"stalled": "true"}; every other target is planned as
    PickPlanner plans it. The ports are declared again, as in TimedPlanner."""

    target = lk.In(PickTarget, queue=8)
    state = lk.In(ServoState, stale=0.1)
    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        lead_s: float = 0.05
        stall_on: int = 2

    def forward(self, target, state):
        targets_seen = getattr(self, "_targets_seen", 0)
        self._targets_seen = targets_seen + 1
        before_the_stall = getattr(self, "_state_before", None)
        self._state_before = state
        if targets_seen != self.params.stall_on or before_the_stall is None:
            return super().forward(target, state)
        plan = super().forward(target, before_the_stall)
        stalled = Header(stamp=plan.header.stamp, frame_id=plan.header.frame_id, meta={"stalled": "true"})
        return plan.replace(header=stalled)


class HandoffProbe(lk.Component):
    """Steps right after the planner in the planner's group (a same-group edge is a plain call): the gap between
    the planner's return and this step's start is what the framework did on the planner's thread (A1, A2, T2)."""

    trajectory = lk.In(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")
        self._last = None

    def forward(self, trajectory):
        started = time.perf_counter_ns()
        if trajectory is None or trajectory is self._last or "returned_ns" not in trajectory.header.meta:
            return
        self._last = trajectory
        returned = int(trajectory.header.meta["returned_ns"])
        self._file.write(json.dumps({"gap_s": (started - returned) * 1e-9, "stamp": trajectory.header.stamp}) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class Spinner(lk.Component):
    """T3's hostile neighbour: spins pure Python for most of its 50 ms period, then runs gc.collect()."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        spin_s: float = 0.045

    def forward(self):
        deadline = time.perf_counter() + self.params.spin_s
        garbage = []
        while time.perf_counter() < deadline:
            garbage.append({"x": [1.0] * 8})
            if len(garbage) > 1000:
                garbage.clear()
        gc.collect()
