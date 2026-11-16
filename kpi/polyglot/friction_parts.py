"""Python ends of the M0.3 and M0.4 polyglot tests (CONTRACT §15 A16, A17, A23, A29;
kpi/polyglot/test_polyglot_friction.py).

Module level, so a deployment can rebuild each part in another process (CONTRACT §4.1: `module:qualname(**params)`).
"""

from __future__ import annotations

import dataclasses
import json
import math

import numpy as np

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import DiagnosticRecord, Header
from robot_msgs.trajectory import JointTrajectory


def _numbers(values) -> list[float | None] | None:
    if values is None:
        return None
    return [None if math.isnan(float(x)) else float(x) for x in values]


class JointStateLog(lk.Component):
    """Writes every JointState it receives as one JSON line: the step time, the names, positions and velocities."""

    value = lk.In(JointState, queue=1000)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, value):
        row = {"now": self.now(), "stamp": value.header.stamp, "name": list(value.name),
               "position": _numbers(value.position), "velocity": _numbers(value.velocity)}
        self._file.write(json.dumps(row) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class GoldenPlan(lk.Component):
    """The golden JointTrajectory (kpi/golden/make_golden.py, a 2-byte and a 3-byte UTF-8 joint name), sent once, on
    the first step: kpi/golden/instances.py's emitter() as a module-level class. WHY (CONTRACT §15 A29, ruler-008):
    emitter() defines its class inside a function, and a component placed in another process is rebuilt as
    module:qualname (§4.1), so A16's dora case could never start."""

    value = lk.Out(JointTrajectory)

    def forward(self):
        from kpi.golden import instances, make_golden

        sent = getattr(self, "_sent", False)
        self._sent = True
        name = "trajectory/JointTrajectory"
        return None if sent else instances.build(name, make_golden.INSTANCES[name])


class SparseStates(lk.Component):
    """A JointState on its steps 1, 2 and 6 (step 0 is the first), position [step]; nothing on the others (A23: a
    writer of an optional queue input that sends now and then). WHY no Params: a tuple does not survive the JSON
    round trip a component placed in another process needs (§4.1)."""

    value = lk.Out(JointState)
    STEPS = (1, 2, 6)

    def forward(self):
        step = getattr(self, "_step", 0)
        self._step = step + 1
        if step not in self.STEPS:
            return None
        return JointState(header=Header(stamp=self.now()), name=("step",), position=np.array([float(step)]))


class PlanSource(lk.Component):
    """A new plan every step, plan k holding k * 1000 + index in every cell of its positions (A17's kept_plan.cpp
    checks every cell against that)."""

    trajectory = lk.Out(JointTrajectory)

    @dataclasses.dataclass(frozen=True)
    class Params:
        samples: int = 100
        joints: int = 7

    def forward(self):
        count = getattr(self, "_count", 0) + 1
        self._count = count
        samples, joints = self.params.samples, self.params.joints
        return JointTrajectory(
            t=np.arange(samples) * 0.01,
            joint_names=tuple(f"j{index + 1}" for index in range(joints)),
            positions=(count * 1000.0 + np.arange(samples * joints, dtype=float)).reshape(samples, joints),
            header=Header(stamp=self.now()),
        )


class VerdictGuard(lk.Component):
    """A guard's verdict as a robot_msgs DiagnosticRecord (CONTRACT §15 A23): header.stamp is the stamp of the state
    it judged (here its own step time), level ok confirms, level error means stop. It confirms while its step time
    is below `confirm_until_s`, then goes silent; from `stop_at_s` on it sends level error instead."""

    verdict = lk.Out(DiagnosticRecord)

    @dataclasses.dataclass(frozen=True)
    class Params:
        confirm_until_s: float = 1e9
        stop_at_s: float = 1e9

    def forward(self):
        now = self.now()
        if now >= self.params.stop_at_s - 1e-9:
            return DiagnosticRecord(header=Header(stamp=now), path="guard", level="error", code="StopRequested",
                                    message="the guard requests a stop")
        if now < self.params.confirm_until_s - 1e-9:
            return DiagnosticRecord(header=Header(stamp=now), path="guard", level="ok")
        return None
