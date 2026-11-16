"""Python ends of the M0.3 polyglot tests (CONTRACT §15 A16, A17, A23; kpi/polyglot/test_polyglot_friction.py).

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
