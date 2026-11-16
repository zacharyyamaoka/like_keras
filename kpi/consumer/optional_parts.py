"""A32's Python deadman (CONTRACT §15 A32; kpi/consumer/test_consumer_friction.py), the twin of
kpi/polyglot/verdict.cpp. A module of its own: until A32 lands, lk.In(T, optional=True) refuses at class creation,
and that must fail A32's test only.
"""

from __future__ import annotations

import dataclasses

import numpy as np

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import DiagnosticRecord, Header


class DeadmanPy(lk.Component):
    """verdict.cpp's guard deadman in Python, the same ports: `verdict` is optional (A32, A23's meaning), so a cell
    with no guard leaves it unwired. Unguarded (no verdict ever received) it moves `step_rad` every step; from the
    first verdict on it moves only while the newest verdict is "ok" and judged a state no older than
    guard_timeout_s. WHY forward takes `verdict` with no default: A32 says forward gets None for an unwired optional
    input, so a library that leaves the argument out fails here, as the consumer's private-attribute workaround did."""

    verdict = lk.In(DiagnosticRecord, optional=True)
    command = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        guard_timeout_s: float = 0.3
        step_rad: float = 0.001

    def activate(self):
        self._guarded, self._position = False, 0.0

    def forward(self, verdict):
        if verdict is not None:
            self._guarded = True  # from the first verdict on, motion needs a fresh "ok"
        # WHY + 1e-9: logical times are sums of ticks; 1.25 - 0.95 is not exactly 0.3 in binary (as verdict.cpp)
        confirmed = (verdict is not None and verdict.level == "ok"
                     and self.now() - verdict.header.stamp <= self.params.guard_timeout_s + 1e-9)
        if not self._guarded or confirmed:
            self._position += self.params.step_rad
        return JointState(header=Header(stamp=self.now()), name=("j1",), position=np.array([self._position]))
