"""Python components of the M0.3 facility tests (CONTRACT §15 A18, A22; kpi/facilities/test_facilities_friction.py), the
Python twins of hand_back.cpp and self_health.cpp; and of ruler M0.7's (A41, A42): the twin of described_params.cpp and
the writer of refused_pushes.cpp.

Module level, so a deployment can rebuild each part in another process (CONTRACT §4.1: `module:qualname(**params)`).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
from robot_msgs.sensor import JointState
from robot_msgs.std import Header

import like_keras as lk


class HandBackPy(lk.Component):
    """Its deactivate writes the file `path` and logs once (A18: a signal must not end a deployment without it)."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def forward(self):
        return None

    def deactivate(self):
        with open(self.params.path, "w", encoding="utf-8") as file:
            file.write("handed back\n")
        self.log.info("handed back")


class SelfHealth(lk.Component):
    """A22's four phases on the logical clock, as self_health.cpp: health("error", "Refused") at the first step,
    health("warn", "Degraded") from 2.5 s, health("ok", "") from 5.5 s, one log.error from 7.5 s. `health` is both
    the facility port (wired from outside) and, called, the component's own level and code."""

    def forward(self):
        phase, now = getattr(self, "_phase", 0), self.now()
        if phase == 0:
            self.health("error", "Refused")
            self._phase = 1
        elif phase == 1 and now >= 2.5:
            self.health("warn", "Degraded")
            self._phase = 2
        elif phase == 2 and now >= 5.5:
            self.health("ok", "")
            self._phase = 3
        elif phase == 3 and now >= 7.5:
            self.log.error("refused: %s", "StartTooFar")
            self._phase = 4


class TieHealth(lk.Component):
    """The Python twin of tie_health.cpp (ruler M0.4; CONTRACT §15 A22, §4.5): health("error", "Braked") at the
    first step, log.error("brake held: %s", "J2") from 1.5 s, health("warn", "Slipping") with a warning from 2.5 s."""

    def forward(self):
        phase, now = getattr(self, "_phase", 0), self.now()
        if phase == 0:
            self.health("error", "Braked")
            self._phase = 1
        elif phase == 1 and now >= 1.5:
            self.log.error("brake held: %s", "J2")
            self._phase = 2
        elif phase == 2 and now >= 2.5:
            self.health("warn", "Slipping")
            self.log.warning("slipping: %s", "J3")
            self._phase = 3


# ---------------------------------------------------------------- ruler M0.7 (CONTRACT §15 A43)


def _default_limits() -> list[float]:
    return [-1.5, 1.5]


def _default_joints() -> list[dict]:
    return [{"name": "shoulder", "offset": 0.25}]


class DescribedParamsPy(lk.Component):
    """The Python twin of described_params.cpp (A41): the same Params, field by field, with the same defaults. WHY
    lists, never tuples: a component placed in another process is rebuilt from its params' JSON round trip (§4.1),
    and a tuple comes back a list."""

    echo = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        gain: float = 0.5
        retries: int = 3
        armed: bool = True
        adapter: str = "0607623A"
        timeout_s: float = math.nan  # NaN = off (A33)
        limits: list = dataclasses.field(default_factory=_default_limits)
        joints: list = dataclasses.field(default_factory=_default_joints)

    def forward(self):
        return None


class SignedSteps(lk.Component):
    """A JointState on each of its first `count` steps (A42): position [k] at step k = 1, 2, 3, ..., the odd ones
    negated (-1, 2, -3, 4, ...), so refused_pushes.cpp's conversion refuses exactly the odd ones."""

    sample = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        count: int = 10

    def forward(self):
        step = getattr(self, "_step", 0) + 1
        self._step = step
        if step > self.params.count:
            return None
        value = float(step) if step % 2 == 0 else -float(step)
        return JointState(header=Header(stamp=self.now()), name=("sample",), position=np.array([value]))
