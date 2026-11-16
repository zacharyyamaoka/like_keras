"""Python components of the M0.3 facility tests (CONTRACT §15 A18, A22; kpi/facilities/test_facilities_friction.py), the
Python twins of hand_back.cpp and self_health.cpp."""

from __future__ import annotations

import dataclasses

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
