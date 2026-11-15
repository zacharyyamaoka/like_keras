"""The empty-step components of criterion 9: every facility must work on them with zero user lines."""

import dataclasses

import like_keras as lk


class Empty(lk.Component):
    """Does nothing each step. Its log, health, metrics, params, lifecycle, recording and introspection are the
    framework's."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        gain: float = 1.0

    def forward(self):
        return None


class Talker(lk.Component):
    """The one user line a log test needs: a log call."""

    def forward(self):
        self.log.info("step")
