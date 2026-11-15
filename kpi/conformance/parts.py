"""Components for the conformance suite (CONTRACT §5): sources that count, and sinks that write what they saw.

Every sink writes one JSON line per step to `path` (`tag`, `now`, and what each input held), so the same part works
in this process (inproc) and in a dora node process. Values ride in a robot_msgs JointState: position[0] is the
sender's step count, header.stamp its now().
"""

from __future__ import annotations

import dataclasses
import json
import time

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


def value(count: int, now: float) -> JointState:
    return JointState(name=("k",), position=[float(count)], header=Header(stamp=now))


def seen(message) -> float | None:
    return None if message is None else float(message.position[0])


@dataclasses.dataclass(frozen=True)
class SinkParams:
    path: str
    tag: str = ""


@dataclasses.dataclass(frozen=True)
class CounterParams:
    limit: int = 1_000_000  # sends this many values, then nothing


class _Writes:
    """Not a component: the file handling every sink shares."""

    def _write(self, **row) -> None:
        if not hasattr(self, "_file"):
            self._file = open(self.params.path, "a", encoding="utf-8")
        row.update(tag=self.params.tag, now=self.now(), wall=time.time())
        self._file.write(json.dumps(row) + "\n")
        self._file.flush()


class Counter(lk.Component):
    value = lk.Out(JointState)
    Params = CounterParams

    def forward(self):
        count = getattr(self, "_count", 0)
        if count >= self.params.limit:
            return None
        self._count = count + 1
        return value(count, self.now())


class CounterPair(lk.Component):
    """Two outputs; `left` is None (not sent) on odd steps (§4.1: a None element is not sent)."""

    left = lk.Out(JointState)
    right = lk.Out(JointState)

    def forward(self):
        count = getattr(self, "_count", 0)
        self._count = count + 1
        return (value(count, self.now()) if count % 2 == 0 else None), value(count, self.now())


class LatestSink(_Writes, lk.Component):
    value = lk.In(JointState)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))


class StaleSink(_Writes, lk.Component):
    value = lk.In(JointState, stale=0.05)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))


class QueueSink2(_Writes, lk.Component):
    value = lk.In(JointState, queue=2)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))


class QueueSink(_Writes, lk.Component):
    value = lk.In(JointState, queue=1000)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))


class GatherSink(_Writes, lk.Component):
    left = lk.In(JointState, queue=10)
    right = lk.In(JointState, queue=10)
    Params = SinkParams

    def forward(self, left, right):
        self._write(left=seen(left), right=seen(right))


class Relay(_Writes, lk.Component):
    """Rate-less upstream: one step per queued message; passes it on and writes that it stepped."""

    value = lk.In(JointState, queue=100)
    out = lk.Out(JointState)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))
        return value


class Downstream(_Writes, lk.Component):
    """No queue input: in a rate-less group it steps when its same-group upstream stepped (§5.2)."""

    value = lk.In(JointState)
    Params = SinkParams

    def forward(self, value):
        self._write(value=seen(value))


class LaggedFront(_Writes, lk.Component):
    """Reads `back`'s previous step (lagged): the same-group cycle front -> back -> front is legal."""

    back = lk.In(JointState, lagged=True)
    out = lk.Out(JointState)
    Params = SinkParams

    def forward(self, back):
        count = getattr(self, "_count", 0)
        self._count = count + 1
        self._write(back=seen(back), count=count)
        return value(count, self.now())


class LaggedBack(lk.Component):
    front = lk.In(JointState)
    out = lk.Out(JointState)

    def forward(self, front):
        return front


class OrderProbe(_Writes, lk.Component):
    """Writes when it steps: the order of steps in a group (§5.3) and of groups at equal times (§5.5)."""

    Params = SinkParams

    def forward(self):
        self._write()


class ChainProbe(_Writes, lk.Component):
    """A link of a same-group chain that writes on activate, step and deactivate (§5.7 lifecycle order)."""

    value = lk.In(JointState)
    out = lk.Out(JointState)
    Params = SinkParams

    def activate(self):
        self._write(event="activate")

    def forward(self, value):
        self._write(event="step")
        return value

    def deactivate(self):
        self._write(event="deactivate")


class RefuseActivate(lk.Component):
    """activate raises: the group never starts, no data frame is sent (§5.7, T7)."""

    value = lk.Out(JointState)

    def activate(self):
        raise RuntimeError("refused on purpose (kpi conformance)")

    def forward(self):
        return value(0, self.now())


@dataclasses.dataclass(frozen=True)
class RaiseParams:
    at_step: int = 3


class RaiseInForward(lk.Component):
    """forward raises at step `at_step`: the group deactivates, health error code Exception (§5.7)."""

    value = lk.Out(JointState)
    Params = RaiseParams

    def forward(self):
        count = getattr(self, "_count", 0)
        self._count = count + 1
        if count == self.params.at_step:
            raise ValueError("raised on purpose at step 3 (kpi conformance)")
        return value(count, self.now())


@dataclasses.dataclass(frozen=True)
class SleepParams:
    path: str
    tag: str = ""
    every: int = 10
    sleep_s: float = 0.025


class Overrunner(_Writes, lk.Component):
    """Every `every`-th step takes `sleep_s`, longer than its period: the missed ticks are skipped (§5.2)."""

    Params = SleepParams

    def forward(self):
        count = getattr(self, "_count", 0)
        self._count = count + 1
        self._write(count=count)
        if count % self.params.every == 0:
            time.sleep(self.params.sleep_s)


class Talker(lk.Component):
    def forward(self):
        self.log.info("step")
