"""Test-side components: logs that write what they receive to a JSON-lines file, and the attach observer.

Imported only once like_keras exists (kpi/support/lk.like_keras() skips first). A file, not a list, so the same
part works when a deployment rebuilds it in another process (CONTRACT §4.1: `module:qualname(**params)`).
"""

from __future__ import annotations

import dataclasses
import json
import math
import time

import like_keras as lk
from robot_msgs.control import ServoCommand, ServoState
from robot_msgs.trajectory import JointTrajectory


def _row(component: lk.Component, value) -> str:
    row = {"now": component.now(), "wall": time.time(), "stamp": value.header.stamp}
    if hasattr(value, "position"):
        row["position"] = [None if math.isnan(x) else float(x) for x in value.position]
    if hasattr(value, "t"):
        row["samples"] = len(value.t)
    return json.dumps(row)


@dataclasses.dataclass(frozen=True)
class LogParams:
    path: str


class CommandLog(lk.Component):
    value = lk.In(ServoCommand, queue=1000)
    Params = LogParams

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, value):
        self._file.write(_row(self, value) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class StateLog(lk.Component):
    value = lk.In(ServoState, queue=1000)
    Params = LogParams

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, value):
        self._file.write(_row(self, value) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class TrajectoryLog(lk.Component):
    value = lk.In(JointTrajectory, queue=100)
    Params = LogParams

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, value):
        self._file.write(_row(self, value) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


def read_log(path) -> list[dict]:
    try:
        with open(path, encoding="utf-8") as file:
            return [json.loads(line) for line in file if line.strip()]
    except FileNotFoundError:
        return []


def observe(address: str, path, name: str = "observer") -> lk.Deployment:
    """A read-only attach client logging the deployment's exposed `command` (it never writes, so it never takes the
    lease). Started and returned; the caller stops it."""
    s = lk.System(name)
    arm = s.add(lk.attach(address), "arm")
    log = s.add(CommandLog(path=str(path)), "log", value=arm.command)
    dep = lk.Deployment(s)
    dep.place(arm, hz=100)
    dep.place(log)
    dep.start()
    return dep
