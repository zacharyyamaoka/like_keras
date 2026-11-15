"""Python ends of the polyglot chains (criterion 8): the same port names and types as the C++ Gain."""

from __future__ import annotations

import dataclasses
import json

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


class JointSource(lk.Component):
    output = lk.Out(JointState)

    def forward(self):
        count = getattr(self, "_count", 0)
        self._count = count + 1
        return JointState(name=("a", "b", "c"), position=[count, -0.5 * count, 0.1 * count],
                          header=Header(stamp=self.now()))


class JointGain(lk.Component):
    """The Python peer of kpi/tasks/cpp_gain/gain.cpp: the same ports, the same law."""

    input = lk.In(JointState)
    output = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        k: float = 2.0

    def forward(self, input):  # noqa: A002 - the port is called input on both sides
        if input is None:
            return None
        return input.replace(position=self.params.k * input.position)


class JointSink(lk.Component):
    input = lk.In(JointState, queue=100)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def forward(self, input):  # noqa: A002
        with open(self.params.path, "a", encoding="utf-8") as file:
            file.write(json.dumps({"now": self.now(), "position": [float(x) for x in input.position]}) + "\n")
