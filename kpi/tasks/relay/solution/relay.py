"""Criterion 4's cross-process task in like_keras: count -> double -> save, three processes, dora between."""

import dataclasses

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


class Count(lk.Component):
    value = lk.Out(JointState)

    def activate(self):
        self.count = 0

    def forward(self):
        self.count += 1
        return JointState(name=("a",), position=[self.count], header=Header(stamp=self.now()))


class Double(lk.Component):
    value = lk.In(JointState, queue=10)
    doubled = lk.Out(JointState)

    def forward(self, value):
        return value.replace(position=2 * value.position)


class Save(lk.Component):
    value = lk.In(JointState, queue=10)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str = "relay.txt"

    def forward(self, value):
        with open(self.params.path, "a") as file:
            file.write(f"{value.position[0]}\n")


def relay(path="relay.txt", realtime=True):
    from like_keras.backends import dora

    s = lk.System("relay")
    count = s.add(Count())
    double = s.add(Double(), value=count.value)
    s.add(Save(path=path), value=double.doubled)
    dep = lk.Deployment(s, backend=dora, realtime=realtime)
    dep.place(s.count, process="source", hz=10)
    dep.place(s.double, process="filter")
    dep.place(s.save, process="sink")
    return dep
