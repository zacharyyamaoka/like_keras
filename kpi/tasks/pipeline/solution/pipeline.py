"""Criterion 4's in-process task in like_keras: count -> double -> save in one process."""

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
    value = lk.In(JointState)
    doubled = lk.Out(JointState)

    def forward(self, value):
        return value.replace(position=2 * value.position)


class Save(lk.Component):
    value = lk.In(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str = "pipeline.txt"

    def forward(self, value):
        with open(self.params.path, "a") as file:
            file.write(f"{value.position[0]}\n")


def pipeline(path="pipeline.txt", realtime=True):
    s = lk.System("pipeline")
    count = s.add(Count())
    double = s.add(Double(), value=count.value)
    s.add(Save(path=path), value=double.doubled)
    dep = lk.Deployment(s, realtime=realtime)
    dep.place(s.count, s.double, s.save, hz=10)
    return dep
