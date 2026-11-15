"""The target source of the reference consumer: pick targets replayed from a JSON-lines file, one per step."""

import dataclasses
import json

import like_keras as lk
from robot_msgs.std import Header

from kpi.tasks.arm_cell.plan import JOINTS, TARGETS, PickTarget


class TargetFile(lk.Component):
    target = lk.Out(PickTarget)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str = str(TARGETS)
        repeat: bool = False  # A1 runs 60 s at 2 Hz: the file again from the top

    def activate(self):
        with open(self.params.path, encoding="utf-8") as file:
            self._targets = [json.loads(line)["position"] for line in file if line.strip()]
        self._next = 0

    def forward(self):
        if self._next == len(self._targets):
            if not self.params.repeat:
                return None
            self._next = 0
        position = self._targets[self._next]
        self._next += 1
        return PickTarget(name=JOINTS, position=position, header=Header(stamp=self.now()))
