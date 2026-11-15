"""The like_keras side of criterion 2's A/B: the same sender -> forwarder -> receiver as raw_dora_node.py, written
in like_keras and placed in three processes under the dora backend; and the 4-component chain for the per-edge cost."""

from __future__ import annotations

import dataclasses
import time

import like_keras as lk
import numpy as np
from robot_msgs.std import Header
from robot_msgs.trajectory import JointTrajectory


def payload(size: int, joints: int = 1) -> JointTrajectory:
    """A JointTrajectory of `size` bytes of float64 (t plus positions): 64 B, 4 KiB, 1 MiB, or the 7 x 1000 plan."""
    samples = max(1, size // (8 * (joints + 1)))
    rng = np.random.default_rng(size)
    return JointTrajectory(t=np.arange(samples, dtype=np.float64), joint_names=tuple(f"j{i}" for i in range(joints)),
                           positions=rng.standard_normal((samples, joints)))


@dataclasses.dataclass(frozen=True)
class SenderParams:
    size: int = 64
    joints: int = 1
    samples: int = 1000


class Sender(lk.Component):
    data = lk.Out(JointTrajectory)
    Params = SenderParams

    def activate(self):
        self._payload = payload(self.params.size, self.params.joints)
        self._sent = 0

    def forward(self):
        if self._sent >= self.params.samples:
            return None
        self._sent += 1
        return self._payload.replace(header=Header(stamp=self.now()))


class Forwarder(lk.Component):
    data = lk.In(JointTrajectory, queue=1000)
    out = lk.Out(JointTrajectory)

    def forward(self, data):
        return data


@dataclasses.dataclass(frozen=True)
class ReceiverParams:
    path: str


class Receiver(lk.Component):
    data = lk.In(JointTrajectory, queue=1000)
    Params = ReceiverParams

    def activate(self):
        self._latencies, self._copies = [], 0

    def forward(self, data):
        self._latencies.append(time.time() - data.header.stamp)
        root = data.positions
        while isinstance(root.base, np.ndarray):
            root = root.base
        self._copies = max(self._copies, 1 if root.base is None else 0)  # owned by numpy: it was copied on receive

    def deactivate(self):
        np.save(self.params.path, np.array(self._latencies))
        with open(self.params.path + ".copies", "w") as file:
            file.write(str(self._copies))


class First(lk.Component):
    out = lk.Out(float)

    def forward(self):
        return 1.0


class Add(lk.Component):
    value = lk.In(float)
    out = lk.Out(float)

    def forward(self, value):
        return value + 1.0


class Last(lk.Component):
    value = lk.In(float)

    def forward(self, value):
        return None
