"""Lane core's tests of the Python API CONTRACT §15 adds to a component: period() (A20), health(level, code) (A22)
and In(T, fresh=True) (A25), at the edges the frozen M0.3 tests leave alone: eager calls, refusals at class creation,
and the health rules that only show in a combination of facts.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_component_api.py
"""

from __future__ import annotations

import dataclasses

import pytest

import like_keras as lk
from robot_msgs.std import Header


class Ticker(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        return Header(stamp=self.now())


# ---------------------------------------------------------------- A20: period()


def test_period_is_zero_when_eager_and_a_port_may_not_take_its_name():
    assert Ticker().period() == 0.0
    with pytest.raises(TypeError, match="'period' collides with a Component member"):

        class Clashing(lk.Component):  # noqa: F841
            period = lk.Out(Header)


# ---------------------------------------------------------------- A25: In(T, fresh=True)


class Every(lk.Component):
    """Sends one Header object on the steps in `on` (0-based), the same object every time."""

    value = lk.Out(Header)

    @dataclasses.dataclass(frozen=True)
    class Params:
        on: tuple = ()

    def forward(self):
        self._step = getattr(self, "_step", -1) + 1
        self._header = getattr(self, "_header", Header(stamp=0.0))
        return self._header if self._step in self.params.on else None


class Seen(lk.Component):
    value = lk.In(Header, fresh=True)

    def activate(self):
        self.seen = []

    def forward(self, value):
        self.seen.append(value is not None)


@pytest.mark.parametrize("same_group", [True, False], ids=["same_group", "across_groups"])
def test_fresh_sees_each_send_once_even_of_one_object(same_group):
    s = lk.System("fresh_once")
    writer = s.add(Every(on=(1, 2, 5)), "writer")
    reader = s.add(Seen(), "reader", value=writer.value)
    dep = lk.Deployment(s, realtime=False)
    if same_group:
        dep.place(writer, reader, hz=10)  # an edge inside a group is read in the same step
    else:
        dep.place(writer, hz=10)
        dep.place(reader, hz=10)  # across groups, at the first step strictly after the send
    dep.start(until=0.75)
    dep.stop()
    arrivals = [step for step, new in enumerate(reader.seen) if new]
    assert arrivals == ([1, 2, 5] if same_group else [2, 3, 6]), reader.seen


def test_fresh_is_refused_on_a_queue():
    with pytest.raises(ValueError, match="fresh is for a latest input"):
        lk.In(Header, queue=4, fresh=True)
