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


# ---------------------------------------------------------------- A22: health(level, code)


class Scripted(lk.Component):
    """One fact per logical second (lockstep, rounds at 1 s, 2 s, ...): the rules of a health round in combination."""

    value = lk.Out(Header)

    def forward(self):
        at = round(self.now(), 6)  # each fact once, at a whole second: it shows in the round one second later
        if at == 0.0:
            self.health("warn", "Degraded")
            self.log.warning("a warning beside the component's own warn")  # tie: the component's code
        elif at == 1.0:
            self.log.error("an error logged while the component says warn")  # worse: error / ErrorLogged
        elif at == 3.0:
            self.health("ok", "")
            self.log.warning("a warning once the component says ok")  # the facility's own: WarningLogged
        elif at == 4.0:
            self.health("error", "Braked")  # held ...
        elif at == 5.5:
            raise RuntimeError("forward failed while braked")  # ... until an Exception, which always wins
        return Header(stamp=self.now())


class Diagnoses(lk.Component):
    value = lk.In("std/DiagnosticRecord", queue=100)

    def activate(self):
        self.rows = []

    def forward(self, value):
        self.rows.append((round(value.header.stamp, 6), value.level, value.code))


def test_health_rounds_combine_the_components_level_with_the_facilitys():
    s = lk.System("scripted")
    subject = s.add(Scripted(), "subject")
    sink = s.add(Diagnoses(), "sink", value=subject.health)
    dep = lk.Deployment(s, realtime=False)
    dep.place(subject, hz=10)
    dep.place(sink)
    dep.start(until=6.5)
    dep.stop()
    assert sink.rows[1:] == [
        (1.0, "warn", "Degraded"),
        (2.0, "error", "ErrorLogged"),
        (3.0, "warn", "Degraded"),  # held: no call in between
        (4.0, "warn", "WarningLogged"),
        (5.0, "error", "Braked"),
        (6.0, "error", "Exception"),
    ], sink.rows


@pytest.mark.parametrize(("level", "code"), [("warning", "Degraded"), ("warn", ""), ("error", "not pascal")])
def test_health_refuses_a_level_or_code_a_record_cannot_carry(level, code):
    with pytest.raises(ValueError, match=r"health\("):
        Ticker().health(level, code)
