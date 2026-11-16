"""Lane core's own tests of the three seams attach needs from the core (CONTRACT §15 A11, docs/friction/contract-006.md
§1-3): an `In` marked optional may stay unwired, `health.code` comes from the component's state, and
`run.subscribe(leaf, port, sink)` is the one way a reader takes a leaf's outputs.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_attach_seams.py

WHY: kpi/attach, T5 `attached` and T8 attach read-only (an observer wires only `state` / `command`), so the proxy's
unwired `trajectory` was refused as unbound at describe(); and T8 needs the refused proxy's health to say LeaseHeld.
"""

from __future__ import annotations

import collections
import dataclasses
import pathlib

import pytest
from robot_msgs.std import DiagnosticRecord, Header

import like_keras as lk
from like_keras.backends import inproc
from like_keras.component import ComponentState

LIBRARY = pathlib.Path(__file__).resolve().parents[1] / "build" / "kpi" / "libempty.so"


class Proxy(lk.Component):
    """An attach proxy's shape: an input it may never be sent (marked optional below, as lk.attach marks every
    mirrored In) and an output it forwards."""

    command = lk.In(Header)
    state = lk.Out(Header)

    def activate(self) -> None:
        self.seen: list[list[str]] = []

    def forward(self, **inputs):
        self.seen.append(sorted(inputs))
        return Header(stamp=self.now())


Proxy.command.optional = True


class Source(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        return Header(stamp=self.now())


class Talker(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        self.log.info("step at %s", self.now())
        return Header(stamp=self.now())


class Keep(lk.Component):
    value = lk.In(Header, queue=100)

    def activate(self) -> None:
        self.rows: list = []

    def forward(self, value):
        self.rows.append(value)


class KeepHealth(Keep):
    value = lk.In(DiagnosticRecord, queue=100)


class Refused(lk.Component):
    """From 0.25 s on it reports an error with its own code, as the attach proxy does on REFUSED (LeaseHeld)."""

    value = lk.Out(Header)

    @dataclasses.dataclass(frozen=True)
    class Params:
        code: str = ""
        then_raise: bool = False

    def forward(self):
        if self.now() >= 0.25:
            self._lk.error, self._lk.code = (
                "write refused: the lease is held by another client",
                self.params.code or None,
            )
        if self.params.then_raise and self.now() >= 1.25:
            raise RuntimeError("forward failed after the refusal")


# ---------------------------------------------------------------- (a) an optional In may stay unwired


def test_an_optional_input_may_stay_unwired_and_forward_never_sees_it():
    s = lk.System("optional_unwired")
    proxy = s.add(Proxy(), "proxy")
    keep = s.add(Keep(), "keep", value=proxy.state)
    dep = lk.Deployment(s, realtime=False)
    dep.place(proxy, hz=10)
    dep.place(keep)
    assert dep.describe()["system"] == "optional_unwired"  # no "is not bound" refusal
    dep.start(until=1.0)
    dep.stop()
    assert len(proxy.seen) == 10 and all(names == [] for names in proxy.seen), proxy.seen
    assert len(keep.rows) == 10


def test_an_optional_input_that_is_wired_is_delivered():
    s = lk.System("optional_wired")
    source = s.add(Source(), "source")
    proxy = s.add(Proxy(), "proxy", command=source.value)
    dep = lk.Deployment(s, realtime=False)
    dep.place(source, proxy, hz=10)
    dep.start(until=1.0)
    dep.stop()
    assert proxy.seen == [["command"]] * 10


def test_an_input_not_marked_optional_is_still_refused_unbound():
    s = lk.System("plain_unwired")
    s.add(Keep(), "keep")
    with pytest.raises(ValueError, match=r"plain_unwired\.keep\.value is not bound"):
        lk.Deployment(s, realtime=False).describe()


# ---------------------------------------------------------------- (b) health.code from the component's state


def health_of(params: dict) -> list:
    s = lk.System("coded")
    subject = s.add(Refused(**params), "subject")
    sink = s.add(KeepHealth(), "sink", value=subject.health)
    dep = lk.Deployment(s, realtime=False)
    dep.place(subject, hz=10)
    dep.place(sink)
    dep.start(until=2.5)  # facilities at 0, 1 and 2
    dep.stop()
    return [(row.header.stamp, row.level, row.code) for row in sink.rows]


def test_health_code_is_the_components_own():
    assert health_of({"code": "LeaseHeld"}) == [
        (0.0, "ok", ""),
        (1.0, "error", "LeaseHeld"),
        (2.0, "error", "LeaseHeld"),
    ]


def test_health_code_defaults_to_exception():
    assert health_of({}) == [(0.0, "ok", ""), (1.0, "error", "Exception"), (2.0, "error", "Exception")]


def test_a_forward_that_raises_reports_exception_whatever_code_was_set():
    assert health_of({"code": "LeaseHeld", "then_raise": True})[-1] == (2.0, "error", "Exception")


def test_reset_clears_the_code():
    state = ComponentState("proxy")
    state.error, state.code = "write refused", "LeaseHeld"
    state.reset()
    assert state.error is None and state.code is None


# ---------------------------------------------------------------- (c) run.subscribe: the one seam readers use


def test_subscribe_takes_outputs_and_facilities_of_python_and_native_leaves(monkeypatch):
    """What attach.serve and the dora Python host subscribe to: an exposed Out, every leaf's health and metrics, and
    (the host) a log port, which reaches the component's logger only through subscribe's refresh."""
    if not LIBRARY.exists():
        raise FileNotFoundError(f"{LIBRARY} is not built: cmake --build /home/bam/like_keras/build -j 1")
    s = lk.System("subscribed")
    talker = s.add(Talker(), "talker")
    s.add(Keep(), "keep", value=talker.value)
    box = s.add(lk.native(LIBRARY, config={"gain": 2.0}), "box")
    wanted = [(talker, "value"), (talker, "log"), (talker, "health"), (talker, "metrics"), (box, "health"),
              (box, "metrics")]  # fmt: skip
    got: dict = collections.defaultdict(list)
    start = inproc.Run.start

    def subscribing(run, until):
        for leaf, port in wanted:
            run.subscribe(leaf, port, lambda value, sent, key=(leaf._lk.name, port): got[key].append((sent, value)))
        return start(run, until)

    monkeypatch.setattr(inproc.Run, "start", subscribing)
    dep = lk.Deployment(s, realtime=False)
    dep.place(talker, hz=10)
    dep.place(s.keep)
    dep.place(box, hz=10)
    dep.start(until=2.5)
    dep.stop()
    assert [sent for sent, _ in got["talker", "value"]] == pytest.approx([tick / 10 for tick in range(25)])
    assert len(got["talker", "log"]) == 25 and all("step at" in row.message for _, row in got["talker", "log"])
    assert [sent for sent, _ in got["talker", "health"]] == [0.0, 1.0, 2.0]
    assert [row.path for _, row in got["talker", "metrics"]] == ["subscribed.talker"] * 3
    assert sorted({row.path for _, row in got["box", "health"]}) == ["subscribed.box.empty", "subscribed.box.talker"]
    # 3 rounds, + the box's own metrics record (A9), + the round drained after the last deactivate (A28, offline)
    assert len(got["box", "health"]) == 8 and len(got["box", "metrics"]) == 12
    assert [row.mode for _, row in got["box", "health"]] == ["operational"] * 6 + ["offline"] * 2
