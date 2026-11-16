"""Lane core's tests of CONTRACT §15 A21: a Python lk.System has log, health and metrics ports that carry every inner
component's records with their full paths, as a native box's do. kpi's frozen test reads the reference arm both
ways in one process; these add nesting, recording, the dora Python host's fan-in, and what stays refused.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_system_facilities.py

The dora case needs a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); elsewhere it skips.
"""

from __future__ import annotations

import json

import pytest

import like_keras as lk
from kpi.facilities.parts import HealthLog, LogLog
from kpi.support.lk import backend, recorded
from robot_msgs.std import Header


class Talker(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        if round(self.now(), 6) == 0.5:
            self.log.info("half a second")
        return Header(stamp=self.now())


class Listener(lk.Component):
    value = lk.In(Header)

    def forward(self, value):
        return None


def chain() -> lk.System:
    """outer: talker -> inner (listener, and a nested System with a second talker): three leaves, two levels."""
    nested = lk.System("nested")
    nested.add(Talker(), "deep")
    inner = lk.System("inner")
    talker = inner.add(Talker(), "talker")
    inner.add(Listener(), "listener", value=talker.value)
    inner.add(nested)
    return inner


LEAVES = {"cell.inner.talker", "cell.inner.listener", "cell.inner.nested.deep"}


def rows(path) -> list[dict]:
    return [json.loads(line) for line in open(path, encoding="utf-8")]


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_a_systems_health_and_log_carry_every_leaf_in_it(backend_name, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System("cell")
    inner = s.add(chain(), "inner")
    s.add(HealthLog(path=str(tmp_path / "health.jsonl")), "health_sink", value=inner.health)
    s.add(LogLog(path=str(tmp_path / "log.jsonl")), "log_sink", value=inner.log)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(inner, process="chain", hz=10)
    dep.place(s.health_sink, s.log_sink, process="sinks")
    try:
        dep.start(until=2.5)
    finally:
        dep.stop()
    health = rows(tmp_path / "health.jsonl")
    assert {row["path"] for row in health} == LEAVES, health
    assert sorted(row["stamp"] for row in health if row["path"] == "cell.inner.listener") == [0.0, 1.0, 2.0]
    said = sorted((row["path"], row["message"]) for row in rows(tmp_path / "log.jsonl"))
    assert said == [("cell.inner.nested.deep", "half a second"), ("cell.inner.talker", "half a second")], said


def test_a_systems_log_records_on_one_channel(tmp_path):
    s = lk.System("cell")
    inner = s.add(chain(), "inner")
    dep = lk.Deployment(s, realtime=False)
    dep.record(tmp_path / "run.mcap", inner.log)
    dep.place(inner, hz=10)
    dep.start(until=1.0)
    dep.stop()
    lines = recorded(tmp_path / "run.mcap", "log")
    assert sorted(row["path"] for row in lines) == ["cell.inner.nested.deep", "cell.inner.talker"], lines


def test_renamed_through_a_parent_it_still_carries_every_leaf():
    """Closure under composition: a parent exposes the child System's log under a name; its own parent wires it."""
    outer = lk.System("outer")
    cell = lk.System("cell")
    inner = cell.add(chain(), "inner")
    cell.expose(inner_log=inner.log)
    outer.add(cell)
    sink = outer.add(LogLog(path="/dev/null"), "sink", value=cell.inner_log)
    dep = lk.Deployment(outer, realtime=False)
    dep.place(cell, hz=10)
    dep.place(sink)
    edges = dep.describe()["edges"]
    assert sorted(edge["src"] for edge in edges if edge["dst"] == "outer.sink.value") == [
        "outer.cell.inner.listener.log", "outer.cell.inner.nested.deep.log", "outer.cell.inner.talker.log"]


def test_exposing_it_at_the_top_is_refused_and_a_child_of_that_name_still_wins():
    top = lk.System("top")
    inner = top.add(chain(), "inner")
    top.expose(inner_health=inner.health)
    with pytest.raises(ValueError, match=r"top\.inner\.health is exposed by top .* v0 exposes one leaf's port"):
        lk.Deployment(top, realtime=False).describe()
    s = lk.System("named")
    talker = s.add(Talker(), "talker")
    s.add(HealthLog(path="/dev/null"), "health", value=talker.health)  # kpi names sinks after facilities
    assert isinstance(s.health, HealthLog)
