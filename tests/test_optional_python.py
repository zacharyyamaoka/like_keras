"""Lane core's test of CONTRACT §15 A32 (consumer-015): `lk.In(T, optional=True)` where the frozen ruler test
(consumer::test_optional_input_in_python, a flat System under lockstep) does not reach: the consumer's own shape, a
Python System (ArmControllerPy's) exposing its child's optional input, added to a parent that leaves it unbound, in
real time; and an optional QUEUE input left unwired, which must neither gate the gather of §5.2 nor be left out of
forward's arguments.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_optional_python.py
"""

from __future__ import annotations

import json
import time

import numpy as np
from robot_msgs.sensor import JointState
from robot_msgs.std import DiagnosticRecord, Header

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog, StepCount


class Guarded(lk.Component):
    """Each step: [1 if a verdict was read else -1, the extra message's position or -1]."""

    verdict = lk.In(DiagnosticRecord, optional=True)
    extra = lk.In(JointState, queue=4, optional=True)
    seen = lk.Out(JointState)

    def forward(self, verdict, extra):
        read = [-1.0 if verdict is None else 1.0, -1.0 if extra is None else float(extra.position[0])]
        return JointState(header=Header(stamp=self.now()), name=("verdict", "extra"), position=np.array(read))


def arm() -> lk.System:
    s = lk.System("arm")
    guarded = s.add(Guarded(), "guarded")
    s.expose(guarded.seen, guarded.verdict, guarded.extra)  # a boundary the parent may leave unbound: optional
    return s


def run(tmp_path, wired: bool) -> list[list[float]]:
    cell = lk.System(f"optional_{'wired' if wired else 'unwired'}")
    inner = cell.add(arm(), "arm")
    cell.add(JointStateLog(path=str(tmp_path / "seen.jsonl")), "sink", value=inner.seen)
    dep = lk.Deployment(cell)
    if wired:
        source = cell.add(StepCount(), "source")
        cell.wire(inner, extra=source.value)
        dep.place(cell.source, hz=50)
    dep.place(cell.arm, hz=50)
    dep.place(cell.sink)
    ports = dep.describe()["components"][f"{cell._lk.name}.arm.guarded"]["ports"]
    assert ports["verdict"]["optional"] and ports["extra"]["optional"] and not ports["seen"]["optional"], ports
    dep.start()
    try:
        time.sleep(0.5)
    finally:
        dep.stop()
    return [json.loads(line)["position"] for line in (tmp_path / "seen.jsonl").read_text().splitlines()]


def test_a_nested_optional_boundary_left_unbound_steps_every_tick_with_none(tmp_path):
    seen = run(tmp_path, wired=False)
    assert len(seen) >= 10, f"{len(seen)} steps in 0.5 s at 50 Hz: an unwired optional queue gated the step"
    assert all(row == [-1.0, -1.0] for row in seen), seen[:5]


def test_the_same_boundary_wired_by_the_parent_delivers(tmp_path):
    seen = run(tmp_path, wired=True)
    extras = [row[1] for row in seen if row[1] > 0]
    assert len(extras) >= 5 and extras == sorted(extras), seen[:10]
    assert all(row[0] == -1.0 for row in seen), "the verdict stayed unwired"
