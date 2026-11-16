"""Lane cpp's test of CONTRACT §15 A23 for an optional QUEUE input of a native box (fix round 1 after the first consumer,
item 1): left unwired by the deployment it reads NULL and never gates the gather of §5.2; wired, it gates it exactly
as a Python queue input does. In one process (inproc) and across processes (dora: lk-dora-host). The box's own half
(an unwired, unexposed one; the host's first push) is tests/cpp/test_friction.cpp's OptionalInput tests.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_optional_queue.py

The dora cases need a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); elsewhere they skip.
WHY: before this fix the unwired box never stepped (0 ticks), and with its port neither wired nor exposed it crashed.
"""

from __future__ import annotations

import json
import pathlib
import subprocess

import pytest

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog
from kpi.support.lk import backend
from robot_msgs.sensor import JointState
from robot_msgs.std import Header

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"


class Positions(lk.Component):
    """Sends position [k] on its k-th step."""

    value = lk.Out(JointState)

    def forward(self):
        self._steps = getattr(self, "_steps", 0) + 1
        return JointState(name=("k",), position=[float(self._steps)], header=Header(stamp=self.now()))


class PythonStepper(lk.Component):
    """tests/cpp/optional_queue.cpp's box in Python, its `extra` a plain (wired) queue input: the reference."""

    extra = lk.In(JointState, queue=4)
    ticks = lk.Out(JointState)

    def forward(self, extra):
        self._steps = getattr(self, "_steps", 0) + 1
        position = [float(self._steps), 1.0 if extra is None else 0.0, 0.0 if extra is None else extra.position[0]]
        return JointState(name=("steps", "extra_null", "extra_position"), position=position,
                          header=Header(stamp=self.now()))


@pytest.fixture(scope="module")
def library() -> pathlib.Path:
    """tests/cpp built (tests/test_cpp_box.py builds the same project): liboptional_queue.so."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "optional_queue", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD / "liboptional_queue.so"


def ticks(path: pathlib.Path) -> list[tuple[float, float, float]]:
    """(step time, extra_null, extra_position) per step."""
    rows = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    return [(round(row["stamp"], 9), row["position"][1], row["position"][2]) for row in rows]


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_an_unwired_optional_queue_never_gates(backend_name, library, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System(f"optional_unwired_{backend_name}")
    box = s.add(lk.native(library, config={"unused": 0}), "box")  # `extra` is left unwired
    s.add(JointStateLog(path=str(tmp_path / "box.jsonl")), "sink", value=box.ticks)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    try:
        dep.start(until=1.05)
    finally:
        dep.stop()
    found = ticks(tmp_path / "box.jsonl")
    assert [row[0] for row in found] == [round(0.1 * step, 9) for step in range(11)], found
    assert all(row[1] == 1.0 for row in found), f"an unwired input read a message: {found}"


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_a_wired_optional_queue_gates_as_a_python_queue_input(backend_name, library, tmp_path, request):
    """A 5 Hz writer into a 10 Hz box: the box steps on the ticks that hold a message and only then, from its first
    tick on (the host wires the input before lk_start), as its Python twin does."""
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System(f"optional_wired_{backend_name}")
    source = s.add(Positions(), "source")
    box = s.add(lk.native(library, config={"unused": 0}), "box", extra=source.value)
    twin = s.add(PythonStepper(), "twin", extra=source.value)
    s.add(JointStateLog(path=str(tmp_path / "box.jsonl")), "box_sink", value=box.ticks)
    s.add(JointStateLog(path=str(tmp_path / "twin.jsonl")), "twin_sink", value=twin.ticks)
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    dep.place(s.source, process="source", hz=5)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.twin, process="twin", hz=10)
    dep.place(s.box_sink, s.twin_sink, process="sinks")
    try:
        dep.start(until=1.05)
    finally:
        dep.stop()
    found, reference = ticks(tmp_path / "box.jsonl"), ticks(tmp_path / "twin.jsonl")
    assert reference == [(0.1, 0.0, 1.0), (0.3, 0.0, 2.0), (0.5, 0.0, 3.0), (0.7, 0.0, 4.0), (0.9, 0.0, 5.0)], reference
    assert found == reference
