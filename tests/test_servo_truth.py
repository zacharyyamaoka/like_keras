"""ServoState.truth across the C++ boundary: a nullable nested GroundTruth (robot_msgs c72d664) with a str -> float
`meta` (d85aa92). Before this test the generated view had no `truth` member while stamping the full schema hash, so a
C++ ServoState reached Python one child short ("zip() argument 2 is shorter than argument 1"), and the generator had
dropped the field without a word. Three halves:

- C++ -> Python, truth null: a C++ component is real-side, so its encoder writes truth as null by default.
- Python -> C++ -> Python, truth present: the C++ decoder accepts a present truth, and its echo comes back equal.
- the generator refuses a field it cannot lay out, naming it, instead of leaving it out of the view.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_servo_truth.py

The dora cases need a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); elsewhere they skip.
"""

from __future__ import annotations

import dataclasses
import pathlib
import subprocess
from collections.abc import Mapping

import numpy as np
import pytest

import like_keras as lk
import robot_msgs
from kpi.support.lk import backend
from like_keras import codec
from robot_msgs.control import GroundTruth, ServoState
from robot_msgs.std import Header

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"
TRUTH = GroundTruth(name=("épaule", "coude"), position=[0.5, -1.25], velocity=[0.0, 2.0], stop_effort=[0.0, 3.5],
                    source="mujoco", meta={"object/cap.z": 0.125, "contacts": 2.0},
                    header=Header(stamp=0.05, frame_id="world", meta={"scene": "pendulum"}))


class TruthSource(lk.Component):
    """Sends a sim-side ServoState, truth present, on every step."""

    state = lk.Out(ServoState)

    def forward(self):
        self._steps = getattr(self, "_steps", 0) + 1
        return ServoState(name=TRUTH.name, position=[0.5, -1.0], effort=[1.0, 2.0], seq=self._steps,
                          header=Header(stamp=self.now(), frame_id="base", meta={"side": "sim"}), truth=TRUTH)


class FrameLog(lk.Component):
    """Writes every ServoState it receives as one LKF frame per line (hex): the whole message, byte for byte."""

    state = lk.In(ServoState, queue=1000)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, state):
        self._file.write(codec.lkf(state).hex() + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


def received(path: pathlib.Path) -> list[ServoState]:
    lines = path.read_text().splitlines() if path.exists() else []
    return [codec.decode(bytes.fromhex(line), ServoState) for line in lines]


@pytest.fixture(scope="module")
def library() -> pathlib.Path:
    """tests/cpp built (tests/test_cpp_box.py builds the same project): libservo_truth.so."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "servo_truth", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD / "libservo_truth.so"


def run(s: lk.System, backend_name: str, *, source: bool) -> None:
    dep = lk.Deployment(s, backend=backend(backend_name), realtime=False)
    if source:
        dep.place(s.source, process="source", hz=10)
    dep.place(s.box, process="box", hz=10)
    dep.place(s.sink, process="sink")
    try:
        dep.start(until=0.55)
    finally:
        dep.stop()


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_a_cpp_servo_state_reaches_python_with_truth_null(backend_name, library, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System(f"servo_truth_null_{backend_name}")
    box = s.add(lk.native(library, config={"unused": 0}), "box")  # `measured` left unwired: the box's own state
    s.add(FrameLog(path=str(tmp_path / "box.lkf")), "sink", state=box.state)
    run(s, backend_name, source=False)
    found = received(tmp_path / "box.lkf")
    assert len(found) >= 5, found
    for step, state in enumerate(found, start=1):
        assert state.truth is None
        assert state.name == ("épaule", "coude") and state.seq == step
        assert np.array_equal(state.position, [0.25, -float(step)])
        assert state.header.frame_id == "base" and dict(state.header.meta) == {"side": "real"}


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_a_present_truth_crosses_into_cpp_and_back_unchanged(backend_name, library, tmp_path, request):
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    s = lk.System(f"servo_truth_echo_{backend_name}")
    source = s.add(TruthSource(), "source")
    box = s.add(lk.native(library, config={"unused": 0}), "box", measured=source.state)
    s.add(FrameLog(path=str(tmp_path / "echo.lkf")), "sink", state=box.state)
    run(s, backend_name, source=True)
    echoed = [state for state in received(tmp_path / "echo.lkf") if state.truth is not None]
    assert len(echoed) >= 4, received(tmp_path / "echo.lkf")
    for state in echoed:
        assert state.truth == TRUTH and dict(state.truth.meta) == {"object/cap.z": 0.125, "contacts": 2.0}
        assert state.header.frame_id == "base" and dict(state.header.meta) == {"side": "sim"}
        expected = ServoState(name=TRUTH.name, position=[0.5, -1.0], effort=[1.0, 2.0], seq=state.seq,
                              header=state.header, truth=TRUTH)
        assert codec.lkf(state) == codec.lkf(expected)  # every child, byte for byte, under the same schema hash


def test_every_hashed_child_has_a_view_member_and_a_visitor_call():
    """The views stamp kHash over every child, so each child must be in the view: one visitor call per child."""
    for cls in codec._cpp_types():
        text = "\n".join(codec._cpp_struct(cls))
        visitor = text[text.index("void fields("):]
        for node in codec.schema(cls):
            assert f" {node.name}{{}};" in text and f"(m.{node.name}" in visitor, (cls.type_name, node.name)
        assert visitor.count("v.") == len(codec.schema(cls)), cls.type_name


@dataclasses.dataclass(frozen=True, eq=False, repr=False)
class NullableNote(robot_msgs.Msg):
    note: str | None = None


@dataclasses.dataclass(frozen=True, eq=False, repr=False)
class NullableTags(robot_msgs.Msg):
    tags: Mapping[str, str] | None = None


@pytest.mark.parametrize(("cls", "field"), [(NullableNote, "note"), (NullableTags, "tags")])
def test_the_generator_refuses_a_field_it_cannot_lay_out(cls, field):
    """The root cause: a child the C++ views had no layout for was skipped, and the view still claimed its hash."""
    assert codec.schema(cls)  # the Python wire lays it out (CONTRACT §7.1) ...
    with pytest.raises(TypeError, match=rf"{cls.type_name}\.{field} .*no C\+\+ view layout"):
        codec._cpp_struct(cls)  # ... the C++ view refuses it, naming it, instead of leaving it out
