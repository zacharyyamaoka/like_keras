"""Criterion 8, Python + C++ reach: five parts, each scored only when every test of that part passes.

part 1 test_mixed_chain_*          Python -> C++ -> Python, the same port names and types, in and across processes
part 2 test_cpp_needs_no_python    the C++ side needs neither Python nor ROS to build or run
part 3 test_one_so_three_placements one .so as a dora node, a GIL-free thread under Python, and on the twin bus
part 4 test_T10_*                  every per-joint type bit-exact Python -> C++ -> Python; a JointTrajectory view read
                                   bit-exact; a schema-hash mismatch refused at connect
part 5 test_cpp_boilerplate        a C++ component is <= 2 files and <= 10 lines that name the framework
"""

from __future__ import annotations

import json
import re
import subprocess
import sys

import numpy as np
import pytest

from kpi.golden import lkf, make_golden
from kpi.support import cells, mcap
from kpi.support.lk import BUILD, KPI, REPO, backend, like_keras, native_library

PER_JOINT = {"sensor/JointState": "joint_state", "control/ServoState": "servo_state", "control/ServoCommand": "servo_command"}


def rows(path) -> list[dict]:
    try:
        return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]
    except FileNotFoundError:
        return []


@pytest.mark.parametrize("backend_name", ["inproc", "dora"])
def test_mixed_chain(backend_name, tmp_path, request):
    """Python source -> C++ Gain (k = 2) -> Python sink, lockstep, against the same chain all in Python."""
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    chosen = backend(backend_name)
    from kpi.polyglot.parts import JointGain, JointSink, JointSource

    outputs = {}
    for language in ("python", "cpp"):
        s = lk.System(f"chain_{language}_{backend_name}")
        source = s.add(JointSource(), "source")
        if language == "cpp":
            gain = s.add(lk.native(native_library("cpp_gain"), config={"k": 2.0}), "gain", input=source.output)
        else:
            gain = s.add(JointGain(k=2.0), "gain", input=source.output)
        path = tmp_path / f"{language}.jsonl"
        s.add(JointSink(path=str(path)), "sink", input=gain.output)
        dep = lk.Deployment(s, backend=chosen, realtime=False)
        dep.place(s.source, process="source", hz=10)
        dep.place(s.gain, process="gain", hz=10)
        dep.place(s.sink, process="sink")
        dep.start(until=1.95)
        dep.stop()
        outputs[language] = rows(path)
    assert len(outputs["cpp"]) >= 15 and outputs["cpp"] == outputs["python"]


def test_cpp_needs_no_python():
    """The component .so files link no Python and no ROS; the build reads no Python (CMake never runs it)."""
    for name in ("cpp_gain", "arm_controller", "empty"):
        library = native_library(name)
        linked = subprocess.run(["ldd", str(library)], capture_output=True, text=True).stdout
        assert "python" not in linked.lower() and "rcl" not in linked and "ros" not in linked.lower(), linked
        symbols = subprocess.run(["nm", "-D", "--undefined-only", str(library)], capture_output=True, text=True).stdout
        assert not re.search(r"\bPy[A-Z_]", symbols), f"{name} calls into Python"
        assert "dora" not in symbols.lower(), f"{name} links dora (only lk-dora-host may)"
    for cmake_file in [REPO / "CMakeLists.txt", REPO / "cmake" / "like_keras.cmake", KPI / "CMakeLists.txt"]:
        if cmake_file.exists():
            text = cmake_file.read_text().lower()
            assert "find_package(python" not in text and not re.search(r"execute_process\([^)]*python", text), cmake_file


def test_one_so_three_placements(tmp_path, request):
    """libarm_controller.so as a dora node (target), as a thread inside this Python process (same_box, real time),
    and stepped on the twin bus (same_box, lockstep): commands flow in each."""
    request.getfixturevalue("private_dora")
    native_library("arm_controller")
    twin = cells.run_lockstep("same_box", tmp_path / "twin", 2.0)
    assert len(twin["command"]) >= 150
    for placement in ("target", "same_box"):
        directory = tmp_path / placement
        directory.mkdir()
        dep = cells.deployment(placement, directory, lockstep=False)
        dep.record(directory / "run.mcap")
        dep.start(until=3.0)
        dep.stop()
        from kpi.support.lk import recorded

        assert len(recorded(directory / "run.mcap", "command")) >= 150, f"{placement}: no commands"


@pytest.mark.parametrize("type_name", sorted(PER_JOINT))
def test_T10_codec_roundtrip(type_name, tmp_path):
    """The golden instance goes Python -> C++ (a fixed-capacity view) -> Python and is recorded: the bytes are the
    golden bytes. The recording of what Python sent is the golden bytes too (both directions, one test)."""
    lk = like_keras()
    from kpi.golden.test_golden import emitter

    prefix = PER_JOINT[type_name]
    s = lk.System("t10")
    emit = s.add(emitter(lk, type_name)(), "emit")
    box = s.add(lk.native(native_library("echo"), config={"unused": 0}), "box", **{f"{prefix}_in": emit.value})
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.emit, hz=10)
    dep.place(s.box, hz=10)
    dep.record(tmp_path / "t10.mcap", emit.value, getattr(box, f"{prefix}_out"))
    dep.start(until=0.45)
    dep.stop()
    recording = mcap.read(tmp_path / "t10.mcap")
    golden = make_golden.golden_path(type_name).read_bytes()
    assert [m.data for m in recording.on("value")] == [golden], "Python's encoding differs from the golden bytes"
    assert [m.data for m in recording.on(f"{prefix}_out")] == [golden], "the C++ round trip changed the bytes"


def test_T10_trajectory_view(tmp_path):
    """The C++ JointTrajectory view reads the right buffers: t, positions (NaN payload included), the stamp,
    N, J and velocities arrive bit for bit."""
    lk = like_keras()
    from kpi.golden.test_golden import emitter
    from kpi.support.lk import recorded

    s = lk.System("t10")
    emit = s.add(emitter(lk, "trajectory/JointTrajectory")(), "emit")
    box = s.add(lk.native(native_library("echo"), config={"unused": 0}), "box", trajectory=emit.value)
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.emit, hz=10)
    dep.place(s.box, hz=10)
    dep.record(tmp_path / "digest.mcap", box.digest)
    dep.start(until=0.45)
    dep.stop()
    digests = recorded(tmp_path / "digest.mcap", "digest")
    assert len(digests) == 1
    plan = make_golden.INSTANCES["trajectory/JointTrajectory"]
    positions = np.asarray(plan["positions"]).ravel()
    expected = [plan["t"][0], plan["t"][-1], positions[1], positions[-1], plan["header"]["stamp"], 3.0, 3.0,
                np.asarray(plan["velocities"]).ravel()[3]]
    assert np.asarray(digests[0]["position"]).tobytes() == np.asarray(expected, dtype="<f8").tobytes()


def test_T10_hash_mismatch_refused(tmp_path):
    """A .so whose port hash differs from the Python type's is refused at connect, naming both hashes."""
    lk = like_keras()
    skew = BUILD / "libfake_native_skew.so"
    if not skew.exists():
        pytest.skip(f"{skew} not built")
    from kpi.polyglot.parts import JointSource

    s = lk.System("skew")
    source = s.add(JointSource(), "source")
    with pytest.raises(ValueError) as refused:
        s.add(lk.native(skew), "box", input=source.output)
        dep = lk.Deployment(s)
        dep.place(s.source, hz=10)
        dep.place(s.box, hz=10)
        dep.describe()
    message = str(refused.value)
    assert "0123456789abcdef" in message and f"{lkf.schema_hash('sensor/JointState'):016x}" in message


def test_cpp_boilerplate():
    """kpi/surface.py --cpp-boilerplate kpi/tasks/cpp_gain: <= 2 files, <= 10 framework lines, and it builds."""
    native_library("cpp_gain")
    result = subprocess.run([sys.executable, str(KPI / "surface.py"), "--cpp-boilerplate", str(KPI / "tasks" / "cpp_gain")],
                            capture_output=True, text=True, timeout=60)
    report = json.loads(result.stdout)
    assert report["files"] <= 2 and report["framework_lines"] <= 10, report
