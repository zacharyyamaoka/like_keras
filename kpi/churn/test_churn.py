"""Criterion 6, insulation from dora churn: parts 1-4 here; part 5 is kpi/conformance green on both backends.

1 dora is named in no first-party file outside src/like_keras/backends/dora and cpp/hosts (and that backend exists)
2 the wire is like_keras's: the golden bytes come out with dora and pyarrow blocked, the hash in describe()
3 dora is used through nodes only: the emitted dataflow has no operator, no `custom:`, restart_policy everywhere
4 a dora CLI of another version is refused within 2 s naming both versions, and a hanging one never hangs start (T11)
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import textwrap
import time

import pytest

from kpi.golden import make_golden
from kpi.support.lk import REPO, dora_backend, like_keras, subprocess_env

DORA_TOKEN = re.compile(r"dora", re.IGNORECASE)


def first_party_files():
    for root in (REPO / "src", REPO / "cpp", REPO / "include", REPO / "cmake"):
        if not root.exists():
            continue
        for path in root.rglob("*"):
            relative = path.relative_to(REPO).as_posix()
            if path.is_file() and not relative.startswith(("src/like_keras/backends/dora", "cpp/hosts")) \
                    and "__pycache__" not in relative and path.suffix in (".py", ".hpp", ".h", ".cpp", ".c", ".cmake", ".txt", ".toml", ".yml", ".yaml", ".json", ".sh"):
                yield path


def test_dora_named_only_in_its_backend():
    assert (REPO / "src" / "like_keras" / "backends" / "dora").is_dir(), "no dora backend yet: nothing is insulated"
    naming = [str(path.relative_to(REPO)) for path in first_party_files() if DORA_TOKEN.search(path.read_text(errors="replace"))]
    assert naming == [], f"files outside the dora backend name dora: {naming}"


def test_wire_is_owned_by_like_keras(tmp_path):
    """With dora, pyarrow and the dora backend blocked, recording a JointState gives the golden bytes."""
    like_keras()
    script = textwrap.dedent(f'''
        import importlib.abc, sys
        class Blocker(importlib.abc.MetaPathFinder):
            def find_spec(self, name, path=None, target=None):
                if name.split(".")[0] in ("dora", "pyarrow") or name.startswith("like_keras.backends.dora"):
                    raise ImportError(name + " is blocked")
        sys.meta_path.insert(0, Blocker())
        import like_keras as lk
        from kpi.golden.instances import emitter
        s = lk.System("wire")
        s.add(emitter(lk, "sensor/JointState")(), "emit")
        dep = lk.Deployment(s, realtime=False)
        dep.place(s.emit, hz=10)
        dep.record({str(tmp_path / "wire.mcap")!r}, s.emit.value)
        dep.start(until=0.25)
        dep.stop()
        from kpi.support import mcap
        frames = [m.data for m in mcap.read({str(tmp_path / "wire.mcap")!r}).on("value")]
        golden = {str(make_golden.golden_path("sensor/JointState"))!r}
        assert frames == [open(golden, "rb").read()], "not the golden bytes"
        print("ok")
    ''')
    result = subprocess.run([sys.executable, "-c", script], cwd=REPO, env=subprocess_env(), capture_output=True,
                            text=True, timeout=120)
    assert result.returncode == 0 and result.stdout.strip().endswith("ok"), result.stderr[-3000:]


def test_nodes_only(tmp_path, private_dora):
    """The dataflow start() writes: nodes only, no operators, no `custom:`, restart_policy always on every node,
    no `_unstable_deploy` without host=, ids from [a-zA-Z0-9_.-]."""
    lk = like_keras()
    dora = dora_backend()
    from kpi.conformance.parts import Counter, QueueSink

    name = f"churn_{os.getpid()}"
    s = lk.System(name)
    counter = s.add(Counter(limit=5), "counter")
    s.add(QueueSink(path=str(tmp_path / "sink.jsonl")), "sink", value=counter.value)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.counter, process="writer", hz=10)
    dep.place(s.sink, process="reader")
    dep.start()
    time.sleep(1.0)
    dep.stop()
    flows = list((REPO / "build" / "like_keras" / name).glob("dataflow.yml"))
    assert flows, "start() wrote no build/like_keras/<system>/dataflow.yml"
    import yaml

    flow = yaml.safe_load(flows[0].read_text())
    text = flows[0].read_text()
    assert "operator" not in text and "custom:" not in text and "_unstable_deploy" not in text, text
    for node in flow["nodes"]:
        assert node.get("restart_policy") == "always", node
        assert re.fullmatch(r"[a-zA-Z0-9_.-]+", node["id"]), node["id"]


@pytest.mark.parametrize("fake", ["other_version", "hangs"])
def test_T11_cli_skew_refused_within_2s(fake, tmp_path, monkeypatch):
    """A `dora` CLI of another version on PATH: start() refuses within 2 s naming both versions; one that hangs
    is refused within 2 s too (2 s timeout), never waited on."""
    lk = like_keras()
    dora = dora_backend()
    from kpi.conformance.parts import Counter

    script = tmp_path / "bin" / "dora"
    script.parent.mkdir()
    body = 'echo "dora-cli 0.4.1"' if fake == "other_version" else "sleep 30"
    script.write_text(f"#!/bin/sh\n{body}\n")
    script.chmod(0o755)
    monkeypatch.setenv("PATH", f"{script.parent}{os.pathsep}{os.environ['PATH']}")
    s = lk.System(f"skew_{fake}")
    s.add(Counter(), "counter")
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.counter, hz=10)
    started = time.monotonic()
    with pytest.raises(Exception) as refused:
        dep.start()
    elapsed = time.monotonic() - started
    dep.stop()
    assert elapsed <= 2.5, f"refused after {elapsed:.2f} s"
    message = str(refused.value)
    assert "0.5.0" in message, message
    if fake == "other_version":
        assert "0.4.1" in message, message


def test_golden_bytes_owned_here():
    """The wire layout is defined by like_keras (kpi/golden), not by dora: the frame carries no dora metadata."""
    frame = make_golden.golden_path("control/ServoState").read_bytes()
    assert frame[:4] == b"LKF1" and b"dora" not in frame.lower()
