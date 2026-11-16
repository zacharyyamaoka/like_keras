"""Lane dora's test of CONTRACT §15 A34 (consumer-017) where the frozen ruler test (consumer::
test_refused_activate_is_final_under_dora, a C++ box in lk-dora-host) does not reach: a PYTHON node that refuses in
activate is final too. Its host marks the refusal and waits for dora's Stop instead of exiting (restart_policy is
`always`, §9.2), so Deployment.start() raises naming it, the node was started once, and the dataflow is stopped, in
real time and in lockstep.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_dora_refusal.py

It needs a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites).
"""

from __future__ import annotations

import dataclasses
import os
import pathlib
import time

import pytest
from robot_msgs.sensor import JointState

import like_keras as lk
from kpi.polyglot.friction_parts import JointStateLog
from kpi.support import procs
from kpi.support.lk import backend
from like_keras.backends.dora import emit

REFUSAL = "REFUSED: the bus lock is held by lane_holder"


class RefuserPy(lk.Component):
    """kpi/consumer/refuser.cpp in Python: notes each start in a file, then refuses in activate, every time."""

    echo = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        starts_path: str

    def activate(self):
        with open(self.params.starts_path, "a", encoding="utf-8") as starts:
            starts.write("activate\n")
        raise RuntimeError(REFUSAL)

    def forward(self):  # never runs
        return None


@pytest.mark.parametrize("realtime", [True, False], ids=["realtime", "lockstep"])
def test_a_python_node_refused_in_activate_is_final(realtime, tmp_path, request):
    request.getfixturevalue("private_dora")
    name = f"refuse_py_{'rt' if realtime else 'ls'}_{os.getpid()}"
    starts = pathlib.Path(tmp_path / "starts")
    s = lk.System(name)
    box = s.add(RefuserPy(starts_path=str(starts)), "box")
    s.add(JointStateLog(path=str(tmp_path / "echo.jsonl")), "reader", value=box.echo)
    dep = lk.Deployment(s, backend=backend("dora"), realtime=realtime)
    dep.place(s.box, process="box", hz=20)
    dep.place(s.reader, process="reader")
    begun = time.monotonic()
    try:
        with pytest.raises(RuntimeError, match=REFUSAL):
            dep.start(until=None if realtime else 5.0)
        took = time.monotonic() - begun
        time.sleep(3.0)  # a node dora restarts refuses again within this (consumer-017: 30-100 restarts a second)
        assert starts.read_text().splitlines() == ["activate"], f"started {starts.read_text().count('activate')} times"
        # WHY its log too: a Python node restarted at once reaches activate only after the launcher's `dora stop`,
        # but every process the node had wrote its "--- box pid" line first (host.log_to)
        log = emit.directory_of(name) / "logs" / "box.log"
        processes = [line for line in log.read_text().splitlines() if line.startswith("--- box pid")]
        assert len(processes) == 1, f"dora restarted the refused node: {processes}"
        left = procs.pids_with("like_keras.backends.dora.host", name)
        assert not left, f"the dataflow was not stopped: its nodes {left} still run 3 s after start() raised"
        assert took < 60.0, f"start() raised after {took:.1f} s"
    finally:
        dep.stop()
