"""Lane cpp's tests of CONTRACT §15 A28 and A18 on the native side: every stop of a native box drains its facility
ports once more after the last deactivate(), so what a C++ component says in deactivate() (the controller's
hand-back line) reaches a reader. The frozen M0.3 test covers signals only (test_sigterm_runs_deactivate); these
cover a plain stop() in real time and in lockstep, SIGINT, and lk-dora-host taking SIGTERM, SIGHUP or SIGINT itself.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_final_drain.py

The lk-dora-host cases need a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); in the shared
network namespace they skip, never touching the production dora. WHY (consumer-012): a C++ box's deactivate line was
lost on every stop, even SIGINT, because the box was destroyed right after lk_stop.
"""

from __future__ import annotations

import json
import os
import signal
import time

import pytest

import like_keras as lk
from kpi.support import procs
from kpi.support.dora_private import private_dora  # noqa: F401  (the lk-dora-host cases' fixture)
from kpi.support.lk import feature_library, recorded


def logged(path) -> list[str]:
    return [row["message"] for row in recorded(path, "log")]


@pytest.mark.parametrize("realtime", [True, False], ids=["realtime", "lockstep"])
def test_a_plain_stop_drains_deactivates_line(realtime, tmp_path):
    s = lk.System(f"plain_stop_{'rt' if realtime else 'ls'}")
    box = s.add(lk.native(feature_library("hand_back"), config={"path": str(tmp_path / "handed_back")}), "box")
    dep = lk.Deployment(s, realtime=realtime)
    dep.record(tmp_path / "run.mcap", box.log)
    dep.place(s.box, hz=20)
    if realtime:
        dep.start()
        time.sleep(0.3)  # a partial second: no facility round has run since the start
    else:
        dep.start(until=0.3)
    dep.stop()
    assert (tmp_path / "handed_back").read_text() == "handed back\n"
    assert "handed back" in logged(tmp_path / "run.mcap"), logged(tmp_path / "run.mcap")


def test_sigint_drains_deactivates_line(tmp_path):
    """The contract's file 3 (signal.pause() in try/finally: stop) ended by SIGINT, which reaches the finally today."""
    child = procs.Child("kpi.facilities.hand_back_child", "--language", "cpp", "--backend", "inproc", "--directory",
                        str(tmp_path), "--name", f"sigint_{os.getpid()}", "--library", str(feature_library("hand_back")),
                        log=tmp_path / "child.log")
    try:
        child.wait_ready(timeout_s=60.0)
        time.sleep(0.5)
        child.signal(signal.SIGINT)
        child.process.wait(timeout=30)
    finally:
        child.stop()
    assert (tmp_path / "handed_back").read_text() == "handed back\n"
    assert "handed back" in logged(tmp_path / "run.mcap"), logged(tmp_path / "run.mcap")


@pytest.mark.parametrize("number", [signal.SIGTERM, signal.SIGHUP, signal.SIGINT], ids=lambda n: signal.Signals(n).name)
def test_lk_dora_host_takes_a_signal_and_its_last_line_reaches_a_reader(number, tmp_path, request):
    """A18 in lk-dora-host: the signal goes to the box's node alone; its sigwait thread ends the node the way dora's
    Stop does (deactivate, the hand-back file), and A28: deactivate's line still reaches a Python sink in another
    process, which keeps running."""
    request.getfixturevalue("private_dora")
    from kpi.facilities.parts import LogLog
    from like_keras.backends import dora

    name = f"node_signal_{number}_{os.getpid()}"
    s = lk.System(name)
    box = s.add(lk.native(feature_library("hand_back"), config={"path": str(tmp_path / "handed_back")}), "box")
    s.add(LogLog(path=str(tmp_path / "log.jsonl")), "sink", value=box.log)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.box, process="box", hz=20)
    dep.place(s.sink, process="sink")
    dep.start()
    try:
        node = procs.wait_for(lambda: next(iter(procs.pids_with("lk-dora-host", name)), None), 10.0, "lk-dora-host")
        time.sleep(0.5)
        os.kill(node, number)
        procs.wait_for((tmp_path / "handed_back").exists, 15.0, "deactivate's file after the signal")
        procs.wait_for(lambda: any("handed back" in line for line in (tmp_path / "log.jsonl").read_text().splitlines())
                       if (tmp_path / "log.jsonl").exists() else False, 15.0, "deactivate's line at the sink")
    finally:
        dep.stop()
    lines = [json.loads(line)["message"] for line in (tmp_path / "log.jsonl").read_text().splitlines()]
    assert "handed back" in lines, lines
