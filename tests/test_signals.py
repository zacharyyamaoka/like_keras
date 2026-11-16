"""Lane core's tests of CONTRACT §15 A18 on the Python side: SIGTERM and SIGHUP end a deployment the way SIGINT does.
The frozen M0.3 test sends them to the contract's file 3 (start, then signal.pause() in try/finally: stop); these
cover a start() the signal reaches while the main thread is still inside it (real time with until, and a lockstep
run), the process's exit status, the application's own handler and nohup's SIG_IGN left alone, the handlers put
back once nothing runs, and the dora Python host taking SIGTERM itself.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_signals.py

The dora case needs a private dora (bwrap --unshare-net, as kpi/run.py runs the dora suites); elsewhere it skips.
WHY (consumer-003): systemctl stop, kill and a dora stop of a Python node send SIGTERM, which ended a same_box
process with no deactivate: no hand-back.
"""

from __future__ import annotations

import os
import signal
import time

import pytest

import like_keras as lk
from kpi.facilities.friction_parts import HandBackPy
from kpi.support import procs
from kpi.support.lk import backend, recorded


def child(mode: str, directory) -> procs.Child:
    started = procs.Child("tests.signal_child", "--mode", mode, "--directory", str(directory),
                          log=directory / "child.log")
    started.wait_ready(timeout_s=60.0)
    return started


def handed_back(directory) -> bool:
    path = directory / "handed_back"
    return path.exists() and path.read_text() == "handed back\n"


@pytest.mark.parametrize("mode", ["until", "lockstep"])
@pytest.mark.parametrize("number", [signal.SIGTERM, signal.SIGHUP], ids=["SIGTERM", "SIGHUP"])
def test_a_signal_inside_start_stops_the_run_and_ends_the_process_by_that_signal(mode, number, tmp_path):
    process = child(mode, tmp_path)
    try:
        process.signal(number)
        code = process.process.wait(timeout=30)
    finally:
        process.stop()
    # WHY death by the signal, not exit(128 + n): systemd counts it as a clean stop, and an exit code as a failure
    assert code == -number, code
    assert handed_back(tmp_path)
    assert any("handed back" in row["message"] for row in recorded(tmp_path / "run.mcap", "log"))


def test_a_signal_while_start_starts_a_thread_still_hands_back_and_ends_by_that_signal(tmp_path):
    """A SIGTERM that unwinds start() while it is starting a thread: stop() must not refuse to join a thread that never
    started ("cannot join thread before it is started", exit 1, nothing handed back: tests/test_signals.py's until
    cases failed 2 in 2, then 1 in 3, at c8d43b9 on the loaded host, fix round 1 after the first consumer)."""
    process = child("starting", tmp_path)
    try:
        code = process.process.wait(timeout=30)
    finally:
        process.stop()
    assert code == -signal.SIGTERM, code
    assert handed_back(tmp_path)


def test_the_applications_own_handler_is_left_alone(tmp_path):
    process = child("own", tmp_path)
    try:
        process.signal(signal.SIGTERM)
        code = process.process.wait(timeout=30)
    finally:
        process.stop()
    assert code == 7  # its handler ran (sys.exit(7)); its finally stopped the deployment, as it chose to
    assert handed_back(tmp_path)


def test_nohups_ignored_sighup_stays_ignored(tmp_path):
    process = child("ignored", tmp_path)
    try:
        process.signal(signal.SIGHUP)
        time.sleep(1.0)  # an ignored SIGHUP: still running, nothing handed back
        assert process.process.poll() is None and not handed_back(tmp_path)
        process.signal(signal.SIGTERM)
        code = process.process.wait(timeout=30)
    finally:
        process.stop()
    assert code == -signal.SIGTERM and handed_back(tmp_path)


def test_the_handlers_are_ours_only_while_a_deployment_runs(tmp_path):
    if signal.getsignal(signal.SIGTERM) != signal.SIG_DFL:
        pytest.skip("this test process has a SIGTERM handler of its own; like_keras leaves it alone")
    s = lk.System("handlers")
    s.add(HandBackPy(path=str(tmp_path / "handed_back")), "box")
    dep = lk.Deployment(s)
    dep.place(s.box, hz=20)
    dep.start()
    try:
        assert signal.getsignal(signal.SIGTERM) != signal.SIG_DFL
    finally:
        dep.stop()
    assert signal.getsignal(signal.SIGTERM) == signal.SIG_DFL and handed_back(tmp_path)


def test_the_dora_python_host_stops_its_groups_on_sigterm(tmp_path, request):
    request.getfixturevalue("private_dora")
    name = f"host_signal_{int(time.time() * 1000) % 100000}"
    s = lk.System(name)
    s.add(HandBackPy(path=str(tmp_path / "handed_back")), "box")
    dep = lk.Deployment(s, backend=backend("dora"))
    dep.place(s.box, process="box", hz=20)
    dep.start()
    try:
        host = procs.wait_for(lambda: next(iter(procs.pids_with("like_keras.backends.dora.host", name)), None), 10.0,
                              "the box's Python host")
        os.kill(host, signal.SIGTERM)
        procs.wait_for(lambda: handed_back(tmp_path), 15.0, "deactivate's file after a SIGTERM to the Python host")
    finally:
        dep.stop()
