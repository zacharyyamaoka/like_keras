"""Lane cpp's regression test for the native always-on node's startup SIGSEGV (seen once in about 8 starts at M3).

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_native_drain.py

WHY: in real time two Python threads drain one native box through lk_take at once, the pump thread (NativeGroup.loop:
the exposed Outs) and the facilities thread (NativeGroup.publish: log, health, metrics). Both took into ONE ctypes
ArrowArray held by the group, so one thread's lk_take could overwrite the other's message between its take and its
release: the same message released twice (ASan: heap-use-after-free in release_built, allocated by Box::take on the
pump thread, freed on the facilities thread). At 8f09ce7 it killed 6 of 102 starts of the always-on node, each
within a second of ready, where both threads drain at once. This test makes both threads hold a taken message at the
same moment and asserts each holds its own.
"""

from __future__ import annotations

import pathlib
import threading

import pytest
from robot_msgs.std import DiagnosticRecord, Metrics

import like_keras as lk
from like_keras.backends import inproc

LIBRARY = pathlib.Path(__file__).resolve().parents[1] / "build" / "kpi" / "libempty.so"


def test_two_threads_draining_one_box_at_once_each_take_their_own_message(monkeypatch):
    if not LIBRARY.exists():
        raise FileNotFoundError(f"{LIBRARY} is not built: cmake --build /home/bam/like_keras/build -j 1")
    s = lk.System("drained")
    box = s.add(lk.native(LIBRARY, config={"gain": 1.0}), "box")
    dep = lk.Deployment(s, realtime=False)
    dep.place(box, hz=10)
    run = inproc.Run(dep._plan())
    (group,) = run.groups
    received: dict[str, list] = {"health": [], "metrics": []}
    for port, rows in received.items():
        group.sinks[port].append(lambda value, sent, rows=rows: rows.append(value))
    taken_at: dict[str, int] = {}
    shared: list[str] = []
    both_taken = threading.Barrier(2, timeout=10.0)
    real_import = inproc._import

    def gated(root_at: int, cls, keep: bool = True):
        """Each thread's first message waits here until the other thread has taken one too."""
        thread = threading.current_thread().name
        if shared:
            return None  # one destination was found shared: decode and release nothing more (no double release)
        if thread not in taken_at:
            taken_at[thread] = root_at
            both_taken.wait()
            if len(set(taken_at.values())) < 2:  # the other thread's take overwrote this one's message
                shared.append(thread)
                return None
        return real_import(root_at, cls, keep)

    monkeypatch.setattr(inproc, "_import", gated)
    failures: list[BaseException] = []

    def drain(port: str) -> None:
        index = group.outs.index(port)
        try:
            group._drain(0.0, index + 1, index)
        except BaseException as error:  # noqa: BLE001 - reported by the test thread below
            failures.append(error)

    group.activate(0.0)  # lk_create, lk_start stepped: the box has no thread of its own
    try:
        threads = [threading.Thread(target=drain, args=(port,), name=port) for port in received]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=30.0)
    finally:
        group.deactivate()
    assert not failures, failures
    assert not shared, f"both threads took into one destination at {taken_at}: a message would be released twice"
    assert sorted(row.path for row in received["health"]) == ["drained.box.empty", "drained.box.talker"]
    assert all(isinstance(row, DiagnosticRecord) for row in received["health"]), received["health"]
    assert sorted(row.path for row in received["metrics"]) == ["drained.box", "drained.box.empty", "drained.box.talker"]
    assert all(isinstance(row, Metrics) for row in received["metrics"]), received["metrics"]


@pytest.mark.parametrize("port", ["health", "metrics"])
def test_one_thread_still_takes_a_whole_round(port):
    """The per-call destination changes nothing for one thread: a round is every record, once."""
    if not LIBRARY.exists():
        raise FileNotFoundError(f"{LIBRARY} is not built: cmake --build /home/bam/like_keras/build -j 1")
    s = lk.System("drained_alone")
    box = s.add(lk.native(LIBRARY, config={"gain": 1.0}), "box")
    dep = lk.Deployment(s, realtime=False)
    dep.place(box, hz=10)
    (group,) = inproc.Run(dep._plan()).groups
    rows: list = []
    group.sinks[port].append(lambda value, sent: rows.append(value.path))
    group.activate(0.0)
    try:
        index = group.outs.index(port)
        group._drain(0.0, index + 1, index)
    finally:
        group.deactivate()
    expected = ["drained_alone.box.empty", "drained_alone.box.talker"]
    assert sorted(rows) == (expected if port == "health" else ["drained_alone.box", *expected])
