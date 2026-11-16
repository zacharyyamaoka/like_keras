"""Lane cpp's probe of the boundary's ladder (CONTRACT §15 A35; tests/test_io_ladder.py): a dedicated host in one
process, as lk-dora-host is, driving a box through the C ABI only.

    python tests/io_ladder_probe.py <libspinner.so> <cpu> <scenario>

SIGXCPU is blocked before any thread exists (lk-dora-host's main does the same), so the box's ladder runs its
sentinel. tests/cpp/spinner.cpp's box starts at 100 Hz with fifo=80 (it never spins here); then, by scenario:
  io_spin            a host thread is put on the ladder (lk_raise, 10 below the RT thread) and spins its own CPU time
                     past RLIMIT_RTTIME's soft limit: the sentinel must demote it before the hard limit's SIGKILL
  stop_while_raising a stand-in daemon (another process) is raised while the slow busctl stand-in holds rtkit's call;
                     lk_stop comes while it is in flight, and this process exits at once, as lk-dora-host does
  repromote_at_stop  io_spin's thread, demoted, waits for the sentinel's re-promotion 10 s later; the slow stand-in
                     holds that call, and lk_stop comes while it is in flight
It prints one JSON line: what each thread ran under when, every realtime thread of this process after lk_stop, the
stand-in daemon's pid; it exits 0 when it lived. The slow stand-in holds the call for LK_SLOW_PID / LK_SLOW_TID,
which this probe sets once only that call is still to come (tests/test_io_ladder.py writes the stand-in).
WHY in shape of the raw node's race tests (fix round 6, slow_busctl.hpp): a race shown only by chance is no test.
"""

from __future__ import annotations

import ctypes
import json
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGXCPU})  # first: every thread from here on inherits it

REALTIME = (os.SCHED_FIFO, os.SCHED_RR)


class Place(ctypes.Structure):
    _fields_ = [("hz", ctypes.c_double), ("cpus", ctypes.POINTER(ctypes.c_int)), ("ncpus", ctypes.c_int),
                ("fifo", ctypes.c_int), ("lock_memory", ctypes.c_int), ("stepped", ctypes.c_int)]


def policy(thread: int) -> list[int] | None:
    try:
        return [os.sched_getscheduler(thread) & ~os.SCHED_RESET_ON_FORK, os.sched_getparam(thread).sched_priority]
    except OSError:
        return None


def realtime_threads() -> dict[str, list[int]]:
    found = {}
    for task in os.listdir("/proc/self/task"):
        seen = policy(int(task))
        if seen is not None and seen[0] in REALTIME:
            found[pathlib.Path(f"/proc/self/task/{task}/comm").read_text().strip() + " " + task] = seen
    return found


def slow_next(pid: int, thread: int) -> None:
    """The slow busctl stand-in holds rtkit's call for this (pid, thread) from now on."""
    os.environ["LK_SLOW_PID"], os.environ["LK_SLOW_TID"] = str(pid), str(thread)


class Spinner(threading.Thread):
    """A host thread on the box's ladder: raised, then it spins its CPU time while realtime (0.4 s at most, past the
    hard limit), then sleeps until the probe ends."""

    def __init__(self, library, box) -> None:
        super().__init__(daemon=True)
        self.library, self.box, self.report, self.spun = library, box, {}, threading.Event()

    def run(self) -> None:
        self.report["raised"] = self.library.lk_raise(self.box, 0, self.native_id, 10)
        self.report["before_spin"] = policy(self.native_id)
        began = time.thread_time()
        while time.thread_time() - began < 0.4 and policy(self.native_id)[0] in REALTIME:
            pass
        self.report["spun_s"] = time.thread_time() - began
        self.report["after_spin"] = policy(self.native_id)
        self.spun.set()
        time.sleep(3600)


def main(library_path: str, cpu: int, scenario: str) -> None:
    library = ctypes.CDLL(library_path)
    library.lk_create.restype = ctypes.c_void_p
    library.lk_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
    for name in ("lk_start",):
        getattr(library, name).argtypes = [ctypes.c_void_p, ctypes.POINTER(Place), ctypes.c_char_p, ctypes.c_size_t]
    library.lk_raise.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    library.lk_stop.argtypes = [ctypes.c_void_p]
    error = ctypes.create_string_buffer(512)
    box = library.lk_create(json.dumps({"spin_at_s": 1e9, "cap_s": 0.3}).encode(), error, len(error))
    assert box, error.value
    cpus = (ctypes.c_int * 1)(cpu)
    assert library.lk_start(box, ctypes.byref(Place(100.0, cpus, 1, 80, 0, 0)), error, len(error)) == 0, error.value
    report: dict = {"scenario": scenario, "at_start": realtime_threads()}
    if scenario in ("io_spin", "repromote_at_stop"):
        spinner = Spinner(library, box)
        spinner.start()
        assert spinner.spun.wait(10.0), "the spinner never finished its spin"
        report["spinner"] = spinner.report
        if scenario == "repromote_at_stop":
            mark = pathlib.Path(os.environ["LK_SLOW_MARK"])
            slow_next(os.getpid(), spinner.native_id)
            deadline = time.monotonic() + 20.0  # the sentinel asks 10 s after the last SIGXCPU
            while not mark.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            report["repromotion_in_flight"] = mark.exists()
        library.lk_stop(box)
        report["spinner"]["after_stop"] = policy(spinner.native_id)
    elif scenario == "stop_while_raising":
        daemon = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)"], start_new_session=True,
                                  stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        report["daemon"] = daemon.pid
        mark = pathlib.Path(os.environ["LK_SLOW_MARK"])
        slow_next(daemon.pid, daemon.pid)
        raising = threading.Thread(target=library.lk_raise, args=(box, daemon.pid, daemon.pid, 15), daemon=True)
        raising.start()
        deadline = time.monotonic() + 10.0
        while not mark.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        report["raise_in_flight"] = mark.exists()
        library.lk_stop(box)
        report["daemon_after_stop"] = policy(daemon.pid)
    report["realtime_after_stop"] = realtime_threads()
    print(json.dumps(report), flush=True)
    os._exit(0)  # as lk-dora-host: _exit right after the stop, a request still in flight left to its fate


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
