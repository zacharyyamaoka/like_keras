"""The RT ladder's two processes (CONTRACT §15 A19; kpi/blackbox/test_blackbox_friction.py).

    python kpi/blackbox/rt_ladder_child.py --oracle
    python kpi/blackbox/rt_ladder_child.py --library build/kpi/librt_ladder.so --cpu N --directory DIR

(run by path with the repo on PYTHONPATH: kpi/blackbox.py takes the name kpi.blackbox, so this folder is no package)

--oracle: what this host offers a process like this one, found by asking for it on a throwaway thread and giving it
back at once: SCHED_FIFO 80 itself (an rtprio limit, or root), else rtkit's SCHED_RR (rtkit on the system bus, this
process in the host's PID namespace, since rtkit names threads by their host thread ids). It prints one JSON line.

The run: rt_ladder.cpp's box in this process (same_box) at 100 Hz with fifo=80 for 4 s, real time, its metrics to
DIR/metrics.jsonl. At 1 s and at the end it notes the scheduling policy of every native thread (the threads that
are not Python's) and this process's RLIMIT_RTTIME. It prints one JSON line, and exits 0 only if it lived through the
box's spin past RLIMIT_RTTIME's soft limit (SIGXCPU must demote, never kill). Run apart from the suite's process so
a policy, a limit or a signal never reaches it.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import resource
import shutil
import subprocess
import threading
import time

RTKIT = ("org.freedesktop.RealtimeKit1", "/org/freedesktop/RealtimeKit1", "org.freedesktop.RealtimeKit1")
INITIAL_PID_NAMESPACE = 0xEFFFFFFC  # PROC_PID_INIT_INO: the host's PID namespace


def on_a_thread(probe):
    """probe() on a fresh thread (the policy it may change is that thread's alone), its result."""
    result = []
    thread = threading.Thread(target=lambda: result.append(probe()))
    thread.start()
    thread.join()
    return result[0]


def rtkit_property(name: str) -> int | None:
    busctl = shutil.which("busctl")
    if busctl is None:
        return None
    try:
        found = subprocess.run([busctl, "--system", "get-property", *RTKIT, name], capture_output=True, text=True,
                               timeout=5)
    except subprocess.TimeoutExpired:
        return None
    parts = found.stdout.split()
    return int(parts[1]) if found.returncode == 0 and len(parts) == 2 else None


def give_back() -> None:
    """This thread back to SCHED_OTHER. WHY reset-on-fork: rtkit grants with SCHED_RESET_ON_FORK set, and an
    unprivileged thread may not clear it (EPERM); the thread ends right after anyway."""
    try:
        os.sched_setscheduler(0, os.SCHED_OTHER | os.SCHED_RESET_ON_FORK, os.sched_param(0))
    except OSError:
        pass


def oracle() -> dict:
    def fifo() -> bool:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
        except OSError:
            return False
        give_back()
        return True

    if on_a_thread(fifo):
        return {"fifo": True, "rtkit": False, "reason": "SCHED_FIFO 80 is permitted here"}
    if os.stat("/proc/self/ns/pid").st_ino != INITIAL_PID_NAMESPACE:
        return {"fifo": False, "rtkit": False, "reason": "a private PID namespace: rtkit cannot name our threads"}
    highest, rttime_max = rtkit_property("MaxRealtimePriority"), rtkit_property("RTTimeUSecMax")
    if not highest or not rttime_max:
        return {"fifo": False, "rtkit": False, "reason": "no rtkit on the system bus"}
    resource.setrlimit(resource.RLIMIT_RTTIME, (rttime_max // 2, rttime_max))  # rtkit grants only under a limit

    def rtkit() -> bool:
        try:
            subprocess.run([shutil.which("busctl"), "--system", "call", *RTKIT, "MakeThreadRealtimeWithPID", "ttu",
                            str(os.getpid()), str(threading.get_native_id()), "1"], capture_output=True, timeout=5)
        except subprocess.TimeoutExpired:
            return False
        granted = os.sched_getscheduler(0) & ~os.SCHED_RESET_ON_FORK == os.SCHED_RR
        give_back()
        return granted

    granted = on_a_thread(rtkit)
    return {"fifo": False, "rtkit": granted, "rtkit_max_priority": highest, "rtkit_rttime_max_us": rttime_max,
            "reason": "rtkit granted SCHED_RR to a probe thread" if granted else "rtkit refused a probe thread"}


def native_threads() -> dict[str, list[int]]:
    """{tid: [policy, priority]} of every thread of this process that is not a Python thread."""
    python = {thread.native_id for thread in threading.enumerate()}
    found = {}
    for task in os.listdir("/proc/self/task"):
        tid = int(task)
        if tid in python:
            continue
        try:
            found[task] = [os.sched_getscheduler(tid) & ~os.SCHED_RESET_ON_FORK, os.sched_getparam(tid).sched_priority]
        except OSError:
            continue
    return found


def run(library: str, cpu: int, directory: pathlib.Path) -> dict:
    import like_keras as lk

    from kpi.facilities.parts import MetricsLog

    s = lk.System("rt_ladder")
    box = s.add(lk.native(library, config={"spin_at_s": 1.5}), "box")
    s.add(MetricsLog(path=str(directory / "metrics.jsonl")), "sink", value=box.metrics)
    dep = lk.Deployment(s)
    dep.place(s.box, process="box", hz=100, cpus=[cpu], fifo=80)
    dep.place(s.sink, process="sinks")
    dep.start()
    try:
        time.sleep(1.0)
        before, rttime = native_threads(), resource.getrlimit(resource.RLIMIT_RTTIME)
        time.sleep(3.0)
        after = native_threads()
    finally:
        dep.stop()
    infinite = resource.RLIM_INFINITY
    return {"threads_before_spin": before, "threads_after_spin": after,
            "rttime_us": [None if value == infinite else value for value in rttime]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--oracle", action="store_true")
    parser.add_argument("--library")
    parser.add_argument("--cpu", type=int)
    parser.add_argument("--directory", type=pathlib.Path)
    arguments = parser.parse_args()
    result = oracle() if arguments.oracle else run(arguments.library, arguments.cpu, arguments.directory)
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
