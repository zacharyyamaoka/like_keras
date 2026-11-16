"""Lane cpp's RT ladder probe (CONTRACT §15 A19; tests/test_rt_ladder.py): a child process that spins past the soft limit.

    python tests/rt_ladder_probe.py <libspinner.so> <cpu> <directory>

tests/cpp/spinner.cpp's box in this process (same_box) at 100 Hz with fifo=80 for 3 s, real time, its metrics to
<directory>/metrics.jsonl; at 1.5 s its forward spins its CPU until it is no longer realtime, or past the hard limit.
It prints one JSON line: every native thread's policy before the spin and after it, every realtime thread of the
process after stop(), and this process's RLIMIT_RTTIME. It exits 0 only if it lived through the spin: the library's
SIGXCPU handler demotes the RT thread; without it, SIGXCPU's default action ends the process, and SIGKILL at the hard
limit ends it when nothing demotes.
"""

from __future__ import annotations

import json
import os
import resource
import sys
import threading
import time

import like_keras as lk


def native_threads() -> dict[str, list[int]]:
    python = {thread.native_id for thread in threading.enumerate()}
    found = {}
    for task in os.listdir("/proc/self/task"):
        if int(task) in python:
            continue
        try:
            found[task] = [os.sched_getscheduler(int(task)) & ~os.SCHED_RESET_ON_FORK,
                           os.sched_getparam(int(task)).sched_priority]
        except OSError:
            continue
    return found


def realtime_threads() -> list[str]:  # every thread of this process, Python's included
    found = []
    for task in os.listdir("/proc/self/task"):
        try:
            if os.sched_getscheduler(int(task)) & ~os.SCHED_RESET_ON_FORK in (os.SCHED_FIFO, os.SCHED_RR):
                found.append(task)
        except OSError:
            continue
    return found


def main(library: str, cpu: int, directory: str) -> None:
    from kpi.facilities.parts import MetricsLog

    s = lk.System("rt_probe")
    box = s.add(lk.native(library, config={"spin_at_s": 1.5, "cap_s": 0.3}), "box")
    s.add(MetricsLog(path=os.path.join(directory, "metrics.jsonl")), "sink", value=box.metrics)
    dep = lk.Deployment(s)
    dep.place(s.box, process="box", hz=100, cpus=[cpu], fifo=80)
    dep.place(s.sink, process="sinks")
    dep.start()
    time.sleep(1.0)
    before, rttime = native_threads(), resource.getrlimit(resource.RLIMIT_RTTIME)
    time.sleep(2.0)
    after = native_threads()
    dep.stop()
    print(json.dumps({"threads_before_spin": before, "threads_after_spin": after,
                      "realtime_after_stop": realtime_threads(),
                      "rttime_us": [None if value == resource.RLIM_INFINITY else value for value in rttime]}),
          flush=True)


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), sys.argv[3])
