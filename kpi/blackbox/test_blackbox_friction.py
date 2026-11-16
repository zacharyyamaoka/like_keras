"""Criterion 3 (black box), ruler M0.3 (CONTRACT §15 A27): a C++ component never names its thread.

  A19 test_rt_ladder_is_the_librarys   the RT thread's scheduling ladder is the library's, with no hook for components

kpi/score.py counts it with every C++ row of criterion 3 (a C++ component is clean only when this passes too). Its box
(rt_ladder.cpp) is built on demand (kpi/support/lk.feature_library) and runs in a child process
(kpi/blackbox/rt_ladder_child.py), so no scheduling policy, limit or signal reaches the suite's process.
"""

from __future__ import annotations

import json
import os
import subprocess

from kpi.support.lk import KPI, REPO, feature_library, kpi_cpus, like_keras, python, subprocess_env

SCHED_OTHER, SCHED_FIFO, SCHED_RR = os.SCHED_OTHER, os.SCHED_FIFO, os.SCHED_RR
NAMES = {SCHED_OTHER: "SCHED_OTHER", SCHED_FIFO: "SCHED_FIFO", SCHED_RR: "SCHED_RR"}


def _child(*arguments: str) -> tuple[int, dict]:
    # WHY by path: kpi/blackbox.py, the static scan, takes the name kpi.blackbox, so this folder is no package
    result = subprocess.run([python(), str(KPI / "blackbox" / "rt_ladder_child.py"), *arguments], cwd=REPO,
                            env=subprocess_env(), capture_output=True, text=True, timeout=120)
    lines = [line for line in result.stdout.splitlines() if line.startswith("{")]
    return result.returncode, json.loads(lines[-1]) if lines else {"stderr": result.stderr[-2000:]}


def test_rt_ladder_is_the_librarys(tmp_path):
    """CONTRACT §15 A19: the RT thread's scheduling ladder is the library's: SCHED_FIFO, else rtkit's SCHED_RR, else
    SCHED_OTHER, reported in the box's metrics (sched_policy as Linux numbers it, and sched_priority, consumer-004's
    names); when rtkit grants, RLIMIT_RTTIME is set soft < hard and SIGXCPU demotes instead of killing. No hook for
    components: rt_ladder.cpp never touches its thread. Placed with fifo=80 in one process, the box must get what
    the host offers a process like it (an oracle child asks for it on a throwaway thread), say so in its metrics
    and on the thread itself, and, under rtkit, live through a spin past the soft limit, demoted. WHY
    (consumer-004): on a host with no rtprio limit the RT thread fell to SCHED_OTHER, where the raw node, on the
    same host with no extra privilege, ran SCHED_RR with the RTTIME guard; a .so in someone's process cannot do it.
    Under kpi/run.py's private PID namespace rtkit cannot name the threads, so there the honest answer is
    SCHED_OTHER, reported."""
    like_keras()
    library = feature_library("rt_ladder")
    code, offered = _child("--oracle")
    assert code == 0 and "fifo" in offered, offered
    expected = SCHED_FIFO if offered["fifo"] else SCHED_RR if offered["rtkit"] else SCHED_OTHER
    code, run = _child("--library", str(library), "--cpu", str(kpi_cpus()[0]), "--directory", str(tmp_path))
    assert code == 0, f"the process did not live through the run (exit {code}): {run}"
    records = [json.loads(line) for line in open(tmp_path / "metrics.jsonl", encoding="utf-8") if line.strip()]
    reported = [row for row in records if "sched_policy" in row["values"]]
    assert reported, f"no metrics record of the box reports sched_policy (A19); names: {records[-1]['values'] if records else None}"
    # the spin shows as the ticker's spun_s metric; sched_policy may sit on the box's own record or the ticker's
    spun_at = min((row["stamp"] for row in records if "spun_s" in row["values"]), default=None)
    before = [row["values"] for row in reported if spun_at is None or row["stamp"] < spun_at]
    after = [row["values"] for row in reported if spun_at is not None and row["stamp"] >= spun_at]
    assert before, "no sched_policy reported before the spin"
    policy, priority = int(before[-1]["sched_policy"]), int(before[-1].get("sched_priority", -1))
    assert policy == expected, (f"the box reports {NAMES.get(policy, policy)}; this host offers {NAMES[expected]} "
                                f"({offered['reason']})")
    threads = run["threads_before_spin"]
    if expected == SCHED_OTHER:
        assert all(found[0] == SCHED_OTHER for found in threads.values()), f"a native thread is realtime: {threads}"
        return
    granted = [found for found in threads.values() if found[0] == expected]
    assert granted, f"no native thread runs {NAMES[expected]}: {threads}"
    if expected == SCHED_FIFO:
        assert priority == 80 and [SCHED_FIFO, 80] in granted, (priority, threads)
        return
    highest = offered["rtkit_max_priority"]
    assert 1 <= priority <= min(80, highest) and [SCHED_RR, priority] in granted, (priority, threads)
    soft, hard = run["rttime_us"]
    assert soft is not None and hard is not None and soft < hard <= offered["rtkit_rttime_max_us"], run["rttime_us"]
    assert after, "the box never spun past the soft limit (no spun_s metric, or no sched_policy after it)"
    assert int(after[-1]["sched_policy"]) == SCHED_OTHER, f"not demoted after SIGXCPU: {after[-1]}"
    assert all(found[0] == SCHED_OTHER for found in run["threads_after_spin"].values()), run["threads_after_spin"]
