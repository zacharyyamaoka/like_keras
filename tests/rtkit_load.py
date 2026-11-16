"""Lane tests that need rtkit grants (CONTRACT §15 A35: tests/test_dora_boundary.py, tests/test_io_ladder.py) share one
rtkit budget with every other lane on this host: rtkit refuses a user's request past its burst limit (25 requests per
20 s) or its concurrent-threads limit, whoever spent it. This tells such a refusal, caused by another process's load,
from a failure of the library, and runs the scenario again instead of failing on it.

WHY (library verifier, 2026-11-16 13:58:50): rtkit's burst limit, hit by a concurrent lane's lk-dora-host, failed the
as-built "re-raised within 3 s" check of tests/test_dora_boundary.py; it passed on re-run. The module's own 21 s pacing
covers only its own requests.

What is never excused: a problem a refused grant cannot explain (a thread realtime when it must not be, a probe that
died), and a refusal that only this test's own requests explain (no other process granted in the same burst window).
An attempt excused every time is skipped naming the refusals, never passed.
"""

from __future__ import annotations

import datetime
import re
import subprocess
import time
from collections.abc import Callable, Iterable

import pytest

GRANT = "[grant] "  # marks a problem a refused rtkit grant explains (a thread never raised, a mutant never exercised)
BURST_S = 20.0  # rtkit-daemon's --actions-burst-sec (default 20; --actions-per-burst-max 25)
TRIES = 3
REFUSED = ("Reached burst limit for user", "Reached maximum concurrent threads limit for user")
GRANTED = re.compile(r"Successfully made thread \d+ of process (\d+) ")
OURS: set[int] = set()  # every process this session's rtkit tests had raised: never "another process"


def outside_refusals(since_s: float, until_s: float, ours: Iterable[int]) -> list[str]:
    """rtkit's refusals for its per-user limits from `since_s` to `until_s` (epoch s), when a process other than `ours`
    (and than every process an earlier attempt named) was granted from one burst window before `since_s` on; else [],
    as when the journal cannot be read (then the failure stands)."""
    OURS.update(ours)
    command = ["journalctl", "-u", "rtkit-daemon", "--no-pager", "-o", "short-iso", "--since",
               f"@{int(since_s - BURST_S)}", "--until", f"@{int(until_s) + 2}"]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return refusals_in(done.stdout.splitlines(), since_s, OURS)


def refusals_in(lines: list[str], since_s: float, ours: set[int]) -> list[str]:
    """outside_refusals() on journal lines (journalctl -o short-iso) that reach one burst window before `since_s`."""
    def at(line: str) -> float:
        try:
            return datetime.datetime.fromisoformat(line.split(" ", 1)[0]).timestamp()
        except ValueError:
            return 0.0

    others = {int(found.group(1)) for line in lines if (found := GRANTED.search(line))} - ours
    refused = [line for line in lines if any(reason in line for reason in REFUSED) and at(line) >= int(since_s)]
    if not others or not refused:
        return []
    return [f"{line} (other processes granted meanwhile: {sorted(others)[:5]})" for line in refused[:3]]


def judged(attempt: Callable[[int], tuple[list[str], Iterable[int]]]) -> list[str]:
    """attempt(index) runs the scenario once and returns (its problems, the processes it asked rtkit to raise). When
    every problem is a GRANT one and rtkit refused grants for another process's load meanwhile, it runs again (at most
    TRIES times; the caller paces its runs); excused every time, the test is skipped naming the refusals. Otherwise the
    problems are returned for the test to assert on: [] when the scenario held."""
    excuses = []
    for index in range(TRIES):
        began = time.time()
        problems, ours = attempt(index)
        if not problems:
            return []
        grant_only = all(problem.startswith(GRANT) for problem in problems)
        refused = outside_refusals(began, time.time(), ours) if grant_only else []
        if not refused:
            return problems
        excuses.append(f"attempt {index}: {problems} while rtkit refused: {refused}")
        print(f"rtkit refused grants for another process's load ({excuses[-1]})"
              + ("; it runs again" if index + 1 < TRIES else ""))
    pytest.skip(f"rtkit refused grants for other processes' load in all {TRIES} attempts (not judged): {excuses}")
