"""Lane core's test of fix round 1 after the first consumer, item 3 (CONTRACT §15 A28, consumer-012, T1): a reader
in the same deployment sees what a native box logged between two facility rounds, as it sees a Python twin's line.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_log_readers.py

WHY: a native box's log lines reached readers only at the 1 Hz facility rounds, and the last partial second's only at
the final drain, after the readers' groups had stopped; a Python component's reach them at once. So a log tap read a
line in `fallback` that it never read in `same_box` (verifier probe a28_swap: ['brake held: J2'] against []). Under
lockstep the box's log port is now drained after each step, so the reader sees the line at the step that logged it;
in real time the facilities thread drains it every 0.1 s.
"""

from __future__ import annotations

import dataclasses
import json
import pathlib
import subprocess
import time

import pytest

import like_keras as lk
from kpi.facilities.parts import LogLog

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"


class LogOnce(lk.Component):
    """tests/cpp/log_once.cpp in Python: one error line, the first step at least at_s after its first step."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        at_s: float = 0.45

    def forward(self):
        first = self._first = getattr(self, "_first", self.now())
        if getattr(self, "_said", False) or self.now() - first < self.params.at_s - 1e-9:
            return
        self._said = True
        self.log.error("brake held: %s", "J2")


@pytest.fixture(scope="module")
def library() -> pathlib.Path:
    """tests/cpp built (tests/test_cpp_box.py builds the same project): liblog_once.so."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "log_once", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD / "liblog_once.so"


def read(directory: pathlib.Path, library: pathlib.Path, language: str, realtime: bool) -> list[tuple]:
    """(the reader's step time, message) of every error line a rate-less reader got from the box or its twin."""
    s = lk.System(f"log_readers_{language}_{'rt' if realtime else 'ls'}")
    if language == "cpp":
        says = s.add(lk.native(library, config={"at_s": 0.45 if not realtime else 0.1}), "says")
    else:
        says = s.add(LogOnce(at_s=0.45 if not realtime else 0.1), "says")
    s.add(LogLog(path=str(directory / f"{language}.jsonl")), "reader", value=says.log)
    dep = lk.Deployment(s, realtime=realtime)
    dep.place(s.says, hz=20)
    dep.place(s.reader)  # rate-less: a step per line, at its send time under lockstep
    if realtime:
        dep.start()
        time.sleep(0.6)  # the line at 0.1 s, the next facility round at 1.0 s: never reached
        dep.stop()
    else:
        try:
            dep.start(until=0.9)  # the line at 0.45 s, the facility rounds at 0.0 s and 1.0 s
        finally:
            dep.stop()
    file = directory / f"{language}.jsonl"
    rows = [json.loads(line) for line in file.read_text().splitlines()] if file.exists() else []
    return [(round(row["now"], 9) if not realtime else None, row["message"]) for row in rows if row["level"] >= 40]


def test_under_lockstep_a_reader_gets_a_native_line_at_the_step_that_logged_it(library, tmp_path):
    python, cpp = read(tmp_path, library, "python", False), read(tmp_path, library, "cpp", False)
    assert python == [(0.45, "brake held: J2")], python
    assert cpp == python


def test_in_real_time_a_reader_gets_a_native_line_before_the_next_facility_round(library, tmp_path):
    python, cpp = read(tmp_path, library, "python", True), read(tmp_path, library, "cpp", True)
    assert python == [(None, "brake held: J2")], python
    assert cpp == python
