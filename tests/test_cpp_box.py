"""Lane cpp's C++ tests of one box (tests/cpp/test_box.cpp: its step clock and the counters its metrics publish),
built and run under pytest so `pytest tests` covers them.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

WHY -j 1 and nice: the rack on this host is powered all day; builds stay off its cores (CONTRACT §1).
"""

from __future__ import annotations

import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"


def test_box_tests_pass():
    configure = ["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"]
    build = ["nice", "-n", "19", "cmake", "--build", str(BUILD), "-j", "1"]
    for command in (configure, build):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    done = subprocess.run([str(BUILD / "box_tests")], capture_output=True, text=True, timeout=300, check=False)
    assert done.returncode == 0, done.stdout[-6000:] + done.stderr[-2000:]
