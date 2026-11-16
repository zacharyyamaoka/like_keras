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


def test_a_component_exports_the_c_abi_and_nothing_else():
    """CONTRACT §15 A28 (consumer-013): lk_component() links every component with a version script, so its .so exports
    the lk_* C ABI and nothing else, whatever it links: here one linking a static library compiled with default
    visibility (as the consumer's mcn_core was; without the script: its 2 functions and the runtime's 5 std:: symbols),
    and the ruler's empty component. WHY: two components in one process bound each other's exported code."""
    configure = ["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"]
    for command in (configure, ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "links_static", "-j", "1"],
                    ["nice", "-n", "19", "cmake", "--build", str(ROOT / "build"), "--target", "empty", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    for library in (BUILD / "liblinks_static.so", ROOT / "build" / "kpi" / "libempty.so"):
        listed = subprocess.run(["nm", "-D", "--defined-only", str(library)], capture_output=True, text=True, check=True)
        names = [line.split()[-1] for line in listed.stdout.splitlines() if line.strip()]
        assert len(names) == 10 and all(name.startswith("lk_") for name in names), (library.name, names)
