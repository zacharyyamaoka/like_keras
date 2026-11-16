"""Lane cpp's tests of the RT thread's scheduling ladder (CONTRACT §15 A19), in the raw-dora node's shape: a probe
child that spins past RLIMIT_RTTIME's soft limit (tests/rt_ladder_probe.py, on tests/cpp/spinner.cpp's box), on the
runtime as built and on mutants of cpp/src/rt.cpp that must fail: without the SIGXCPU handler SIGXCPU's default
action ends the process; with a handler that demotes nothing, or with soft == hard, the hard limit's SIGKILL does.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_rt_ladder.py

It needs what the ruler's oracle calls rtkit's route (kpi/blackbox/rt_ladder_child.py --oracle: the host's PID
namespace, rtkit on the system bus, no rtprio limit); elsewhere it skips, naming why. No timing is asserted (the rack
is powered on this host). WHY mutants: the raw node's ladder took six verify-and-fix rounds; a test that a mutant
also passes would not have caught any of them.
"""

from __future__ import annotations

import json
import pathlib
import signal
import subprocess

import pytest

from kpi.support.lk import kpi_cpus, python, subprocess_env

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"

# name: (text in cpp/src/rt.cpp, its replacement, how the probe must end)
MUTANTS = {
    "no_sigxcpu_handler": ("::sigaction(SIGXCPU, &action, &ladder.previous);", ";", -signal.SIGXCPU),
    "handler_demotes_nothing": ("if (thread != 0 && realtime(thread) && demote(thread))", "if (false)",
                                -signal.SIGKILL),
    "soft_equals_hard": ("limit.rlim_cur < hard ? limit.rlim_cur : hard / 2", "hard", -signal.SIGKILL),
}


def run(*arguments: str, timeout_s: float = 120.0) -> tuple[int, dict]:
    done = subprocess.run([python(), *arguments], cwd=ROOT, env=subprocess_env(), capture_output=True, text=True,
                          timeout=timeout_s)
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return done.returncode, json.loads(lines[-1]) if lines else {"stderr": done.stderr[-2000:]}


@pytest.fixture(scope="module")
def rtkit_route():
    code, offered = run(str(ROOT / "kpi" / "blackbox" / "rt_ladder_child.py"), "--oracle")
    if code != 0 or offered.get("fifo") or not offered.get("rtkit"):
        pytest.skip(f"this host does not offer rtkit's route to this process: {offered}")
    return offered


def probe(library: pathlib.Path, directory: pathlib.Path) -> tuple[int, dict]:
    directory.mkdir(parents=True, exist_ok=True)
    return run(str(ROOT / "tests" / "rt_ladder_probe.py"), str(library), str(kpi_cpus()[0]), str(directory))


def reported(directory: pathlib.Path) -> list[float]:
    rows = [json.loads(line) for line in (directory / "metrics.jsonl").read_text().splitlines() if line.strip()]
    return [row["values"]["sched_policy"] for row in rows if "sched_policy" in row["values"]]


@pytest.fixture(scope="module")
def built():
    """tests/cpp built (tests/test_cpp_box.py builds the same project): libspinner.so and its object."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "spinner", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD


def test_the_runtime_lives_through_a_spin_past_the_soft_limit_demoted(rtkit_route, built, tmp_path):
    code, found = probe(built / "libspinner.so", tmp_path)
    assert code == 0, f"the probe died (exit {code}): {found}"
    assert any(policy == 2 for policy, _ in found["threads_before_spin"].values()), found
    soft, hard = found["rttime_us"]
    assert soft is not None and hard is not None and soft < hard <= rtkit_route["rtkit_rttime_max_us"], found
    assert all(policy == 0 for policy, _ in found["threads_after_spin"].values()), found
    assert found["realtime_after_stop"] == [], "a grant outlived stop()"
    policies = reported(tmp_path)
    assert policies[0] == 2.0 and policies[-1] == 0.0, f"the box's sched_policy over the run: {policies}"


@pytest.fixture(scope="module")
def objects(tmp_path_factory):
    """abi.cpp and the spinner compiled once; each mutant recompiles rt.cpp only."""
    directory = tmp_path_factory.mktemp("rt_ladder_mutants")
    for source in (ROOT / "cpp" / "src" / "abi.cpp", ROOT / "tests" / "cpp" / "spinner.cpp"):
        compile_object(source, directory / f"{source.stem}.o")
    return directory


def compile_object(source: pathlib.Path, output: pathlib.Path) -> None:
    command = ["nice", "-n", "19", "g++", "-std=c++20", "-O2", "-fPIC", "-fvisibility=hidden",
               "-fvisibility-inlines-hidden", f"-I{ROOT / 'cpp' / 'include'}", f"-I{ROOT / 'cpp' / 'src'}", "-c",
               str(source), "-o", str(output)]
    done = subprocess.run(command, capture_output=True, text=True, timeout=900)
    assert done.returncode == 0, done.stderr[-3000:]


@pytest.mark.parametrize("mutant", list(MUTANTS))
def test_a_mutant_of_the_ladder_dies_in_the_spin(mutant, rtkit_route, objects, tmp_path):
    text, replacement, ending = MUTANTS[mutant]
    source = (ROOT / "cpp" / "src" / "rt.cpp").read_text()
    assert source.count(text) == 1, f"the mutated line moved: {text!r}"
    (tmp_path / "rt.cpp").write_text(source.replace(text, replacement))
    compile_object(tmp_path / "rt.cpp", tmp_path / "rt.o")
    library = tmp_path / "librt_ladder_mutant.so"
    link = ["nice", "-n", "19", "g++", "-shared", "-pthread", "-o", str(library), str(tmp_path / "rt.o"),
            str(objects / "abi.o"), str(objects / "spinner.o"),
            f"-Wl,--version-script={ROOT / 'cmake' / 'lk_component.map'}"]
    done = subprocess.run(link, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stderr[-3000:]
    code, found = probe(library, tmp_path / "run")
    assert code == ending, f"{mutant}: the probe ended {code}, expected {ending}: {found}"
