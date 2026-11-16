"""Lane cpp's tests of the boundary's ladder (CONTRACT §15 A35): a host's own threads and its transport's daemon on the
box's ladder, in the shape of the raw-dora node's race tests (moteus_controller_node cpp/tests: rttime_probe,
daemon_probe, slow_busctl.hpp; fix rounds 4-6, credit). A probe child (tests/io_ladder_probe.py) is a dedicated host,
as lk-dora-host is: SIGXCPU blocked in every thread, so the box runs its sentinel.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_io_ladder.py

- a host thread that runs past RLIMIT_RTTIME's soft limit is demoted by the sentinel and the process lives;
- lk_stop while a grant for the daemon is in flight (a slow busctl stand-in holds rtkit's call): after the host has
  exited, the daemon has no realtime thread;
- lk_stop while the sentinel's re-promotion is in flight (held the same way): no thread of the host is realtime after.
Each runs on the runtime as built and on a mutant of cpp/src/rt.cpp that must fail it. It needs rtkit's route (the
host's PID namespace, rtkit on the system bus, no rtprio limit; kpi/blackbox/rt_ladder_child.py --oracle), else it
skips naming why. No timing is asserted (the rack is powered on this host). rtkit's per-user limits are shared with
every lane on this host: a check that needs a grant, failed while rtkit refused grants for another process's load,
runs the probe again (tests/rtkit_load.py).
"""

from __future__ import annotations

import json
import os
import pathlib
import signal
import subprocess
import time

import pytest

from kpi.support.lk import kpi_cpus, python, subprocess_env
from tests.rtkit_load import GRANT, judged

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build" / "tests-cpp"
SLOW_S = 0.5  # how long the stand-in holds the call: inside lk_stop's 1 s wait, so the runtime as built ends it
PACE_S = 21.0  # before a probe runs again for rtkit's load: its burst window and then some (tests/rtkit_load.py)

# WHY a shell stand-in first on PATH: the ladder spawns `busctl` by name (rt.cpp), and only the one call the probe
# names (LK_SLOW_PID, LK_SLOW_TID) is held; it marks that it holds it, so lk_stop comes while it is in flight
STAND_IN = """#!/bin/sh
case " $* " in
  *" MakeThreadRealtimeWithPID ttu $LK_SLOW_PID $LK_SLOW_TID "*) : > "$LK_SLOW_MARK"; sleep "$LK_SLOW_S";;
esac
exec /usr/bin/busctl "$@"
"""

# name: (text in cpp/src/rt.cpp, its replacement, the scenario it must fail)
MUTANTS = {
    "sentinel_demotes_nothing": ("      sweep(self, ladder.sentinel_tid.load(), true,",
                                 "      if (false) sweep(self, ladder.sentinel_tid.load(), true,", "io_spin"),
    "stop_waits_for_nothing": ("    settle(ending);\n", "    (void)ending;\n", "stop_while_raising"),
    "stop_waits_for_others_only": ("    return call.first != 0 && std::count(",
                                   "    return call.first != 0 && call.second != ::getpid() && std::count(",
                                   "repromote_at_stop"),
}


@pytest.fixture(scope="module")
def rtkit_route():
    done = subprocess.run([python(), str(ROOT / "kpi" / "blackbox" / "rt_ladder_child.py"), "--oracle"], cwd=ROOT,
                          env=subprocess_env(), capture_output=True, text=True, timeout=120)
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    offered = json.loads(lines[-1]) if lines else {}
    if done.returncode != 0 or offered.get("fifo") or not offered.get("rtkit"):
        pytest.skip(f"this host does not offer rtkit's route to this process: {offered or done.stderr[-500:]}")
    return offered


@pytest.fixture(scope="module")
def spinner():
    """tests/cpp's libspinner.so (tests/test_rt_ladder.py builds the same target)."""
    for command in (["cmake", "-S", str(ROOT / "tests" / "cpp"), "-B", str(BUILD), "-DCMAKE_BUILD_TYPE=Release"],
                    ["nice", "-n", "19", "cmake", "--build", str(BUILD), "--target", "spinner", "-j", "1"]):
        done = subprocess.run(command, capture_output=True, text=True, timeout=900, check=False)
        assert done.returncode == 0, done.stdout[-3000:] + done.stderr[-3000:]
    return BUILD / "libspinner.so"


def probe(library: pathlib.Path, scenario: str, directory: pathlib.Path) -> tuple[int, dict]:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "busctl").write_text(STAND_IN)
    (directory / "busctl").chmod(0o755)
    env = subprocess_env() | {"PATH": f"{directory}{os.pathsep}{os.environ.get('PATH', os.defpath)}",
                              "LK_SLOW_MARK": str(directory / "held"), "LK_SLOW_S": str(SLOW_S),
                              "LK_SLOW_PID": "0", "LK_SLOW_TID": "0"}
    done = subprocess.run([python(), str(ROOT / "tests" / "io_ladder_probe.py"), str(library), str(kpi_cpus()[0]),
                           scenario], cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    return done.returncode, json.loads(lines[-1]) if lines else {"stderr": done.stderr[-2000:]}


def failures(scenario: str, code: int, found: dict) -> list[str]:
    """What the run got wrong; [] when the ladder held. GRANT marks what a grant rtkit refused explains
    (tests/rtkit_load.py). A stand-in daemon is checked after the host has exited and the held call has had time to
    land (SLOW_S and then some), and killed."""
    wrong = [] if code == 0 else [f"the probe died (exit {code})"]
    tried = code != 0 or bool(found.get("at_start"))  # else: below an RT thread rtkit never raised, nothing is asked
    if not tried:
        wrong.append(f"{GRANT}the box's RT thread was never raised (rtkit), so the scenario was not tried")
    if tried and scenario in ("io_spin", "repromote_at_stop"):
        spun = found.get("spinner", {})
        if spun.get("raised") != 1 or (spun.get("before_spin") or [0])[0] != os.SCHED_RR:
            wrong.append(f"{GRANT}the host thread was not raised: {spun}")
        if code == 0 and (spun.get("after_spin") or [1])[0] != os.SCHED_OTHER:
            wrong.append(f"the sentinel did not demote the spinning thread: {spun}")
        if code == 0 and not any(name.startswith("lk-sentinel") for name in found.get("at_start", {})):
            wrong.append(f"{GRANT}no realtime sentinel at the start: {found.get('at_start')}")
    if tried and scenario == "repromote_at_stop":
        if not found.get("repromotion_in_flight"):
            wrong.append("the sentinel's re-promotion never started (no call held)")
        if (found.get("spinner", {}).get("after_stop") or [1])[0] != os.SCHED_OTHER:
            wrong.append(f"a re-promotion in flight at lk_stop outlived it: {found.get('spinner')}")
    if tried and scenario == "stop_while_raising" and not found.get("raise_in_flight"):
        wrong.append("the daemon's grant was never in flight (no call held)")
    if scenario == "stop_while_raising":
        daemon = found.get("daemon")
        if daemon:
            time.sleep(SLOW_S + 1.0)
            try:
                after = os.sched_getscheduler(daemon) & ~os.SCHED_RESET_ON_FORK
            finally:
                os.kill(daemon, signal.SIGKILL)
            if after != os.SCHED_OTHER:
                wrong.append(f"the daemon's grant outlived lk_stop and its host: policy {after} after the host exited")
    if code == 0 and found.get("realtime_after_stop"):
        wrong.append(f"realtime threads after lk_stop: {found['realtime_after_stop']}")
    return wrong


def judged_probe(library: pathlib.Path, scenario: str, directory: pathlib.Path, verdict) -> list[str]:
    """probe() judged by `verdict(wrong, found)` (problems, [] when it holds), run again while rtkit refuses grants for
    another process's load (tests/rtkit_load.py), each run PACE_S after the last."""
    def attempt(index: int):
        if index:
            time.sleep(PACE_S)
        code, found = probe(library, scenario, directory / f"attempt{index}")
        wrong = failures(scenario, code, found)
        problems = verdict(wrong, found)
        print(f"{scenario} attempt {index}: wrong {wrong}, judged {problems}; {json.dumps(found)[:600]}")
        return problems, [pid for pid in (found.get("pid"), found.get("daemon")) if pid]

    return judged(attempt)


@pytest.mark.parametrize("scenario", ["io_spin", "stop_while_raising", "repromote_at_stop"])
def test_the_ladder_holds(scenario, rtkit_route, spinner, tmp_path):
    assert judged_probe(spinner, scenario, tmp_path, lambda wrong, found: wrong) == []


@pytest.fixture(scope="module")
def objects(tmp_path_factory, spinner):
    """abi.cpp and the spinner compiled once; each mutant recompiles rt.cpp only (as tests/test_rt_ladder.py)."""
    directory = tmp_path_factory.mktemp("io_ladder_mutants")
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
def test_a_mutant_of_the_ladder_fails(mutant, rtkit_route, objects, tmp_path):
    text, replacement, scenario = MUTANTS[mutant]
    source = (ROOT / "cpp" / "src" / "rt.cpp").read_text()
    assert source.count(text) == 1, f"the mutated line moved: {text!r}"
    (tmp_path / "rt.cpp").write_text(source.replace(text, replacement))
    compile_object(tmp_path / "rt.cpp", tmp_path / "rt.o")
    library = tmp_path / "libio_ladder_mutant.so"
    link = ["nice", "-n", "19", "g++", "-shared", "-pthread", "-o", str(library), str(tmp_path / "rt.o"),
            str(objects / "abi.o"), str(objects / "spinner.o"),
            f"-Wl,--version-script={ROOT / 'cmake' / 'lk_component.map'}"]
    done = subprocess.run(link, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stderr[-3000:]

    def caught(wrong: list[str], found: dict) -> list[str]:  # caught: a problem no refused grant explains
        if any(not reason.startswith(GRANT) for reason in wrong):
            return []
        return [f"{GRANT}{mutant} passed {scenario} (or only a grant failed: {wrong})"]

    assert judged_probe(library, scenario, tmp_path / "run", caught) == []
