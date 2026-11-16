"""Lane dora's test of lk-dora-host's boundary (CONTRACT §15 A35), in the shape of the raw-dora node's keep-holding
dataflow test (moteus_controller_node tests/test_twin_dataflow.py, fix round 6, credit): the reference arm in
lk-dora-host, its planner in `main` and a subscriber in a process of its own, on a private dora; once the daemon is
raised the subscriber's node is SIGKILLed (a peer leaves, dora restarts it), 4 s later the deployment stops. The private daemon's
thread policies are sampled every 20 ms throughout. Then:
- the RT thread and the sentinel are realtime, RX, TX and dora's threads below them, the daemon's data-path threads
  below those, and the box record says io_sched_policy realtime;
- within 0.5 s of the departure no daemon thread is realtime, and within 3 s they are again (the disconnect passed);
- after the stop no daemon thread is realtime.
It runs on lk-dora-host as built and on a mutant that never watches its sibling nodes, which must fail (dora 0.5
closes no input for a peer it restarts, so only the watch sees it leave).

A shared daemon is never touched (A35): the same daemon also runs a foreign dataflow (one plain dora node, not
like_keras: tests/foreign_dora_node.py), the reference arm runs in lk-dora-host on it, and the daemon's threads are
sampled every 20 ms from before the start until after the stop: none is ever realtime, while lk-dora-host's own RT
thread is. It runs on lk-dora-host as built and on a mutant whose private_daemon() takes any `dora daemon` for
private, which must fail (library verifier, the round after A31-A42: that mutant passed every kpi and lane test;
its scenario, test_lkv_a35.py::test_shared_daemon_*, credit).

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_dora_boundary.py

It needs a private network namespace in the host's PID namespace (`bwrap --unshare-net`: the private dora, and rtkit
naming host thread ids) and rtkit's route; elsewhere it skips naming why. No timing is asserted beyond the 0.5 s and
3 s windows, which are 10x what was measured on the powered rig (33 ms and 0.24 s). rtkit's per-user limits are
shared with every lane on this host: a check that needs a grant, failed while rtkit refused grants for another
process's load, runs the scenario again (tests/rtkit_load.py).
"""

from __future__ import annotations

import json
import os
import pathlib
import shutil
import signal
import subprocess
import threading
import time

import pytest

from kpi.support.lk import kpi_cpus, native_library, python, subprocess_env
from tests.rtkit_load import GRANT, judged

ROOT = pathlib.Path(__file__).resolve().parents[1]
DORA_CXX = pathlib.Path(os.environ.get("LK_DORA_CXX_DIR", ROOT / "build" / "dora-cxx-0.5.0"))
REALTIME = (os.SCHED_FIFO, os.SCHED_RR)
RAISED_WITHIN_S, STOP_AFTER_S = 15.0, 4.0
LOOKS_PASSED_S = 6.0  # past the elevator's looks at the daemon (0 s, 1 s and 3 s after the boundary is raised)
# WHY 21 s between runs: rtkit allows a user 25 requests per 20 s (its burst limit), one run asks about 18 (the RT
# thread, the sentinel, six I/O threads, five daemon threads, five again after the departure), and every lane shares
# the budget (the raw node paces its dataflow tests the same way, fix round 6)
PACE_S = 21.0
last_run = [0.0]
NEVER_WATCHES = ("    if (!paused && siblings.left()) {", "    if (false) {")  # never watches its siblings
# private_daemon() takes any `dora daemon` for private, whatever its other nodes run
TAKES_ANY_DAEMON = ('           (argv[at + 1] == "daemon" && !nodes.empty() && std::all_of(',
                    '           (argv[at + 1] == "daemon") || (!nodes.empty() && std::all_of(')


@pytest.fixture(scope="module")
def rtkit_route():
    done = subprocess.run([python(), str(ROOT / "kpi" / "blackbox" / "rt_ladder_child.py"), "--oracle"], cwd=ROOT,
                          env=subprocess_env(), capture_output=True, text=True, timeout=120)
    lines = [line for line in done.stdout.splitlines() if line.startswith("{")]
    offered = json.loads(lines[-1]) if lines else {}
    if done.returncode != 0 or offered.get("fifo") or not offered.get("rtkit"):
        pytest.skip(f"this host does not offer rtkit's route to this process: {offered or done.stderr[-500:]}")
    return offered


def policies(pid: int) -> dict[str, list[int]]:
    """'<comm> <tid>' -> [policy, priority] for every thread of `pid`."""
    found = {}
    for task in os.listdir(f"/proc/{pid}/task"):
        try:
            name = pathlib.Path(f"/proc/{pid}/task/{task}/comm").read_text().strip()
            found[f"{name} {task}"] = [os.sched_getscheduler(int(task)) & ~os.SCHED_RESET_ON_FORK,
                                       os.sched_getparam(int(task)).sched_priority]
        except OSError:
            continue
    return found


def realtime(pid: int) -> list[int]:
    return sorted(priority for policy, priority in policies(pid).values() if policy in REALTIME)


class Sampler(threading.Thread):
    """The daemon's realtime threads every 20 ms, as (seconds since the start, their priorities)."""

    def __init__(self, daemon: int) -> None:
        super().__init__(daemon=True)
        self.pid, self.samples, self.halt = daemon, [], threading.Event()

    def run(self) -> None:
        began = time.monotonic()
        while not self.halt.is_set():
            self.samples.append((time.monotonic() - began, realtime(self.pid)))
            self.halt.wait(0.02)

    def done(self) -> list:
        self.halt.set()
        self.join()
        return self.samples


def arm_cell(name: str, tmp_path: pathlib.Path, metrics: bool):
    """The reference arm in lk-dora-host, its planner in `main`, a subscriber in a process of its own (and, with
    `metrics`, a reader of the box record), as a dora Deployment."""
    import like_keras as lk
    from like_keras.backends import dora

    from kpi.consumer.boundary_parts import SeqLog
    from kpi.facilities.parts import MetricsLog
    from kpi.support import cells
    from kpi.tasks.arm_cell.planner import PickPlanner
    from kpi.tasks.arm_cell.targets import TargetFile

    native_library("arm_controller")
    tmp_path.mkdir(parents=True, exist_ok=True)
    s = lk.System(name)
    targets = s.add(TargetFile(repeat=True), "targets")
    arm = s.add(lk.native(cells.cell_module().LIBRARY, config=cells.config(tmp_path, lockstep=False)), "arm")
    planner = s.add(PickPlanner(), "planner", target=targets.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.add(SeqLog(path=str(tmp_path / "states.jsonl")), "subscriber", value=arm.state)
    if metrics:
        s.add(MetricsLog(path=str(tmp_path / "metrics.jsonl")), "metrics_reader", value=arm.metrics)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=2)
    dep.place(s.planner)
    dep.place(s.subscriber, process="subscriber")
    if metrics:
        dep.place(s.metrics_reader, process="metrics_reader")
    return dep


def paced() -> None:
    time.sleep(max(0.0, last_run[0] + PACE_S - time.monotonic()))
    last_run[0] = time.monotonic()


def run(tmp_path, private_dora) -> dict:
    """The departure scenario above; what was seen, as data."""
    from kpi.support import procs

    daemon = private_dora.daemon_pid
    name = f"boundary_lane_{os.getpid()}"
    dep = arm_cell(name, tmp_path, metrics=True)
    seen: dict = {"ours": [daemon]}
    sampler = Sampler(daemon)
    paced()
    sampler.start()
    began = time.monotonic()
    dep.start()
    try:
        seen["ours"] += procs.pids_with("lk-dora-host", name)
        try:
            procs.wait_for(lambda: realtime(daemon) or None, RAISED_WITHIN_S, "the private daemon raised (rtkit)")
        except TimeoutError as never:
            seen["never_raised"] = str(never)
            return seen
        time.sleep(0.5)  # every data-path thread, not only the first
        host = procs.pids_with("lk-dora-host", name)[0]
        seen["host"], seen["daemon_before"] = policies(host), realtime(daemon)
        victim = next(pid for pid in procs.pids_with("like_keras.backends.dora.host", name)
                      if open(f"/proc/{pid}/cmdline", "rb").read().rstrip(b"\0").endswith(b"subscriber"))
        seen["killed_at_s"] = time.monotonic() - began
        os.kill(victim, signal.SIGKILL)
        time.sleep(STOP_AFTER_S)
    finally:
        seen["stop_at_s"] = time.monotonic() - began
        dep.stop()
        time.sleep(1.0)
        seen["daemon_after_stop"] = realtime(daemon)
        seen["samples"] = sampler.done()
    rows = [json.loads(line) for line in (tmp_path / "metrics.jsonl").read_text().splitlines() if line.strip()]
    seen["io_sched_policy"] = [row["values"].get("io_sched_policy") for row in rows if row["path"] == f"{name}.arm"]
    return seen


def failures(seen: dict) -> list[str]:
    """What the departure run got wrong; GRANT marks what a grant rtkit refused explains (tests/rtkit_load.py)."""
    wrong = [f"daemon threads realtime after the stop: {seen['daemon_after_stop']}"] if seen["daemon_after_stop"] else []
    if "never_raised" in seen:
        return [f"{GRANT}{seen['never_raised']}", *wrong]
    host = sorted(priority for policy, priority in seen["host"].values() if policy in REALTIME)
    if host.count(20) != 2 or host.count(10) < 3:
        wrong.append(f"{GRANT}lk-dora-host: the RT thread and the sentinel at 20, RX, TX and dora's at 10 expected: "
                     f"{host}")
    if not seen["daemon_before"] or max(seen["daemon_before"]) >= 10:
        wrong.append(f"the private daemon was not raised below the I/O threads: {seen['daemon_before']}")
    if os.SCHED_RR not in seen["io_sched_policy"]:
        wrong.append(f"{GRANT}the box record never said io_sched_policy realtime: {seen['io_sched_policy']}")
    killed, after = seen["killed_at_s"], [(at, len(rt)) for at, rt in seen["samples"] if at > seen["killed_at_s"]]
    paused = next((at for at, count in after if count == 0), None)
    if paused is None or paused > killed + 0.5:
        wrong.append(f"the daemon stayed realtime after a peer left at {killed:.2f} s (first 0 at {paused})")
    again = next((at for at, count in after if paused is not None and at > paused and count > 0), None)
    if again is None or again > killed + 3.0:
        wrong.append(f"{GRANT}the daemon was not raised again once the departure had passed (at {again})")
    return wrong


def mutant_host(tmp_path: pathlib.Path, mutation: tuple[str, str], monkeypatch) -> None:
    """lk-dora-host built from cpp/hosts/dora_host.cpp with one `mutation` (its text, its replacement), first on PATH
    for this test (emit.native_host(): PATH first)."""
    if not (DORA_CXX / "lib" / "libdora_node_api_cxx.a").exists():
        pytest.skip(f"dora's cxx node API is not built in {DORA_CXX} (cpp/hosts/build_dora_cxx.sh)")
    source = (ROOT / "cpp" / "hosts" / "dora_host.cpp").read_text()
    assert source.count(mutation[0]) == 1, f"the mutated line moved: {mutation[0]!r}"
    (tmp_path / "dora_host.cpp").write_text(source.replace(*mutation))
    host = tmp_path / "bin" / "lk-dora-host"
    host.parent.mkdir()
    command = ["nice", "-n", "19", "g++", "-std=c++20", "-O2", "-pthread", f"-I{DORA_CXX / 'include'}",
               f"-I{ROOT / 'cpp' / 'include'}", str(tmp_path / "dora_host.cpp"),
               str(DORA_CXX / "include" / "dora-node-api.cc"), str(DORA_CXX / "lib" / "libdora_node_api_cxx.a"),
               "-ldl", "-lm", "-lrt", "-o", str(host)]
    done = subprocess.run(command, capture_output=True, text=True, timeout=900)
    assert done.returncode == 0, done.stderr[-3000:]
    monkeypatch.setenv("PATH", f"{host.parent}{os.pathsep}{os.environ['PATH']}")
    assert shutil.which("lk-dora-host") == str(host)


def test_a_departed_peer_pauses_the_private_daemon(rtkit_route, private_dora, tmp_path):
    def attempt(index: int):
        seen = run(tmp_path / f"attempt{index}", private_dora)
        print({key: value for key, value in seen.items() if key != "samples"})
        return failures(seen), seen["ours"]

    assert judged(attempt) == []


def test_a_host_that_never_watches_its_peers_fails(rtkit_route, private_dora, tmp_path, monkeypatch):
    mutant_host(tmp_path, NEVER_WATCHES, monkeypatch)

    def attempt(index: int):
        seen = run(tmp_path / f"run{index}", private_dora)
        wrong = failures(seen)
        print(f"the mutant fails: {wrong}")
        caught = any("stayed realtime after a peer left" in reason for reason in wrong)
        return ([] if caught else [f"{GRANT}the mutant that never watches its peers was not caught: {wrong}"],
                seen["ours"])

    assert judged(attempt) == []


# ------------------------------------------------------------------ a shared daemon (A35: never touched)


def foreign_dataflow(private_dora, directory: pathlib.Path) -> None:
    """One foreign dora node (not like_keras) started detached on the private daemon, which is then SHARED."""
    from kpi.support import procs

    directory.mkdir(parents=True, exist_ok=True)
    # WHY /usr/bin/env: with the venv's python (a symlink to /usr/bin/python3.12) as its path, dora ran the node
    # without the venv (ModuleNotFoundError: dora); it died at once and the daemon was private again
    flow = {"nodes": [{"id": "foreign", "path": "/usr/bin/env",
                       "args": f"{python()} {ROOT / 'tests' / 'foreign_dora_node.py'}",
                       "inputs": {"tick": "dora/timer/millis/200"}}]}
    (directory / "foreign.yml").write_text(json.dumps(flow))
    done = subprocess.run([private_dora.cli, "start", str(directory / "foreign.yml"), "--name", "foreign", "--detach"],
                          cwd=private_dora.workdir, env=subprocess_env(), capture_output=True, text=True, timeout=120)
    assert done.returncode == 0, done.stdout + done.stderr
    procs.wait_for(lambda: foreign_nodes(private_dora.daemon_pid), 30.0, "the foreign node on the private daemon")


def foreign_nodes(daemon: int) -> list[int]:
    found = []
    for task in os.listdir(f"/proc/{daemon}/task"):
        for child in pathlib.Path(f"/proc/{daemon}/task/{task}/children").read_text().split():
            try:
                if b"foreign_dora_node.py" in pathlib.Path(f"/proc/{child}/cmdline").read_bytes():
                    found.append(int(child))
            except OSError:
                continue
    return found


def shared_run(tmp_path, private_dora) -> dict:
    """The arm in lk-dora-host on a private daemon that also runs a foreign dataflow; what was seen, as data."""
    from kpi.support import procs

    daemon = private_dora.daemon_pid
    name = f"shared_lane_{os.getpid()}"
    dep = arm_cell(name, tmp_path, metrics=False)
    seen: dict = {"ours": [daemon]}
    paced()
    foreign_dataflow(private_dora, tmp_path / "foreign")
    sampler = Sampler(daemon)
    sampler.start()
    try:
        dep.start()
        try:
            seen["ours"] += procs.pids_with("lk-dora-host", name)
            procs.wait_for(lambda: len((tmp_path / "states.jsonl").read_text().splitlines()) >= 100
                           if (tmp_path / "states.jsonl").exists() else None, 60.0, "states at the subscriber")
            time.sleep(LOOKS_PASSED_S)
            seen["host"] = policies(procs.pids_with("lk-dora-host", name)[0])
            seen["foreign_at_end"] = foreign_nodes(daemon)
        finally:
            dep.stop()
    finally:
        subprocess.run([private_dora.cli, "stop", "--name", "foreign"], cwd=private_dora.workdir, env=subprocess_env(),
                       capture_output=True, text=True, timeout=60)
        procs.wait_for(lambda: not foreign_nodes(daemon), 30.0, "the foreign node gone")
        time.sleep(1.0)
        samples = sampler.done()
    seen["samples"] = len(samples)
    seen["daemon_realtime"] = [(round(at, 2), rt) for at, rt in samples if rt]
    return seen


def shared_failures(seen: dict) -> list[str]:
    wrong = [] if seen["foreign_at_end"] else ["the foreign node did not live through the run: the daemon was not shared"]
    host = sorted(priority for policy, priority in seen["host"].values() if policy in REALTIME)
    if host.count(20) < 1:
        wrong.append(f"{GRANT}lk-dora-host's RT thread was never raised (rtkit), so its ladder was not tried: {host}")
    if seen["daemon_realtime"]:
        wrong.append(f"a SHARED daemon was raised in {len(seen['daemon_realtime'])} of {seen['samples']} samples: "
                     f"{seen['daemon_realtime'][:5]}")
    return wrong


def test_a_shared_daemon_is_never_raised(rtkit_route, private_dora, tmp_path):
    def attempt(index: int):
        seen = shared_run(tmp_path / f"attempt{index}", private_dora)
        print({key: value for key, value in seen.items() if key != "daemon_realtime"}, seen["daemon_realtime"][:5])
        return shared_failures(seen), seen["ours"]

    assert judged(attempt) == []


def test_a_host_that_raises_a_shared_daemon_fails(rtkit_route, private_dora, tmp_path, monkeypatch):
    mutant_host(tmp_path, TAKES_ANY_DAEMON, monkeypatch)

    def attempt(index: int):
        seen = shared_run(tmp_path / f"run{index}", private_dora)
        wrong = shared_failures(seen)
        print(f"the mutant fails: {wrong}")
        caught = any("a SHARED daemon was raised" in reason for reason in wrong)
        return ([] if caught else [f"{GRANT}the mutant that takes any daemon for private was not caught: {wrong}"],
                seen["ours"])

    assert judged(attempt) == []
