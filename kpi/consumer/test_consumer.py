"""The consumer's acceptance tests (CONTRACT §10; criterion 1 = T1-T8 and T12, each in every placement it names).

Every test runs the frozen reference consumer (kpi/tasks/arm_cell) on the fake bus. Placements: `target` (the C++
controller in its own process, dora between), `same_box` (the C++ chain on a GIL-free thread in this process),
`fallback` (all Python); `attached` is the always-on form with planners attached over its socket.
A test id passes only if every parametrization of it passes (kpi/score.py). Timing assertions run in the timing
tier only (KPI_TIER=timing: rack unpowered or the fixed cloud VM); elsewhere they skip and score 0.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from kpi.support import arm_cell_oracle, cells, procs
from kpi.support.lk import BUILD, KPI, binary, like_keras, lk_command, native_library, recorded, kpi_cpus
from kpi.tasks.arm_cell.fake_bus import EVENT_POSITION, EVENT_QUERY, EVENT_STOP, MODE_POSITION, MODE_TIMEOUT, FakeBoards
from kpi.tasks.arm_cell.jtc_law import TrajectoryLaw

LOCKSTEP_S = 4.0
TICK_S = 0.01
TOLERANCE_RAD = 1e-9  # T1


def _dora_if(placement: str, request) -> None:
    if placement in cells.NEEDS_DORA:
        request.getfixturevalue("private_dora")


def _assert_same_commands(left: list[dict], right: list[dict], what: str) -> None:
    left_stamps, left_positions = cells.command_rows(left)
    right_stamps, right_positions = cells.command_rows(right)
    assert len(left) == len(right) > 0, f"{what}: {len(left)} vs {len(right)} commands"
    assert np.array_equal(left_stamps, right_stamps), f"{what}: command stamps differ"
    worst = float(np.max(np.abs(left_positions - right_positions)))
    assert worst <= TOLERANCE_RAD, f"{what}: commands differ by {worst} rad (T1 allows {TOLERANCE_RAD})"


def _position_events(boards: FakeBoards, pid: int | None = None) -> np.ndarray:
    events = boards.events()
    events = events[(events["kind"] == EVENT_POSITION) & (events["t"] > 0.0)]
    return events if pid is None else events[events["pid"] == pid]


def _jitter_s(events: np.ndarray) -> np.ndarray:
    """Tick-start jitter as the device saw it: when the frame went out (wall) minus its scheduled time (now())."""
    return events["wall"] - events["t"]


def _sample(plan: dict, tau: float) -> np.ndarray:
    """The ruler's own JTC law at plan time tau (the law the reference controller vendors)."""
    law = TrajectoryLaw()
    samples = len(plan["t"])
    law.replace_input(np.asarray(plan["t"]), np.asarray(plan["positions"]).reshape(samples, -1),
                      None if plan["velocities"] is None else np.asarray(plan["velocities"]).reshape(samples, -1), None)
    law.tick(0.0)
    if tau > 0.0:
        law.tick(tau)
    return law.position


def _assert_plans_start_on_time(plans: list[dict], commands: list[dict]) -> None:
    """Every plan sent is received and starts at the first tick >= its header.stamp, within one tick (A1)."""
    stamps, positions = cells.command_rows(commands)
    assert plans, "no plan was sent"
    for plan in plans[:-1]:
        first = int(np.searchsorted(stamps, plan["header"]["stamp"] - 1e-9))
        moved = _sample(plan, TICK_S)
        started = [k for k in (first + 1, first + 2) if k < len(stamps) and np.allclose(positions[k], moved, atol=1e-6)]
        assert started, f"the plan stamped {plan['header']['stamp']:.3f} did not start within one tick of its stamp"


# ---------------------------------------------------------------- T1, A3: lockstep, no timing


def test_T1_swap(tmp_path, request):
    """All three placements, lockstep, recorded targets: the same command on every plan tick, to 1e-9 rad."""
    request.getfixturevalue("private_dora")
    runs = {placement: cells.run_lockstep(placement, tmp_path / placement, LOCKSTEP_S)
            for placement in ("fallback", "same_box", "target")}
    for placement in ("same_box", "target"):
        _assert_same_commands(runs["fallback"]["command"], runs[placement]["command"], f"fallback vs {placement}")


def test_A3_fallback_matches_reference(tmp_path):
    """All Python under A1's load (a 7 x 1000 plan every 0.5 s for 60 s), lockstep: the commands of the ruler's
    hand-stepped schedule (kpi/support/arm_cell_oracle.py) and, once the .so is built, those of same_box (T1)."""
    until = 60.0
    fallback = cells.run_lockstep("fallback", tmp_path / "fallback", until, repeat=True)
    expected = arm_cell_oracle.run(until, repeat=True)
    assert len(fallback["trajectory"]) == len(expected.plans) >= 100, "plans sent"
    for plan in fallback["trajectory"]:
        assert np.asarray(plan["positions"]).size == 7 * 1000, "each plan is 7 joints x 1000 samples"
    stamps, positions = cells.command_rows(fallback["command"])
    expected_stamps = np.array([command.header.stamp for command in expected.commands])
    expected_positions = np.array([command.position for command in expected.commands])
    common = min(len(stamps), len(expected_stamps))
    assert common >= len(expected_stamps) - 1, f"{len(stamps)} commands, the schedule has {len(expected_stamps)}"
    assert np.array_equal(stamps[:common], expected_stamps[:common]), "tick times differ from i / hz"
    worst = float(np.max(np.abs(positions[:common] - expected_positions[:common])))
    assert worst <= TOLERANCE_RAD, f"fallback differs from the reference schedule by {worst} rad"
    if (BUILD / "libarm_controller.so").exists():
        same_box = cells.run_lockstep("same_box", tmp_path / "same_box", until, repeat=True)
        _assert_same_commands(fallback["command"], same_box["command"], "fallback vs same_box under A1's load")


# ---------------------------------------------------------------- A1, A2, T2, T3: the timing tier


def _probe_cell(placement: str, directory, *, repeat: bool, spinner: bool = False):
    """cell.py's wiring plus the hand-off probe (and T3's spinner) in the planner's group."""
    lk = like_keras()
    from kpi.consumer.parts import HandoffProbe, Spinner, TimedPlanner
    from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
    from kpi.tasks.arm_cell.targets import TargetFile

    config = cells.config(directory, lockstep=False)
    if placement == "fallback":
        arm = ArmControllerPy(config=config)
    else:
        arm = lk.native(native_library("arm_controller"), config=config)
    s = lk.System("pick_cell")
    targets = s.add(TargetFile(repeat=repeat), "targets")
    s.add(arm, "arm")
    planner = s.add(TimedPlanner(), "planner", target=targets.target, state=arm.state)
    s.add(HandoffProbe(path=str(directory / "handoff.jsonl")), "probe", trajectory=planner.trajectory)
    s.wire(arm, trajectory=planner.trajectory)
    s.expose(arm.state, arm.command)
    if spinner:
        s.add(Spinner(), "spinner")
    if placement == "target":
        from like_keras.backends import dora

        dep = lk.Deployment(s, backend=dora)
    else:
        dep = lk.Deployment(s)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=2)
    dep.place(s.planner, s.probe)
    if spinner:
        dep.place(s.spinner, hz=20)
    return dep, s


def _gaps(directory) -> np.ndarray:
    rows = [json.loads(line) for line in open(directory / "handoff.jsonl", encoding="utf-8") if line.strip()]
    return np.array([row["gap_s"] for row in rows])


def _run_realtime(dep, seconds: float) -> None:
    dep.start()
    try:
        time.sleep(seconds)
    finally:
        dep.stop()


def _gil_probe():
    """Every OS thread that ran Python while the probe was on (sys.monitoring sees every thread of the interpreter).
    A native thread that never takes the GIL never runs Python, so it never shows up here."""
    seen: set[int] = set()
    tool = sys.monitoring.PROFILER_ID

    def on_start(code, offset):
        seen.add(threading.get_native_id())

    sys.monitoring.use_tool_id(tool, "kpi-gil-probe")
    sys.monitoring.register_callback(tool, sys.monitoring.events.PY_START, on_start)
    sys.monitoring.set_events(tool, sys.monitoring.events.PY_START)

    def stop() -> set[int]:
        sys.monitoring.set_events(tool, 0)
        sys.monitoring.free_tool_id(tool)
        return seen

    return stop


def _python_thread_ids() -> set[int]:
    return {thread.native_id for thread in threading.enumerate() if thread.native_id is not None}


@pytest.mark.timing
def test_A1_handoff_over_dora(tmp_path, request):
    """target: a 7 x 1000 plan every 0.5 s for 60 s while the node ticks at 100 Hz."""
    request.getfixturevalue("private_dora")
    dep, cell = _probe_cell("target", tmp_path, repeat=True)
    dep.record(tmp_path / "a1.mcap", cell.arm.command, cell.planner.trajectory)
    _run_realtime(dep, 60.0)
    gaps = _gaps(tmp_path)
    assert len(gaps) >= 100 and np.percentile(gaps, 99) <= 1e-3, f"hand-off p99 {np.percentile(gaps, 99) * 1e3:.3f} ms"
    _assert_plans_start_on_time(recorded(tmp_path / "a1.mcap", "trajectory"), recorded(tmp_path / "a1.mcap", "command"))
    jitter = _jitter_s(_position_events(FakeBoards(tmp_path / "arm.boards")))
    assert np.percentile(jitter, 99) <= 0.2e-3 and jitter.max() <= 1e-3, "tick-start jitter (T3 limits)"


@pytest.mark.timing
def test_A2_same_planner_in_one_process(tmp_path):
    """same_box: A1's load; the push p99 <= 0.1 ms; the RT thread never runs Python (never takes the GIL)."""
    dep, cell = _probe_cell("same_box", tmp_path, repeat=True)
    dep.record(tmp_path / "a2.mcap", cell.arm.command, cell.planner.trajectory)
    stop_probe = _gil_probe()
    dep.start()
    try:
        time.sleep(60.0)
        python_threads = _python_thread_ids()
    finally:
        dep.stop()
        ran_python = stop_probe()
    native = [int(task) for task in os.listdir("/proc/self/task") if int(task) not in python_threads]
    assert not (ran_python - python_threads - {threading.main_thread().native_id}), "a native thread ran Python"
    assert native, "no native thread found (the C++ chain must run on its own thread)"
    gaps = _gaps(tmp_path)
    assert len(gaps) >= 100 and np.percentile(gaps, 99) <= 0.1e-3, f"push p99 {np.percentile(gaps, 99) * 1e3:.3f} ms"
    _assert_plans_start_on_time(recorded(tmp_path / "a2.mcap", "trajectory"), recorded(tmp_path / "a2.mcap", "command"))
    jitter = _jitter_s(_position_events(FakeBoards(tmp_path / "arm.boards")))
    assert np.percentile(jitter, 99) <= 0.2e-3 and jitter.max() <= 1e-3, "tick-start jitter (T3 limits)"


@pytest.mark.timing
@pytest.mark.parametrize("placement", ["target", "same_box"])
def test_T2_send_cost(placement, tmp_path, request):
    """1000 sends of a 7 x 1000 plan while the node ticks: p99 <= 0.1 ms in-process, <= 1 ms over dora."""
    _dora_if(placement, request)
    targets = tmp_path / "targets.jsonl"
    lines = open(KPI / "tasks" / "arm_cell" / "targets.jsonl", encoding="utf-8").read().splitlines()
    targets.write_text("\n".join(lines[index % len(lines)] for index in range(1000)) + "\n")
    # 1000 targets at 2 Hz would take 500 s; here the targets group runs at 50 Hz (20 s).
    lk = like_keras()
    from kpi.consumer.parts import HandoffProbe, TimedPlanner
    from kpi.tasks.arm_cell.targets import TargetFile

    config = cells.config(tmp_path, lockstep=False)
    s = lk.System("pick_cell")
    source = s.add(TargetFile(path=str(targets)), "targets")
    arm = s.add(lk.native(native_library("arm_controller"), config=config), "arm")
    planner = s.add(TimedPlanner(), "planner", target=source.target, state=arm.state)
    s.add(HandoffProbe(path=str(tmp_path / "handoff.jsonl")), "probe", trajectory=planner.trajectory)
    s.wire(arm, trajectory=planner.trajectory)
    if placement == "target":
        from like_keras.backends import dora

        dep = lk.Deployment(s, backend=dora)
    else:
        dep = lk.Deployment(s)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=kpi_cpus(), fifo=80)
    dep.place(s.targets, hz=50)
    dep.place(s.planner, s.probe)
    _run_realtime(dep, 22.0)
    gaps = _gaps(tmp_path)
    limit = 1e-3 if placement == "target" else 0.1e-3
    assert len(gaps) >= 990, f"{len(gaps)} of 1000 sends measured"
    assert np.percentile(gaps, 99) <= limit, f"send p99 {np.percentile(gaps, 99) * 1e3:.3f} ms > {limit * 1e3} ms"


@pytest.mark.timing
@pytest.mark.parametrize("placement", ["target", "same_box"])
def test_T3_isolation(placement, tmp_path, request):
    """The planner's process spins pure Python with gc.collect() every 50 ms under kpi/load.py --cpu 24, for 10 min
    (KPI_T3_SECONDS): tick-start jitter p99 <= 0.2 ms, max <= 1 ms."""
    _dora_if(placement, request)
    seconds = float(os.environ.get("KPI_T3_SECONDS", "600"))
    dep, _ = _probe_cell(placement, tmp_path, repeat=True, spinner=True)
    load = subprocess.Popen([sys.executable, str(KPI / "load.py"), "--cpu", "24", "--seconds", str(seconds + 10)],
                            env={**os.environ, "KPI_TIER": "timing"})
    try:
        _run_realtime(dep, seconds)
    finally:
        load.terminate()
        load.wait(timeout=10)
    jitter = _jitter_s(_position_events(FakeBoards(tmp_path / "arm.boards")))
    assert len(jitter) >= 0.95 * seconds * 100, "ticks recorded"
    assert np.percentile(jitter, 99) <= 0.2e-3, f"jitter p99 {np.percentile(jitter, 99) * 1e3:.3f} ms"
    assert jitter.max() <= 1e-3, f"jitter max {jitter.max() * 1e3:.3f} ms"


# ---------------------------------------------------------------- T4-T8, T12: real time, process kills


def _node_pid(process: str) -> int | None:
    found = [pid for pid in procs.pids_with(process)
             if any(host in open(f"/proc/{pid}/cmdline", "rb").read().decode(errors="replace")
                    for host in ("lk-dora-host", "like_keras.backends.dora.host"))]
    return found[0] if found else None


def _top(address: str, seconds: float) -> str:
    """`lk top ADDRESS` for `seconds`, its text output (it prints plain text when stdout is not a terminal)."""
    process = subprocess.Popen(lk_command("top", address), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(seconds)
    process.send_signal(signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()
    return output


@pytest.mark.timing
def test_T4_boundary_stall(tmp_path, private_dora):
    """kill -STOP the dora daemon for 4 s mid-trajectory: the trajectory finishes on time, then holds; 0 missed
    ticks; InputStale visible on lk top during the stall (attach does not go through the daemon)."""
    dep = cells.deployment("target", tmp_path, lockstep=False, repeat=True)
    address = dep.describe()["attach"]["socket"]
    dep.start()
    try:
        boards = FakeBoards(tmp_path / "arm.boards")
        procs.wait_for(lambda: len(_position_events(boards)) > 150, 10.0, "the node ticking")
        stopped_at = time.time()
        os.kill(private_dora.daemon_pid, signal.SIGSTOP)
        try:
            top = _top(address, 3.0)
            time.sleep(max(0.0, 4.0 - (time.time() - stopped_at)))
        finally:
            os.kill(private_dora.daemon_pid, signal.SIGCONT)
        time.sleep(1.0)
    finally:
        dep.stop()
    events = _position_events(boards)
    stall = events[(events["wall"] >= stopped_at) & (events["wall"] <= stopped_at + 4.0)]
    assert len(stall) >= 395, f"{len(stall)} ticks in the 4 s stall"
    assert np.allclose(np.diff(stall["t"]), TICK_S, atol=1e-6), "a tick was missed during the stall"
    settled = stall[np.argmax(stall["q0"] == stall["q0"][-1])]
    assert settled["wall"] <= stopped_at + 0.4 + 0.05, "the trajectory in flight did not finish on time"
    assert np.all(stall["q0"][stall["wall"] >= settled["wall"]] == stall["q0"][-1]), "the arm did not hold"
    assert "InputStale" in top, "lk top did not show InputStale during the stall"


def _wait_commands(log, count: int, timeout_s: float) -> list[dict]:
    from kpi.support.parts import read_log

    return procs.wait_for(lambda: (rows := read_log(log)) and len(rows) >= count and rows, timeout_s, f"{count} commands")


def _restarted_commands_within(log, since_wall: float, timeout_s: float) -> float:
    """Seconds from `since_wall` until the commands move again (a new plan runs)."""
    from kpi.support.parts import read_log

    def moved():
        rows = [row for row in read_log(log) if row["wall"] >= since_wall]
        for previous, row in zip(rows, rows[1:]):
            if not np.allclose(previous["position"], row["position"], atol=1e-9):
                return row["wall"]
        return None

    return procs.wait_for(moved, timeout_s, "commands after the restart") - since_wall


def _assert_holds(log, start_wall: float, end_wall: float) -> None:
    from kpi.support.parts import read_log

    rows = [row for row in read_log(log) if start_wall <= row["wall"] <= end_wall]
    assert len(rows) >= 10, "no commands while the planner was down"
    assert all(np.allclose(rows[0]["position"], row["position"], atol=1e-12) for row in rows), "the arm did not hold"


@pytest.mark.parametrize("placement", ["target", "attached"])
def test_T5_planner_crash(placement, tmp_path, request):
    """kill -9 the planner, restart it: the arm holds; the restarted planner commands again within 1 s."""
    like_keras()
    from kpi.support.parts import observe

    native_library("arm_controller")
    log = tmp_path / "commands.jsonl"
    if placement == "target":
        request.getfixturevalue("private_dora")
        dep = cells.deployment("target", tmp_path, lockstep=False, repeat=True)
        address = dep.describe()["attach"]["socket"]
        dep.start()
        controller = planner = None
    else:
        dep = None
        controller = procs.Child("kpi.tasks.arm_cell.always_on", "--name", f"arm_t5_{os.getpid()}", "--language", "cpp",
                                 "--config", json.dumps(cells.config(tmp_path, lockstep=False)),
                                 "--cpus", str(kpi_cpus()[0]))
        address = controller.wait_ready()["attach"]["socket"]
        planner = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", address)
        planner.wait_ready()
    observer = observe(address, log)
    try:
        _wait_commands(log, 200, 15.0)
        if placement == "target":
            victim = procs.wait_for(lambda: _node_pid("main"), 5.0, "the planner's node")
            os.kill(victim, signal.SIGKILL)
            killed = time.time()
            procs.wait_for(lambda: (pid := _node_pid("main")) and pid != victim and pid, 10.0, "dora restarting the planner")
            restarted = time.time()
        else:
            planner.signal(signal.SIGKILL)
            killed = time.time()
            time.sleep(2.0)
            planner = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", address)
            planner.wait_ready()
            restarted = time.time()
        if restarted - killed > 0.7:  # the plan in flight (<= 0.4 s) has ended: the arm must be holding
            _assert_holds(log, killed + 0.5, restarted - 0.1)
        assert _restarted_commands_within(log, restarted, 5.0) <= 1.0, "the restarted planner did not command within 1 s"
    finally:
        observer.stop()
        for child in (planner, controller):
            if child is not None:
                child.stop()
        if dep is not None:
            dep.stop()


@pytest.mark.parametrize("placement", ["target"])
def test_T6_node_crash(placement, tmp_path, request):
    """kill -9 the node: the fake boards are in timeout mode within 0.25 s; on restart take_over, no mode 0, no bare stop."""
    request.getfixturevalue("private_dora")
    dep = cells.deployment(placement, tmp_path, lockstep=False, repeat=True)
    dep.start()
    try:
        boards = FakeBoards(tmp_path / "arm.boards")
        procs.wait_for(lambda: len(_position_events(boards)) > 100, 10.0, "the node ticking")
        victim = procs.wait_for(lambda: _node_pid("arm-controller"), 5.0, "the controller node")
        os.kill(victim, signal.SIGKILL)
        killed = time.time()
        last = _position_events(boards, victim)["wall"].max()
        assert last <= killed + 0.01, "the board was fed after the kill"
        assert all(mode == MODE_TIMEOUT for mode in boards.modes(at=last + 0.25 + 1e-3)), "boards not in timeout in 0.25 s"
        successor = procs.wait_for(lambda: (pid := _node_pid("arm-controller")) and pid != victim and pid, 15.0,
                                   "dora restarting the controller")
        procs.wait_for(lambda: len(_position_events(boards, successor)) > 50, 10.0, "the restarted node ticking")
    finally:
        dep.stop()
    events = boards.events()
    after = events[events["pid"] == successor]
    kinds = list(after["kind"][:3])
    assert kinds[0] == EVENT_QUERY, "the restart did not begin with take_over's query"
    for index, kind in enumerate(after["kind"]):
        if kind == EVENT_STOP:
            assert index + 1 < len(after) and after["kind"][index + 1] == EVENT_POSITION, "a bare stop"
    assert np.all(after["mode_after"][after["kind"] == EVENT_POSITION] == MODE_POSITION), "a board rested in mode 0"


@pytest.mark.parametrize("placement", ["target", "same_box"])
def test_T7_exclusivity(placement, tmp_path, request):
    """Start a second node on the same bus: refused in activate before the port opens; exits non-zero having sent 0 frames."""
    _dora_if(placement, request)
    dep = cells.deployment(placement, tmp_path, lockstep=False)
    dep.start()
    try:
        boards = FakeBoards(tmp_path / "arm.boards")
        procs.wait_for(lambda: len(_position_events(boards)) > 20, 10.0, "the first node ticking")
        first = set(boards.events()["pid"])
        second = procs.Child("kpi.tasks.arm_cell.always_on", "--name", f"arm_t7_{os.getpid()}", "--language", "cpp",
                             "--config", json.dumps(cells.config(tmp_path, lockstep=False)), "--cpus",
                             str(kpi_cpus()[0]), "--backend", "dora" if placement == "target" else "inproc")
        if placement == "same_box":
            assert second.process.wait(timeout=20) != 0, "the second node did not exit non-zero"
        else:
            time.sleep(5.0)  # dora restarts the refused node; each attempt must refuse again
            second.stop()
        intruders = set(boards.events()["pid"]) - first
        assert not intruders, f"the second node sent frames (pids {sorted(intruders)})"
    finally:
        dep.stop()


def test_T8_claim(tmp_path):
    """Two attached planners write at once: only the lease holder's trajectories run; the other gets REFUSED (its
    proxy's health error, code LeaseHeld); when the holder stops, the inputs retract and the arm holds."""
    like_keras()
    from kpi.support.parts import observe

    native_library("arm_controller")
    negated = tmp_path / "negated.jsonl"
    negated.write_text("".join(json.dumps({"position": [-x for x in json.loads(line)["position"]]}) + "\n"
                               for line in open(KPI / "tasks" / "arm_cell" / "targets.jsonl", encoding="utf-8")
                               if line.strip()))
    controller = procs.Child("kpi.tasks.arm_cell.always_on", "--name", f"arm_t8_{os.getpid()}", "--language", "cpp",
                             "--config", json.dumps(cells.config(tmp_path, lockstep=False)), "--cpus", str(kpi_cpus()[0]))
    address = controller.wait_ready()["attach"]["socket"]
    log = tmp_path / "commands.jsonl"
    holder = second = observer = None
    try:
        observer = observe(address, log)
        holder = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", address, "--name", "holder",
                             "--record", str(tmp_path / "holder.mcap"))
        holder.wait_ready()
        _wait_commands(log, 100, 15.0)
        second = procs.Child("kpi.tasks.arm_cell.attached_planner", "--address", address, "--targets", str(negated),
                             "--name", "second", "--record", str(tmp_path / "second.mcap"))
        second.wait_ready()
        overlap_from = time.time()
        time.sleep(3.0)
        second.stop()
        holder_stopped = time.time()
        holder.stop()
        time.sleep(2.0)
    finally:
        for child in (second, holder):
            if child is not None:
                child.stop()
        if observer is not None:
            observer.stop()
        controller.stop()
    from kpi.support.parts import read_log

    targets = [json.loads(line)["position"] for line in open(KPI / "tasks" / "arm_cell" / "targets.jsonl") if line.strip()]
    finals = [row["position"] for row in read_log(log) if overlap_from <= row["wall"] <= holder_stopped]
    reached = {tuple(np.round(position, 6)) for position in finals}
    assert not any(tuple(np.round([-x for x in target], 6)) in reached for target in targets), "the second planner's plan ran"
    health = recorded(tmp_path / "second.mcap", "health")
    assert any(record["level"] == "error" and record["code"] == "LeaseHeld" for record in health), "no LeaseHeld refusal"
    _assert_holds(log, holder_stopped + 0.25 + 0.5, holder_stopped + 2.0)


@pytest.mark.parametrize("placement", ["target", "same_box"])
def test_T12_introspection(placement, tmp_path, request):
    """`lk top <address>` shows every R14 metric: tick p50/p99/max, overruns, exchange p95, bus errors, watchdog
    lapses, drops, and every component's health."""
    _dora_if(placement, request)
    dep = cells.deployment(placement, tmp_path, lockstep=False, repeat=True)
    address = dep.describe()["attach"]["socket"]
    dep.start()
    try:
        time.sleep(3.0)
        output = _top(address, 3.0)
    finally:
        dep.stop()
    for name in ("step_p50_s", "step_p99_s", "step_max_s", "overruns", "exchange_p95_s", "bus_errors",
                 "watchdog_lapses", "drops.", "period_error_p99_s"):
        assert name in output, f"lk top does not show {name}"
    for component in ("arm", "planner", "targets"):
        assert component in output, f"lk top does not show {component}"


# ---------------------------------------------------------------- T9: the C++ chain's overhead


def test_T9_allocations():
    """0 heap allocations per tick over 100k ticks of the C++ chain, no lock taken on the RT thread (alloc_probe)."""
    probe = binary("alloc_probe")
    result = subprocess.run([str(probe), "--gtest_brief=1"], capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stdout[-2000:] + result.stderr[-2000:]


@pytest.mark.timing
def test_T9_per_edge():
    """<= 100 ns per edge of the 4-component C++ chain over hand-written calls (bench_chain)."""
    from kpi.bench_overhead import cxx_per_edge_ns

    per_edge = cxx_per_edge_ns()
    assert per_edge is not None and per_edge <= 100.0, f"{per_edge} ns per edge"
