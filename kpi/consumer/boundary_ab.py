"""A35's A/B (CONTRACT §15 A35; consumer-018): like_keras's `target` against a raw-dora reference written in kpi, on
the same twin scenario, with one oracle for both sides. Used by kpi/consumer/test_consumer_friction.py (timing tier).

The scenario, the same on both sides (the consumer's ab_compare.py, rebuilt on kpi's own reference cell):
- the reference arm (kpi/tasks/arm_cell: bus -> jtc -> guard on the fake bus) at 100 Hz: like_keras's
  libarm_controller.so in lk-dora-host; raw dora's kpi/consumer/raw_arm_node.py `arm`, the same laws in Python;
- every 0.5 s the next row of targets.jsonl becomes a 7 x 1000 minimum-jerk plan (plan_pick) from the newest joint
  state, stamped now (lead 0), so it runs from the first tick after it arrives;
- a subscriber in a process of its own keeps every joint state it receives (seq, arrival).

What is compared (per run, then paired):
    states lost   1 - received / the seq span, at the subscriber (dora drops nothing for it: queue 1000)
    hand-off      the planner's return (raw: the send call) -> the start of the tick that first ran the plan, read
                  off the fake bus's own event log (the tick whose command is the plan's start; found as the tick two
                  exchanges before the one that carries the plan's second sample, since bus.command is lagged)
    within noise  the upper edge of a bootstrap 95 % CI of mean(like_keras - raw) over the pairs is at most the
                  raw side's own spread over the pairs (max - min), or a floor: 0.1 % of states (the raw node's own
                  0-0.1 %, consumer-018), 1 ms of hand-off p99 (A1's bound)
Numbers are written to build/kpi/a35_ab.json, labelled with the tier and what rtkit offered.
"""

from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import time

import numpy as np

TICK_S = 0.01  # the arm's 100 Hz
LOST_FLOOR = 0.001  # 0.1 % of states: the raw node's own loss (consumer-018: 0-0.1 %)
HANDOFF_FLOOR_S = 1e-3  # A1: hand-off p99 <= 1 ms
MATCH_RAD = 1e-9  # a command equals the plan's sample (both sides run the same law in double precision)
MEASURABLE_RAD = 1e-7  # a plan whose second sample is this close to its start cannot be told from a hold


def plan_row(plan, returned_wall: float) -> dict:
    """What the oracle needs of a plan, cheap to note on the planner's thread: when it left (epoch s), its stamp, and
    joint 1's first and last position (the rest of joint 1's column is plan_pick's minimum-jerk curve between them)."""
    return {"returned_wall": returned_wall, "stamp": float(plan.header.stamp),
            "q0_start": float(plan.positions[0, 0]), "q0_end": float(plan.positions[-1, 0])}


def _joint1(row: dict):
    """(t, joint 1's positions) of the plan `row` notes, as plan_pick computes them."""
    from kpi.tasks.arm_cell.plan import DURATION_S, SAMPLES

    t = np.linspace(0.0, DURATION_S, SAMPLES)
    u = t / DURATION_S
    s = u**3 * (10.0 - 15.0 * u + 6.0 * u**2)
    return t, row["q0_start"] + s * (row["q0_end"] - row["q0_start"])


def read_rows(path) -> list[dict]:
    try:
        with open(path, encoding="utf-8") as file:
            return [json.loads(line) for line in file if line.strip()]
    except FileNotFoundError:
        return []


def states_lost(path) -> dict:
    rows = read_rows(path)
    if not rows:
        return {"received": 0, "lost_fraction": 1.0}
    seqs = np.unique([row["seq"] for row in rows])
    span = int(seqs[-1] - seqs[0] + 1)
    arrivals = np.sort([row["wall"] for row in rows])
    return {"received": len(rows), "span": span, "lost": span - len(seqs), "lost_fraction": (span - len(seqs)) / span,
            "worst_arrival_gap_s": float(np.diff(arrivals).max()) if len(arrivals) > 1 else None}


def handoffs(boards_path, plans_path) -> dict:
    """Each plan's hand-off (s), from the fake bus's event log: see the module docstring."""
    from kpi.tasks.arm_cell.fake_bus import EVENT_POSITION, FakeBoards

    boards = FakeBoards(boards_path)
    events = boards.events()
    boards.close()
    events = events[(events["kind"] == EVENT_POSITION) & (events["t"] > 0.0)]
    plans = sorted(read_rows(plans_path), key=lambda row: row["returned_wall"])
    found, unmeasurable, missing = [], 0, 0
    for index, row in enumerate(plans):
        t, column = _joint1(row)
        due_by = plans[index + 1]["returned_wall"] + 1.0 if index + 1 < len(plans) else np.inf
        first = int(np.searchsorted(events["wall"], row["returned_wall"]))
        adopted = None
        for at in range(first, len(events) - 2):
            if events["wall"][at] > due_by:
                break
            tau = events["t"][at + 1] - events["t"][at]
            expected = float(np.interp(tau, t, column))
            if abs(expected - row["q0_start"]) <= MEASURABLE_RAD:
                continue
            starts_here = abs(events["q0"][at + 1] - row["q0_start"]) <= MATCH_RAD  # plan time 0: its start
            if starts_here and abs(events["q0"][at + 2] - expected) <= MATCH_RAD:
                adopted = at
                break
        if adopted is None:
            if abs(row["q0_end"] - row["q0_start"]) <= MEASURABLE_RAD:
                unmeasurable += 1
            else:
                missing += 1
            continue
        found.append(float(events["wall"][adopted] - row["returned_wall"]))
    values = np.array(found)
    return {"plans": len(plans), "measured": len(found), "unmeasurable": unmeasurable, "not_adopted": missing,
            "p50_s": float(np.percentile(values, 50)) if len(values) else None,
            "p99_s": float(np.percentile(values, 99)) if len(values) else None,
            "max_s": float(values.max()) if len(values) else None}


POLICY_NAMES = {0: "OTHER", 1: "FIFO", 2: "RR", 3: "BATCH", 5: "IDLE"}


def thread_policies(pid: int) -> dict[str, int]:
    """'<policy> <rt priority>' -> how many threads of process `pid` run under it (/proc/<pid>/task/*/stat)."""
    counts: dict[str, int] = {}
    for task in pathlib.Path(f"/proc/{pid}/task").glob("*"):
        try:
            fields = (task / "stat").read_text().rsplit(")", 1)[1].split()
        except (OSError, IndexError):
            continue
        key = f"{POLICY_NAMES.get(int(fields[38]), fields[38])} {fields[37]}"
        counts[key] = counts.get(key, 0) + 1
    return counts


def _wait_rows(path, count: int, timeout_s: float, what: str) -> None:
    deadline = time.monotonic() + timeout_s
    while len(read_rows(path)) < count:
        if time.monotonic() > deadline:
            raise TimeoutError(f"{what}: fewer than {count} rows in {path} after {timeout_s} s")
        time.sleep(0.1)


def _policies_of(*needles: str) -> dict[str, int]:
    from kpi.support import procs

    pids = procs.pids_with(*needles)
    return thread_policies(pids[0]) if pids else {}


def run_raw(dora, directory: pathlib.Path, seconds: float, config: dict, name: str) -> dict:
    """The raw-dora side: kpi/consumer/raw_arm_node.py's three nodes, a private dataflow, `seconds` of running."""
    from kpi.support.lk import REPO, subprocess_env
    from kpi.tasks.arm_cell.plan import TARGETS

    directory.mkdir(parents=True, exist_ok=True)
    states, plans = directory / "states.jsonl", directory / "plans.jsonl"
    env = {"PYTHONPATH": os.pathsep.join([str(REPO / "src"), str(REPO)]), "AB_CONFIG": json.dumps(config),
           "AB_TARGETS": str(TARGETS), "AB_PLANS": str(plans), "AB_STATES": str(states),
           "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}

    def node(role: str, inputs: dict, outputs: list[str]) -> dict:
        return {"id": f"raw_{role}", "path": "/usr/bin/env", "env": env, "inputs": inputs, "outputs": outputs,
                "args": f"{sys.executable} -m kpi.consumer.raw_arm_node {role}"}

    dataflow = {"nodes": [
        node("arm", {"tick": "dora/timer/millis/10",
                     "trajectory": {"source": "raw_planner/trajectory", "queue_size": 1}}, ["state"]),
        node("planner", {"tick": "dora/timer/millis/500", "state": {"source": "raw_arm/state", "queue_size": 1}},
             ["trajectory"]),
        node("subscriber", {"state": {"source": "raw_arm/state", "queue_size": 1000}}, []),
    ]}
    path = directory / "dataflow.yml"
    path.write_text(json.dumps(dataflow, indent=1))  # YAML is a superset of JSON
    started = subprocess.run([dora.cli, "start", str(path), "--name", name, "--detach"], cwd=directory,
                             env=subprocess_env(), capture_output=True, text=True, timeout=120)
    assert started.returncode == 0, f"raw dora start: {started.stdout[-1000:]}{started.stderr[-1000:]}"
    try:
        _wait_rows(states, 1, 60.0, "the raw arm's first state")
        time.sleep(min(3.0, seconds))
        policies = _policies_of("kpi.consumer.raw_arm_node", "arm")
        time.sleep(max(0.0, seconds - 3.0))
    finally:
        subprocess.run([dora.cli, "stop", "--name", name, "--grace-duration", "5s"], cwd=directory,
                       env=subprocess_env(), capture_output=True, timeout=60)
    return {"states": states_lost(states), "handoff": handoffs(config["bus"]["board_file"], plans),
            "arm_threads": policies, "load": os.getloadavg()}


def run_lk(directory: pathlib.Path, seconds: float, config: dict, name: str, cpus: list[int]) -> dict:
    """like_keras's `target`: the reference arm in lk-dora-host, WallPlanner in `main`, SeqLog in its own process."""
    import like_keras as lk
    from like_keras.backends import dora

    from kpi.consumer.boundary_parts import SeqLog, WallPlanner
    from kpi.support.cells import cell_module
    from kpi.tasks.arm_cell.targets import TargetFile

    directory.mkdir(parents=True, exist_ok=True)
    states, plans = directory / "states.jsonl", directory / "plans.jsonl"
    s = lk.System(name)
    targets = s.add(TargetFile(repeat=True), "targets")
    arm = s.add(lk.native(cell_module().LIBRARY, config=config), "arm")
    planner = s.add(WallPlanner(path=str(plans)), "planner", target=targets.target, state=arm.state)
    s.wire(arm, trajectory=planner.trajectory)
    s.add(SeqLog(path=str(states)), "subscriber", value=arm.state)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.arm, process="arm-controller", hz=100, cpus=cpus, fifo=80)
    dep.place(s.targets, hz=2)
    dep.place(s.planner)
    dep.place(s.subscriber, process="subscriber")
    dep.start()
    try:
        _wait_rows(states, 1, 60.0, "the like_keras arm's first state")
        time.sleep(min(3.0, seconds))
        policies = _policies_of("lk-dora-host", name)
        time.sleep(max(0.0, seconds - 3.0))
    finally:
        dep.stop()
    return {"states": states_lost(states), "handoff": handoffs(config["bus"]["board_file"], plans),
            "arm_threads": policies, "load": os.getloadavg()}


def within_noise(lk_values: list[float], raw_values: list[float], floor: float) -> dict:
    """Paired: the upper edge of a bootstrap 95 % CI of mean(like_keras - raw) against max(floor, raw's spread)."""
    differences = np.asarray(lk_values, dtype=float) - np.asarray(raw_values, dtype=float)
    generator = np.random.default_rng(0)  # WHY a fixed seed: the same runs always give the same verdict
    means = generator.choice(differences, size=(2000, len(differences)), replace=True).mean(axis=1)
    upper = float(np.percentile(means, 97.5))
    band = max(floor, float(np.max(raw_values) - np.min(raw_values)))
    return {"like_keras": list(map(float, lk_values)), "raw": list(map(float, raw_values)),
            "upper_95_of_mean_difference": upper, "band": band, "within": upper <= band}
