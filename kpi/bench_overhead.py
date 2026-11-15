"""Criterion 2, speed: what like_keras adds on top of the transport. JSON on stdout.

    nice -n 19 python kpi/bench_overhead.py --pin 20,21            # the timing tier: KPI_TIER=timing, rack unpowered
    nice -n 19 python kpi/bench_overhead.py --smoke                 # anywhere: proves the harness, scores nothing

Four numbers (CONTRACT §11 criterion 2):
1 cross-process: p99 latency of sender -> forwarder -> receiver through like_keras (dora backend) minus the same
  three nodes on raw dora 0.5.0 (kpi/bench/raw_dora_node.py), in the same run, interleaved A/B pairs, payloads
  64 B / 4 KiB / 1 MiB / 7 x 1000, open loop at 1 kHz, timed from the scheduled send. Python senders. (The C++
  sender pair needs the dora cxx API built by the dora lane: reported null until then.)
2 per edge: the 4-component C++ chain (build/kpi/bench_chain, Google Benchmark) and Python chain (inproc lockstep)
  minus the same calls by hand, divided by the 3 edges.
3 copies and CPU: receive-side copies (0 when the received array is a view on the transport's buffer) and the
  forwarder's CPU % against raw dora's.
4 import: cold `import like_keras`, median of 10 runs.
Everything not measured is null, and a null scores 0. Outside the timing tier every number is labelled
"rack powered, cores 22-23, not the timing tier" and kpi/score.py does not score it.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import platform
import statistics
import subprocess
import sys
import tempfile
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "src"), str(REPO)]

PAYLOADS = {"64B": (64, 1), "4KiB": (4096, 1), "1MiB": (1 << 20, 1), "7x1000": (8 * 8 * 1000, 7)}
NOT_TIMING = "rack powered, cores 22-23, not the timing tier"


def importable() -> bool:
    probe = subprocess.run([sys.executable, "-c", "import like_keras"], env=dict(os.environ, PYTHONPATH=str(REPO / "src")),
                           capture_output=True)
    return probe.returncode == 0


def import_ms(runs: int = 10) -> float | None:
    times = []
    for _ in range(runs):
        result = subprocess.run([sys.executable, "-X", "importtime", "-c", "import like_keras"], cwd="/",
                                env=dict(os.environ, PYTHONPATH=str(REPO / "src")), capture_output=True, text=True)
        if result.returncode != 0:
            return None
        lines = [line for line in result.stderr.splitlines() if line.rstrip().endswith("| like_keras")]
        if not lines:
            return None
        times.append(int(lines[-1].split("|")[1]) / 1000.0)
    return statistics.median(times)


def python_per_edge_ns(steps: int = 20000) -> float | None:
    if not importable():
        return None
    script = f"""
import time, json
import like_keras as lk
from kpi.bench.lk_nodes import First, Add, Last
first, add, last = First(), Add(), Last()
hand = []
for _ in range(5):
    started = time.perf_counter()
    for _ in range({steps}):
        last.forward(add.forward(add.forward(first.forward())))
    hand.append(time.perf_counter() - started)
chain = []
for _ in range(5):
    s = lk.System("edges")
    a = s.add(First(), "a"); b = s.add(Add(), "b", value=a.out); c = s.add(Add(), "c", value=b.out); s.add(Last(), "d", value=c.out)
    dep = lk.Deployment(s, realtime=False)
    dep.place(s.a, s.b, s.c, s.d, hz=1000)
    started = time.perf_counter()
    dep.start(until={steps} / 1000)
    chain.append(time.perf_counter() - started)
    dep.stop()
print(json.dumps((min(chain) - min(hand)) / {steps} / 3 * 1e9))
"""
    result = subprocess.run([sys.executable, "-c", script], cwd=REPO, capture_output=True, text=True,
                            env=dict(os.environ, PYTHONPATH=f"{REPO / 'src'}:{REPO}"), timeout=600)
    return json.loads(result.stdout) if result.returncode == 0 else None


def cxx_per_edge_ns() -> float | None:
    binary = REPO / "build" / "kpi" / "bench_chain"
    if not binary.exists():
        return None
    result = subprocess.run([str(binary), "--benchmark_format=json", "--benchmark_repetitions=5",
                             "--benchmark_report_aggregates_only=true"], capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        return None
    rows = {row["name"]: row["cpu_time"] for row in json.loads(result.stdout)["benchmarks"]
            if row.get("aggregate_name") == "median"}
    hand = next((value for name, value in rows.items() if name.startswith("BM_Hand")), None)
    chain = next((value for name, value in rows.items() if name.startswith("BM_LkChain")), None)
    return None if hand is None or chain is None else (chain - hand) / 3.0


def steady_p99(latencies) -> float:
    """p99 after the first 10 % of samples: dora starts nodes in no order, so the first sends can queue behind a
    receiver that is not up yet (the same cut for both sides of the A/B)."""
    import numpy as np

    return float(np.percentile(latencies[len(latencies) // 10 :], 99))


def cpu_seconds(pid: int) -> float:
    fields = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def raw_run(work: pathlib.Path, dora, size: int, samples: int, pin: list[int]) -> dict:
    out = work / "raw.npy"
    node = REPO / "kpi" / "bench" / "raw_dora_node.py"

    def command(role: str, core: int | None) -> tuple[str, str]:
        # WHY /usr/bin/env: dora resolves a node's `path` through symlinks, and the venv's python is a symlink to
        # the system one; run directly it would lose the venv (no pyarrow). env execs the venv path as given.
        pinned = f"taskset -c {core} " if core is not None else ""
        return "/usr/bin/env", f"{pinned}{sys.executable} {node} {role}"

    cores = (pin + [None, None])[:2]
    sender, forwarder, receiver = command(f"sender {size} {samples}", cores[0]), command("forwarder", cores[1]), \
        command(f"receiver {out}", None)
    flow = work / "raw.yml"
    flow.write_text(f"""nodes:
  - id: sender
    path: {sender[0]}
    args: {sender[1]}
    outputs: [data, done]
  - id: forwarder
    path: {forwarder[0]}
    args: {forwarder[1]}
    inputs:
      data: {{source: sender/data, queue_size: 1000}}
      done: sender/done
    outputs: [data, done]
  - id: receiver
    path: {receiver[0]}
    args: {receiver[1]}
    inputs:
      data: {{source: forwarder/data, queue_size: 1000}}
      done: forwarder/done
""")
    name = f"raw_{size}_{time.monotonic_ns()}"
    started = subprocess.run([dora.cli, "start", str(flow), "--name", name, "--detach"], cwd=work, capture_output=True,
                             text=True, timeout=30)
    if started.returncode != 0:
        return {"error": started.stderr[-500:]}
    from kpi.support.procs import pids_with

    deadline = time.monotonic() + samples / 1000.0 + 30
    forwarder = None
    cpu_start = None
    while time.monotonic() < deadline and not out.exists():
        if forwarder is None:
            found = pids_with(str(node), "forwarder")
            if found:
                forwarder, cpu_start, wall_start = found[0], cpu_seconds(found[0]), time.monotonic()
        time.sleep(0.2)
    cpu = None
    if forwarder is not None:
        try:
            cpu = (cpu_seconds(forwarder) - cpu_start) / (time.monotonic() - wall_start) * 100.0
        except OSError:
            cpu = None
    subprocess.run([dora.cli, "stop", "--name", name], cwd=work, capture_output=True, timeout=30)
    if not out.exists():
        return {"error": "the raw receiver wrote nothing"}
    import numpy as np

    latencies = np.load(out)
    return {"p99_s": steady_p99(latencies), "received": int(len(latencies)), "sent": samples,
            "copies": int((work / "raw.npy.copies").read_text()), "forwarder_cpu_pct": cpu}


def lk_run(work: pathlib.Path, size: int, joints: int, samples: int, pin: list[int]) -> dict:
    if not importable():
        return {"error": "like_keras is not importable"}
    script = f"""
import json, time
import like_keras as lk
from like_keras.backends import dora
from kpi.bench.lk_nodes import Forwarder, Receiver, Sender
s = lk.System("bench_{size}_{time.monotonic_ns()}")
sender = s.add(Sender(size={size}, joints={joints}, samples={samples}), "sender")
forwarder = s.add(Forwarder(), "forwarder", data=sender.data)
s.add(Receiver(path={str(work / 'lk.npy')!r}), "receiver", data=forwarder.out)
dep = lk.Deployment(s, backend=dora)
dep.place(s.sender, process="sender", hz=1000, cpus={pin[:1] or None})
dep.place(s.forwarder, process="forwarder", cpus={pin[1:2] or None})
dep.place(s.receiver, process="receiver")
dep.start(until={samples} / 1000 + 3.0)
dep.stop()
"""
    result = subprocess.run([sys.executable, "-c", script], cwd=REPO, capture_output=True, text=True,
                            env=dict(os.environ, PYTHONPATH=f"{REPO / 'src'}:{REPO}"), timeout=samples / 1000 + 120)
    out = work / "lk.npy"
    if result.returncode != 0 or not out.exists():
        return {"error": (result.stderr or "no output")[-500:]}
    import numpy as np

    latencies = np.load(out)
    return {"p99_s": steady_p99(latencies), "received": int(len(latencies)), "sent": samples,
            "copies": int((work / "lk.npy.copies").read_text()), "forwarder_cpu_pct": None}


def cross_process(pairs: int, samples: int, payloads: list[str], pin: list[int]) -> dict:
    from kpi.support.dora_private import PrivateDora

    base = REPO / "build" / "kpi" / "tmp"
    base.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="bench-", dir=base) as scratch:
        try:
            dora = PrivateDora(REPO / "build" / "kpi" / f"dora-{os.getpid()}").start()
        except BaseException as refused:  # pytest.skip in the shared namespace
            return {"error": f"no private dora: {refused}"}
        try:
            out = {}
            for name in payloads:
                size, joints = PAYLOADS[name]
                rows = []
                for pair in range(pairs):
                    work = pathlib.Path(scratch) / f"{name}_{pair}"
                    work.mkdir()
                    order = ("raw", "lk") if pair % 2 == 0 else ("lk", "raw")  # interleaved A/B
                    result = {}
                    for side in order:
                        side_dir = work / side
                        side_dir.mkdir()
                        result[side] = (raw_run(side_dir, dora, size, samples, pin) if side == "raw"
                                        else lk_run(side_dir, size, joints, samples, pin))
                    rows.append(result)
                out[name] = summarise(rows)
            return out
        finally:
            dora.stop()


def summarise(rows: list[dict]) -> dict:
    raw = [row["raw"].get("p99_s") for row in rows]
    lk = [row["lk"].get("p99_s") for row in rows]
    summary = {"pairs": rows}
    if None in raw or None in lk:
        summary["added_p99_s"] = None
        return summary
    added = [b - a for a, b in zip(raw, lk)]
    summary.update({"raw_p99_s": statistics.median(raw), "lk_p99_s": statistics.median(lk),
                    "added_p99_s": max(added),  # the conservative edge until the bootstrap CI has >= 5 pairs
                    "sent_equals_received": all(row["lk"]["received"] == row["lk"]["sent"] for row in rows)})
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pin", default="", help="cores for the sender and the forwarder, e.g. 20,21 (never 22)")
    parser.add_argument("--pairs", type=int, default=5)
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--smoke", action="store_true", help="one pair, 64 B, 2000 samples: the harness, not a score")
    parser.add_argument("--no-cross-process", action="store_true")
    arguments = parser.parse_args(argv)
    pin = [int(core) for core in arguments.pin.split(",") if core]
    if 22 in pin:
        raise SystemExit("refused: core 22 is RESERVED_CPUS (the shared dora daemon stalls there)")
    timing = os.environ.get("KPI_TIER") == "timing" and not arguments.smoke
    report: dict = {"host": platform.node(), "tier": "timing" if timing else NOT_TIMING,
                    "import_ms": import_ms(), "python_per_edge_ns": python_per_edge_ns(),
                    "cxx_per_edge_ns": cxx_per_edge_ns(), "cxx_sender_pairs": None}
    if arguments.no_cross_process:
        report["cross_process"] = None
    elif timing:
        report["cross_process"] = cross_process(arguments.pairs, arguments.samples, list(PAYLOADS), pin)
    elif arguments.smoke:
        report["cross_process"] = cross_process(1, 2000, ["64B"], pin)
    else:
        report["cross_process"] = None
        report["note"] = "cross-process A/B runs only with KPI_TIER=timing (or --smoke for the harness)"
    print(json.dumps(report, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
