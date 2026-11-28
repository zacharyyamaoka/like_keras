"""Measure read(), write() and round-trip latency of the pendulum's HardwareInterface on each backend; write one JSON.

    bash /home/bam/like_keras/examples/hw_backends/run.sh measure --cases mujoco_nominal,mujoco_twin,analytical,kinematic
    bash /home/bam/like_keras/examples/hw_backends/run.sh measure --cases real --ticks 2000 --trials 100

What is timed, per case, all through the HardwareInterface a script uses (so its JointTable mapping is in every number):
- ``read`` and ``write``: each call on its own, over `--ticks` ticks of a write-then-read loop that holds the current position
  (on real: a hold at the opening position, so the board never moves).
- ``round_trip``: `--trials` steps, each from rest (`--settle-ticks`, `--settle-s`). Each writes a target `--step-deg` away at t0, then loops write(same) / read() until the first
  read whose position has moved `--detect-deg` toward the target; the wall time from t0 to that read returning is one sample
  (and the ticks it took, which on a simulated backend is plant time). For the remote cases the core does the stepping, so this is
  the request/response round trip through the attach socket plus the plant.
- ``loop_hz``: ticks per second of the unpaced write/read loop.
- ``step``: one step response (time, position) for the report's plot.
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import socket
import time
from pathlib import Path
from typing import Any

import numpy as np

from examples.hw_backends.backends import (AnalyticalConfig, KinematicConfig, MujocoConfig, RealConfig, RemoteConfig, hardware,
                                           hold)

OUT = Path("/home/bam/like_keras/reports/media/hw-backends-2026-11-28")


def stats(samples_s: list[float]) -> dict[str, Any]:
    ms = np.asarray(samples_s, dtype=float) * 1e3
    if ms.size == 0:
        return {"n": 0}
    return {"n": int(ms.size), "p50": float(np.percentile(ms, 50)), "p95": float(np.percentile(ms, 95)),
            "p99": float(np.percentile(ms, 99)), "max": float(ms.max()), "mean": float(ms.mean()), "min": float(ms.min()),
            # a fixed log-spaced histogram (1 us .. 10 s) so every case's chart shares its x axis
            "hist": np.histogram(np.clip(ms, 1e-3, 1e4), bins=np.logspace(-3, 4, 71))[0].tolist()}


def measure(hw, *, ticks: int, trials: int, step_rad: float, detect_rad: float, settle_ticks: int, settle_s: float,
            max_ticks: int) -> dict[str, Any]:
    clock = time.perf_counter
    state = hw.read()
    origin = float(state.position[0])
    command = hold(state)
    reads, writes = [], []
    began = clock()
    for _ in range(ticks):
        t = clock()
        hw.write(command)
        t1 = clock()
        state = hw.read()
        t2 = clock()
        writes.append(t1 - t)
        reads.append(t2 - t1)
    loop_hz = ticks / (clock() - began)
    assert np.isfinite(float(state.position[0])), "read() returned a NaN position"

    rtts, rtt_ticks, missed = [], [], 0
    step_trace = None
    for trial in range(trials):
        for _ in range(settle_ticks):  # let the last step settle (a simulated plant's time moves only with writes)
            hw.write(command)
        if settle_s:
            end = clock() + settle_s
            while clock() < end:
                hw.write(command)
                time.sleep(0.01)
        state = hw.read()
        q0 = float(state.position[0])
        # alternate origin + step and origin - step: never more than one step from where the board was found, and every step is
        # 2 x step long, so a plant stuck in its friction band (the twin of record's LuGre) still breaks away
        target = origin + (step_rad if trial % 2 == 0 else -step_rad)
        direction = 1.0 if target >= q0 else -1.0
        command = hold(state, target - q0)
        trace = [] if trial == 0 else None
        t0 = clock()
        for k in range(1, max_ticks + 1):
            hw.write(command)
            state = hw.read()
            now = clock()
            q = float(state.position[0])
            if trace is not None:
                trace.append((now - t0, q - q0))
            if direction * (q - q0) >= detect_rad:
                rtts.append(now - t0)
                rtt_ticks.append(k)
                break
        else:
            missed += 1
        if trace is not None:  # the plot: keep stepping 1 s of plant time past the detection
            for _ in range(int(1.0 / max(hw.period_s, 1e-3))):
                hw.write(command)
                state = hw.read()
                trace.append((clock() - t0, float(state.position[0]) - q0))
            step_trace = {"t_s": [round(t, 6) for t, _ in trace], "dq_deg": [round(math.degrees(q), 5) for _, q in trace],
                          "target_deg": math.degrees(target - q0), "period_s": hw.period_s}
    hw.write(hold(state, origin - float(state.position[0])))
    return {"read": stats(reads), "write": stats(writes), "round_trip": stats(rtts),
            "round_trip_ticks": {"p50": float(np.median(rtt_ticks)) if rtt_ticks else None,
                                 "max": int(max(rtt_ticks)) if rtt_ticks else None},
            "missed": missed, "loop_hz": loop_hz, "step": step_trace, "period_s": hw.period_s, "simulated": hw.simulated,
            "backend": hw.backend_name, "origin_deg": math.degrees(origin)}


def cases(names: list[str], address: str | None, fingerprint: str | None) -> dict[str, tuple[Any, Any, str]]:
    from examples.hw_backends.description import nominal, twin_of_record  # noqa: PLC0415

    table: dict[str, Any] = {}
    for name in names:
        if name == "mujoco_nominal":
            table[name] = (MujocoConfig(), nominal(), "1. MuJoCo, nominal (frictionless)")
        elif name == "mujoco_twin":
            table[name] = (MujocoConfig(), twin_of_record(), "2. MuJoCo, twin of record (realistic)")
        elif name == "analytical":
            table[name] = (AnalyticalConfig(), nominal(), "3. Analytical ODE, nominal")
        elif name == "analytical_twin":
            table[name] = (AnalyticalConfig(), twin_of_record(), "3b. Analytical ODE, twin-of-record numbers")
        elif name == "kinematic":
            table[name] = (KinematicConfig(), None, "4. Kinematic (measured = commanded)")
        elif name == "real":
            table[name] = (RealConfig(), None, "5. Real: rack CAN 11 on fdcanusb 272813EF")
        elif name.startswith("remote"):
            if not address or not fingerprint:
                raise SystemExit(f"{name} needs --address and --fingerprint")
            table[name] = (RemoteConfig(address, fingerprint), None, f"6. Remote ({name})")
        else:
            raise SystemExit(f"unknown case {name!r}")
    return table


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cases", default="mujoco_nominal,mujoco_twin,analytical,analytical_twin,kinematic")
    parser.add_argument("--ticks", type=int, default=2000)
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--step-deg", type=float, default=2.0)
    parser.add_argument("--detect-deg", type=float, default=0.02)
    parser.add_argument("--settle-ticks", type=int, default=100, help="plant periods between trials: the step starts at rest")
    parser.add_argument("--settle-s", type=float, default=0.0, help="wall-clock settle between trials (real: 0.2)")
    parser.add_argument("--max-ticks", type=int, default=500)
    parser.add_argument("--address", default=None)
    parser.add_argument("--fingerprint", default=None)
    parser.add_argument("--label", default=None, help="a remote case's label in the report")
    parser.add_argument("--note", default="", help="how the case ran (e.g. which host the core ran on)")
    parser.add_argument("--out", default=str(OUT / "latency.json"), help="merged into: one key per case")
    arguments = parser.parse_args(argv)
    out = Path(arguments.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    merged = json.loads(out.read_text()) if out.exists() else {"cases": {}}
    for name, (config, description, label) in cases(arguments.cases.split(","), arguments.address, arguments.fingerprint).items():
        print(f"[measure] {name}: {label}", flush=True)
        hw = hardware(config, description)
        try:
            result = measure(hw, ticks=arguments.ticks, trials=arguments.trials, step_rad=math.radians(arguments.step_deg),
                             detect_rad=math.radians(arguments.detect_deg), settle_ticks=arguments.settle_ticks,
                             settle_s=arguments.settle_s, max_ticks=arguments.max_ticks)
        finally:
            hw.close()
        result.update(label=arguments.label or label, note=arguments.note, config=repr(config),
                      description=None if description is None else {"label": description.label,
                                                                     "digest": description.parameters_digest,
                                                                     **description.numbers()},
                      host=socket.gethostname(), python=platform.python_version(), utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                      settings={"ticks": arguments.ticks, "trials": arguments.trials, "step_deg": arguments.step_deg,
                                "detect_deg": arguments.detect_deg})
        merged["cases"][name] = result
        r, w, rt = result["read"], result["write"], result["round_trip"]
        print(f"  read p50 {r['p50']:.4f} ms p99 {r['p99']:.4f} | write p50 {w['p50']:.4f} ms p99 {w['p99']:.4f} | "
              f"rtt p50 {rt.get('p50', float('nan')):.4f} ms p99 {rt.get('p99', float('nan')):.4f} (ticks {result['round_trip_ticks']['p50']}) | "
              f"{result['loop_hz']:.0f} Hz | missed {result['missed']}", flush=True)
        out.write_text(json.dumps(merged, indent=1))
    print(f"[measure] wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
