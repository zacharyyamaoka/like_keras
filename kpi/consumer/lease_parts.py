"""A36's processes (CONTRACT §15 A36; consumer-019; kpi/consumer/test_consumer_friction.py): an always-on box with
two exposed inputs, and attach clients that each write one of them.

    python -m kpi.consumer.lease_parts serve --name lease_kpi --path rows.jsonl
    python -m kpi.consumer.lease_parts write --address unix:... --port a --base 1000 --name writer_a --record a.mcap

`serve` runs TwoInputs alone (inproc, real time, 100 Hz), exposing its inputs `a` and `b`: the always-on form (§10)
with a planner's input and a guard's. `write` attaches to it and writes one input, `port`, at 20 Hz: base + 1, base +
2, ...; it records its proxy's health (a refused write is `error` / `LeaseHeld`, naming the holder: §4.5, T8).
Each prints {"ready": true, ...} once started and stops on SIGTERM or SIGINT.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import signal
import time

import like_keras as lk
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


def _first(value) -> float | None:
    return None if value is None else float(value.position[0])


class TwoInputs(lk.Component):
    """Two latest inputs, `a` and `b`; every step it writes one JSON line: the wall time and what each read (None
    when nothing was written yet, or when its writer's lease lapsed and it was retracted: §5.4, §9.3)."""

    a = lk.In(JointState)
    b = lk.In(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        path: str

    def activate(self):
        self._file = open(self.params.path, "a", encoding="utf-8")

    def forward(self, a, b):
        self._file.write(json.dumps({"wall": time.time(), "a": _first(a), "b": _first(b)}) + "\n")
        self._file.flush()

    def deactivate(self):
        self._file.close()


class Count(lk.Component):
    """base + 1, base + 2, ... one per step."""

    value = lk.Out(JointState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        base: float = 0.0

    def forward(self):
        self._count = getattr(self, "_count", 0) + 1
        return JointState(name=("value",), position=[self.params.base + self._count], header=Header(stamp=self.now()))


def serve(arguments) -> lk.Deployment:
    s = lk.System(arguments.name)
    box = s.add(TwoInputs(path=arguments.path), "box")
    s.expose(box.a, box.b)
    dep = lk.Deployment(s)
    dep.place(s.box, hz=100)
    return dep


def write(arguments) -> lk.Deployment:
    s = lk.System(arguments.name)
    proxy = s.add(lk.attach(arguments.address), "arm")
    counter = s.add(Count(base=arguments.base), "count")
    s.wire(proxy, **{arguments.port: counter.value})
    dep = lk.Deployment(s)
    dep.place(s.count, hz=20)
    dep.place(proxy, hz=100)  # the proxy: steps at 100 Hz, sending what is new
    dep.record(arguments.record, proxy.health)
    return dep


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("role", choices=("serve", "write"))
    parser.add_argument("--name", required=True)
    parser.add_argument("--path", default="")
    parser.add_argument("--address", default="")
    parser.add_argument("--port", default="a")
    parser.add_argument("--base", type=float, default=0.0)
    parser.add_argument("--record", default="")
    arguments = parser.parse_args(argv)
    dep = serve(arguments) if arguments.role == "serve" else write(arguments)
    plan = dep.describe()
    dep.start()
    print(json.dumps({"ready": True, "attach": plan.get("attach")}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    raise SystemExit(main())
