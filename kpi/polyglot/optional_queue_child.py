"""A23's optional queue input, deployed in a process of its own (ruler M0.4; kpi/polyglot/test_polyglot_friction.py).

    python -m kpi.polyglot.optional_queue_child --library SO --backend inproc|dora --wiring unwired|wired \
        --directory DIR --name NAME

kpi/polyglot/optional_queue.cpp's box at 10 Hz, lockstep until 1.05 s, its two reports written to DIR/idle.jsonl and
DIR/gated.jsonl, and the box's ports as describe() lists them to DIR/ports.json. `wired`: a SparseStates source
(steps 1, 2 and 6) writes the box's exposed optional queue input `extra`; `unwired`: nothing does.
WHY a child process: on a library where an unwired optional queue input crashes the box (a SIGSEGV in the gather,
library verifier round 1), the crash must fail this one test, never take the pytest process, and every other test of
the suite, with it.
"""

from __future__ import annotations

import argparse
import json
import pathlib

import like_keras as lk

UNTIL_S = 1.05
BOX_HZ = 10.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--library", required=True)
    parser.add_argument("--backend", choices=("inproc", "dora"), required=True)
    parser.add_argument("--wiring", choices=("unwired", "wired"), required=True)
    parser.add_argument("--directory", type=pathlib.Path, required=True)
    parser.add_argument("--name", required=True)
    arguments = parser.parse_args()
    from kpi.polyglot.friction_parts import JointStateLog, SparseStates

    directory = arguments.directory
    s = lk.System(arguments.name)
    if arguments.wiring == "wired":
        source = s.add(SparseStates(), "source")
        box = s.add(lk.native(arguments.library, config={"unused": 0}), "box", extra=source.value)
    else:
        box = s.add(lk.native(arguments.library, config={"unused": 0}), "box")
    s.add(JointStateLog(path=str(directory / "idle.jsonl")), "idle_sink", value=box.idle_steps)
    s.add(JointStateLog(path=str(directory / "gated.jsonl")), "gated_sink", value=box.gated_steps)
    if arguments.backend == "dora":
        from like_keras.backends import dora

        dep = lk.Deployment(s, backend=dora, realtime=False)
    else:
        dep = lk.Deployment(s, realtime=False)
    dep.place(s.box, process="box", hz=BOX_HZ)
    if arguments.wiring == "wired":
        dep.place(s.source, process="source", hz=BOX_HZ)
    dep.place(s.idle_sink, s.gated_sink, process="sinks")
    ports = next(component["ports"] for path, component in dep.describe()["components"].items()
                 if path.endswith(".box"))
    (directory / "ports.json").write_text(json.dumps(ports))
    try:
        dep.start(until=UNTIL_S)
    finally:
        dep.stop()
    print(json.dumps({"done": True}), flush=True)


if __name__ == "__main__":
    main()
