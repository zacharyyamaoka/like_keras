"""A process ended by a signal while its deployment runs (CONTRACT §15 A18; tests/test_signals.py).

    python -m tests.signal_child --mode MODE --directory DIR

The deployment holds kpi's HandBackPy, whose deactivate writes DIR/handed_back, and Ready, which prints
{"ready": true} on its first step. Modes, each the way an application is written:
  until      start(until=600) in real time: the main thread waits inside start()
  lockstep   start(until=1e9) under lockstep: the main thread runs every step inside start()
  own        the application installed its own SIGTERM handler first: like_keras leaves it alone
  ignored    SIGHUP is SIG_IGN (nohup): like_keras leaves it ignored
"""

from __future__ import annotations

import argparse
import json
import pathlib
import signal
import sys

import like_keras as lk
from kpi.facilities.friction_parts import HandBackPy
from robot_msgs.std import Header


class Ready(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        if not getattr(self, "_said", False):
            self._said = True
            print(json.dumps({"ready": True}), flush=True)
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("until", "lockstep", "own", "ignored"), required=True)
    parser.add_argument("--directory", type=pathlib.Path, required=True)
    arguments = parser.parse_args()
    if arguments.mode == "own":
        signal.signal(signal.SIGTERM, lambda number, frame: sys.exit(7))  # the application's own way to end
    if arguments.mode == "ignored":
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    s = lk.System("signalled")
    box = s.add(HandBackPy(path=str(arguments.directory / "handed_back")), "box")
    s.add(Ready(), "ready")
    dep = lk.Deployment(s, realtime=arguments.mode != "lockstep")
    dep.record(arguments.directory / "run.mcap", box.log)
    dep.place(s.box, s.ready, hz=20)
    if arguments.mode in ("until", "lockstep"):
        dep.start(until=600.0 if arguments.mode == "until" else 1e9)
        return
    dep.start()
    try:
        signal.pause()
        signal.pause()  # "ignored": the first signal (SIGHUP) does not end it
    finally:
        dep.stop()


if __name__ == "__main__":
    main()
