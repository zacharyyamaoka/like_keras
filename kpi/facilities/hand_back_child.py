"""A deployment ended by a signal (CONTRACT §15 A18; kpi/facilities/test_facilities_friction.py's test_sigterm_runs_deactivate).

    python -m kpi.facilities.hand_back_child --language py|cpp --backend inproc|dora --directory DIR --name NAME [--library SO]

The contract's file 3 form (§4.6): start, then signal.pause() inside try/finally: stop(). It installs no signal
handler of its own: that is the point. SIGINT reaches the finally through KeyboardInterrupt; A18 makes SIGTERM and
SIGHUP end it the same way, so the box's deactivate writes DIR/handed_back. Under inproc the box's log port is
recorded to DIR/run.mcap, which must hold deactivate's last line. Prints {"ready": true} once started.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import signal

import like_keras as lk


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--language", choices=("py", "cpp"), required=True)
    parser.add_argument("--backend", choices=("inproc", "dora"), required=True)
    parser.add_argument("--directory", type=pathlib.Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--library")
    arguments = parser.parse_args()
    handed_back = str(arguments.directory / "handed_back")
    s = lk.System(arguments.name)
    if arguments.language == "cpp":
        box = s.add(lk.native(arguments.library, config={"path": handed_back}), "box")
    else:
        from kpi.facilities.friction_parts import HandBackPy

        box = s.add(HandBackPy(path=handed_back), "box")
    if arguments.backend == "dora":
        from like_keras.backends import dora

        dep = lk.Deployment(s, backend=dora)
    else:
        dep = lk.Deployment(s)
        dep.record(arguments.directory / "run.mcap", box.log)
    dep.place(s.box, process="box", hz=20)
    dep.start()
    print(json.dumps({"ready": True, "pid": os.getpid()}), flush=True)
    try:
        signal.pause()
    finally:
        dep.stop()


if __name__ == "__main__":
    main()
