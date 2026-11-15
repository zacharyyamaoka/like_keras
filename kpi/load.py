"""The 24-core load of T3 (CONTRACT §10): stdlib multiprocessing spinners (stress-ng is not installed here).

    python3 /home/bam/like_keras/kpi/load.py --cpu 24 --seconds 610

WHY a refusal above 2 processes outside the timing tier: the rack on this host is powered all day and the live-rig
cap is two worker processes across all lanes; the timing tier (KPI_TIER=timing) runs only with the rack unpowered
or on the fixed cloud VM.
"""

from __future__ import annotations

import argparse
import multiprocessing
import os
import sys
import time


def spin(deadline: float) -> None:
    value = 0
    while time.monotonic() < deadline:
        value = (value * 1103515245 + 12345) & 0x7FFFFFFF


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cpu", type=int, required=True)
    parser.add_argument("--seconds", type=float, default=600.0)
    arguments = parser.parse_args(argv)
    if arguments.cpu > 2 and os.environ.get("KPI_TIER") != "timing":
        print("refused: more than 2 spinners outside the timing tier (KPI_TIER=timing)", file=sys.stderr)
        return 2
    deadline = time.monotonic() + arguments.seconds
    workers = [multiprocessing.Process(target=spin, args=(deadline,), daemon=True) for _ in range(arguments.cpu)]
    for worker in workers:
        worker.start()
    try:
        for worker in workers:
            worker.join()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
