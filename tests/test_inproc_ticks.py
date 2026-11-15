"""Lane core's own tests of inproc's real-time tick loop (CONTRACT §5.2), beside the ruler's kpi/conformance.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

§5.2: an overrun skips the missed ticks and never catches up with a burst. A step whose forward BEGAN more than half
a period late gives up the next tick, so no two forwards start closer than half a period.
WHY this test (fix round 1, M1): the loop read the lateness at the thread's wake-up, before the input reads. A thread
preempted between waking and forward (measured 8.7 ms at 100 Hz, rack powered, cores 22-23, not the timing tier) then
stepped again 0.61 ms later, and kpi/conformance's test_overrun_skips_never_bursts[inproc] saw a burst in wall time.
"""

from __future__ import annotations

import itertools
import time

import like_keras as lk
from like_keras.backends import inproc

HZ = 100.0
DELAY_S = 0.007  # between the wake-up and forward: past half a period (5 ms), short of a whole one (10 ms)


class Stamp(lk.Component):
    """Records when its forward began, first thing, on the clock the tick loop uses."""

    def activate(self) -> None:
        self.entries: list[float] = []

    def forward(self):
        self.entries.append(time.monotonic())


def test_a_forward_that_begins_late_gives_up_the_next_tick(monkeypatch):
    calls = itertools.count()
    original = inproc.Box.step

    def late_step(box, now):
        if next(calls) % 4 == 1:  # every fourth step is held up after the wake-up, before its inputs and forward
            time.sleep(DELAY_S)
        return original(box, now)

    monkeypatch.setattr(inproc.Box, "step", late_step)
    system = lk.System("late_forward")
    stamp = Stamp()
    system.add(stamp, "stamp")
    deployment = lk.Deployment(system)
    deployment.place(system.stamp, hz=HZ)
    deployment.start(until=1.2)
    deployment.stop()
    gaps = [later - earlier for earlier, later in zip(stamp.entries, stamp.entries[1:])]
    assert len(gaps) >= 20, f"only {len(gaps) + 1} steps in 1.2 s at {HZ:.0f} Hz"
    closest = min(gaps)
    assert closest > 0.5 / HZ, f"two forwards began {closest * 1e3:.2f} ms apart at {HZ:.0f} Hz: a burst (§5.2)"
