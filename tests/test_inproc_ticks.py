"""Lane core's own tests of inproc's real-time tick loop (CONTRACT §5.2), beside the ruler's kpi/conformance.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

§5.2: an overrun skips the missed ticks (counted in metrics.overruns) and never catches up with a burst.
- A tick is skipped, and counted, only when its deadline had already passed as the step before it ended.
- Never a burst: the next forward never begins within half a period of the last one's beginning (on the loop's clock).
  A step that began late DELAYS the next one to half a period after it; it does not give that tick up.
WHY (fix round 1, M1): the loop read the lateness at the thread's wake-up, before the input reads. A thread preempted
between waking and forward (measured 8.7 ms at 100 Hz, rack powered, cores 22-23, not the timing tier) then stepped
again 0.61 ms later, and kpi/conformance's test_overrun_skips_never_bursts[inproc] saw a burst in wall time.
WHY (fix round 2, M1): round 1 kept the half-period rule by skipping the next tick, and counted it as an overrun
though its deadline had not passed. That reported Overrun in 23 of 30 health periods at 50 Hz on the loaded host and
failed kpi/facilities::test_health[python-inproc] in 7 of 24 runs; §5.2 counts only missed ticks.
"""

from __future__ import annotations

import itertools
import time

import like_keras as lk
from like_keras.backends import inproc


class Stamp(lk.Component):
    """Records its scheduled time and when its forward began, first thing, on the clock the tick loop uses."""

    def activate(self) -> None:
        self.entries: list[float] = []
        self.ticks: list[tuple[float, float]] = []  # (now(): the scheduled epoch time, forward began: monotonic)

    def forward(self):
        began = time.monotonic()
        self.entries.append(began)
        self.ticks.append((self.now(), began))


def hold_up(monkeypatch, every: int, delay_s: float) -> list[float]:
    """Hold every `every`-th step up `delay_s` after the wake-up, before its inputs and forward. Returns, filled as
    the run goes, when each forward began on the loop's own clock (Box.entered, the time the burst rule reads)."""
    calls, entered = itertools.count(), []
    original = inproc.Box.step

    def late_step(box, now):
        if next(calls) % every == 1:
            time.sleep(delay_s)
        result = original(box, now)
        entered.append(box.entered)
        return result

    monkeypatch.setattr(inproc.Box, "step", late_step)
    return entered


def assert_never_a_burst(entered: list[float], observed: list[float], hz: float) -> None:
    """§5.2 never a burst. On the loop's clock, exactly: no two forwards begin within half a period. As the forward
    itself sees it, within the ruler's slack (kpi/conformance test_overrun_skips_never_bursts: 4 ms at 100 Hz).
    WHY two thresholds: the forward reads the clock after the loop does, and a preemption in between is invisible to
    the loop (once 3.5 ms, rack powered, cores 22-23, not the timing tier), while the rule now places the next forward
    at exactly half a period after a late one."""
    loop_gaps = [later - earlier for earlier, later in itertools.pairwise(entered)]
    closest = min(loop_gaps)
    assert closest >= 0.5 / hz, f"two forwards began {closest * 1e3:.2f} ms apart at {hz:.0f} Hz: a burst (§5.2)"
    seen = min(later - earlier for earlier, later in itertools.pairwise(observed))
    assert seen > 0.4 / hz, f"the forward saw two steps {seen * 1e3:.2f} ms apart at {hz:.0f} Hz: a burst (§5.2)"


def run_stamp(name: str, hz: float, seconds: float) -> tuple[Stamp, float]:
    """Run one Stamp at `hz` in real time; returns it and the offset that maps its now() onto time.monotonic()."""
    system = lk.System(name)
    stamp = Stamp()
    system.add(stamp, "stamp")
    deployment = lk.Deployment(system)
    deployment.place(system.stamp, hz=hz)
    deployment.start()
    run = deployment._run  # WHY a private read: the deadline of tick k is mono0 + k / hz, and now() is epoch0 + k / hz
    time.sleep(seconds)
    deployment.stop()
    offset = run.mono0 - run.epoch0
    return stamp, offset


def test_a_forward_that_begins_late_never_has_the_next_begin_within_half_a_period(monkeypatch):
    hz = 100.0
    entered = hold_up(monkeypatch, every=4, delay_s=0.007)  # past half a period (5 ms), short of a whole one (10 ms)
    stamp, _ = run_stamp("late_forward", hz, 1.2)
    assert len(stamp.entries) >= 21, f"only {len(stamp.entries)} steps in 1.2 s at {hz:.0f} Hz"
    assert_never_a_burst(entered, stamp.entries, hz)


def test_a_late_step_that_ends_before_the_next_deadline_skips_no_tick(monkeypatch):
    hz, period = 10.0, 0.1
    # 60 ms late: past half a period (50 ms), and 40 ms short of the next deadline, so neither edge is within reach of
    # the scheduler noise on the loaded host (rack powered, cores 22-23, not the timing tier).
    entered = hold_up(monkeypatch, every=3, delay_s=0.06)
    margin_s = 0.02
    stamp, offset = run_stamp("late_not_overrun", hz, 2.0)
    late, wrongly_skipped = 0, []
    for (now, began), (following, _) in itertools.pairwise(stamp.ticks):
        deadline = now + offset
        if began - deadline <= 0.5 * period or began + margin_s >= deadline + period:
            continue  # not held past half a period, or ended too near the next deadline to judge
        late += 1
        if round((following - now) * hz) != 1:
            wrongly_skipped.append(
                f"tick at {now - stamp.ticks[0][0]:.1f} s began {(began - deadline) * 1e3:.0f} ms late"
            )
    assert late >= 3, f"only {late} steps were held between half a period and the next deadline: nothing measured"
    assert not wrongly_skipped, (
        f"{len(wrongly_skipped)} of {late} steps that ended before the next deadline gave that tick up (§5.2 skips "
        f"only missed ticks): {wrongly_skipped[:3]}"
    )
    skipped = sum(round((later - earlier) * hz) - 1 for (earlier, _), (later, _) in itertools.pairwise(stamp.ticks))
    assert stamp._lk.overruns >= skipped, f"{skipped} ticks skipped, {stamp._lk.overruns} counted in overruns"
    assert_never_a_burst(entered, stamp.entries, hz)
