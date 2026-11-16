"""CONTRACT §5 on every backend (criterion 6 part 5; the equivalence gate's semantics).

Each test runs on `inproc` and on `dora` (dora 0.5.0, a private coordinator and daemon, kpi/support/dora_private).
Under dora the sending and receiving groups sit in different processes, so every rule is checked across a process
boundary too. Lockstep tests (`realtime=False`) check exact values; the few real-time tests check what only a
clock can show (overruns, staleness reported in health, FIFO delivery under load, now() as epoch time). Where such a
test also bounds how late or how often steps run on the wall clock, that half is a `timing`-marked twin
(`..._wall_clock`): a loaded host cannot judge it (CONTRACT §15 A14).
The ruler's reading of the open points (rate-less groups under lockstep, tick times i / hz) is in
docs/friction/contract-002.md.
"""

from __future__ import annotations

import json
import time

import pytest

from kpi.support.lk import backend as load_backend
from kpi.support.lk import like_keras

BACKENDS = ["inproc", "dora"]
EPOCH_SLACK_S = 0.05  # how far now() may sit from the wall clock it is mapped to (the offset sampled at start, §5.6)


def rows(path, tag: str | None = None) -> list[dict]:
    try:
        lines = open(path, encoding="utf-8").read().splitlines()
    except FileNotFoundError:
        return []
    found = [json.loads(line) for line in lines if line.strip()]
    return [row for row in found if tag is None or row["tag"] == tag]


@pytest.fixture
def make(request, tmp_path):
    """make(system_name, realtime) -> (lk, parts, System, Deployment factory) for this test's backend."""
    backend_name = request.node.callspec.params["backend"]
    lk = like_keras()
    if backend_name == "dora":
        request.getfixturevalue("private_dora")
    backend = load_backend(backend_name)
    from kpi.conformance import parts

    def build(name: str):
        system = lk.System(f"{name}_{backend_name}")

        def deployment(realtime: bool = False):
            return lk.Deployment(system, backend=backend, realtime=realtime)

        return system, deployment

    build.lk, build.parts, build.backend = lk, parts, backend_name
    build.process = (lambda name: name) if backend_name == "dora" else (lambda name: "main")
    return build


def sink(tmp_path, name="sink.jsonl"):
    return str(tmp_path / name)


@pytest.mark.parametrize("backend", BACKENDS)
def test_latest_reads_newest_and_never_waits(make, tmp_path, backend):
    """§5.4 latest: the receiver reads the newest value at the start of its step; a value sent at t is seen at the
    first step strictly after t. A 100 Hz writer, a 10 Hz reader: at t = i/10 it sees the value sent at t - 0.01."""
    s, deployment = make("latest")
    counter = s.add(make.parts.Counter(), "counter")
    s.add(make.parts.LatestSink(path=sink(tmp_path)), "reader", value=counter.value)
    dep = deployment()
    dep.place(s.counter, process=make.process("writer"), hz=100)
    dep.place(s.reader, process=make.process("reader"), hz=10)
    dep.start(until=1.0)
    dep.stop()
    seen = [(row["now"], row["value"]) for row in rows(sink(tmp_path))]
    assert seen[0] == (0.0, None), "the first step sees nothing: nothing was sent strictly before 0"
    for now, got in seen[1:]:
        assert got == round(now * 100) - 1, f"at {now} the reader saw {got}"


@pytest.mark.parametrize("backend", BACKENDS)
def test_visibility_strictly_after(make, tmp_path, backend):
    """§5.5: equal rates, the writer placed first: at t the reader sees the value of t - period, never t's."""
    s, deployment = make("visible")
    counter = s.add(make.parts.Counter(), "counter")
    s.add(make.parts.LatestSink(path=sink(tmp_path)), "reader", value=counter.value)
    dep = deployment()
    dep.place(s.counter, process=make.process("writer"), hz=10)
    dep.place(s.reader, process=make.process("reader"), hz=10)
    dep.start(until=1.0)
    dep.stop()
    seen = [row["value"] for row in rows(sink(tmp_path))]
    assert seen == [None] + list(range(len(seen) - 1)), seen


@pytest.mark.parametrize("backend", BACKENDS)
def test_tick_times_and_place_order(make, tmp_path, backend):
    """§5.5: tick i of an hz group runs at logical i / hz; ticks at equal times run in place() order."""
    s, deployment = make("order")
    s.add(make.parts.OrderProbe(path=sink(tmp_path), tag="first"), "first")
    s.add(make.parts.OrderProbe(path=sink(tmp_path), tag="second"), "second")
    dep = deployment()
    dep.place(s.second, hz=20)  # placed first: at equal times it runs first, whatever its name
    dep.place(s.first, hz=10)
    dep.start(until=0.99)
    dep.stop()
    found = rows(sink(tmp_path))
    assert [row["now"] for row in found if row["tag"] == "second"] == [i / 20 for i in range(20)]
    assert [row["now"] for row in found if row["tag"] == "first"] == [i / 10 for i in range(10)]
    position = {(row["tag"], row["now"]): index for index, row in enumerate(found)}
    for i in range(10):
        assert position[("second", i / 10)] < position[("first", i / 10)], f"at {i / 10} the later-placed group ran first"


@pytest.mark.parametrize("backend", BACKENDS)
def test_queue_drop_oldest(make, tmp_path, backend):
    """§5.4 queue(2): drop-oldest beyond 2; a 10 Hz reader of a 100 Hz writer pops one per step: 8, 18, 28, ..."""
    s, deployment = make("queue")
    counter = s.add(make.parts.Counter(), "counter")
    s.add(make.parts.QueueSink2(path=sink(tmp_path)), "reader", value=counter.value)
    dep = deployment()
    dep.place(s.counter, process=make.process("writer"), hz=100)
    dep.place(s.reader, process=make.process("reader"), hz=10)
    dep.start(until=1.0)
    dep.stop()
    seen = [row["value"] for row in rows(sink(tmp_path))]
    assert seen == [10 * i + 8 for i in range(len(seen))] and len(seen) >= 8, seen


@pytest.mark.parametrize("backend", BACKENDS)
def test_gather_pops_one_from_each(make, tmp_path, backend):
    """§5.2: a component with two queue inputs steps only when both hold a message, popping one from each."""
    s, deployment = make("gather")
    left = s.add(make.parts.Counter(), "left")
    right = s.add(make.parts.Counter(), "right")
    s.add(make.parts.GatherSink(path=sink(tmp_path)), "gather", left=left.value, right=right.value)
    dep = deployment()
    dep.place(s.left, process=make.process("left"), hz=10)
    dep.place(s.right, process=make.process("right"), hz=5)
    dep.place(s.gather, process=make.process("gather"))
    dep.start(until=2.0)
    dep.stop()
    pairs = [(row["left"], row["right"]) for row in rows(sink(tmp_path))]
    assert pairs and all(a == b for a, b in pairs) and [a for a, _ in pairs] == list(range(len(pairs))), pairs
    assert len(pairs) >= 9, f"{len(pairs)} gathered steps for 10 right-hand messages"


@pytest.mark.parametrize("backend", BACKENDS)
def test_rateless_group_steps_per_message(make, tmp_path, backend):
    """§5.2: a rate-less group steps once per queued message; a same-group downstream with no queue input steps
    after its upstream stepped, in the same step."""
    s, deployment = make("rateless")
    counter = s.add(make.parts.Counter(limit=7), "counter")
    relay = s.add(make.parts.Relay(path=sink(tmp_path), tag="relay"), "relay", value=counter.value)
    s.add(make.parts.Downstream(path=sink(tmp_path), tag="down"), "down", value=relay.out)
    dep = deployment()
    dep.place(s.counter, process=make.process("writer"), hz=10)
    dep.place(s.relay, s.down, process=make.process("reader"))
    dep.start(until=2.0)
    dep.stop()
    found = rows(sink(tmp_path))
    assert [row["value"] for row in found if row["tag"] == "relay"] == list(range(7))
    assert [row["value"] for row in found if row["tag"] == "down"] == list(range(7))
    order = [row["tag"] for row in found]
    assert order == ["relay", "down"] * 7, order


@pytest.mark.parametrize("backend", BACKENDS)
def test_lagged_input_reads_previous_step(make, tmp_path, backend):
    """§4.1, §5.3: a same-group cycle is legal when one input is lagged; it reads the previous step's value; the
    order is derived (front -> back), never written."""
    s, deployment = make("lagged")
    front = s.add(make.parts.LaggedFront(path=sink(tmp_path)), "front")
    back = s.add(make.parts.LaggedBack(), "back", front=front.out)
    s.wire(front, back=back.out)
    dep = deployment()
    dep.place(s.front, s.back, hz=10)
    dep.start(until=1.0)
    dep.stop()
    seen = [(row["count"], row["back"]) for row in rows(sink(tmp_path))]
    assert seen[0] == (0, None)
    assert all(back == count - 1 for count, back in seen[1:]), seen


@pytest.mark.parametrize("backend", BACKENDS)
def test_forward_return_values(make, tmp_path, backend):
    """§4.1: several Outs return a tuple in declaration order; a None element is not sent."""
    s, deployment = make("returns")
    pair = s.add(make.parts.CounterPair(), "pair")
    s.add(make.parts.QueueSink(path=sink(tmp_path), tag="left"), "left", value=pair.left)
    s.add(make.parts.QueueSink(path=sink(tmp_path), tag="right"), "right", value=pair.right)
    dep = deployment()
    dep.place(s.pair, process=make.process("writer"), hz=10)
    dep.place(s.left, s.right, process=make.process("reader"))
    dep.start(until=0.95)
    dep.stop()
    assert [row["value"] for row in rows(sink(tmp_path), "right")] == list(range(10))
    assert [row["value"] for row in rows(sink(tmp_path), "left")] == list(range(0, 10, 2))


@pytest.mark.parametrize("backend", BACKENDS)
def test_stale_latest_reads_none(make, tmp_path, backend):
    """§4.1 stale=0.05: a latest value older than 0.05 s on the receiver's clock is delivered as None."""
    s, deployment = make("stale")
    counter = s.add(make.parts.Counter(limit=3), "counter")
    s.add(make.parts.StaleSink(path=sink(tmp_path)), "reader", value=counter.value)
    dep = deployment()
    dep.place(s.counter, process=make.process("writer"), hz=10)
    dep.place(s.reader, process=make.process("reader"), hz=100)
    dep.start(until=0.5)
    dep.stop()
    seen = {round(row["now"], 6): row["value"] for row in rows(sink(tmp_path))}
    assert seen[0.21] == 2 and seen[0.24] == 2, "fresh within 0.05 s"
    assert all(seen[t] is None for t in seen if t >= 0.26), "stale after 0.05 s"


@pytest.mark.parametrize("backend", BACKENDS)
def test_lifecycle_order(make, tmp_path, backend):
    """§5.7: start activates a group's components in derived order before its first step; stop deactivates in reverse."""
    s, deployment = make("lifecycle")
    counter = s.add(make.parts.Counter(), "counter")
    first = s.add(make.parts.ChainProbe(path=sink(tmp_path), tag="a"), "a", value=counter.value)
    second = s.add(make.parts.ChainProbe(path=sink(tmp_path), tag="b"), "b", value=first.out)
    s.add(make.parts.ChainProbe(path=sink(tmp_path), tag="c"), "c", value=second.out)
    dep = deployment()
    dep.place(s.counter, s.a, s.b, s.c, hz=10)
    dep.start(until=0.3)
    dep.stop()
    events = [(row["tag"], row["event"]) for row in rows(sink(tmp_path))]
    assert events[:3] == [("a", "activate"), ("b", "activate"), ("c", "activate")]
    assert events[-3:] == [("c", "deactivate"), ("b", "deactivate"), ("a", "deactivate")]
    steps = [tag for tag, event in events if event == "step"]
    assert steps[:3] == ["a", "b", "c"], steps


@pytest.mark.parametrize("backend", BACKENDS)
def test_activate_raising_sends_nothing(make, tmp_path, backend):
    """§5.7: activate raising: the group never starts and sends no data frame (inproc: start raises; dora: the
    process exits non-zero)."""
    s, deployment = make("refuse")
    refuser = s.add(make.parts.RefuseActivate(), "refuser")
    s.add(make.parts.QueueSink(path=sink(tmp_path)), "reader", value=refuser.value)
    dep = deployment(realtime=True)
    dep.place(s.refuser, process=make.process("refuser"), hz=100)
    dep.place(s.reader, process=make.process("reader"))
    try:
        if make.backend == "inproc":
            with pytest.raises((Exception, SystemExit)):  # inproc: the launching process is the one refusing
                dep.start(until=0.5)
        else:
            dep.start()
            time.sleep(2.0)
    finally:
        dep.stop()
    assert rows(sink(tmp_path)) == [], "a refused component sent data"


def _raise_in_forward(make, tmp_path) -> tuple[list[dict], list[dict], list[dict]]:
    """Two 20 Hz groups in one process for 3 s of wall time, one raising in forward at its step 3: what the reader
    got from the raiser, from the counter, and the raiser's health records."""
    from kpi.facilities.parts import HealthLog

    s, deployment = make("raises")
    raiser = s.add(make.parts.RaiseInForward(), "raiser")
    counter = s.add(make.parts.Counter(), "counter")
    s.add(make.parts.QueueSink(path=sink(tmp_path), tag="raiser"), "from_raiser", value=raiser.value)
    s.add(make.parts.QueueSink(path=sink(tmp_path), tag="counter"), "from_counter", value=counter.value)
    s.add(HealthLog(path=sink(tmp_path, "health.jsonl")), "health", value=raiser.health)
    dep = deployment(realtime=True)
    dep.place(s.raiser, process=make.process("work"), hz=20)
    dep.place(s.counter, process=make.process("work"), hz=20)
    dep.place(s.from_raiser, s.from_counter, s.health, process=make.process("reader"))
    dep.start()
    time.sleep(3.0)
    dep.stop()
    return rows(sink(tmp_path), "raiser"), rows(sink(tmp_path), "counter"), rows(sink(tmp_path, "health.jsonl"))


@pytest.mark.parametrize("backend", BACKENDS)
def test_forward_exception_goes_error(make, tmp_path, backend):
    """§5.7: an exception in forward deactivates that group; health goes error with code Exception; the process
    keeps serving health and the other groups keep stepping: the counter's values still arrive after the error was
    reported. How many steps the other group made in 3 s of wall time is the timing twin's
    (test_forward_exception_goes_error_wall_clock, CONTRACT §15 A14)."""
    from_raiser, from_counter, health = _raise_in_forward(make, tmp_path)
    assert [row["value"] for row in from_raiser] == [0, 1, 2], "the raising group kept stepping"
    errors = [row for row in health
              if row["level"] == "error" and row["code"] == "Exception" and "on purpose" in row["message"]]
    assert errors, "health did not go error with code Exception"
    reported = min(row["stamp"] for row in errors)
    assert any(row["wall"] > reported for row in from_counter), "the other group stopped when the raiser's group did"


@pytest.mark.timing
@pytest.mark.parametrize("backend", BACKENDS)
def test_forward_exception_goes_error_wall_clock(make, tmp_path, backend):
    """Timing tier (rack unpowered or the fixed cloud VM): test_forward_exception_goes_error, and the other group
    made at least 40 of its 60 steps in the 3 s."""
    from_raiser, from_counter, health = _raise_in_forward(make, tmp_path)
    assert [row["value"] for row in from_raiser] == [0, 1, 2], "the raising group kept stepping"
    assert len(from_counter) >= 40, "the other group stopped"
    assert any(row["level"] == "error" and row["code"] == "Exception" and "on purpose" in row["message"] for row in health)


@pytest.mark.parametrize("backend", BACKENDS)
def test_overrun_skips_never_bursts(make, tmp_path, backend):
    """§5.2: an overrun skips the missed ticks (counted in metrics.overruns) and never catches up with a burst."""
    from kpi.facilities.parts import MetricsLog

    s, deployment = make("overrun")
    slow = s.add(make.parts.Overrunner(path=sink(tmp_path)), "slow")
    s.add(MetricsLog(path=sink(tmp_path, "metrics.jsonl")), "metrics", value=slow.metrics)
    dep = deployment(realtime=True)
    dep.place(s.slow, process=make.process("work"), hz=100)
    dep.place(s.metrics, process=make.process("reader"))
    dep.start()
    time.sleep(2.5)
    dep.stop()
    steps = rows(sink(tmp_path))
    nows = [row["now"] for row in steps]
    gaps = [later - earlier for earlier, later in zip(nows, nows[1:])]
    assert any(gap > 0.015 for gap in gaps), "no tick was skipped after a 25 ms step at 100 Hz"
    assert all(gap > 0.005 for gap in gaps), "a burst: two steps closer than half a period"
    walls = [row["wall"] for row in steps]
    assert all(later - earlier > 0.004 for earlier, later in zip(walls, walls[1:])), "a burst in wall time"
    overruns = [row["values"]["overruns"] for row in rows(sink(tmp_path, "metrics.jsonl")) if "overruns" in row["values"]]
    assert overruns and max(overruns) > 0, "metrics.overruns not counted"


@pytest.mark.parametrize("backend", BACKENDS)
def test_queue_fifo_and_drops_counted(make, tmp_path, backend):
    """§5.4 under a real clock: a queue edge across processes delivers in order, sent = received while it keeps
    up; a queue(2) behind a slow reader drops oldest and counts it in metrics.drops.<port>."""
    from kpi.facilities.parts import MetricsLog

    s, deployment = make("fifo")
    counter = s.add(make.parts.Counter(limit=500), "counter")
    s.add(make.parts.QueueSink(path=sink(tmp_path), tag="all"), "everything", value=counter.value)
    small = s.add(make.parts.QueueSink2(path=sink(tmp_path), tag="small"), "small", value=counter.value)
    s.add(MetricsLog(path=sink(tmp_path, "metrics.jsonl")), "metrics", value=small.metrics)
    dep = deployment(realtime=True)
    dep.place(s.counter, process=make.process("writer"), hz=500)
    dep.place(s.everything, s.metrics, process=make.process("reader"))
    dep.place(s.small, process=make.process("slow"), hz=10)
    dep.start()
    time.sleep(3.0)
    dep.stop()
    assert [row["value"] for row in rows(sink(tmp_path), "all")] == list(range(500)), "FIFO, sent = received"
    drops = [row["values"].get("drops.value", 0) for row in rows(sink(tmp_path, "metrics.jsonl"))]
    assert drops and max(drops) > 0, "drops not counted in metrics.drops.value"


@pytest.mark.parametrize("backend", BACKENDS)
def test_stale_reported_in_health(make, tmp_path, backend):
    """§4.1, §8: a stale latest input is listed in health.stale_inputs with level warn, code InputStale."""
    from kpi.facilities.parts import HealthLog

    s, deployment = make("stalehealth")
    counter = s.add(make.parts.Counter(limit=5), "counter")
    reader = s.add(make.parts.StaleSink(path=sink(tmp_path)), "reader", value=counter.value)
    s.add(HealthLog(path=sink(tmp_path, "health.jsonl")), "health", value=reader.health)
    dep = deployment(realtime=True)
    dep.place(s.counter, process=make.process("writer"), hz=10)
    dep.place(s.reader, process=make.process("reader"), hz=50)
    dep.place(s.health, process=make.process("reader"))
    dep.start()
    time.sleep(3.0)
    dep.stop()
    health = rows(sink(tmp_path, "health.jsonl"))
    assert any(row["level"] == "warn" and row["code"] == "InputStale" and "value" in row["stale_inputs"] for row in health)


def _scheduled_clock(make, tmp_path) -> tuple[list[dict], float, float]:
    """A 50 Hz probe for 1.5 s of wall time: what it wrote each step (now, wall), and the wall clock just before
    start() and just after stop()."""
    s, deployment = make("clock")
    s.add(make.parts.OrderProbe(path=sink(tmp_path)), "probe")
    dep = deployment(realtime=True)
    dep.place(s.probe, process=make.process("probe"), hz=50)
    started = time.time()
    dep.start()
    time.sleep(1.5)
    dep.stop()
    return rows(sink(tmp_path)), started, time.time()


def _assert_on_the_period_grid(found: list[dict]) -> None:
    nows = [row["now"] for row in found]
    periods = [later - earlier for earlier, later in zip(nows, nows[1:])]
    assert all(abs(period - 0.02) < 1e-6 or period > 0.03 for period in periods), "now() is not the scheduled deadline"


@pytest.mark.parametrize("backend", BACKENDS)
def test_now_is_scheduled_time(make, tmp_path, backend):
    """§5.6: now() is the step's scheduled time: epoch seconds in real time (a deadline, not a measurement). What
    holds however late a step runs: every now() is epoch time inside the run, never after the wall clock of its own
    step (a deadline comes first), and on the period grid. How late steps run (|now - wall| < 50 ms) and how many run
    (>= 50 in 1.5 s) is the timing twin's (test_now_is_scheduled_time_wall_clock, CONTRACT §15 A14)."""
    found, started, stopped = _scheduled_clock(make, tmp_path)
    assert len(found) >= 3, f"{len(found)} steps in 1.5 s"
    assert all(started - EPOCH_SLACK_S < row["now"] < stopped for row in found), "now() is not epoch time in the run"
    assert all(row["now"] - row["wall"] < EPOCH_SLACK_S for row in found), "now() came after its step: a measurement"
    _assert_on_the_period_grid(found)


@pytest.mark.timing
@pytest.mark.parametrize("backend", BACKENDS)
def test_now_is_scheduled_time_wall_clock(make, tmp_path, backend):
    """Timing tier (rack unpowered or the fixed cloud VM): test_now_is_scheduled_time, with every step within 50 ms of
    its deadline and at least 50 of the 75 steps run."""
    found, _, _ = _scheduled_clock(make, tmp_path)
    assert len(found) >= 50
    assert all(abs(row["now"] - row["wall"]) < EPOCH_SLACK_S for row in found), "now() is not epoch time"
    _assert_on_the_period_grid(found)


@pytest.mark.parametrize("backend", BACKENDS)
def test_ticks_logged_as_int64(make, tmp_path, backend):
    """§5.6: ticks are int64 counts; LogRecord.tick is the component's step count when it logged."""
    from kpi.facilities.parts import LogLog

    s, deployment = make("ticks")
    talker = s.add(make.parts.Talker(), "talker")
    s.add(LogLog(path=sink(tmp_path)), "logs", value=talker.log)
    dep = deployment()
    dep.place(s.talker, process=make.process("talker"), hz=10)
    dep.place(s.logs, process=make.process("reader"))
    dep.start(until=1.0)
    dep.stop()
    ticks = [row["tick"] for row in rows(sink(tmp_path)) if row["message"] == "step"]
    assert len(ticks) >= 9 and ticks[0] in (0, 1), ticks
    assert ticks == list(range(ticks[0], ticks[0] + len(ticks))), "one tick per step, counted as integers"
