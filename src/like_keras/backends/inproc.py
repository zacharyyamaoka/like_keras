"""The in-process backend (CONTRACT §9.1): every group a thread, every edge between groups a lock-free slot, and
under lockstep one logical clock stepping every group in time order on the calling thread (§5.5).

Every `process` collapses into this one, with the same slot semantics as across processes, so moving a component
between processes is one edit of `process=` or of `backend=`, never both.
"""

from __future__ import annotations

import collections
import heapq
import itertools
import json
import math
import sys
import threading
import time
from typing import Any

from like_keras.backends import address  # noqa: F401  (part of the backend's surface: describe() prints it)
from like_keras.component import message_type, path

NAME = "inproc"
VERSION = "0.1.0"
CAPABILITIES = ("latest", "queue", "lagged", "lockstep", "mixed_process")


def start(plan, until: float | None):
    run = Run(plan)
    run.start(until)
    return run


class Latest:
    """latest (CONTRACT §5.4): the newest value and its send time, and the one before it, so a reader stepping at t
    sees only what was sent strictly before t. One tuple swap per write: no lock, no wait on either side."""

    __slots__ = ("cell",)

    def __init__(self) -> None:
        self.cell = (None, -math.inf, None, -math.inf)

    def write(self, value: Any, sent: float) -> None:
        current = self.cell
        self.cell = (value, sent, *current[:2]) if sent > current[1] else (value, sent, *current[2:])

    def read(self, now: float, strict: bool) -> tuple[Any, float]:
        value, sent, before, before_sent = self.cell
        return (value, sent) if not strict or sent < now else (before, before_sent)


class Queue:
    """queue(n) (CONTRACT §5.4): drop-oldest beyond n, drops counted; one per edge (single producer, single
    consumer). WHY `arriving` under lockstep: a message sent at t must not push out one the reader stepping at t
    is still due; it joins the queue only once a step strictly after t could see it."""

    __slots__ = ("ready", "arriving", "drops", "wake", "lockstep")

    def __init__(self, size: int, lockstep: bool, wake=None) -> None:
        self.ready, self.arriving = collections.deque(maxlen=size), collections.deque()
        self.drops, self.wake, self.lockstep = 0, wake, lockstep

    def push(self, value: Any, sent: float) -> None:
        if self.lockstep:
            self._arrive(sent, True)
            self.arriving.append((sent, value))
        else:
            self._admit((sent, value))
        if self.wake is not None:
            self.wake(sent)

    def has(self, now: float, strict: bool) -> bool:
        self._arrive(now, strict)
        return bool(self.ready)

    def take(self, now: float, strict: bool) -> Any:
        return self.ready.popleft()[1] if self.has(now, strict) else None

    def _arrive(self, now: float, strict: bool) -> None:
        while self.arriving and (self.arriving[0][0] < now or not strict and self.arriving[0][0] == now):
            self._admit(self.arriving.popleft())

    def _admit(self, item: tuple[float, Any]) -> None:
        if len(self.ready) == self.ready.maxlen:
            self.drops += 1
        self.ready.append(item)


class Box:
    """One Python leaf at run time: where each input reads from, where each output goes, and its step."""

    def __init__(self, component) -> None:
        self.component, self.state = component, component._lk
        self.outs = type(component)._lk_outs
        self.queues: list[tuple[str, Queue, bool]] = []
        self.latest: list[tuple[str, Latest, bool, float | None]] = []
        self.sinks = {name: [] for name, port in type(component)._lk_ports.items() if port.output}
        self.upstream: list[Box] = []  # same-group writers: in a rate-less group they let this one step (§5.2)

    def ready(self, now: float) -> bool:
        return all(slot.has(now, strict) for _, slot, strict in self.queues)

    def step(self, now: float) -> None:
        state = self.state
        inputs = {name: slot.take(now, strict) for name, slot, strict in self.queues}
        for name, slot, strict, stale in self.latest:
            value, sent = slot.read(now, strict)
            if value is not None:
                state.ages[name] = now - sent
                if stale is not None and now - sent > stale:  # §4.1: delivered as None, reported in health
                    value = None
                    state.stale.add(name)
            inputs[name] = value
        state.now = now
        self.entered = time.monotonic()  # when forward really began: Group.loop's burst rule reads it (§5.2)
        started = time.perf_counter()
        result = self.component.forward(**inputs)
        state.durations.append(time.perf_counter() - started)
        state.steps += 1
        if result is not None and self.outs:
            values = (result,) if len(self.outs) == 1 else result
            for name, value in zip(self.outs, values, strict=True):
                if value is not None:  # None is not sent (§4.1)
                    for sink in self.sinks[name]:
                        sink(value, now)


class Group:
    """One place() call on one thread: its components as plain calls in the derived order (CONTRACT §5.1-5.3)."""

    def __init__(self, run: Run, placed, index: int) -> None:
        self.run, self.index, self.hz, self.dead = run, index, placed.hz, False
        self.boxes = [run.boxes[id(leaf)] for leaf in placed.order]
        self.wake = threading.Event()

    def activate(self, now: float) -> None:
        done = []
        for box in self.boxes:
            try:
                box.component.activate()
            except Exception as error:
                for activated in reversed(done):
                    _deactivate(activated)
                error.add_note(f"raised by {path(box.component)}.activate(): its group never started (CONTRACT §5.7)")
                raise
            done.append(box)
            box.state.mode, box.state.started = "operational", now

    def deactivate(self) -> None:
        for box in reversed(self.boxes):
            _deactivate(box)

    def notify(self, sent: float) -> None:
        """A message reached a queue input of this rate-less group: one step, at its send time under lockstep."""
        if self.run.lockstep:
            heapq.heappush(self.run.heap, (sent, 1, next(self.run.sequence), self.index, 0))
        else:
            self.wake.set()

    def step(self, now: float) -> bool:
        """One step of the group; True when a component stepped on a queued message."""
        stepped, triggered, box = set(), False, None
        self.entered = None  # when the step's first forward began (monotonic), None when nothing stepped
        try:
            for box in self.boxes:
                if box.queues:
                    if not box.ready(now):
                        continue
                    triggered = True
                elif self.hz is None and not any(id(writer) in stepped for writer in box.upstream):
                    continue
                box.step(now)
                stepped.add(id(box))
                if self.entered is None:
                    self.entered = box.entered
        except Exception as error:  # §5.7: the group stops; health says why; the process keeps serving
            box.state.error = f"{type(error).__name__}: {error}"
            box.component.log.error("forward raised; its group stops: %s", error, exc_info=error)
            self.dead = True
            self.deactivate()
        return triggered

    def loop(self) -> None:
        """Real time. hz: absolute deadlines from the deployment's start; an overrun skips the missed ticks, counted,
        never caught up with a burst (§5.2). Rate-less: steps while queued messages let a component step."""
        run, tick = self.run, 0
        while not self.dead:
            if self.hz is None:
                if run.stopping.is_set():
                    return
                self.wake.wait(0.1)
                self.wake.clear()
                while self.step(run.epoch0 + time.monotonic() - run.mono0) and not self.dead:
                    pass
                continue
            deadline = run.mono0 + tick / self.hz
            if run.stopping.wait(max(0.0, deadline - time.monotonic())):
                return
            started = time.monotonic()
            for box in self.boxes:
                box.state.period_errors.append(started - deadline)
            self.step(run.epoch0 + tick / self.hz)
            # WHY two skips: past deadlines are missed ticks, and a step whose forward began more than half a period
            # late also gives up the next tick, so no two forwards ever start closer than half a period (§5.2: never
            # a burst). WHY the forward's start, not the wake-up: a thread preempted between waking and forward
            # (8.7 ms at 100 Hz, rack powered) stepped again 0.61 ms later (tests/test_inproc_ticks.py).
            began = started if self.entered is None else self.entered
            began_tick, ended_tick = (began - run.mono0) * self.hz, (time.monotonic() - run.mono0) * self.hz
            following = max(tick + 1, math.floor(began_tick + 0.5) + 1, math.floor(ended_tick) + 1)
            for box in self.boxes:
                box.state.overruns += following - tick - 1
            tick = following


def _deactivate(box: Box) -> None:
    if box.state.mode != "operational":
        return
    box.state.mode = "offline"
    try:
        box.component.deactivate()
    except Exception as error:  # deactivate must not raise; if it does, say so and keep shutting down
        box.component.log.error("deactivate raised: %s", error, exc_info=error)


class NativeGroup:
    """A native .so's own RT thread, driven through its C ABI (CONTRACT §6.3, §9.1). This is the seam: the .so is
    created through ctypes here, so its parameter errors surface at start(); the data path (lk_push/lk_take over
    Arrow C Data structs, lk_step under lockstep) lands with the C++ runtime (CONTRACT §12, M2)."""

    def __init__(self, run: Run, placed, index: int) -> None:
        import ctypes

        (self.component,) = placed.order
        self.index, self.hz, self.dead, self.wake, self.placed = index, placed.hz, False, threading.Event(), placed
        library = type(self.component)._lk_impl
        self.calls = ctypes.CDLL(library)  # lk_create, lk_start, lk_wait, lk_stop: may block, so the GIL is released
        # WHY PyDLL for push/take/step: they never block, and CDLL's release and re-take of the GIL could wait a
        # whole 5 ms switch interval behind a spinning Python thread (CONTRACT §6.3).
        self.hot = ctypes.PyDLL(library)
        self.calls.lk_create.restype = ctypes.c_void_p
        self.calls.lk_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
        self.calls.lk_destroy.argtypes = [ctypes.c_void_p]

    def activate(self, now: float) -> None:
        import ctypes

        error = ctypes.create_string_buffer(1024)
        params = json.dumps(self.component._lk_params()).encode()
        box = self.calls.lk_create(params, error, len(error))
        if not box:
            raise RuntimeError(f"{path(self.component)}: lk_create refused: {error.value.decode(errors='replace')}")
        self.calls.lk_destroy(box)
        raise NotImplementedError(
            f"{path(self.component)}: running a native .so in-process lands with the C++ "
            "runtime (CONTRACT §12, M2); describe() and the checks already cover it"
        )

    def deactivate(self) -> None:
        pass


class Run:
    """A started deployment in this process."""

    def __init__(self, plan) -> None:
        from robot_msgs.std import DiagnosticRecord, Header, Metrics

        self.records = (Header, DiagnosticRecord, Metrics)
        self.plan, self.lockstep, self.stopped = plan, not plan.realtime, False
        self.stopping, self.threads, self.server = threading.Event(), [], None
        self.heap: list = []
        self.sequence = itertools.count()
        self.epoch0 = self.mono0 = 0.0
        self.boxes = {
            id(leaf): Box(leaf) for group in plan.groups if group.language == "python" for leaf in group.order
        }
        for box in self.boxes.values():
            box.state.reset()
        self.groups = [
            (Group if placed.language == "python" else NativeGroup)(self, placed, index)
            for index, placed in enumerate(plan.groups)
        ]
        group_of = {
            id(leaf): group for group, placed in zip(self.groups, plan.groups, strict=True) for leaf in placed.order
        }
        for edge in plan.graph.edges:
            self._connect(edge, group_of[id(edge.src)], group_of[id(edge.dst)])
        self.inputs = {
            name: [self._slot(ref.owner, ref.name, group_of[id(ref.owner)], None) for ref in refs]
            for name, refs in plan.graph.exposed_in.items()
        }  # what an attach client writes
        self.outputs = {}  # what an attach client reads: the newest value of each exposed Out
        for name, ref in plan.graph.exposed_out.items():
            self.outputs[name] = Latest()
            self._box(ref.owner).sinks[ref.name].append(self.outputs[name].write)
        from like_keras.record import Recorder

        self.recorders = []
        for file, channels in plan.recordings:
            self.recorders.append(Recorder(file))
            for leaf, port, topic in channels:
                kind = message_type(type(leaf)._lk_ports[port].type)
                self._box(leaf).sinks[port].append(self.recorders[-1].channel(topic, kind))
        for box in self.boxes.values():
            box.state.log_sinks = tuple(box.sinks["log"])

    def _box(self, leaf) -> Box:
        if id(leaf) not in self.boxes:
            raise NotImplementedError(f"{path(leaf)}: a native component's data path lands with the C++ runtime (M2)")
        return self.boxes[id(leaf)]

    def _slot(self, leaf, name: str, consumer, producer):
        """The slot an input reads: latest or queue(n); strict (sent < now) unless written earlier in this step."""
        box, port = self._box(leaf), type(leaf)._lk_ports[name]
        same = consumer is producer
        if port.queue:
            wake = None if same or consumer.hz is not None else consumer.notify
            slot = Queue(port.queue, self.lockstep, wake)
            box.queues.append((name, slot, not same and consumer.hz is not None))
            return slot
        slot = Latest()
        box.latest.append((name, slot, port.lagged or not same, port.stale))
        return slot

    def _connect(self, edge, producer, consumer) -> None:
        slot = self._slot(edge.dst, edge.dst_port, consumer, producer)
        writer = self._box(edge.src)
        writer.sinks[edge.src_port].append(slot.push if isinstance(slot, Queue) else slot.write)
        if consumer is producer and not type(edge.dst)._lk_ports[edge.dst_port].lagged:
            self._box(edge.dst).upstream.append(writer)

    # ------------------------------------------------------------ running

    def start(self, until: float | None) -> None:
        self.epoch0, self.mono0 = (0.0, 0.0) if self.lockstep else (time.time(), time.monotonic())
        activated = []
        try:
            for group in self.groups:  # §5.7: activate every group's components before any first step
                group.activate(self.epoch0)
                activated.append(group)
        except BaseException:
            for group in reversed(activated):
                group.deactivate()
            self._close()
            raise
        if self.lockstep:
            self._lockstep(until)
            return
        attach = sys.modules.get("like_keras.attach")  # the attach socket, when its module is installed (§9.3)
        if attach is not None and (self.inputs or self.outputs):
            self.server = attach.serve(self)
        workers = [*(group.loop for group in self.groups if isinstance(group, Group)), self._facilities]
        for index, target in enumerate(workers):
            self.threads.append(
                threading.Thread(target=target, name=f"lk-{path(self.plan.system)}-{index}", daemon=True)
            )
            self.threads[-1].start()
        if until is not None:
            self.stopping.wait(until)

    def _lockstep(self, until: float) -> None:
        """One logical clock (§5.5): tick i of an hz group at i / hz, ticks at equal times in place() order, a
        rate-less group at the send time of each message that reaches it, after every tick at that time."""
        heap = self.heap
        for group in self.groups:
            if group.hz:
                heapq.heappush(heap, (0.0, 0, group.index, group.index, 0))
        heapq.heappush(heap, (0.0, -1, 0, 0, 0))  # the facilities, once a logical second
        while heap and heap[0][0] < until:
            now, kind, _, index, tick = heapq.heappop(heap)
            if kind < 0:
                self._publish(now)
                heapq.heappush(heap, (float(tick + 1), -1, 0, 0, tick + 1))
                continue
            group = self.groups[index]
            if not group.dead:
                group.step(now)
            if kind == 0 and not group.dead:
                heapq.heappush(heap, ((tick + 1) / group.hz, 0, index, index, tick + 1))

    def _facilities(self) -> None:
        second = 0
        while not self.stopping.wait(max(0.0, self.mono0 + second - time.monotonic())):
            self._publish(self.epoch0 + second)
            second += 1

    def _publish(self, now: float) -> None:
        """health and metrics of every component, once a second (CONTRACT §8); the recordings written out."""
        for box in self.boxes.values():
            state = box.state
            durations, state.durations = state.durations, []
            lateness, state.period_errors = state.period_errors, []
            stale, state.stale = state.stale, set()
            warning, state.warning = state.warning, None
            overrun, state.overruns_reported = state.overruns - state.overruns_reported, state.overruns
            if box.sinks["health"]:
                record = self._health(box, now, stale, overrun, warning)
                for sink in box.sinks["health"]:
                    sink(record, now)
            if box.sinks["metrics"]:
                record = self._metrics(box, now, durations, lateness)
                for sink in box.sinks["metrics"]:
                    sink(record, now)
        for recorder in self.recorders:
            recorder.flush()

    def _health(self, box: Box, now: float, stale: set[str], overrun: int, warning: str | None):
        state, (header, diagnostic, _) = box.state, self.records
        level, code, message = "ok", "", ""
        if state.error:
            level, code, message = "error", "Exception", state.error
        elif stale:
            level, code, message = "warn", "InputStale", f"stale: {', '.join(sorted(stale))}"
        elif overrun:
            level, code, message = "warn", "Overrun", f"{overrun} ticks skipped"
        elif warning:
            level, code, message = "warn", "WarningLogged", warning
        elif state.mode == "initialization":
            level = "unknown"
        return diagnostic(
            header=header(stamp=now),
            path=path(box.component),
            level=level,
            code=code,
            message=message,
            mode=state.mode,
            expected_period_s=1.0,
            uptime_s=now - state.started if state.mode != "initialization" else 0.0,
            stale_inputs=tuple(sorted(stale)),
        )

    def _metrics(self, box: Box, now: float, durations: list[float], lateness: list[float]):
        state, (header, _, metrics) = box.state, self.records
        values = {
            "steps": state.steps,
            "step_p50_s": _percentile(durations, 50),
            "step_p99_s": _percentile(durations, 99),
            "step_max_s": max(durations, default=math.nan),
            "period_error_p99_s": _percentile(lateness, 99),
            "overruns": state.overruns,
        }
        values |= {f"input_age_s.{name}": state.ages.get(name, math.nan) for name, *_ in box.latest}
        values |= {f"drops.{name}": slot.drops for name, slot, _ in box.queues}
        values |= state.custom
        return metrics(
            header=header(stamp=now),
            path=path(box.component),
            names=tuple(values),
            values=[float(value) for value in values.values()],
        )

    def stop(self) -> None:
        if self.stopped:
            return
        self.stopped = True
        self.stopping.set()
        for group in self.groups:
            group.wake.set()
        for thread in self.threads:
            thread.join()
        if self.server is not None:
            self.server.close()
        for group in reversed(self.groups):
            group.deactivate()
        self._close()

    def _close(self) -> None:
        for box in self.boxes.values():
            box.state.log_sinks = ()
        for recorder in self.recorders:
            recorder.close()


def _percentile(values: list[float], percent: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * percent / 100))]
