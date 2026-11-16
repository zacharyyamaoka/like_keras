"""The in-process backend (CONTRACT §9.1): every group a thread, every edge between groups a lock-free slot, and
under lockstep one logical clock stepping every group in time order on the calling thread (§5.5).

Every `process` collapses into this one, with the same slot semantics as across processes, so moving a component
between processes is one edit of `process=` or of `backend=`, never both.
"""

from __future__ import annotations

import collections
import functools
import heapq
import itertools
import json
import logging
import math
import struct
import sys
import threading
import time
import types
from typing import Any

from like_keras.backends import address  # noqa: F401  (part of the backend's surface: describe() prints it)
from like_keras.component import FACILITY_PORTS, message_type, path

NAME = "inproc"
SEVERITY = {"ok": 0, "unknown": 1, "warn": 2, "error": 3}  # what "the worse" of two health levels means (§15 A22)
VERSION = "0.1.0"
CAPABILITIES = ("latest", "queue", "lagged", "lockstep", "mixed_process")


def start(plan, until: float | None):
    run = Run(plan)
    try:
        run.start(until)
    except BaseException:  # WHY: a signal (CONTRACT §15 A18) or Ctrl-C may unwind a lockstep run; never leave it half up
        run.stop()
        raise
    return run


class Latest:
    """latest (CONTRACT §5.4): the newest value, its send time and its write count, and the same of the one before
    it, so a reader stepping at t sees only what was sent strictly before t. One tuple swap per write: no lock, no
    wait on either side. WHY a count: In(fresh=True) (§15 A25) asks whether a write arrived since the last step,
    which a send time cannot say when one writer sends twice at the same logical time."""

    __slots__ = ("cell",)

    def __init__(self) -> None:
        self.cell = (None, -math.inf, 0, None, -math.inf, 0)

    def write(self, value: Any, sent: float) -> None:
        current = self.cell
        written = current[2] + 1
        self.cell = (value, sent, written, *current[:3]) if sent > current[1] else (value, sent, written, *current[3:])

    def read(self, now: float, strict: bool) -> tuple[Any, float, int]:
        cell = self.cell
        return cell[:3] if not strict or cell[1] < now else cell[3:]


class Queue:
    """queue(n) (CONTRACT §5.4): drop-oldest beyond n, drops counted; one per edge (single producer, single
    consumer). WHY `arriving` under lockstep: a message sent at t must not push out one the reader stepping at t
    is still due; it joins the queue only once a step strictly after t could see it."""

    __slots__ = ("arriving", "drops", "lockstep", "ready", "wake")

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
        self.fresh: dict[str, int] = {}  # In(fresh=True) inputs -> the write count forward saw last (§15 A25)
        self.sinks = {name: [] for name, port in type(component)._lk_ports.items() if port.output}
        self.upstream: list[Box] = []  # same-group writers: in a rate-less group they let this one step (§5.2)

    def ready(self, now: float) -> bool:
        return all(slot.has(now, strict) for _, slot, strict in self.queues)

    def step(self, now: float) -> None:
        state = self.state
        inputs = {name: slot.take(now, strict) for name, slot, strict in self.queues}
        fresh = self.fresh
        for name, slot, strict, stale in self.latest:
            value, sent, written = slot.read(now, strict)
            if value is not None:
                state.ages[name] = now - sent
                if stale is not None and now - sent > stale:  # §4.1: delivered as None, reported in health
                    value = None
                    state.stale.add(name)
            if name in fresh:  # §15 A25: None unless a write arrived since the last step
                value, fresh[name] = (value if written != fresh[name] else None), written
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
        for box in self.boxes:
            box.state.period = 1.0 / placed.hz if placed.hz else 0.0  # period(), from activate() on (§15 A20)
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
        run, tick, not_before = self.run, 0, -math.inf
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
            if not _sleep_until(max(deadline, not_before), run.stopping):
                return
            started = time.monotonic()
            for box in self.boxes:
                box.state.period_errors.append(started - deadline)
            self.step(run.epoch0 + tick / self.hz)
            # WHY skip only past deadlines: §5.2 skips and counts the MISSED ticks. Round 1 also gave up the next tick
            # after a forward that began over half a period late, and counted it, though its deadline had not passed:
            # Overrun in 23 of 30 health periods at 50 Hz on the loaded host (verifier, M1 fix round 2).
            ended_tick = (time.monotonic() - run.mono0) * self.hz
            following = max(tick + 1, math.floor(ended_tick) + 1)
            for box in self.boxes:
                box.state.overruns += following - tick - 1
            tick = following
            # WHY never a burst by delay: the next forward waits for its deadline AND half a period after this one's
            # forward began. WHY the forward's start, not the wake-up: a thread preempted between waking and forward
            # (8.7 ms at 100 Hz, rack powered) stepped again 0.61 ms later (tests/test_inproc_ticks.py).
            began = started if self.entered is None else self.entered
            not_before = began + 0.5 / self.hz


def _sleep_until(moment: float, stopping: threading.Event) -> bool:
    """Wait until time.monotonic() reaches `moment`; False when the run is stopping. WHY a loop: a timed wait that
    returns a hair early would start a step before its deadline, or within half a period of the last (§5.2)."""
    while (remaining := moment - time.monotonic()) > 0:
        if stopping.wait(remaining):
            return False
    return not stopping.is_set()


def _deactivate(box: Box) -> None:
    if box.state.mode != "operational":
        return
    box.state.mode = "offline"
    try:
        box.component.deactivate()
    except Exception as error:  # deactivate must not raise; if it does, say so and keep shutting down
        box.component.log.error("deactivate raised: %s", error, exc_info=error)


class NativeGroup:
    """A native .so's System on its own RT thread inside this process, driven through its C ABI (CONTRACT §6.3, §9.1).
    Python never runs on that thread and that thread never takes the GIL: Python reaches the box only through
    lk_push and lk_take (lk_step under lockstep), which never block, and a pump thread hands its outputs to readers."""

    def __init__(self, run: Run, placed, index: int) -> None:
        import ctypes

        (self.component,) = placed.order
        self.run, self.index, self.hz, self.placed, self.dead, self.last = run, index, placed.hz, placed, False, 0.0
        self.wake, self.box, self.refusals = threading.Event(), None, 0
        ports, cls = type(self.component)._lk_description["ports"], type(self.component)
        self.ins = [name for name, spec in ports.items() if spec["dir"] == "in"]
        self.queued = {name for name in self.ins if ports[name]["queue"]}
        self.outs = [name for name, spec in ports.items() if spec["dir"] == "out"] + list(FACILITY_PORTS)
        self.types = [message_type(cls._lk_ports[name].type) for name in self.outs]
        self.sinks = {name: [] for name in self.outs}
        self.waiting: dict[str, list] = {name: [] for name in self.ins}  # lockstep: (sent, value) not yet visible
        self.calls = ctypes.CDLL(cls._lk_impl)  # lk_create, lk_start, lk_wait, lk_stop: they block, the GIL is released
        # WHY PyDLL for push/take/step: they never block, and CDLL's release and re-take of the GIL could wait a
        # whole 5 ms switch interval behind a spinning Python thread (CONTRACT §6.3).
        self.hot = ctypes.PyDLL(cls._lk_impl)
        pointer, text = ctypes.c_void_p, ctypes.c_char_p
        for function, argtypes, restype in (
            (self.calls.lk_create, [text, text, ctypes.c_size_t], pointer),
            (self.calls.lk_start, [pointer, pointer, text, ctypes.c_size_t], ctypes.c_int),
            (self.calls.lk_wait, [pointer, ctypes.c_int], ctypes.c_int),
            (self.calls.lk_stop, [pointer], None),
            (self.calls.lk_destroy, [pointer], None),
            (self.hot.lk_step, [pointer, ctypes.c_double], ctypes.c_int),
            (self.hot.lk_push, [pointer, ctypes.c_uint32, pointer], ctypes.c_int),
            (self.hot.lk_take, [pointer, ctypes.c_uint32, pointer], ctypes.c_int),
        ):
            function.argtypes, function.restype = argtypes, restype

    def activate(self, now: float) -> None:
        import ctypes

        error = ctypes.create_string_buffer(1024)
        box = self.calls.lk_create(json.dumps(self.component._lk_params()).encode(), error, len(error))
        if not box:
            raise RuntimeError(f"{path(self.component)}: lk_create refused: {error.value.decode(errors='replace')}")
        cpus = list(self.placed.cpus or [])
        place = _arrow().Place(
            self.hz or 0.0,
            (ctypes.c_int * len(cpus))(*cpus) if cpus else None,
            len(cpus),
            self.placed.fifo or 0,
            0,
            int(self.run.lockstep),
        )
        if self.calls.lk_start(box, ctypes.byref(place), error, len(error)) != 0:
            self.calls.lk_destroy(box)
            raise RuntimeError(f"{path(self.component)}: {error.value.decode(errors='replace')} (CONTRACT §5.7)")
        self.box = box

    def deactivate(self) -> None:
        box = self.box
        if box:
            self.calls.lk_stop(box)  # the RT thread joined, the components deactivated in reverse order
            # WHY once more (CONTRACT §15 A28): what the box logged in its last partial second and in deactivate(),
            # and its last health and metrics, on every stop: at the last step's time under lockstep, else now
            now = self.last if self.run.lockstep else self.run.epoch0 + time.monotonic() - self.run.mono0
            self._drain(now, len(self.outs), len(self.outs) - len(FACILITY_PORTS))
            self.box = None
            self.calls.lk_destroy(box)

    def receive(self, name: str):
        """The writer of one exposed input: under lockstep held until a step strictly after its send time (a
        rate-less group's queue: at it), in real time pushed at once from the writer's thread."""
        port, queued = self.ins.index(name), name in self.queued

        def write(value, sent: float) -> None:
            if self.run.lockstep:
                self.waiting[name].append((sent, value))
                if queued and self.hz is None:
                    heapq.heappush(self.run.heap, (sent, 1, next(self.run.sequence), self.index, 0))
            elif self.box is not None:
                self._push(port, value)

        return write

    def _push(self, port: int, value) -> None:
        root = 0 if value is None else _export(value)  # None: retracted (CONTRACT §5.4)
        if self.hot.lk_push(self.box, port, root) < 0 and not self.refusals:
            self.refusals += 1
            self.component.log.error(
                "%s refused a %s on %s", path(self.component), type(value).__name__, self.ins[port]
            )

    def step(self, now: float) -> bool:
        """Lockstep: deliver what is due, one step through lk_step on this thread, hand on what it sent at `now`."""
        if self.dead or self.box is None:
            return False
        self.last = now
        for port, name in enumerate(self.ins):
            waiting, strict = self.waiting[name], not (name in self.queued and self.hz is None)
            due = 0
            while due < len(waiting) and (waiting[due][0] < now or not strict and waiting[due][0] == now):
                due += 1
            for _, value in waiting[:due] if name in self.queued else waiting[due - 1 : due]:
                self._push(port, value)
            del waiting[:due]
        if self.hot.lk_step(self.box, now) != 0:
            self.dead = True  # §5.7: the box deactivated itself; its health says why
        self._drain(now, len(self.outs) - len(FACILITY_PORTS))
        return False

    def loop(self) -> None:
        """Real time: the box steps on its own RT thread; this thread waits on its wake and hands outputs on."""
        run = self.run
        while not run.stopping.is_set() and self.box is not None:
            if self.calls.lk_wait(self.box, 100) > 0:
                self._drain(run.epoch0 + time.monotonic() - run.mono0, len(self.outs) - len(FACILITY_PORTS))

    def publish(self, now: float) -> None:
        """The facilities thread (CONTRACT §8): the box's log, health and metrics records, under this path."""
        if self.box is not None:
            self._drain(now, len(self.outs), len(self.outs) - len(FACILITY_PORTS))

    def _drain(self, sent: float, end: int, begin: int = 0) -> None:
        import ctypes

        # WHY a destination per call, never one per group: the pump thread (loop) and the facilities thread (publish)
        # drain at once, and a shared struct let one thread's lk_take overwrite the other's message between its take
        # and its release: a double release, heap-use-after-free under ASan, and the always-on node's startup
        # SIGSEGV (6 in 102 starts at 8f09ce7, rack powered, cores 22-23).
        received = _arrow().Array()  # where lk_take moves each output (it is copied out and released at once)
        taken, facility = ctypes.addressof(received), len(self.outs) - len(FACILITY_PORTS)
        for port in range(begin, end):
            name = self.outs[port]
            if port >= facility and name != "log" and not self.sinks[name]:
                continue  # nobody reads it: the box need not build the records
            while self.hot.lk_take(self.box, port, taken) == 1:
                try:
                    value = _import(taken, self.types[port], keep=bool(self.sinks[name]) or name == "log")
                except ValueError as refused:  # robot_msgs refused what the box sent: drop it, say so once
                    if not self.refusals:
                        self.component.log.error(
                            "%s sent an invalid %s on %s: %s",
                            path(self.component),
                            self.types[port].type_name,
                            name,
                            refused,
                        )
                    self.refusals += 1
                    continue
                if value is None:
                    continue
                if port >= facility:  # the box names its components; the records carry the full path
                    value = value.replace(path=".".join(filter(None, (path(self.component), value.path))))
                if port > facility and self.run.lockstep:  # health and metrics, never a log line (wall time)
                    value = _at_tick(value, sent)
                if name == "log" and value.level >= logging.WARNING:  # as a Python component's (component._Log)
                    logging.getLogger("like_keras").log(value.level, "%s: %s", value.path, value.message)
                for sink in self.sinks[name]:
                    sink(value, sent)


def _at_tick(record, tick: float):
    """A native box's health or metrics record stamped at its facility tick, uptime_s shifted with it.
    WHY (CONTRACT §15 A10): under lockstep the box stamps its last step's time (0.98 s at the tick 1.0, at 50 Hz)
    where a Python component is stamped at the tick, so a Python <-> C++ swap shifted every stamp by one period.
    The native process host does the same (cpp/hosts, restamp)."""
    shift = tick - record.header.stamp
    if getattr(record, "mode", None) == "operational":  # the box sends uptime 0 unless operational
        record = record.replace(uptime_s=record.uptime_s + shift)
    return record.replace(header=record.header.replace(stamp=tick))


@functools.cache
def _arrow():
    """The two C structs of CONTRACT §6.3 through ctypes, and the release an exported message carries."""
    import ctypes

    class Array(ctypes.Structure):  # the Arrow C Data Interface's ArrowArray
        _fields_ = [(name, ctypes.c_int64) for name in ("length", "null_count", "offset", "n_buffers", "n_children")]
        _fields_ += [
            (name, ctypes.c_void_p) for name in ("buffers", "children", "dictionary", "release", "private_data")
        ]

    class Place(ctypes.Structure):  # lk_place
        _fields_ = [
            ("hz", ctypes.c_double),
            ("cpus", ctypes.POINTER(ctypes.c_int)),
            ("ncpus", ctypes.c_int),
            ("fifo", ctypes.c_int),
            ("lock_memory", ctypes.c_int),
            ("stepped", ctypes.c_int),
        ]

    alive: dict[int, list] = {}

    @ctypes.CFUNCTYPE(None, ctypes.c_void_p)
    def release(root):
        # WHY here and never on the RT thread: the box hands a consumed message back to a host thread (lk_push,
        # lk_take, lk_step or lk_destroy), and only there does this drop the references that kept its buffers alive.
        words = (ctypes.c_int64 * 10).from_address(root)
        alive.pop(words[9], None)
        words[8] = 0

    caller = ctypes.CFUNCTYPE(None, ctypes.c_void_p)
    return types.SimpleNamespace(
        Array=Array,
        Place=Place,
        alive=alive,
        release=release,
        keys=itertools.count(1),
        release_address=ctypes.cast(release, ctypes.c_void_p).value,
        caller=caller,
    )


def _export(value) -> int:
    """`value` as an Arrow C Data struct array of length 1 (CONTRACT §7.1) whose array buffers ARE its frozen numpy
    arrays (no copy, §6.3); the address of its root, which lk_push moves into the box."""
    import numpy as np

    from like_keras import codec

    cls, nodes = type(value), []
    _one(codec.Node("", "+s", children=codec.schema(cls), message=cls), value, nodes, codec, np)
    buffers = [buffer for node in nodes for buffer in node[2]]
    layout = _layout(cls, nodes, len(buffers))
    # WHY one block for the small buffers (validity, offsets, scalars, strings), and the node table from the type's
    # cached layout with three vector writes (CONTRACT §15 A26): building the table field by field cost more than
    # the walk itself, 50 of 73 us p50 for a 7 x 1000 plan (rack powered, cores 22-23, not the timing tier).
    pointers, small, arrays, parts, end = [0] * len(buffers), [], [], [], 0
    for at, buffer in enumerate(buffers):
        if type(buffer) is np.ndarray:
            if buffer.nbytes:
                arrays.append(buffer)
                pointers[at] = buffer.__array_interface__["data"][0]
        elif buffer:
            small.append(at)
            pointers[at] = end  # a spot in the block, resolved once the block has an address
            pad = -len(buffer) % 8  # 8-byte aligned, as Arrow buffers are
            parts += (buffer, PADDING[pad])
            end += len(buffer) + pad
    held = np.frombuffer(b"".join(parts) or bytes(8), np.uint8)
    pointers = np.array(pointers, np.int64)
    pointers[small] += held.__array_interface__["data"][0]
    words = layout.template.copy()
    base = words.__array_interface__["data"][0]
    words[layout.relative] += base
    words[layout.lengths] = [node[0] for node in nodes]
    words[layout.nulls] = [node[1] for node in nodes]
    words[layout.pointers] = pointers
    arrow = _arrow()
    key = words[9] = next(arrow.keys)  # the root's private_data: what its release drops
    arrow.alive[key] = (words, held, arrays)
    return base


_LAYOUTS: dict[tuple, types.SimpleNamespace] = {}
PADDING = tuple(bytes(size) for size in range(8))


def _layout(cls, nodes: list, buffer_count: int) -> types.SimpleNamespace:
    """A message type's ArrowArray table with every field that never changes filled in: the counts, the release,
    the buffer and child pointers relative to the table's start. WHY per type: a type's nodes have one shape
    whatever its values (a None field still has its node, empty), so only lengths, null counts and buffer
    addresses differ from one message to the next. The key also holds the counts, so a shape never seen is built."""
    import numpy as np

    key = (cls, len(nodes), buffer_count)
    if (layout := _LAYOUTS.get(key)) is not None:
        return layout
    count = len(nodes)
    template = np.zeros(10 * count + buffer_count + sum(len(node[3]) for node in nodes), np.int64)
    relative, buffer_at, kid_at = [], 0, 0
    for index, (_, _, parts, kids) in enumerate(nodes):
        row = template[10 * index : 10 * index + 10]
        row[3:6] = len(parts), len(kids), 80 * count + 8 * buffer_at
        row[8] = _arrow().release_address
        relative.append(10 * index + 5)
        if kids:
            row[6] = 80 * count + 8 * buffer_count + 8 * kid_at
            relative.append(10 * index + 6)
        for kid in kids:
            template[10 * count + buffer_count + kid_at] = 80 * kid
            relative.append(10 * count + buffer_count + kid_at)
            kid_at += 1
        buffer_at += len(parts)
    rows = np.arange(count) * 10
    layout = types.SimpleNamespace(template=template, relative=np.array(relative), lengths=rows, nulls=rows + 1,
                                   pointers=np.arange(10 * count, 10 * count + buffer_count))
    _LAYOUTS[key] = layout
    return layout


_CONSTANT: dict[tuple, tuple] = {}  # (schema node id, which values are None, start) -> that subtree's nodes


def _nodes(node, values: list, nodes: list, codec, np) -> int:
    """codec's layout (its _encode, field for field) as Arrow nodes over the values' own buffers: (length,
    null count, buffers, child indices), depth first. A buffer is bytes, or a numpy array sent as it is."""
    # WHY a cache: a subtree whose values are all None, or empty lists and maps, is the same Arrow nodes every time,
    # yet the walk built them anew for each plan: limits, channels, accelerations and effort are 26 of a 7 x 1000
    # plan's 42 nodes (contract-005 §5; A2 and T2 allow 0.1 ms per push, M2 verifier round 1). The buffers are the
    # same bytes, so the layout is unchanged (kpi/golden, T10).
    if node.children and all(value is None or node.format != "+s" and len(value) == 0 for value in values):
        # WHY the start index in the key: a subtree sits at the same place in every message of its type, so its
        # cached nodes carry their absolute child indices and are appended as they are (CONTRACT §15 A26)
        base = len(nodes)
        key = (id(node), tuple(value is None for value in values), base)
        if (subtree := _CONSTANT.get(key)) is None:
            built: list = []
            _walk(node, values, built, codec, np)
            subtree = _CONSTANT[key] = tuple(
                (
                    length,
                    nulls,
                    tuple(b"" if isinstance(part, np.ndarray) else part for part in parts),
                    tuple(base + kid for kid in kids),
                )
                for length, nulls, parts, kids in built
            )  # an empty numpy buffer and b"" both export as a null pointer
        nodes += subtree
        return base
    return _walk(node, values, nodes, codec, np)


def _one(node, value, nodes: list, codec, np) -> int:
    """_nodes for one value, an exported message: the same nodes, without building a column per field. WHY: the walk
    over columns of one cost 24 of a 7 x 1000 plan's ~50 us export (CONTRACT §15 A26; rack powered, cores 22-23)."""
    kind = node.format
    if node.children and (value is None or kind != "+s" and len(value) == 0) or kind in ("b", "+m"):
        return _nodes(node, [value], nodes, codec, np)  # the cached constant subtrees, and the rare kinds
    index = len(nodes)
    present = value is not None
    entry = (1, 0 if present else 1, [b"" if present else b"\x00"], [])
    nodes.append(entry)
    buffers, kids = entry[2], entry[3]
    if kind in codec.NUMBER:  # struct packs a float's bits as they are: a NaN keeps its payload
        buffers.append(struct.pack("<d" if kind == "g" else "<q", value if present else 0))
    elif kind == "u":
        encoded = value.encode() if present else b""
        buffers += [struct.pack("<2i", 0, len(encoded)), encoded]
    elif kind == "+l":
        item = node.children[0]
        if item.format in codec.NUMBER:  # WHY ascontiguousarray: a frozen C-order array comes back as itself
            data = np.ascontiguousarray(value, codec.NUMBER[item.format]).ravel()
            buffers.append(struct.pack("<2i", 0, len(data)))
            kids.append(len(nodes))
            nodes.append((len(data), 0, [None, data], []))
        else:
            buffers.append(struct.pack("<2i", 0, len(value)))
            kids.append(_nodes(item, list(value), nodes, codec, np))
    else:  # +s
        for child in node.children:
            column = getattr(value, child.name) if present else None if child.nullable else codec._zero(child)
            kids.append(_one(child, column, nodes, codec, np))
    return index


def _walk(node, values: list, nodes: list, codec, np) -> int:
    index = len(nodes)
    entry = (len(values), sum(value is None for value in values), [codec._validity(values)], [])
    nodes.append(entry)
    kind, buffers, kids = node.format, entry[2], entry[3]
    if kind in codec.NUMBER:  # struct packs a float's bits as they are: a NaN keeps its payload
        buffers.append(
            struct.pack(
                f"<{len(values)}{'d' if kind == 'g' else 'q'}", *(0 if value is None else value for value in values)
            )
        )
    elif kind == "b":
        buffers.append(np.packbits(np.array([bool(value) for value in values]), bitorder="little").tobytes())
    elif kind == "u":
        encoded = [b"" if value is None else value.encode() for value in values]
        buffers += [codec._offsets(map(len, encoded)), b"".join(encoded)]
    elif kind == "+l":
        item, rows = node.children[0], [() if value is None else value for value in values]
        if item.format in codec.NUMBER:  # WHY ascontiguousarray: a frozen C-order array comes back as itself
            arrays = [np.ascontiguousarray(row, codec.NUMBER[item.format]).ravel() for row in rows]
            buffers.append(codec._offsets(map(len, arrays)))
            data = arrays[0] if len(arrays) == 1 else np.concatenate([np.empty(0, codec.NUMBER[item.format]), *arrays])
            kids.append(len(nodes))
            nodes.append((len(data), 0, [None, data], []))
        else:
            buffers.append(codec._offsets(map(len, rows)))
            kids.append(_nodes(item, [element for row in rows for element in row], nodes, codec, np))
    elif kind == "+s":
        for child in node.children:
            column = [
                getattr(value, child.name) if value is not None else None if child.nullable else codec._zero(child)
                for value in values
            ]
            kids.append(_nodes(child, column, nodes, codec, np))
    elif kind == "+m":
        rows = [list(value.items()) if value is not None else [] for value in values]
        buffers.append(codec._offsets(map(len, rows)))
        pairs = [pair for row in rows for pair in row]
        key, item = node.children[0].children
        kids.append(len(nodes))
        entries = (len(pairs), 0, [None], [])
        nodes.append(entries)
        entries[3].extend(
            (
                _nodes(key, [name for name, _ in pairs], nodes, codec, np),
                _nodes(item, [element for _, element in pairs], nodes, codec, np),
            )
        )
    return index


def _import(root_at: int, cls, keep: bool = True):
    """The message lk_take moved to `root_at`, copied out (a per-joint message is small) and released at once."""
    import ctypes

    from like_keras import codec

    arrow = _arrow()
    root = arrow.Array.from_address(root_at)
    try:
        if not keep:
            return None
        parts: list[bytes] = []
        node = codec.Node("", "+s", children=codec.schema(cls), message=cls)
        _parts(node, root, parts, arrow, ctypes)
        return codec._decode(node, root.length, iter(parts))[0]
    finally:
        if root.release:
            arrow.caller(root.release)(root_at)


def _parts(node, array, parts: list, arrow, ctypes) -> None:
    """The array's buffers, depth first, each its exact size, in the order codec._decode reads an LKF frame."""
    length, buffers = array.length, (ctypes.c_void_p * array.n_buffers).from_address(array.buffers)
    if array.offset:
        raise ValueError("an Arrow array with an offset: the box's encoder never writes one")

    def read(at: int, size: int) -> bytes:
        return ctypes.string_at(buffers[at], size) if size and buffers[at] else b""

    parts.append(read(0, (length + 7) // 8) if array.null_count else b"")
    kind = node.format
    if kind in ("g", "l"):
        parts.append(read(1, 8 * length))
    elif kind == "b":
        parts.append(read(1, (length + 7) // 8))
    elif kind == "u":
        offsets = read(1, 4 * (length + 1))
        parts += [offsets, read(2, int.from_bytes(offsets[-4:], "little", signed=True))]
    elif kind in ("+l", "+m"):
        parts.append(read(1, 4 * (length + 1)))
    children = (ctypes.c_void_p * array.n_children).from_address(array.children) if array.n_children else ()
    for child, child_address in zip(node.children, children, strict=True):
        _parts(child, arrow.Array.from_address(child_address), parts, arrow, ctypes)


class Run:
    """A started deployment in this process."""

    def __init__(self, plan) -> None:
        from robot_msgs.std import DiagnosticRecord, Header, Metrics

        self.records = (Header, DiagnosticRecord, Metrics)
        self.plan, self.lockstep, self.stopped = plan, not plan.realtime, False
        self.stopping, self.threads, self.server = threading.Event(), [], None
        self.heap: list = []
        self.slots: dict[tuple[int, str], Latest | Queue] = {}  # (leaf, input) -> the one slot it reads
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
        self.natives = {id(group.component): group for group in self.groups if isinstance(group, NativeGroup)}
        for edge in plan.graph.edges:
            self._connect(edge, group_of[id(edge.src)], group_of[id(edge.dst)])
        self.inputs = {
            name: [self._slot(ref.owner, ref.name, group_of[id(ref.owner)], None) for ref in refs]
            for name, refs in plan.graph.exposed_in.items()
        }  # what an attach client writes
        self.outputs = {}  # what an attach client reads: the newest value of each exposed Out
        for name, ref in plan.graph.exposed_out.items():
            self.outputs[name] = Latest()
            self.subscribe(ref.owner, ref.name, self.outputs[name].write)
        from like_keras.record import Recorder

        self.recorders = []
        for file, channels in plan.recordings:
            self.recorders.append(Recorder(file))
            for leaf, port, topic in channels:
                kind = message_type(type(leaf)._lk_ports[port].type)
                self.subscribe(leaf, port, self.recorders[-1].channel(topic, kind))
        for box in self.boxes.values():
            box.state.log_sinks = tuple(box.sinks["log"])

    def subscribe(self, leaf, port: str, sink) -> None:
        """`sink(value, sent)` gets every value `leaf` sends on `port`, a facility port (log, health, metrics) and a
        native box's included, on the thread that sends it; it must hand the value over and return, never block.
        WHY one seam (CONTRACT §15 A11): attach, `lk top` and a transport backend's Python host subscribe here, so
        a rework of the boxes (M4) keeps them working."""
        box = self._box(leaf)
        box.sinks[port].append(sink)
        if port == "log" and isinstance(box, Box):  # a Python logger reads its sinks from its state (component._Log)
            box.state.log_sinks = tuple(box.sinks["log"])

    def _box(self, leaf) -> Box | NativeGroup:
        """Where a leaf's outputs go: its Box, or for a native .so its group (`sinks` by port either way)."""
        return self.boxes[id(leaf)] if id(leaf) in self.boxes else self.natives[id(leaf)]

    def _slot(self, leaf, name: str, consumer, producer):
        """The slot an input reads: latest or queue(n); strict (sent < now) unless written earlier in this step. A
        native box keeps its own slots (CONTRACT §5.1): its input is a push through the C ABI."""
        if id(leaf) in self.natives:
            write = self.natives[id(leaf)].receive(name)
            return types.SimpleNamespace(write=write, push=write)
        box, port = self._box(leaf), type(leaf)._lk_ports[name]
        same = consumer is producer
        wake = None if same or consumer.hz is not None else consumer.notify
        strict = not same and consumer.hz is not None if port.queue else port.lagged or not same
        if (slot := self.slots.get((id(leaf), name))) is not None:
            # WHY one slot for several writers: only a System's own facility port fans in (CONTRACT §15 A21), and a box
            # reads one slot per input. Strict if any writer is in another group; woken if any writer can wake it.
            for entries in (box.queues, box.latest):
                entries[:] = [(*entry[:2], entry[2] or strict, *entry[3:]) if entry[0] == name else entry
                              for entry in entries]
            if port.queue and slot.wake is None:
                slot.wake = wake
            return slot
        if port.queue:
            slot = Queue(port.queue, self.lockstep, wake)
            box.queues.append((name, slot, strict))
        else:
            slot = Latest()
            box.latest.append((name, slot, strict, port.stale))
            if port.fresh:
                box.fresh[name] = 0
        self.slots[(id(leaf), name)] = slot
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
        workers = [*(group.loop for group in self.groups), self._facilities]
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
            logged, state.logged_error = state.logged_error, None
            overrun, state.overruns_reported = state.overruns - state.overruns_reported, state.overruns
            if box.sinks["health"]:
                record = self._health(box, now, stale, overrun, warning, logged)
                for sink in box.sinks["health"]:
                    sink(record, now)
            if box.sinks["metrics"]:
                record = self._metrics(box, now, durations, lateness)
                for sink in box.sinks["metrics"]:
                    sink(record, now)
        for native in self.natives.values():
            native.publish(now)
        for recorder in self.recorders:
            recorder.flush()

    def _health(self, box: Box, now: float, stale: set[str], overrun: int, warning: str | None, logged: str | None):
        state, (header, diagnostic, _) = box.state, self.records
        level, code, message = "ok", "", ""
        if state.error:
            level, code, message = "error", "Exception", state.error
        elif logged:  # CONTRACT §15 A22: until a round in which no error was logged
            level, code, message = "error", "ErrorLogged", logged
        elif stale:
            level, code, message = "warn", "InputStale", f"stale: {', '.join(sorted(stale))}"
        elif overrun:
            level, code, message = "warn", "Overrun", f"{overrun} ticks skipped"
        elif warning:
            level, code, message = "warn", "WarningLogged", warning
        elif state.mode == "initialization":
            level = "unknown"
        # CONTRACT §15 A22, as the C++ runtime: the worse of the component's own health(level, code) and the
        # facility's; on a tie the component's code says more, but an Exception always wins (it no longer runs)
        if state.own is not None and code != "Exception" and SEVERITY[state.own[0]] >= max(SEVERITY[level], 1):
            (level, code), message = state.own, ""
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
