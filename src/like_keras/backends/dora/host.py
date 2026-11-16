"""A Python process of a dora deployment (CONTRACT §9.2): `python -m like_keras.backends.dora.host <plan.json> <process>`.

It rebuilds its components as `module:qualname(**params)`, runs one thread per group with the inproc runtime (the
same slots, stepping and facilities), receives on its own RX thread (`node.next(timeout=0.1)`, which releases the
GIL) and sends from its own TX thread: no group thread calls into dora.

Under lockstep (§5.5) the process steps its groups on one logical clock and crosses processes conservatively: every
frame carries its sender's step time `lk_t` (and `lk_h`, the time before which the sender will send nothing more on
that output); an output with nothing to send that step gets an empty frame carrying that promise. The process runs
an event at t only once every input has promised everything before t, so a value sent at t is seen at the first
step strictly after t, never earlier, never later.

`Link` (the dora node, the lk_hello handshake, Arrow in and out) is shared with the gateway.
"""

from __future__ import annotations

import collections
import heapq
import importlib
import json
import math
import os
import pathlib
import sys
import threading
import time
import traceback
import types
import uuid

HELLO = "lk_hello"
HELLO_TIMEOUT_S = 2.0  # CONTRACT §6.4/§9.2: a peer that never says hello is a refusal within 2 s, never a hang
FOREVER = 1 << 62  # a promise of no more frames, in ns


def ns(seconds: float) -> int:
    return FOREVER if seconds == math.inf else round(seconds * 1e9)


def mark(document: dict, kind: str, node: str, text: str = "") -> None:
    """markers/<run>/<kind>.<node>: what the launcher waits on (started, refused, done: stepped to `until`; exit;
    failed: why). WHY a folder per run: a node of the run before, still exiting, never marks this one."""
    try:
        (pathlib.Path(document["markers"]) / f"{kind}.{node}").write_text(text)
    except OSError:
        pass


def fail(document: dict, node: str, message: str, code: int, final: bool = True) -> None:
    """Refuse loudly and exit; a final refusal (a skewed peer) is marked so a waiting launcher raises at once."""
    print(f"like_keras {node}: {message}", file=sys.stderr, flush=True)
    if final:
        mark(document, "failed", node, f"{node}: {message}")
    mark(document, "exit", node)
    os._exit(code)


# ---------------------------------------------------------------- Arrow in and out (CONTRACT §7.1)


def to_arrow(value):
    """A robot_msgs message as a pyarrow struct array of length 1 whose buffers ARE its frozen numpy arrays (§7.1:
    no copy; dora's copy into its sample is the only one, as with raw dora)."""
    import numpy as np

    from like_keras import codec
    from like_keras.backends import inproc

    root = codec.Node("", "+s", children=codec.schema(type(value)), message=type(value))
    nodes: list = []
    inproc._nodes(root, [value], nodes, codec, np)  # the layout inproc hands a native box: one walk, one source
    return _build(root, 0, nodes)


def _build(node, index: int, nodes: list):
    import pyarrow as pa

    length, nulls, buffers, kids = nodes[index]
    children = _children(node)
    built = [_build(child, kid, nodes) for child, kid in zip(children, kids, strict=True)]
    first, *rest = buffers
    arrow = [None if first is None or not len(first) else pa.py_buffer(first)]
    arrow += [pa.py_buffer(b"" if part is None else part) for part in rest]
    return pa.Array.from_buffers(arrow_type(node), length, arrow, null_count=nulls, children=built or None)


def _children(node) -> tuple:
    return node.children


_TYPES: dict = {}


def arrow_type(node):
    import pyarrow as pa

    if node not in _TYPES:
        kind = node.format
        if kind in ("g", "l", "b", "u"):
            built = {"g": pa.float64(), "l": pa.int64(), "b": pa.bool_(), "u": pa.utf8()}[kind]
        elif kind == "+l":
            (item,) = node.children
            built = pa.list_(pa.field(item.name, arrow_type(item), nullable=item.nullable))
        elif kind == "+m":
            key, item = node.children[0].children
            built = pa.map_(arrow_type(key), pa.field(item.name, arrow_type(item), nullable=item.nullable))
        else:
            built = pa.struct(
                [pa.field(child.name, arrow_type(child), nullable=child.nullable) for child in node.children]
            )
        _TYPES[node] = built
    return _TYPES[node]


def from_arrow(array, cls):
    """The message a received struct array carries: numpy views over the transport's buffers, through
    robot_msgs.adopt (§7.3: no copy; the views keep the dora sample alive through their base chain)."""
    from like_keras import codec

    root = codec.Node("", "+s", children=codec.schema(cls), message=cls)
    parts: list = []
    _parts(root, array, parts)
    return codec._decode(root, len(array), iter(parts))[0]


def _parts(node, array, parts: list) -> None:
    """This array's buffers, each its exact size, depth first: the order codec._decode reads an LKF frame."""
    if array.offset:
        raise ValueError("a received Arrow array with an offset: dora never sends one")
    length, own = len(array), array.buffers()[: array.type.num_buffers]

    def exact(buffer, size: int):
        return memoryview(buffer)[:size] if buffer is not None and size else b""

    parts.append(exact(own[0], (length + 7) // 8) if array.null_count else b"")
    kind = node.format
    if kind in ("g", "l"):
        parts.append(exact(own[1], 8 * length))
    elif kind == "b":
        parts.append(exact(own[1], (length + 7) // 8))
    elif kind == "u":
        offsets = exact(own[1], 4 * (length + 1))
        parts += [offsets, exact(own[2], int.from_bytes(bytes(offsets[-4:]), "little", signed=True) if length else 0)]
    elif kind in ("+l", "+m"):
        parts.append(exact(own[1], 4 * (length + 1)))
        _parts(node.children[0], array.values, parts)
    if kind == "+s":
        for index, child in enumerate(node.children):
            _parts(child, array.field(index), parts)


# ---------------------------------------------------------------- the dora node and the lk_hello handshake


class Link:
    """This process's dora node: init under a watchdog, the lk_hello handshake, the RX thread, sends.

    WHY an acknowledged handshake: a peer counts as ready only once its hello names this incarnation's nonce, so
    nothing is sent before the peer that must receive it is listening (a restarted node starts a new nonce; every
    node answers a nonce it has not seen with its own hello)."""

    def __init__(self, document: dict, name: str) -> None:
        self.document, self.name, self.closed = document, name, False
        self.node_spec = document["nodes"][name]
        self.peers = set(self.node_spec["peers"])
        self.nonce = uuid.uuid4().hex
        self.seen: dict[str, str] = {}  # peer -> the nonce of its hello this node verified
        self.acked: set[str] = set()  # peers whose latest hello names this node's nonce
        self.ready, self.stopping = threading.Event(), threading.Event()
        self.handler, self.buffered = None, []  # frames before the process is built wait here, in order
        self.lock = threading.RLock()
        import faulthandler

        import dora
        from dora import Node

        # WHY faulthandler, not a timer thread: Node() holds the GIL while it waits for the daemon, so no Python
        # thread could fire; faulthandler's watchdog is a C thread (it dumps the stack, then exits 1)
        print(
            f"init_dora_node: a dora {document['versions']['dora']} daemon must answer within {HELLO_TIMEOUT_S} s",
            file=sys.stderr,
        )
        faulthandler.dump_traceback_later(HELLO_TIMEOUT_S, exit=True, file=sys.stderr)
        self.node = Node()
        faulthandler.cancel_dump_traceback_later()
        self.versions = {
            "like_keras": document["versions"]["like_keras"],
            "abi": document["versions"]["abi"],
            "backend": "dora",
            "dora": getattr(dora, "__version__", "unknown"),
        }
        self.ports = {}  # output id -> its schema hash, as robot_msgs computes it here
        from like_keras import codec
        from like_keras.component import message_type

        for output, kind in self.node_spec["outputs"].items():
            self.ports[output] = f"{codec.schema_hash(message_type(kind)):016x}"
        self.expected = {}  # (peer, its output id) -> the hash this node computes for the type it reads there
        for input_id, spec in self.node_spec["inputs"].items():
            self.expected[(spec["source"].split("/")[0], input_id)] = (
                f"{codec.schema_hash(message_type(spec['type'])):016x}"
            )
        self.say_hello()
        self.started = time.monotonic()
        self.rx = threading.Thread(target=self._receive, name=f"lk-rx-{name}", daemon=True)
        self.rx.start()
        if not self.peers:
            self.ready.set()

    def say_hello(self) -> None:
        print(f"lk_hello sent ({self.nonce[:8]}; seen {sorted(self.seen)})", file=sys.stderr)
        hello = {**self.versions, "node": self.name, "nonce": self.nonce, "ports": self.ports, "seen": self.seen}
        self.node.send_output(HELLO, json.dumps(hello).encode())

    def wait_ready(self) -> None:
        """Hold data until every peer's hello verified and named this node (2 s, else exit 3)."""
        if not self.ready.wait(max(0.0, HELLO_TIMEOUT_S - (time.monotonic() - self.started))):
            missing = sorted(self.peers - self.acked)
            mine = f"like_keras {self.versions['like_keras']}, dora {self.versions['dora']}"
            fail(
                self.document,
                self.name,
                f"no verified lk_hello from {missing} within {HELLO_TIMEOUT_S} s "
                f"(this node: {mine}; theirs: unknown, they never answered)",
                3,
                final=False,
            )

    def _hello(self, peer: str, value) -> None:
        theirs = json.loads(bytes(value.to_numpy(zero_copy_only=False)).decode())
        for key in ("like_keras", "abi", "backend", "dora"):
            if theirs.get(key) != self.versions[key]:
                fail(
                    self.document,
                    self.name,
                    f"node {peer} runs {key} {theirs.get(key)}, node {self.name} runs "
                    f"{key} {self.versions[key]}: every node of a deployment runs the same versions",
                    3,
                )
        for (source, output), ours in self.expected.items():
            if source == peer and theirs["ports"].get(output) != ours:
                fail(
                    self.document,
                    self.name,
                    f"{output}: node {peer} sends schema hash {theirs['ports'].get(output)}"
                    f", node {self.name} reads it as {ours}: rebuild against the same robot_msgs",
                    3,
                )
        print(
            f"lk_hello from {peer} ({theirs['nonce'][:8]}; names this node: "
            f"{theirs.get('seen', {}).get(self.name) == self.nonce})",
            file=sys.stderr,
        )
        changed = self.seen.get(peer) != theirs["nonce"]
        self.seen[peer] = theirs["nonce"]
        if theirs.get("seen", {}).get(self.name) == self.nonce:
            self.acked.add(peer)
        if changed:
            self.say_hello()
        if self.acked >= self.peers:
            self.ready.set()

    def _receive(self) -> None:
        while not self.closed:
            event = self.node.next(timeout=0.1)
            if event is None:
                self.stopping.set()
                return
            kind = event["type"]
            if kind == "INPUT":
                ident = event["id"]
                if ident.startswith(HELLO + "."):
                    self._hello(ident[len(HELLO) + 1 :], event["value"])
                else:
                    event["metadata"]["lk_arrived"] = time.monotonic()  # a value's age counts from here (stale=)
                    with self.lock:
                        if self.handler is None:
                            self.buffered.append((ident, event["value"], event["metadata"]))
                        else:
                            self.handler(ident, event["value"], event["metadata"])
            elif kind in ("STOP", "ALL_INPUTS_CLOSED"):
                self.stopping.set()
                return

    def receive_with(self, handler) -> None:
        """From now on frames go to `handler(input id, array, metadata)`, those buffered first."""
        with self.lock:
            for frame in self.buffered:
                handler(*frame)
            self.buffered.clear()
            self.handler = handler

    def send(self, output: str, value, metadata: dict | None = None) -> None:
        if value is None:  # an empty frame: a promise under lockstep, a retraction in real time
            import pyarrow as pa

            array = pa.nulls(0)
        else:
            array = to_arrow(value)
        self.node.send_output(output, array, metadata)


def decoder(spec: dict):
    """value(pyarrow) -> message for one input, with a native box's facility paths prefixed (abi.h)."""
    from like_keras.component import message_type

    cls, prefix = message_type(spec["type"]), spec.get("prefix")

    def decode(array):
        value = from_arrow(array, cls)
        if prefix:
            value = value.replace(path=".".join(filter(None, (prefix, value.path))))
        return value

    return decode


# ---------------------------------------------------------------- the process


def rebuild(spec: dict, name: str):
    """`module:qualname(**params)` (CONTRACT §4.1), named with its full path so logs and records carry it."""
    module, qualname = spec["impl"].split(":")
    built = importlib.import_module(module)
    for part in qualname.split("."):
        built = getattr(built, part)
    component = built(**spec["params"])
    component._lk.name = name
    component.log.name = name
    return component


class _Input:
    """One dora input of this process: where it delivers, and (lockstep) what it promised."""

    def __init__(self, spec: dict, slots: list) -> None:
        self.decode = decoder(spec)
        self.slots = slots  # (slot, how, group): how is "latest", "queue" (an hz group's) or "trigger" (rate-less)
        self.horizon = 0  # ns: every frame with an earlier step time has arrived (nothing is sent before t = 0)
        self.strict: collections.deque = collections.deque()  # (ns, value) for latest and queue slots: due after t
        self.trigger: collections.deque = collections.deque()  # (ns, value) for rate-less queues: due at t
        self.triggers = any(how == "trigger" for _, how, _ in slots)
        self.reads = any(how != "trigger" for _, how, _ in slots)


class Host:
    def __init__(self, document: dict, process: str, link: Link) -> None:
        from like_keras import deployment
        from like_keras.backends import inproc
        from like_keras.system import Edge

        self.document, self.process, self.link = document, process, link
        self.lockstep, node = not document["realtime"], document["nodes"][process]
        leaves = {
            leaf: rebuild(document["components"][leaf], leaf) for group in node["groups"] for leaf in group["order"]
        }
        placed = [
            deployment.Group(
                [leaves[leaf] for leaf in group["order"]],
                process,
                None,
                group["hz"],
                group["cpus"],
                group["fifo"],
                "",
                [leaves[leaf] for leaf in group["order"]],
                "python",
            )
            for group in node["groups"]
        ]
        edges = [
            Edge(leaves[a], port_a, leaves[b], port_b, "", index)
            for index, (a, port_a, b, port_b) in enumerate(document["edges"])
            if a in leaves and b in leaves
        ]
        system = types.SimpleNamespace(_lk=types.SimpleNamespace(name=document["system"], parent=None))
        graph = types.SimpleNamespace(edges=edges, exposed_in={}, exposed_out={})
        plan = types.SimpleNamespace(
            system=system,
            groups=placed,
            graph=graph,
            recordings=[],
            realtime=not self.lockstep,
            crossing={},
            datasheet={},
        )
        self.run = run = inproc.Run(plan)
        group_of = {id(leaf): group for group, where in zip(run.groups, placed) for leaf in where.order}
        self.inputs: dict[str, _Input] = {}
        for input_id, spec in node["inputs"].items():
            slots = []
            for leaf, port in spec["deliver"]:
                consumer = group_of[id(leaves[leaf])]
                slot = run._slot(leaves[leaf], port, consumer, None)
                how = "latest" if isinstance(slot, inproc.Latest) else "trigger" if consumer.hz is None else "queue"
                if how == "trigger" and self.lockstep:
                    slot.wake = None  # WHY: under lockstep the step is scheduled when the frame arrives (_absorb)
                slots.append((slot, how, consumer))
            self.inputs[input_id] = _Input(spec, slots)
        readers = collections.Counter()
        for other in document["nodes"].values():
            for input_id, spec in other["inputs"].items():
                readers[input_id] = max(readers[input_id], spec["queue"])
        self.outbox = {output: collections.deque(maxlen=max(1, readers[output])) for output in node["outputs"]}
        self.frames: list = []  # lockstep: (output, value) sent in the event being stepped
        self.promised = dict.fromkeys(node["outputs"], -1)
        self.wake = threading.Event()
        for output in node["outputs"]:  # run.subscribe: a log port's sinks reach the component's logger too
            leaf, port = output.rsplit(".", 1)
            run.subscribe(leaves[leaf], port, self._sink(output))
        self.backlog: list = []
        self.incoming = collections.deque()  # lockstep: (input id, ns, horizon ns, value or None), from RX
        self.arrived = threading.Event()
        link.receive_with(self._frame)

    def _sink(self, output: str):
        if self.lockstep:
            frames = self.frames
            return lambda value, sent: frames.append((output, value))
        box, wake = self.outbox[output], self.wake

        def send_later(value, sent) -> None:  # a group thread: a deque append and a wake, never a transport call
            box.append(value)
            wake.set()

        return send_later

    # ------------------------------------------------------------ receiving (the RX thread)

    def _frame(self, input_id: str, array, metadata: dict) -> None:
        entry = self.inputs.get(input_id)
        if entry is None:
            return
        try:
            value = entry.decode(array) if len(array) else None
        except (ValueError, TypeError) as refused:  # robot_msgs refused it: drop it, say so
            print(f"like_keras {self.process}: {input_id}: refused a frame: {refused}", file=sys.stderr, flush=True)
            return
        if self.lockstep:
            sent = metadata.get("lk_t", 0)
            horizon = metadata.get("lk_h", sent) if value is not None else sent
            self.incoming.append((entry, sent, horizon, value))
            self.arrived.set()
            return
        with self.link.lock:
            if not self.link.ready.is_set() or not self.started:
                self.backlog.append((entry, value, metadata.get("lk_arrived")))
                return
            self._deliver_now(entry, value)

    started = False

    def _deliver_now(self, entry: _Input, value, arrived: float | None = None) -> None:
        run = self.run
        # on the receiver's clock (CONTRACT §4.1 stale=), from when the frame arrived
        now = run.epoch0 + (time.monotonic() if arrived is None else arrived) - run.mono0
        for slot, how, _ in entry.slots:
            if value is None:  # retracted (CONTRACT §5.4): a latest reads None, a queue is emptied
                if how == "latest":
                    slot.write(None, now)
                else:
                    slot.ready.clear()
                    slot.arriving.clear()
            elif how == "latest":
                slot.write(value, now)
            else:
                slot.push(value, now)

    # ------------------------------------------------------------ real time

    def _transmit(self) -> None:
        """The TX thread: the newest unsent value of each output (a queue reader's depth for queues), to dora."""
        while True:
            self.wake.wait(0.1)
            self.wake.clear()
            for output, box in self.outbox.items():
                while box:
                    self.link.send(output, box.popleft())
            if self.link.stopping.is_set() and not any(self.outbox.values()):
                return

    def _drain_backlog(self) -> None:
        with self.link.lock:
            for entry, value, arrived in self.backlog:
                self._deliver_now(entry, value, arrived)
            self.backlog.clear()

    def run_realtime(self) -> int:
        # WHY before start: what arrived while this process was built (an arm's state) must be in the slots when
        # the first steps run, or a restarted planner skips its first target (T5: command again within 1 s).
        # start() samples the clock again; both samples map monotonic to epoch time alike.
        self.run.epoch0, self.run.mono0 = time.time(), time.monotonic()
        self._drain_backlog()
        try:
            self.run.start(None)
        except Exception as error:  # §5.7: activate raising: exit non-zero having sent no data frame
            mark(self.document, "refused", self.process, f"{type(error).__name__}: {error}")
            raise
        mark(self.document, "started", self.process)
        self._drain_backlog()
        with self.link.lock:
            self._drain_backlog()
            self.started = True
        sender = threading.Thread(target=self._transmit, name=f"lk-tx-{self.process}", daemon=True)
        sender.start()
        self.link.stopping.wait()
        self.run.stop()
        self.wake.set()
        sender.join(timeout=5)
        return 0

    # ------------------------------------------------------------ lockstep (CONTRACT §5.5)

    def _absorb(self) -> None:
        while self.incoming:
            entry, sent, horizon, value = self.incoming.popleft()
            entry.horizon = max(entry.horizon, horizon)
            if value is None:
                continue
            if entry.reads:
                entry.strict.append((sent, value))
            if entry.triggers:
                entry.trigger.append((sent, value))
                for _, how, group in entry.slots:
                    if how == "trigger":  # a rate-less group steps at the send time of each message (§5.2)
                        group.notify(sent / 1e9)

    def _due(self, now_ns: int) -> None:
        """Hand each slot what is due by `now`, oldest first: sent strictly before it, or at it for a rate-less
        queue (its step at t sees the message sent at t). WHY only now: a queue filled ahead of time would drop
        what its reader is still due (drop-oldest, §5.4)."""
        for entry in self.inputs.values():
            while entry.strict and entry.strict[0][0] < now_ns:
                sent, value = entry.strict.popleft()
                for slot, how, _ in entry.slots:
                    if how == "latest":
                        slot.write(value, sent / 1e9)
                    elif how == "queue":
                        slot.push(value, sent / 1e9)
            while entry.trigger and entry.trigger[0][0] <= now_ns:
                sent, value = entry.trigger.popleft()
                for slot, how, _ in entry.slots:
                    if how == "trigger":
                        slot.push(value, sent / 1e9)

    def _promise(self) -> int:
        heap = self.run.heap
        promise = ns(heap[0][0]) if heap else FOREVER
        return min([promise] + [entry.horizon for entry in self.inputs.values() if entry.triggers])

    def _flush(self, now_ns: int, promise: int) -> None:
        """Send this event's frames, the last per output carrying the promise; promise-only frames elsewhere."""
        last = {output: index for index, (output, _) in enumerate(self.frames)}
        for index, (output, value) in enumerate(self.frames):
            metadata = {"lk_t": now_ns}
            if last[output] == index:
                metadata["lk_h"] = promise
                self.promised[output] = promise
            self.link.send(output, value, metadata)
        self.frames.clear()
        for output, before in self.promised.items():
            if before < promise:
                self.link.send(output, None, {"lk_t": promise})
                self.promised[output] = promise

    def run_lockstep(self, until: float) -> int:
        run, heap = self.run, self.run.heap
        activated = []
        try:
            for group in run.groups:
                group.activate(0.0)
                activated.append(group)
        except Exception as error:  # noqa: BLE001 - §5.7: any refusal in activate stops the process, named
            for group in reversed(activated):
                group.deactivate()
            fail(self.document, self.process, f"activate refused: {type(error).__name__}: {error}", 1)
        mark(self.document, "started", self.process)
        for group in run.groups:
            if group.hz:
                heapq.heappush(heap, (0.0, 0, group.index, group.index, 0))
        heapq.heappush(heap, (0.0, -1, 0, 0, 0))  # the facilities, once a logical second (as inproc)
        end = ns(until)
        while True:
            self._absorb()
            now = heap[0][0] if heap else math.inf
            # WHY the triggers' promises too: a message still on its way can step a rate-less group before `until`
            if now >= until and all(entry.horizon >= end for entry in self.inputs.values() if entry.triggers):
                break
            now_ns = min(ns(now), end)
            if any(entry.horizon < now_ns for entry in self.inputs.values()) or now >= until:
                if self.link.stopping.is_set():
                    break
                self._flush(now_ns, self._promise())  # what upstream promised may let this node promise more
                self.arrived.wait(0.1)
                self.arrived.clear()
                continue
            self._due(now_ns)
            now, kind, _, index, tick = heapq.heappop(heap)
            if kind < 0:
                run._publish(now)
                heapq.heappush(heap, (float(tick + 1), -1, 0, 0, tick + 1))
            else:
                group = run.groups[index]
                if not group.dead:
                    group.step(now)
                if kind == 0 and not group.dead:
                    heapq.heappush(heap, ((tick + 1) / group.hz, 0, index, index, tick + 1))
            self._flush(now_ns, self._promise())
        self._flush(ns(until), FOREVER)
        mark(self.document, "done", self.process)
        self.link.stopping.wait()
        run.stop()
        return 0


def log_to(document: dict, node: str) -> None:
    """stderr to <dataflow>/logs/<node>.log as well: refusals stay readable after dora restarts the node."""
    folder = pathlib.Path(document["directory"]) / "logs"
    folder.mkdir(exist_ok=True)
    sys.stderr = open(folder / f"{node}.log", "a", buffering=1)  # noqa: SIM115 - for the process's lifetime
    print(f"--- {node} pid {os.getpid()} at {time.strftime('%H:%M:%S')}", file=sys.stderr)


def main(argv: list[str]) -> int:
    document = json.loads(pathlib.Path(argv[1]).read_text())
    process = argv[2]
    log_to(document, process)
    try:
        link = Link(document, process)
        try:
            host = Host(document, process, link)
        except Exception as error:  # noqa: BLE001 - a component that cannot be rebuilt here never will be: say so
            traceback.print_exc()
            fail(document, process, f"cannot build its components: {type(error).__name__}: {error}", 1)
        link.wait_ready()
        if not document["realtime"]:
            return host.run_lockstep(document["until"])
        try:
            return host.run_realtime()
        except Exception as error:  # noqa: BLE001 - §5.7: activate raising: exit non-zero having sent no data frame
            print(f"like_keras {process}: {type(error).__name__}: {error}", file=sys.stderr, flush=True)
            return 1
    finally:
        mark(document, "exit", process)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
