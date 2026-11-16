"""Attach (CONTRACT §9.3): a running deployment's exposed ports on a unix socket, the IP-camera idea, same host in v0.

    arm = s.add(lk.attach("unix:/run/user/1000/like_keras/arm_controller.sock"), "arm")
    planner = s.add(PickPlanner(), target=targets.target, state=arm.state)   # an exposed Out: subscribed
    s.wire(arm, trajectory=planner.trajectory)                               # an exposed In: written, under the lease

Frames: `u32 length (kind + payload) | u8 kind | payload`. HELLO and DESCRIBE carry JSON, SUB a JSON list of port
names, DATA and WRITE a u16 port index (into the datasheet's `exposed`, in order) and one LKF message, LEASE nothing,
REFUSED JSON {reason, holder}. A subscriber gets each Out's cached latest value at once; a client whose 16-frame send
queue fills is dropped; the deployment never waits on a client. One client at a time holds the write lease (its
first WRITE or LEASE takes it; every WRITE and the proxy's heartbeat, each lease_s / 3, renew it); a holder silent for
lease_s loses it and every input it wrote is retracted, so the controller holds (R5).

The wire format is not public (CONTRACT §3): `lk ls` and `lk top` (like_keras.cli) are its other readers.
"""

from __future__ import annotations

import collections
import dataclasses
import json
import os
import selectors
import socket
import struct
import sys
import threading
import time
from typing import Any

from like_keras.component import FACILITY_PORTS, Component, In, Out, path

HELLO, DESCRIBE, SUB, DATA, WRITE, LEASE, REFUSED, CONTROL = range(1, 9)
FACILITIES = "lk.facilities"  # SUB name of every component's health and metrics (lk top)
FACILITY_PORT = 0xFFFF  # their DATA port index
QUEUE_FRAMES = 16


def frame(kind: int, payload: bytes = b"") -> bytes:
    return struct.pack("<IB", len(payload) + 1, kind) + payload


def frames(buffer: bytearray):
    """Pop every complete frame from `buffer`: (kind, payload)."""
    while len(buffer) >= 5:
        length, kind = struct.unpack_from("<IB", buffer)
        if len(buffer) < 4 + length:
            return
        payload = bytes(buffer[5 : 4 + length])
        del buffer[: 4 + length]
        yield kind, payload


def socket_path(address: str) -> str:
    if not isinstance(address, str) or not address.startswith("unix:") or len(address) <= len("unix:"):
        raise ValueError(
            f"attach address {address!r}: pass the explicit unix:<path> that describe() or `lk ls` prints "
            "(CONTRACT §9.3: nothing is discovered)"
        )
    return address[len("unix:") :]


def exposed_ports(sheet: dict) -> dict[str, dict]:
    """name -> the port sheet (dir, type, hash, queue, stale) of the leaf port behind each exposed name."""
    ports = {}
    for name, leaf_port in sheet["exposed"].items():
        leaf, port = leaf_port.split(", ")[0].rsplit(".", 1)
        ports[name] = sheet["components"][leaf]["ports"][port]
    return ports


def request(address: str, timeout_s: float = 1.0) -> tuple[dict, dict, socket.socket]:
    """Connect, say HELLO, ask DESCRIBE: (their hello, the datasheet, the open socket)."""
    connection = socket.socket(socket.AF_UNIX)
    connection.settimeout(timeout_s)
    connection.connect(socket_path(address))
    connection.sendall(frame(HELLO, json.dumps({"client": f"pid {os.getpid()}"}).encode()) + frame(DESCRIBE))
    buffer, replies = bytearray(), {}
    while DESCRIBE not in replies:
        chunk = connection.recv(65536)
        if not chunk:
            raise ConnectionError(f"{address} closed before it described itself")
        buffer += chunk
        replies.update({kind: json.loads(payload) for kind, payload in frames(buffer) if kind in (HELLO, DESCRIBE)})
    return replies.get(HELLO, {}), replies[DESCRIBE], connection


# ---------------------------------------------------------------- the deployment's side


class _Client:
    def __init__(self, connection: socket.socket) -> None:
        self.connection, self.inbox, self.outbox = connection, bytearray(), collections.deque()
        self.name, self.lease_s, self.subscribed, self.wrote = "?", 0.25, set(), set()


class Server:
    """The attach socket of one deployment: one I/O thread; producers only append and wake it (never block)."""

    def __init__(self, sheet: dict, inputs: dict[str, list], clock, writable: bool = True) -> None:
        self.sheet, self.inputs, self.clock, self.writable = sheet, inputs, clock, writable
        self.names = list(sheet["exposed"])
        self.ports = exposed_ports(sheet)
        self.cached: dict[int, Any] = {}  # port index -> its latest value
        self.news: collections.deque = collections.deque()
        self.holder, self.expires, self.refused = None, 0.0, collections.Counter()
        self.heard = {name: time.monotonic() for name, port in self.ports.items() if port["dir"] == "out"}
        self.rates = {leaf: group["hz"] for group in sheet["groups"] for leaf in group["order"]}
        self.path = socket_path(sheet["attach"]["socket"])
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        if os.path.exists(self.path):
            probe = socket.socket(socket.AF_UNIX)
            try:
                probe.connect(self.path)
                raise RuntimeError(f"{self.path} is served by another running deployment named {sheet['system']!r}")
            except (ConnectionRefusedError, FileNotFoundError):
                os.unlink(self.path)  # a dead deployment's socket
            finally:
                probe.close()
        self.listener = socket.socket(socket.AF_UNIX)
        self.listener.bind(self.path)
        self.listener.listen(16)
        self.listener.setblocking(False)
        self.wake_read, self.wake_write = os.pipe()
        os.set_blocking(self.wake_read, False)
        os.set_blocking(self.wake_write, False)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.listener, selectors.EVENT_READ, None)
        self.selector.register(self.wake_read, selectors.EVENT_READ, "wake")
        self.closed = False
        self.thread = threading.Thread(target=self._loop, name=f"lk-attach-{sheet['system']}", daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ producers (any thread)

    def publish(self, name: str, value) -> None:
        self.news.append((name, value))
        self._wake()

    def facility(self, value) -> None:
        self.news.append((None, value))
        self._wake()

    def _wake(self) -> None:
        try:
            os.write(self.wake_write, b"!")
        except (BlockingIOError, OSError):
            pass  # a wake is already pending (or the server closed)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self._wake()
        self.thread.join(timeout=5)
        for key in list(self.selector.get_map().values()):
            if isinstance(key.data, _Client):
                key.data.connection.close()
        self.listener.close()
        for fd in (self.wake_read, self.wake_write):
            os.close(fd)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

    # ------------------------------------------------------------ the I/O thread

    def _loop(self) -> None:
        next_record = time.monotonic() + 1.0
        while not self.closed:
            now = time.monotonic()
            if self.holder is not None and now > self.expires:
                self._retract(self.holder)  # the holder went silent for lease_s (crashed, killed, slow)
            if now >= next_record:
                self._records()
                next_record += 1.0
            timeout = min(next_record, self.expires if self.holder else next_record) - now
            for key, events in self.selector.select(max(0.0, timeout)):
                if key.data is None:
                    self._accept()
                elif key.data == "wake":
                    self._drain_wake()
                elif events & selectors.EVENT_READ:
                    self._read(key.data)
                if isinstance(key.data, _Client) and events & selectors.EVENT_WRITE:
                    self._flush(key.data)

    def _accept(self) -> None:
        try:
            connection, _ = self.listener.accept()
        except BlockingIOError:
            return
        connection.setblocking(False)
        self.selector.register(connection, selectors.EVENT_READ, _Client(connection))

    def _drain_wake(self) -> None:
        from like_keras import codec

        try:
            while os.read(self.wake_read, 4096):
                pass
        except BlockingIOError:
            pass
        # WHY the newest only, encoded only for a subscriber: an exposed Out is latest (a reader wants the newest),
        # and encoding is the costly part on a busy host; it shares the GIL with the deployment's own threads
        newest, records = {}, []
        while self.news:
            name, value = self.news.popleft()
            if name is None:
                records.append(value)
            else:
                newest[name] = value
                self.heard[name] = time.monotonic()
        clients = self._clients()
        for name, value in newest.items():
            index = self.names.index(name)
            self.cached[index] = value
            readers = [client for client in clients if name in client.subscribed]
            if readers:
                data = frame(DATA, struct.pack("<H", index) + codec.lkf(value))
                for client in readers:
                    self._send(client, data)
        readers = [client for client in clients if FACILITIES in client.subscribed]
        for value in records if readers else ():
            data = frame(DATA, struct.pack("<H", FACILITY_PORT) + codec.lkf(value))
            for client in readers:
                self._send(client, data)

    def _clients(self) -> list[_Client]:
        return [key.data for key in self.selector.get_map().values() if isinstance(key.data, _Client)]

    def _read(self, client: _Client) -> None:
        try:
            chunk = client.connection.recv(1 << 20)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            chunk = b""
        if not chunk:
            self._drop(client)
            return
        client.inbox += chunk
        for kind, payload in frames(client.inbox):
            self._handle(client, kind, payload)

    def _handle(self, client: _Client, kind: int, payload: bytes) -> None:
        from like_keras.backends import address

        if kind == HELLO:
            hello = json.loads(payload or b"{}")
            client.name, client.lease_s = str(hello.get("client", "?")), float(hello.get("lease_s", 0.25))
            reply = {"like_keras": self.sheet["like_keras"], "abi": self.sheet["abi"], "system": self.sheet["system"],
                     "backend": self.sheet["backend"], "describe": self.sheet["hash"], "address": address(
                         self.sheet["system"])}
            self._send(client, frame(HELLO, json.dumps(reply).encode()))
        elif kind == DESCRIBE:
            self._send(client, frame(DESCRIBE, json.dumps(self.sheet, default=str).encode()))
        elif kind == SUB:
            from like_keras import codec

            for name in json.loads(payload):
                client.subscribed.add(name)
                index = self.names.index(name) if name in self.names else None
                if index in self.cached:  # the cached latest value, at once
                    self._send(client, frame(DATA, struct.pack("<H", index) + codec.lkf(self.cached[index])))
        elif kind in (WRITE, LEASE):
            self._write(client, payload if kind == WRITE else None)
        elif kind == CONTROL:
            self._send(client, frame(REFUSED, json.dumps({"reason": "CONTROL is not served in v0", "holder": None})
                                     .encode()))

    def _write(self, client: _Client, payload: bytes | None) -> None:
        from like_keras import codec
        from like_keras.component import message_type

        name = None
        if payload is not None:
            (index,) = struct.unpack_from("<H", payload)
            name = self.names[index] if index < len(self.names) else None
            if name is None or self.ports[name]["dir"] != "in" or not self.writable:
                self._send(client, frame(REFUSED, json.dumps({"reason": f"port {index} is not a writable input",
                                                              "holder": None}).encode()))
                return
        now = time.monotonic()
        if self.holder is not None and self.holder is not client and now <= self.expires:
            if name is not None:
                self.refused[name] += 1
            self._send(client, frame(REFUSED, json.dumps({"reason": "lease held", "holder": self.holder.name,
                                                          "port": name}).encode()))
            return
        self.holder, self.expires = client, now + client.lease_s
        if name is None:
            return
        value = codec.decode(payload[2:], message_type(self.ports[name]["type"]))
        client.wrote.add(name)
        sent = self.clock()
        for slot in self.inputs.get(name, []):
            (slot.push if hasattr(slot, "ready") else slot.write)(value, sent)

    def _retract(self, holder: _Client) -> None:
        """The lease lapsed: every input the holder wrote reads None (a queue is emptied) until its next value."""
        sent = self.clock()
        for name in holder.wrote:
            for slot in self.inputs.get(name, []):
                if hasattr(slot, "ready"):
                    slot.ready.clear()
                    slot.arriving.clear()
                else:
                    slot.write(None, sent)
        holder.wrote.clear()
        self.holder = None

    def _send(self, client: _Client, data: bytes) -> None:
        if len(client.outbox) >= QUEUE_FRAMES:
            print(f"like_keras attach {self.sheet['system']}: dropped {client.name}: {QUEUE_FRAMES} frames unread",
                  file=sys.stderr, flush=True)
            self._drop(client)  # a slow client is dropped; the deployment never waits on one
            return
        client.outbox.append(data)
        self._flush(client)

    def _flush(self, client: _Client) -> None:
        try:
            while client.outbox:
                sent = client.connection.send(client.outbox[0])
                if sent < len(client.outbox[0]):
                    client.outbox[0] = client.outbox[0][sent:]
                    break
                client.outbox.popleft()
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            self._drop(client)
            return
        events = selectors.EVENT_READ | (selectors.EVENT_WRITE if client.outbox else 0)
        try:
            self.selector.modify(client.connection, events, client)
        except (KeyError, ValueError):
            pass

    def _drop(self, client: _Client) -> None:
        if self.holder is client:
            self._retract(client)
        try:
            self.selector.unregister(client.connection)
        except (KeyError, ValueError):
            pass
        client.connection.close()

    def _records(self) -> None:
        """The socket's own health and metrics, path = the System's (it has no facility ports of its own): lease
        refusals, and InputStale for an exposed Out that stopped arriving (a transport stall, T4)."""
        from robot_msgs.std import DiagnosticRecord, Header, Metrics

        now, stamp = time.monotonic(), time.time()
        stale = []
        for name, heard in self.heard.items():
            leaf, port = self.sheet["exposed"][name].rsplit(".", 1)
            # WHY these ages: five periods of the producing group, or a facility's STALE rule (5 s, CONTRACT §8)
            limit = 5.0 if port in FACILITY_PORTS else max(5.0 / self.rates[leaf], 0.1) if self.rates.get(leaf) else None
            if limit is not None and now - heard > limit:
                stale.append(name)
        level, code = ("warn", "InputStale") if stale else ("ok", "")
        system = self.sheet["system"]
        self.news.append((None, DiagnosticRecord(header=Header(stamp=stamp), path=system, level=level, code=code,
                                                 message=f"stale: {', '.join(stale)}" if stale else "",
                                                 mode="operational", stale_inputs=tuple(stale))))
        names = [f"refused_writes.{name}" for name, port in self.ports.items() if port["dir"] == "in"]
        values = [float(self.refused[name.split(".", 1)[1]]) for name in names] + [float(len(self._clients()))]
        self.news.append((None, Metrics(header=Header(stamp=stamp), path=system, names=(*names, "clients"),
                                        values=values)))
        self._drain_wake()


def serve(run) -> Server:
    """The inproc deployment's attach socket (CONTRACT §9.1): its exposed ports, and every component's health
    and metrics for `lk top`. Producers hand values over in their own sinks; the socket thread does the rest."""
    plan = run.plan
    server = Server(plan.datasheet, run.inputs, clock=lambda: run.epoch0 + time.monotonic() - run.mono0)
    for name, ref in plan.graph.exposed_out.items():
        run._box(ref.owner).sinks[ref.name].append(lambda value, sent, name=name: server.publish(name, value))
    for leaf in plan.graph.leaves:
        for port in ("health", "metrics"):
            run._box(leaf).sinks[port].append(lambda value, sent: server.facility(value))
    return server


# ---------------------------------------------------------------- the client's side: lk.attach


def attach(address: str, lease_s: float = 0.25) -> Component:
    """A component mirroring a running deployment's exposed ports, reached at `address` (CONTRACT §4.5): an exposed
    In becomes an In you wire into, an exposed Out an Out you subscribe from. Place it in an hz group."""
    socket_path(address)
    if not lease_s > 0:
        raise ValueError(f"attach(lease_s={lease_s!r}): a lease is a time in seconds, > 0")
    _, sheet, connection = request(address)
    connection.close()
    ports = {}
    for name, port in exposed_ports(sheet).items():
        ports[name] = In(port["type"]) if port["dir"] == "in" else Out(port["type"])  # latest: the proxy samples
        # WHY optional: a reader (lk top, an observer) attaches without writing; an input nobody wires is never
        # sent, so it needs no writer (the core's unbound check honours it: docs/friction/contract-006.md)
        ports[name].optional = port["dir"] == "in"
    cls = type(sheet["system"], (_Attached,), {**ports, "__module__": __name__, "_lk_impl": "like_keras:attach"})
    return cls(address=address, lease_s=lease_s)


class _Attached(Component):
    """The proxy: each step sends what is new on its inputs and returns what arrived on its outputs."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        address: str
        lease_s: float = 0.25

    def activate(self) -> None:
        self._link = _Link(self.params.address, self.params.lease_s, path(self), type(self)._lk_outs)
        self._refused_at, self._drops = -1.0, 0

    def forward(self, **inputs: Any):
        link = self._link
        if link.drops > self._drops:
            self._drops = link.drops
            self.log.warning("reconnected to %s (the deployment dropped this slow reader)", self.params.address)
        for name, value in inputs.items():
            if value is not None and value is not link.sent.get(name):
                link.write(name, value)
        if link.refusals:
            refusal = link.refusals.popleft()
            message = f"write refused: the lease is held by {refusal.get('holder')} ({refusal.get('reason')})"
            self.log.error(message)
            self._lk.error, self._lk.code, self._refused_at = message, "LeaseHeld", time.monotonic()
        elif self._refused_at > 0 and time.monotonic() - self._refused_at > 1.0:
            self._lk.error, self._lk.code, self._refused_at = None, None, -1.0  # no refusal in the last period
        outs = type(self)._lk_outs
        news = [link.take(name) for name in outs]
        return news[0] if len(news) == 1 else tuple(news) if news else None

    def deactivate(self) -> None:
        self._link.close()

    def _lk_params(self) -> dict[str, Any]:
        return {"address": self.params.address, "lease_s": self.params.lease_s}


class _Link:
    """The proxy's connection: one I/O thread reading DATA and REFUSED, sending WRITEs and the lease heartbeat.
    WHY it reconnects: the deployment drops a client that reads too slowly (and may restart); the proxy comes back
    and subscribes again instead of going silent (the IP-camera idea)."""

    def __init__(self, address: str, lease_s: float, name: str, outs: tuple[str, ...]) -> None:
        self.address, self.name, self.outs, self.lease_s = address, name, outs, lease_s
        self.connection = self._connect()
        self.names, self.ports = list(self.sheet["exposed"]), exposed_ports(self.sheet)
        self.latest: dict[str, Any] = {}
        self.sent: dict[str, Any] = {}
        self.pending: collections.deque = collections.deque()
        self.refusals: collections.deque = collections.deque(maxlen=16)
        self.writing, self.closed, self.last_sent, self.drops = False, False, 0.0, 0
        self.wake_read, self.wake_write = socket.socketpair()
        self.first = threading.Event()  # every subscribed Out has its cached latest value
        self.thread = threading.Thread(target=self._loop, name=f"lk-attach-{name}", daemon=True)
        self.thread.start()
        # WHY wait (activate may block): the first steps then see the deployment's state, so an attached planner
        # does not skip its first target (T5: a restarted planner commands again within 1 s)
        self.first.wait(0.5 if outs else 0.0)

    def _connect(self) -> socket.socket:
        _, self.sheet, connection = request(self.address)
        connection.sendall(frame(HELLO, json.dumps({"client": self.name, "lease_s": self.lease_s}).encode())
                           + frame(SUB, json.dumps(list(self.outs)).encode()))
        connection.settimeout(None)
        return connection

    def write(self, name: str, value) -> None:
        self.sent[name] = value
        self.pending.append((name, value))
        self.wake_write.send(b"!")

    def take(self, name: str):
        return self.latest.pop(name, None)

    def close(self) -> None:
        self.closed = True
        self.wake_write.send(b"!")
        self.thread.join(timeout=5)
        self.connection.close()
        self.wake_read.close()
        self.wake_write.close()

    def _loop(self) -> None:
        while not self.closed:
            try:
                self._serve()
            except OSError:
                pass
            self.connection.close()
            while not self.closed:  # dropped or restarted: subscribe again
                try:
                    self.connection = self._connect()
                    self.drops += 1
                    break
                except OSError:
                    time.sleep(0.2)

    def _serve(self) -> None:
        from like_keras import codec
        from like_keras.component import message_type

        buffer = bytearray()
        selector = selectors.DefaultSelector()
        selector.register(self.connection, selectors.EVENT_READ, "socket")
        selector.register(self.wake_read, selectors.EVENT_READ, "wake")
        heartbeat = self.lease_s / 3
        try:
            while not self.closed:
                for key, _ in selector.select(heartbeat):
                    if key.data == "wake":
                        self.wake_read.recv(4096)
                        continue
                    chunk = self.connection.recv(1 << 20)
                    if not chunk:
                        return
                    buffer += chunk
                    for kind, payload in frames(buffer):
                        if kind == DATA:
                            (index,) = struct.unpack_from("<H", payload)
                            name = self.names[index]
                            self.latest[name] = codec.decode(payload[2:], message_type(self.ports[name]["type"]))
                            if not self.first.is_set() and all(out in self.latest for out in self.outs):
                                self.first.set()
                        elif kind == REFUSED:
                            self.refusals.append(json.loads(payload))
                out = b""
                while self.pending:
                    name, value = self.pending.popleft()
                    out += frame(WRITE, struct.pack("<H", self.names.index(name)) + codec.lkf(value))
                    self.writing = True
                if not out and self.writing and time.monotonic() - self.last_sent >= heartbeat:
                    out = frame(LEASE)
                if out:
                    self.connection.sendall(out)
                    self.last_sent = time.monotonic()
        finally:
            selector.close()
