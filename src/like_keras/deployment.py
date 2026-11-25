"""Deployment: where components run (CONTRACT §4.4). Each `place()` call is one group, one thread; each distinct
`process` one OS process (collapsed into this one under `inproc`); the backend carries what crosses.

`describe()` and `start()` run every check of §4.3 first and raise one ValueError listing every failure (the texts
are system.PROBLEMS), then `describe()` returns the plan as JSON-ready data: the device datasheet.
"""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import math
import os
import pkgutil
import re
import signal
import sys
import threading
import time
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any

from like_keras import backends
from like_keras.backends import inproc
from like_keras.component import (
    ABI,
    FACILITY_PORTS,
    Component,
    ComponentState,
    PortRef,
    _Health,
    _Log,
    as_ref,
    is_message,
    message_type,
    path,
    site,
    type_name,
)
from like_keras.system import Edge, Graph, System, graph, leaves_of, out_leaves, port_of, problem

VERSION = "0.1.0"
# WHY 22: on this machine a shared transport daemon runs there and has stalled for up to 4 s (CONTRACT §4.3 check 4).
RESERVED_CPUS = [22]


@dataclass
class Group:
    """One place() call: one thread running its components in the derived order (CONTRACT §5.1, §5.3)."""

    leaves: list[Component]
    process: str = "main"
    host: str | None = None
    hz: float | None = None
    cpus: list[int] | None = None
    fifo: int | None = None
    at: str = ""
    order: list[Component] = field(default_factory=list)
    language: str = "python"


@dataclass
class Plan:
    """What a backend runs: the checked groups and edges, the recordings, and the datasheet describe() returns."""

    system: System
    groups: list[Group]
    graph: Graph
    crossing: dict[Edge, str | None]  # None (same group), "group", "process" or "language"
    recordings: list[tuple[str, list[tuple[Component, str, str]]]]  # file -> (leaf, Out port, MCAP topic)
    realtime: bool
    datasheet: dict[str, Any] = field(default_factory=dict)


class Deployment:
    def __init__(
        self,
        system: System,
        backend: ModuleType = inproc,
        realtime: bool = True,
        bind: Mapping[type[Component] | str, Component] | None = None,
    ) -> None:
        """`bind` (P5) chooses what runs in place of a role or a node: `{Arm: TwinArm(), "arm": lk.native(SO)}`. A
        class key binds every node of that class, a node path (`"arm"`, `"arm.io"`) one node and wins; a value is a
        template, so each Deployment builds its own instance (a System value is used as given)."""
        self._system, self._backend, self._realtime, self._at = system, backend, realtime, site()
        self._places: list[tuple[tuple[Component, ...], dict[str, Any], str]] = []
        self._recordings: list[tuple[str, list[PortRef], str]] = []
        self._run = None
        self._stopping = threading.RLock()  # a stop in progress on another thread finishes before a signal ends us
        # WHY resolved here, not in the System (P5): the System wires roles and stays the same in sim, on the robot
        # and in a test; which implementation runs is this deployment's one line. `cell.arm` reads the bound one.
        self._bind = None if bind is None else dict(bind)
        self._swaps, self._bound, self._bound_by, self._bind_failures = _resolve(system, self._bind or {}, self._at)
        _apply(system, self._swaps)

    def place(
        self,
        *components: Component,
        process: str = "main",
        host: str | None = None,
        hz: float | None = None,
        cpus: Sequence[int] | None = None,
        fifo: int | None = None,
    ) -> Deployment:
        """One group: these components on one thread, in `process` (on `host`), stepping at `hz` (or, without hz,
        once per message on a queue input). `cpus` and `fifo` apply to a native group's real-time thread."""
        cpus = None if cpus is None else list(cpus)
        self._places.append(
            (components, {"process": process, "host": host, "hz": hz, "cpus": cpus, "fifo": fifo}, site())
        )
        return self

    def record(self, path: str | os.PathLike, *ports: PortRef) -> Deployment:
        """Record to MCAP: these Outs, or with none every edge between groups and every exposed Out (CONTRACT §8)."""
        self._recordings.append((os.fspath(path), [as_ref(port) for port in ports], site()))
        return self

    def start(self, until: float | None = None) -> Deployment:
        """Real time: launch and return at once, or with `until` run that long and stop. Lockstep (realtime=False):
        run to logical time `until` at full speed and stop (CONTRACT §5.5)."""
        if self._run is not None:
            raise RuntimeError(f"{path(self._system)} is running: stop() it before starting again")
        if not self._realtime and until is None:
            raise ValueError("lockstep (realtime=False) runs to a logical time: start(until=<seconds>)")
        main = threading.current_thread() is threading.main_thread()
        if main:
            _ending.install()
        _ending.starting += main
        try:
            self._run = self._backend.start(self._plan(), None if self._realtime else until)
            _ending.running.append(self)
        except _Ended as ended:  # a signal unwound the start; the backend stopped what it had started
            self.stop()
            _ending.end(ended.signal)
        finally:
            _ending.starting -= main
        if until is not None:
            try:  # WHY short sleeps, not an Event: a signal handler may stop() meanwhile, and must take no lock here
                deadline = time.monotonic() + until if self._realtime else 0.0
                while self._run is not None and (left := deadline - time.monotonic()) > 0:
                    time.sleep(min(left, 0.05))
            finally:
                self.stop()
        return self

    def stop(self) -> None:
        """Deactivate in reverse order, join the threads, close the recordings. Idempotent."""
        main = threading.current_thread() is threading.main_thread()
        with self._stopping:
            _ending.stops += main  # WHY first: a signal from here on waits for this stop to finish
            try:
                run, self._run = self._run, None
                if run is not None:
                    run.stop()
            finally:
                _ending.stops -= main
                if self in _ending.running:
                    _ending.running.remove(self)
        if main and not _ending.stops:
            _ending.after_stop()

    def describe(self) -> dict[str, Any]:
        """Run every check (CONTRACT §4.3), then return the plan: the device datasheet."""
        return self._plan().datasheet

    def _plan(self) -> Plan:
        _apply(self._system, self._swaps)  # this deployment's bindings, whatever another Deployment bound since
        found, backend = graph(self._system), self._backend
        failures = [*self._bind_failures, *found.failures]
        groups = self._groups(found, failures)
        group_of = {id(leaf): group for group in groups for leaf in group.leaves}
        for group in groups:
            failures += _check_group(group, found.edges, backend)
        crossing = {}
        for edge in (edge for edge in found.edges if id(edge.src) in group_of and id(edge.dst) in group_of):
            source, target = group_of[id(edge.src)], group_of[id(edge.dst)]
            crossing[edge] = next(
                (
                    kind
                    for kind, differs in (
                        ("language", source.language != target.language),
                        ("process", source.process != target.process),
                        ("group", source is not target),
                    )
                    if differs
                ),
                None,
            )
            failures += _check_edge(edge, crossing[edge])
        failures += _check_processes(groups, backend, self._realtime, self._at)
        recordings = [
            (file, self._recorded(found, crossing, [self._moved(port) for port in ports], at, failures))
            for file, ports, at in self._recordings
        ]
        if failures:
            raise ValueError(
                f"{path(self._system)}: {len(failures)} problem(s) found before start (CONTRACT §4.3):\n"
                + "\n".join(f"- {failure}" for failure in failures)
            )
        plan = Plan(self._system, groups, found, crossing, recordings, self._realtime)
        plan.datasheet = _datasheet(plan, backend, None if self._bind is None else self._bound_by)
        return plan

    def _moved(self, ref: PortRef) -> PortRef:
        """A port of a node this deployment bound, as the port of what runs there (P5)."""
        bound = self._bound.get(id(ref.owner))
        return ref if bound is None else PortRef(bound, ref.name)

    def _groups(self, found: Graph, failures: list[str]) -> list[Group]:
        for leaf in found.leaves:  # the logger carries the path its component has in this deployment
            leaf.log.name = path(leaf)
        if not self._places:
            return [Group(list(found.leaves), at="no place() call: one group, no hz")]
        known, placed, groups = {id(leaf) for leaf in found.leaves}, {}, []
        for components, options, at in self._places:
            groups.append(Group([], **options, at=at))
            bound = [self._bound.get(id(component), component) for component in components]  # a role: what runs
            for leaf in (leaf for component in bound for leaf in leaves_of(component)):
                if id(leaf) not in known:
                    failures.append(problem("foreign_leaf", child=path(leaf), system=path(self._system), at=at))
                elif id(leaf) in placed:
                    failures.append(problem("placed_twice", child=path(leaf), first=placed[id(leaf)], at=at))
                else:
                    placed[id(leaf)] = at
                    groups[-1].leaves.append(leaf)
        failures += [
            problem("never_placed", child=path(leaf), at=leaf._lk.site)
            for leaf in found.leaves
            if id(leaf) not in placed
        ]
        return [group for group in groups if group.leaves]

    def _recorded(self, found: Graph, crossing: dict, ports: list[PortRef], at: str, failures: list[str]):
        """(leaf, Out port, MCAP topic) per recorded port; the topic is `<path>.<port>` as the user named it."""
        chosen: dict[tuple[int, str], tuple[Component, str, str]] = {}

        def add(ref: PortRef, topic: str, explicit: bool) -> None:
            kind = port_of(ref).type
            if not is_message(kind):
                if explicit:
                    failures.append(problem("record_type", port=ref, got=type_name(kind), at=at))
                return
            for leaf in out_leaves(ref):  # a System's own log/health/metrics: every leaf's, on one topic (§15 A21)
                topic = getattr(type(leaf.owner), "_lk_topics", {}).get(leaf.name, topic)  # a replay keeps its channel
                chosen.setdefault((id(leaf.owner), leaf.name), (leaf.owner, leaf.name, topic))

        for ref in ports:
            if port_of(ref, at).output:
                add(ref, str(ref), True)
            else:
                failures.append(problem("record_input", port=ref, at=at))
        if not ports:
            for name, ref in found.exposed_out.items():
                add(ref, f"{path(self._system)}.{name}", False)
            for edge in (edge for edge, crosses in crossing.items() if crosses):
                add(PortRef(edge.src, edge.src_port), f"{path(edge.src)}.{edge.src_port}", False)
        return list(chosen.values())


# ---------------------------------------------------------------- bindings (P5: roles and bindings)


def _declared(system: System) -> dict[str, Component]:
    """A System's children as its builder made them, whatever a Deployment has bound in their place since."""
    declared = system.__dict__.get("_lk_declared", {})
    return {name: declared.get(name, child) for name, child in system._children.items()}


def _nodes(system: System, prefix: str = "") -> Iterator[tuple[System, str, str, Component]]:
    """(parent, name, node path below the top, declared node) for every node of the tree, depth first."""
    if not isinstance(system, System):  # v0 refuses a non-System in describe(), as before: nothing to bind here
        return
    for name, node in _declared(system).items():
        yield system, name, prefix + name, node
        if isinstance(node, System):
            yield from _nodes(node, f"{prefix}{name}.")


def _resolve(top: System, bind: dict, at: str):
    """What runs in place of which node: (swaps, {id(node): implementation}, {id(leaf): datasheet rows}, failures).
    A node path wins over a class key, the most specific class key over a wider one, and a bound System is
    replaced whole (class keys inside it are moot, not unmatched)."""
    swaps, bound, rows, failures = [], {}, {}, []
    paths = {key: value for key, value in bind.items() if isinstance(key, str)}
    classes = {key: value for key, value in bind.items() if isinstance(key, type) and issubclass(key, Component)}
    failures += [problem("bind_key", key=repr(key), at=at) for key in bind if key not in paths and key not in classes]
    nodes = list(_nodes(top))
    known = sorted(node_path for _, _, node_path, _ in nodes)
    for key in paths.keys() - set(known):
        failures.append(problem("bind_path", key=key, system=path(top), nodes=", ".join(known), at=at))
    for key in classes:
        if not any(isinstance(node, key) for *_, node in nodes):
            failures.append(problem("bind_class", key=key.__qualname__, system=path(top), at=at))
    replaced: list[str] = []
    for system, name, node_path, node in nodes:
        if any(node_path.startswith(f"{done}.") for done in replaced):
            continue
        key = node_path if node_path in paths else next((k for k in type(node).__mro__ if k in classes), None)
        if key is None:
            continue
        replaced.append(node_path)
        how = f'bind["{key}"]' if isinstance(key, str) else f"bind[{key.__qualname__}]"
        implementation = _implementation(bind[key], how, system, node, node_path, at, failures)
        if implementation is not None:
            swaps.append((system, name, node, implementation))
            bound[id(node)] = implementation
            role = f"System {node._lk.name}" if isinstance(node, System) else _impl(node)
            rows.update({id(leaf): {"role": role, "bound_by": how} for leaf in leaves_of(implementation)})
    return swaps, bound, rows, failures


def _impl(component: Component) -> str:
    return getattr(type(component), "_lk_impl", f"{type(component).__module__}:{type(component).__qualname__}")


def _implementation(value, how: str, system: System, node: Component, node_path: str, at: str, failures: list[str]):
    """This deployment's own instance of a bound value, checked against the node's ports; None when refused."""
    if isinstance(value, System):
        if value._lk.parent not in (None, system):
            failures.append(problem("bind_parent", how=how, got=value._lk.name, parent=path(value._lk.parent), at=at))
            return None
        implementation = value
    elif isinstance(value, Component):
        # WHY a copy: a profile dict serves many Deployments and tests, as module:qualname(**params) rebuilds a
        # component in another process; nothing is shared but the frozen params (state is made in activate)
        implementation = copy.copy(value)
        implementation._lk = ComponentState(value._lk.name)
        implementation.log, implementation.health = _Log(implementation), _Health(implementation, "health")
    else:
        failures.append(problem("bind_value", how=how, got=type(value).__name__, at=at))
        return None
    want, got = _ports(node), _ports(implementation)
    wrong = []
    for name, port in want.items():
        have = got.get(name)
        if have is None:
            wrong.append(f"no port {name!r} ({_side(port)})")
        elif have.output != port.output or type_name(have.type) != type_name(port.type):
            wrong.append(f"{name} is {_side(have)}, the node's is {_side(port)}")
        elif have.hash is not None:
            from like_keras import codec

            ours = codec.schema_hash(message_type(have.type))
            if have.hash != ours:
                wrong.append(f"{name} was built with schema hash {have.hash:016x}; this robot_msgs has {ours:016x}")
    for name, port in got.items():
        if name not in want and not port.output and not port.optional:
            wrong.append(f"its input {name!r} ({_side(port)}) is not a port of the node, so nothing could feed it")
    if wrong:
        shown = f"System {implementation._lk.name}" if isinstance(implementation, System) else _impl(implementation)
        failures.append(problem("bind_ports", how=how, impl=shown, node=node_path, wrong="; ".join(wrong), at=at))
        return None
    return implementation


def _ports(component: Component) -> dict[str, Any]:
    """The ports another component wires to: a leaf's declared ones, a System's exposed ones."""
    if isinstance(component, System):
        names = list(component._exposed)
    else:
        names = [name for name in type(component)._lk_ports if name not in FACILITY_PORTS]
    return {name: port_of(PortRef(component, name)) for name in names}


def _side(port) -> str:
    return f"{'Out' if port.output else 'In'}[{type_name(port.type)}]"


def _apply(top: System, swaps: list) -> None:
    """Put every node back as declared, then this deployment's implementations in their places."""
    for system, name, _, node in list(_nodes(top)):
        if system._children[name] is not node:
            _swap(system, name, system._children[name], node)
    for system, name, node, implementation in swaps:
        system.__dict__.setdefault("_lk_declared", {})[name] = node
        _swap(system, name, node, implementation)


def _swap(system: System, name: str, old: Component, new: Component) -> None:
    """`new` takes `old`'s place in `system`: its name, its site, every binding and exposure that named it."""
    system._children[name] = new
    new._lk.name, new._lk.parent, new._lk.site = name, system, old._lk.site
    if name in FACILITY_PORTS:  # as System._claim does: the child wins over the class's facility port
        system.__dict__[name] = new

    def moved(ref: PortRef) -> PortRef:
        return PortRef(new, ref.name) if ref.owner is old else ref

    system._bindings = [dataclasses.replace(b, dst=moved(b.dst), src=moved(b.src)) for b in system._bindings]
    system._exposed = {key: (moved(ref), at) for key, (ref, at) in system._exposed.items()}


def _check_group(group: Group, edges: list[Edge], backend: ModuleType) -> list[str]:
    failures, at, children = [], group.at, ", ".join(path(leaf) for leaf in group.leaves)
    natives = [path(leaf) for leaf in group.leaves if leaf._lk_language == "native"]
    group.language = "native" if natives else "python"
    cpu_count = os.cpu_count() or 1
    if natives and len(natives) < len(group.leaves):
        failures.append(problem("language", child=", ".join(natives), at=at))
    if not re.fullmatch(r"[a-zA-Z0-9_.-]+", group.process):
        failures.append(problem("process", process=group.process, at=at))
    elif group.process in {module.name for module in pkgutil.iter_modules(backends.__path__)}:
        failures.append(problem("backend_name", process=group.process, at=at))
    if group.hz is not None and not (isinstance(group.hz, (int, float)) and math.isfinite(group.hz) and group.hz > 0):
        failures.append(problem("hz", child=children, hz=group.hz, at=at))
    if group.host is not None and "host" not in backend.CAPABILITIES:
        failures.append(problem("host", host=group.host, backend=backend.NAME, at=at))
    if group.fifo is not None and not (isinstance(group.fifo, int) and 1 <= group.fifo <= 99):
        failures.append(problem("fifo", fifo=group.fifo, at=at))
    if group.cpus is not None and set(group.cpus) & set(RESERVED_CPUS):
        failures.append(problem("reserved_cpu", cpus=group.cpus, reserved=RESERVED_CPUS, at=at))
    if group.cpus is not None and any(not 0 <= cpu < cpu_count for cpu in group.cpus):
        failures.append(problem("cpu_count", cpus=group.cpus, last=cpu_count - 1, at=at))
    return failures + _derive_order(group, edges)


def _derive_order(group: Group, edges: list[Edge]) -> list[str]:
    """Topological order over same-group edges, lagged inputs ignored (CONTRACT §5.3); ties keep place() order.
    A rate-less group may hold no cycle at all, lagged or not, and each of its components must be able to step."""
    index = {id(leaf): position for position, leaf in enumerate(group.leaves)}
    rateless = group.hz is None
    flow = [
        edge for edge in edges if id(edge.src) in index and id(edge.dst) in index and (rateless or not _lagged(edge))
    ]
    waiting = {id(leaf): sum(edge.dst is leaf for edge in flow) for leaf in group.leaves}
    order, ready = [], [leaf for leaf in group.leaves if not waiting[id(leaf)]]
    while ready:
        order.append(min(ready, key=lambda leaf: index[id(leaf)]))
        ready.remove(order[-1])
        for edge in (edge for edge in flow if edge.src is order[-1]):
            waiting[id(edge.dst)] -= 1
            ready += [edge.dst] if not waiting[id(edge.dst)] else []
    stuck = {id(leaf) for leaf in group.leaves} - {id(leaf) for leaf in order}
    group.order = order + [leaf for leaf in group.leaves if id(leaf) in stuck]
    if stuck:
        cycle = [edge for edge in flow if id(edge.src) in stuck and id(edge.dst) in stuck]
        while ends := {id(edge.dst) for edge in cycle} - {id(edge.src) for edge in cycle}:
            cycle = [edge for edge in cycle if id(edge.dst) not in ends]  # keep the cycle, not what hangs below it
        closing = max(cycle, key=lambda edge: edge.order)
        port = f"{path(closing.dst)}.{closing.dst_port}"
        return [problem("rateless_cycle" if rateless else "cycle", port=port, at=closing.at, first=group.at)]
    failures, steppable = [], set()
    for leaf in group.order if rateless else ():
        ports, upstream = (
            type(leaf)._lk_ports,
            {id(edge.src) for edge in flow if edge.dst is leaf and not _lagged(edge)},
        )
        if any(ports[name].queue for name in type(leaf)._lk_ins) or upstream & steppable:
            steppable.add(id(leaf))
        elif leaf._lk_language == "python":
            kind = "never_steps" if type(leaf)._lk_ins else "source"
            failures.append(problem(kind, child=path(leaf), first=leaf._lk.site, at=group.at))
    return failures


def _lagged(edge: Edge) -> bool:
    return type(edge.dst)._lk_ports[edge.dst_port].lagged


def _check_edge(edge: Edge, crosses: str | None) -> list[str]:
    port, src = f"{path(edge.dst)}.{edge.dst_port}", f"{path(edge.src)}.{edge.src_port}"
    kind = type(edge.src)._lk_ports[edge.src_port].type
    if crosses and _lagged(edge):
        return [problem("lagged", port=port, src=src, at=edge.at)]
    if crosses in ("process", "language") and not is_message(kind):
        return [problem("not_message", port=port, got=type_name(kind), crosses=crosses, at=edge.at)]
    return []


def _check_processes(groups: list[Group], backend: ModuleType, realtime: bool, at: str) -> list[str]:
    capabilities = backend.CAPABILITIES
    failures = [] if realtime or "lockstep" in capabilities else [problem("lockstep", backend=backend.NAME, at=at)]
    for process in dict.fromkeys(group.process for group in groups):
        members = [group for group in groups if group.process == process]
        natives = [path(leaf) for group in members if group.language == "native" for leaf in group.leaves]
        if (
            "mixed_process" not in capabilities
            and natives
            and len(natives) < sum(len(group.leaves) for group in members)
        ):
            places = ", ".join(group.at for group in members)
            failures.append(
                problem("mixed_process", child=", ".join(natives), process=process, backend=backend.NAME, at=places)
            )
    if "processes" in capabilities:  # each process rebuilds its components as module:qualname(**params)
        for leaf in (leaf for group in groups if group.language == "python" for leaf in group.leaves):
            params = leaf._lk_params()
            # WHY repr (CONTRACT §15 A33's Python side): NaN != NaN, so a natural "NaN = off" default never compared
            # equal after its JSON round trip; json writes and reads NaN and +-Infinity as Python's json does
            again = json.loads(json.dumps(params, default=str))
            if "<locals>" in type(leaf).__qualname__ or repr(again) != repr(params):
                failures.append(problem("rebuild", child=path(leaf), at=leaf._lk.site))
    return failures


# ---------------------------------------------------------------- SIGTERM and SIGHUP (CONTRACT §15 A18)


class _Ended(BaseException):
    """Raised by the signal handler into a start() in progress on the main thread, so it unwinds through the backend,
    which stops what it had started."""

    def __init__(self, number: int) -> None:
        super().__init__(f"signal {number}")
        self.signal = number


class _Ending:
    """SIGTERM and SIGHUP end a deployment the way SIGINT does: every running deployment stops (each component's
    deactivate runs: a controller hands the arm back), then the process ends as the signal would have ended it.
    WHY (consumer-003): systemctl stop, kill and a transport stopping a Python process send SIGTERM, which ended a
    process with no deactivate. Installed by start() only where the application installed nothing (SIG_DFL: never over its
    own handler, nor over nohup's SIG_IGN); put back once no deployment runs. WHY re-raise rather than exit(143):
    systemd counts death by SIGTERM or SIGHUP as a clean stop, and an exit code of 143 as a failure."""

    SIGNALS = (signal.SIGTERM, signal.SIGHUP)

    def __init__(self) -> None:
        self.running: list[Deployment] = []
        self.starting = self.stops = 0  # start()s and stop()s in progress on the main thread
        self.pending: int | None = None  # a signal that came during a stop on the main thread

    def install(self) -> None:
        for number in self.SIGNALS:
            if signal.getsignal(number) == signal.SIG_DFL:
                signal.signal(number, self.handler)

    def handler(self, number: int, frame) -> None:
        if self.stops:
            self.pending = number  # the main thread is stopping a deployment: it ends the process when done
        elif self.starting:
            raise _Ended(number)
        else:
            self.end(number)

    def end(self, number: int) -> None:
        for deployment in reversed(self.running[:]):
            deployment.stop()
        self.restore()
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except (AttributeError, OSError, ValueError):
                pass
        signal.raise_signal(number)  # SIG_DFL again: the process ends as the signal ends it

    def after_stop(self) -> None:
        if self.pending is not None:
            self.end(self.pending)
        elif not self.running:
            self.restore()

    def restore(self) -> None:
        for number in self.SIGNALS:
            if signal.getsignal(number) == self.handler:
                signal.signal(number, signal.SIG_DFL)


_ending = _Ending()


# ---------------------------------------------------------------- the datasheet (CONTRACT §4.4)


def _port_sheet(port) -> dict[str, Any]:
    from like_keras import codec

    hash_value = f"{codec.schema_hash(message_type(port.type)):016x}" if is_message(port.type) else None
    return {
        "dir": "out" if port.output else "in",
        "type": type_name(port.type),
        "hash": hash_value,
        "queue": port.queue,
        "stale": port.stale,
        "lagged": port.lagged,
        "fresh": port.fresh,
        "optional": port.optional,
    }


def _datasheet(plan: Plan, backend: ModuleType, bound: dict[int, dict[str, str]] | None = None) -> dict[str, Any]:
    group_of = {id(leaf): group for group in plan.groups for leaf in group.leaves}

    def transport(edge: Edge) -> str:  # a call inside a group, a slot inside a process, else the backend's transport
        separate = group_of[id(edge.src)].process != group_of[id(edge.dst)].process
        return (
            "call"
            if not plan.crossing[edge]
            else backend.NAME
            if separate and "processes" in backend.CAPABILITIES
            else "slot"
        )

    def leaf_sheet(leaf: Component) -> dict[str, Any]:
        impl = getattr(type(leaf), "_lk_impl", f"{type(leaf).__module__}:{type(leaf).__qualname__}")
        params = json.loads(json.dumps(leaf._lk_params(), default=str))
        ports = {name: _port_sheet(port) for name, port in type(leaf)._lk_ports.items()}
        sheet = {"impl": impl, "language": leaf._lk_language, "params": params, "ports": ports}
        if bound is not None:  # P5: what was bound here and by which key; v0's sheet (no bind=) is unchanged
            sheet.update(bound.get(id(leaf), {"bound_by": "default"}))
        return sheet

    ignored = [group for group in plan.groups if group.language == "python" and (group.fifo or group.cpus)]
    sheet = {
        "like_keras": VERSION,
        "abi": ABI,
        "backend": {"name": backend.NAME, "version": backend.VERSION, "capabilities": list(backend.CAPABILITIES)},
        "system": path(plan.system),
        "components": {path(leaf): leaf_sheet(leaf) for leaf in plan.graph.leaves},
        "edges": [
            {
                "src": f"{path(edge.src)}.{edge.src_port}",
                "dst": f"{path(edge.dst)}.{edge.dst_port}",
                "crosses": plan.crossing[edge] or "none",
                "transport": transport(edge),
                "at": edge.at,
            }
            for edge in plan.graph.edges
        ],
        "groups": [
            {
                "process": group.process,
                "host": group.host,
                "language": group.language,
                "hz": group.hz,
                "cpus": group.cpus,
                "fifo": group.fifo,
                "order": [path(leaf) for leaf in group.order],
            }
            for group in plan.groups
        ],
        "exposed": {
            **{name: str(ref) for name, ref in plan.graph.exposed_out.items()},
            **{name: ", ".join(map(str, refs)) for name, refs in plan.graph.exposed_in.items()},
        },
        "attach": {"socket": backend.address(path(plan.system))},
        "ignored": [
            f"fifo/cpus of the Python group placed at {group.at}: not applied; a GIL-holding thread at "
            "SCHED_FIFO inverts priorities (CONTRACT D7)"
            for group in ignored
        ],
    }
    sheet["hash"] = hashlib.sha256(json.dumps(sheet, sort_keys=True, default=str).encode()).hexdigest()
    return sheet
