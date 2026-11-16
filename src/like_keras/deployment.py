"""Deployment: where components run (CONTRACT §4.4). Each `place()` call is one group, one thread; each distinct
`process` one OS process (collapsed into this one under `inproc`); the backend carries what crosses.

`describe()` and `start()` run every check of §4.3 first and raise one ValueError listing every failure (the texts
are system.PROBLEMS), then `describe()` returns the plan as JSON-ready data: the device datasheet.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import pkgutil
import re
from collections.abc import Sequence
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any

from like_keras import backends
from like_keras.backends import inproc
from like_keras.component import ABI, Component, PortRef, as_ref, is_message, message_type, path, site, type_name
from like_keras.system import Edge, Graph, System, graph, leaves_of, port_of, problem

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
    def __init__(self, system: System, backend: ModuleType = inproc, realtime: bool = True) -> None:
        self._system, self._backend, self._realtime, self._at = system, backend, realtime, site()
        self._places: list[tuple[tuple[Component, ...], dict[str, Any], str]] = []
        self._recordings: list[tuple[str, list[PortRef], str]] = []
        self._run = None

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
        self._run = self._backend.start(self._plan(), until)
        if until is not None:
            self.stop()
        return self

    def stop(self) -> None:
        """Deactivate in reverse order, join the threads, close the recordings. Idempotent."""
        run, self._run = self._run, None
        if run is not None:
            run.stop()

    def describe(self) -> dict[str, Any]:
        """Run every check (CONTRACT §4.3), then return the plan: the device datasheet."""
        return self._plan().datasheet

    def _plan(self) -> Plan:
        found, backend = graph(self._system), self._backend
        failures = list(found.failures)
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
            (file, self._recorded(found, crossing, ports, at, failures)) for file, ports, at in self._recordings
        ]
        if failures:
            raise ValueError(
                f"{path(self._system)}: {len(failures)} problem(s) found before start (CONTRACT §4.3):\n"
                + "\n".join(f"- {failure}" for failure in failures)
            )
        plan = Plan(self._system, groups, found, crossing, recordings, self._realtime)
        plan.datasheet = _datasheet(plan, backend)
        return plan

    def _groups(self, found: Graph, failures: list[str]) -> list[Group]:
        for leaf in found.leaves:  # the logger carries the path its component has in this deployment
            leaf.log.name = path(leaf)
        if not self._places:
            return [Group(list(found.leaves), at="no place() call: one group, no hz")]
        known, placed, groups = {id(leaf) for leaf in found.leaves}, {}, []
        for components, options, at in self._places:
            groups.append(Group([], **options, at=at))
            for leaf in (leaf for component in components for leaf in leaves_of(component)):
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
            leaf = ref
            while isinstance(leaf.owner, System):
                leaf = leaf.owner._exposed[leaf.name][0]
            kind = port_of(leaf).type
            if is_message(kind):
                topic = getattr(type(leaf.owner), "_lk_topics", {}).get(leaf.name, topic)  # a replay keeps its channel
                chosen.setdefault((id(leaf.owner), leaf.name), (leaf.owner, leaf.name, topic))
            elif explicit:
                failures.append(problem("record_type", port=ref, got=type_name(kind), at=at))

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
            if "<locals>" in type(leaf).__qualname__ or json.loads(json.dumps(params, default=str)) != params:
                failures.append(problem("rebuild", child=path(leaf), at=leaf._lk.site))
    return failures


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
    }


def _datasheet(plan: Plan, backend: ModuleType) -> dict[str, Any]:
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
        return {"impl": impl, "language": leaf._lk_language, "params": params, "ports": ports}

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
