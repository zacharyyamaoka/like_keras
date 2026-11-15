"""System: components wired from outside, in the subscribe direction (CONTRACT §4.2); and the flat graph of leaves
and edges a Deployment places, with every wiring check of §4.3 catalogued in PROBLEMS.

    s.add(PickPlanner(), "planner", target=targets.target, state=arm.state)   # the child's inputs, named
    s.wire(arm, trajectory=planner.trajectory)                                # inputs of a child added earlier

A System is a component: the ports it exposes are its ports when a parent adds it (closure under composition).
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Any, TypeVar

from like_keras.component import FACILITY_PORTS, Component, PortRef, as_ref, message_type, path, site, snake, type_name

C = TypeVar("C", bound=Component)
_ORDER = itertools.count()  # binding order across Systems: a cycle is reported at the binding that closed it

# Every refusal of CONTRACT §4.3 in one place: what is wrong, where (the file:line of the call), then the fix.
PROBLEMS = {
    # wiring calls, refused at the call
    "not_component": "add() takes a Component, got {got} (at {at}). Fix: add an instance, e.g. add(PickPlanner())",
    "has_parent": "{child} is already in {parent} (added at {first}; again at {at}). Fix: a component sits in one System",
    "not_child": "{port}: {child} is not a child of {system} (at {at}). Fix: expose its input in {parent} and wire that port",
    "foreign": "{port} is not a port of a child of {system} (at {at}). Fix: expose a port of one of its children",
    "name": "{name!r} is not a name (at {at}). Fix: use letters, digits and _ (it becomes an attribute)",
    "taken": '{port} is taken (at {first}; again at {at}). Fix: pass another name, e.g. add(component, "{name}_2")',
    "member": "{port} (at {at}) collides with a System member. Fix: pick another name",
    "no_port": "{port} (at {at}): {owner} has no port {name!r}; its inputs: {ins}; its outputs: {outs}. Fix: use one of them",
    "not_input": "{port} is an output (at {at}). Fix: add()/wire() bind inputs: name one of its In ports",
    "writer": "{port} <- {src} (at {at}): a writer is an Out of a sibling or an input {system} exposes. Fix: expose {src} first",
    "twice": "{port} is already bound to {src} (at {first}; again at {at}): an input has one writer. Fix: merge the two in a component of yours",
    "type": "{port} takes {want} but {src} sends {got} (at {at}). Fix: wire an Out of {want}",
    "hash": "{port} <- {src} (at {at}): {side} was built with schema hash {theirs} for {want}; this robot_msgs has {ours}. Fix: rebuild the .so",
    # the graph, refused by describe() and start()
    "no_forward": "{child} (added at {at}) has no forward(). Fix: define forward(self, <its inputs>)",
    "unbound": "{port} is not bound (added at {at}). Fix: bind it with add(..., {name}=<an Out>) or wire(...), or expose it",
    "exposed_wired": "{port} is exposed by {system} (at {first}) and also wired to {src} (at {at}). Fix: an attach client writes an exposed input; drop the expose or the wire",
    "foreign_leaf": "{child} is not in {system} (placed at {at}). Fix: place components of the deployed System",
    "placed_twice": "{child} is placed twice (at {first} and {at}). Fix: place each component exactly once",
    "never_placed": "{child} is never placed (added at {at}). Fix: place it; once any place() exists, every component is placed exactly once",
    "language": "{child} shares a group with Python components (placed at {at}). Fix: one group holds one language: place {child} on its own",
    "process": "process {process!r} (placed at {at}) is not a valid name. Fix: use only [a-zA-Z0-9_.-]",
    "backend_name": "process {process!r} (placed at {at}) is the name of a backend. Fix: name the process something else",
    "hz": "{child}: hz={hz!r} (placed at {at}) is not a rate. Fix: hz > 0, or no hz for a group stepped by its queue inputs",
    "host": "host={host!r} (placed at {at}): the {backend} backend runs every process on this machine. Fix: drop host= or use a backend that places hosts",
    "fifo": "fifo={fifo!r} (placed at {at}) is outside 1-99. Fix: a SCHED_FIFO priority is 1..99",
    "reserved_cpu": "cpus={cpus} (placed at {at}) holds reserved core {reserved}. Fix: pick cpus without {reserved}",
    "cpu_count": "cpus={cpus} (placed at {at}): this machine has cpus 0-{last}. Fix: pick cpus up to {last}",
    "cycle": "{port} closes a same-group cycle (bound at {at}; group placed at {first}). Fix: mark one input of the cycle lagged=True, or place its components in different groups",
    "rateless_cycle": "{port} closes a cycle in a rate-less group (bound at {at}; placed at {first}). Fix: a cycle needs a rate: give the group hz=...",
    "source": "{child} is a source (no inputs) in a group without hz (placed at {at}). Fix: give its group hz=...",
    "never_steps": "{child} can never step in its rate-less group (added at {first}; placed at {at}). Fix: give the group hz=..., or make an input a queue (In(..., queue=n))",
    "lagged": "{port} is lagged=True but its writer {src} runs in another group (bound at {at}). Fix: an edge between groups is already delayed: drop lagged=True or place both in one group",
    "not_message": "{port} carries {got} across a {crosses} (bound at {at}). Fix: only robot_msgs types cross a process or a language: send a robot_msgs message",
    "mixed_process": "{child} shares process {process!r} with Python components (placed at {at}). Fix: under {backend} a native .so runs in a process of its own: give it process=...",
    "rebuild": "{child} (added at {at}) cannot be rebuilt in another process. Fix: define its class at module level and give it JSON-able Params",
    "lockstep": "the {backend} backend cannot run lockstep (realtime=False) (at {at}). Fix: realtime=True or another backend",
    "record_input": "{port} is an input (recorded at {at}). Fix: record the Out that feeds it",
    "record_type": "{port} carries {got} (recorded at {at}). Fix: record ports of robot_msgs types; only they have a wire layout",
}


def problem(kind: str, **fields: Any) -> str:
    return PROBLEMS[kind].format(**fields)


@dataclass(frozen=True)
class Binding:
    dst: PortRef  # an input of a child (or of a nested System's boundary)
    src: PortRef  # an Out of a sibling, or an input this System exposes
    at: str  # file:line of the add/wire call
    order: int


class System(Component):
    """A tree of components and the edges between them, wired from outside. It has no forward: its children run
    in the order the build derives from the edges (CONTRACT §5.3)."""

    def __init__(self, name: str) -> None:
        super().__init__()
        _identifier(name, site())
        self._lk.name = name
        self._children: dict[str, Component] = {}
        self._bindings: list[Binding] = []
        self._exposed: dict[str, tuple[PortRef, str]] = {}  # boundary name -> (the child's port, file:line)

    def add(self, component: C, name: str | None = None, **inputs: PortRef) -> C:
        """Add a child and bind its inputs: `input name = the Out that feeds it`. Returns the child."""
        at = site()
        if not isinstance(component, Component):
            raise TypeError(problem("not_component", got=type(component).__name__, at=at))
        if component._lk.parent is not None:
            parent = path(component._lk.parent)
            raise ValueError(
                problem("has_parent", child=path(component), parent=parent, first=component._lk.site, at=at)
            )
        name = name or (component._lk.name if isinstance(component, System) else snake(type(component).__name__))
        self._claim(name, at, component)
        component._lk.name, component._lk.parent, component._lk.site = name, self, at
        self._children[name] = component
        self._bind(component, inputs, at)
        return component

    def wire(self, component: Component, **inputs: PortRef) -> None:
        """Bind inputs of a child added earlier: for cycles and forward references."""
        at = site()
        if component._lk.parent is not self:
            port, parent = f"{path(component)}.{next(iter(inputs), '')}", path(component._lk.parent or component)
            raise ValueError(
                problem("not_child", port=port, child=path(component), system=path(self), parent=parent, at=at)
            )
        self._bind(component, inputs, at)

    def expose(self, *ports: PortRef, **renamed: PortRef) -> None:
        """Make children's ports this System's boundary: an exposed Out is readable from outside, an exposed In is
        written from outside (a parent's wiring, or an attach client at the top)."""
        at = site()
        for name, port in [*((as_ref(port).name, port) for port in ports), *renamed.items()]:
            ref = as_ref(port)
            if ref.owner._lk.parent is not self:
                raise ValueError(problem("foreign", port=ref, system=path(self), at=at))
            port_of(ref, at)
            self._claim(name, at, PortRef(self, name))
            self._exposed[name] = (ref, at)

    def __getattr__(self, name: str):
        if not name.startswith("_"):
            if name in self.__dict__.get("_children", ()):
                return self._children[name]
            if name in self.__dict__.get("_exposed", ()):
                return PortRef(self, name)
        return super().__getattr__(name)

    def __dir__(self):
        return [*super().__dir__(), *self._children, *self._exposed]

    def _claim(self, name: str, at: str, value: Component | PortRef) -> None:
        """Reserve `name` for a child or an exposed port: unique, and no System member (they are attributes)."""
        _identifier(name, at)
        if name in self._children or name in self._exposed:
            first = self._children[name]._lk.site if name in self._children else self._exposed[name][1]
            raise ValueError(problem("taken", port=f"{path(self)}.{name}", name=name, first=first, at=at))
        if name in FACILITY_PORTS:  # a System has no log/health/metrics of its own: the name is free, and the
            self.__dict__[name] = value  # instance attribute must win over the Component class's port descriptor
        elif hasattr(System, name) or name == "params":
            raise ValueError(problem("member", port=f"{path(self)}.{name}", at=at))

    def _bind(self, component: Component, inputs: dict[str, Any], at: str) -> None:
        for name, source in inputs.items():
            dst, src = PortRef(component, name), as_ref(source)
            dst_port, src_port = port_of(dst, at), port_of(src, at)
            if dst_port.output:
                raise ValueError(problem("not_input", port=dst, at=at))
            sibling = src.owner is not self and src.owner._lk.parent is self
            if not (sibling and src_port.output or src.owner is self and not src_port.output):
                raise ValueError(problem("writer", port=dst, src=src, system=path(self), at=at))
            for binding in self._bindings:
                if binding.dst == dst:
                    raise ValueError(problem("twice", port=dst, src=binding.src, first=binding.at, at=at))
            _check_types(dst, dst_port, src, src_port, at)
            self._bindings.append(Binding(dst, src, at, next(_ORDER)))


def _identifier(name: str, at: str) -> None:
    if not isinstance(name, str) or not name.isidentifier():
        raise ValueError(problem("name", name=name, at=at))


def port_of(ref: PortRef, at: str = ""):
    """The declared port behind a PortRef, through a System's exposed ports."""
    owner = ref.owner
    if isinstance(owner, System) and ref.name in owner._exposed:
        return port_of(owner._exposed[ref.name][0], at)
    port = None if isinstance(owner, System) else type(owner)._lk_ports.get(ref.name)
    if port is None:
        ports = owner._exposed if isinstance(owner, System) else type(owner)._lk_ports
        ins = [name for name in ports if not port_of(PortRef(owner, name)).output]
        outs = [name for name in ports if name not in ins]
        raise ValueError(
            problem(
                "no_port",
                port=ref,
                owner=path(owner),
                name=ref.name,
                at=at,
                ins=", ".join(ins) or "none",
                outs=", ".join(outs) or "none",
            )
        )
    return port


def _check_types(dst: PortRef, dst_port, src: PortRef, src_port, at: str) -> None:
    want, got = dst_port.type, src_port.type
    same = type_name(want) == type_name(got) if isinstance(want, str) or isinstance(got, str) else issubclass(got, want)
    if not same:
        raise ValueError(problem("type", port=dst, src=src, want=type_name(want), got=type_name(got), at=at))
    if dst_port.hash is not None or src_port.hash is not None:  # a native .so: its compiled schema must be ours
        from like_keras import codec

        ours = codec.schema_hash(message_type(want))
        for side, port in ((dst, dst_port), (src, src_port)):
            if port.hash is not None and port.hash != ours:
                raise ValueError(
                    problem(
                        "hash",
                        port=dst,
                        src=src,
                        side=side,
                        theirs=f"{port.hash:016x}",
                        want=type_name(want),
                        ours=f"{ours:016x}",
                        at=at,
                    )
                )


# ---------------------------------------------------------------- the flat graph a Deployment places


@dataclass(frozen=True)
class Edge:
    src: Component  # a leaf
    src_port: str
    dst: Component  # a leaf
    dst_port: str
    at: str
    order: int


@dataclass
class Graph:
    leaves: list[Component]
    edges: list[Edge]
    exposed_out: dict[str, PortRef]  # top-level boundary name -> the leaf Out behind it
    exposed_in: dict[str, list[PortRef]]  # top-level boundary name -> the leaf Ins it feeds (attach writes them)
    failures: list[str]


def leaves_of(component: Component) -> list[Component]:
    if not isinstance(component, System):
        return [component]
    return [leaf for child in component._children.values() for leaf in leaves_of(child)]


def graph(top: System) -> Graph:
    """Resolve every binding through the System tree into leaf-to-leaf edges (CONTRACT §4.3 checks 1 and 6)."""
    systems, stack = [], [top]
    while stack:
        systems.append(stack.pop())
        stack += [child for child in systems[-1]._children.values() if isinstance(child, System)]
    writer = {binding.dst: binding for system in systems for binding in system._bindings}
    exposed_as = {
        inner: (PortRef(system, name), at) for system in systems for name, (inner, at) in system._exposed.items()
    }

    def find_writer(ref: PortRef) -> tuple[Binding | None, PortRef]:
        """The binding that feeds an input (the first one up its chain of exposures), and the outermost exposure."""
        chain = [ref]
        while chain[-1] in exposed_as:
            chain.append(exposed_as[chain[-1]][0])
        return next((writer[link] for link in chain if link in writer), None), chain[-1]

    def leaf_out(ref: PortRef) -> PortRef | None:
        """Down through exposed Outs (and up through boundary inputs used as sources) to the leaf Out."""
        while isinstance(ref.owner, System):
            inner = ref.owner._exposed[ref.name][0]
            if port_of(inner).output:
                ref = inner
                continue
            binding, ref = find_writer(ref)
            if binding is None:
                return None  # fed by a boundary input of the top System: an attach client writes it
            ref = binding.src
        return ref

    leaves, edges, failures = leaves_of(top), [], []
    exposed_in: dict[str, list[PortRef]] = {}
    for leaf in leaves:
        if leaf._lk_language == "python" and type(leaf).forward is Component.forward:
            failures.append(problem("no_forward", child=path(leaf), at=leaf._lk.site))
        for name in type(leaf)._lk_ins:
            ref = PortRef(leaf, name)
            binding, outermost = find_writer(ref)
            if outermost.owner is top and binding is not None:
                expose_at = exposed_as[ref][1]
                failures.append(
                    problem(
                        "exposed_wired", port=ref, system=path(top), src=binding.src, first=expose_at, at=binding.at
                    )
                )
            elif binding is None and outermost.owner is top:
                exposed_in.setdefault(outermost.name, []).append(ref)
            elif binding is None:
                failures.append(problem("unbound", port=ref, name=name, at=leaf._lk.site))
            elif (src := leaf_out(binding.src)) is None:
                exposed_in.setdefault(find_writer(binding.src)[1].name, []).append(ref)
            else:
                edges.append(Edge(src.owner, src.name, leaf, name, binding.at, binding.order))
    exposed_out = {
        name: leaf_out(PortRef(top, name)) for name, (inner, _) in top._exposed.items() if port_of(inner).output
    }
    return Graph(leaves, edges, exposed_out, exposed_in, failures)
