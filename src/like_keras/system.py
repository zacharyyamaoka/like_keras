"""System: components wired from outside, in the subscribe direction (CONTRACT §4.2); and the flat graph of leaves
and edges a Deployment places, with every wiring check of §4.3 catalogued in PROBLEMS.

    s.add(PickPlanner(), "planner", target=targets.target, state=arm.state)   # the child's inputs, named
    s.wire(arm, trajectory=planner.trajectory)                                # inputs of a child added earlier

or, the same wiring as calls (P1, the Keras form), inside a block:

    with lk.System("pick_cell") as s:
        planner = PickPlanner()
        planner(target=targets.target, state=arm.state)   # the call binds inputs; it runs nothing
        arm(trajectory=planner.trajectory)
        s.expose(arm.state)

A component joins the block it is made in, or the one where it is first called, exposed or named as a source; when
the block closes each child is named after the local variable holding it (else as `add` names it), and the calls
are bound in the order they were written, exactly as `add`/`wire` bind them.

A System is a component: the ports it exposes are its ports when a parent adds it (closure under composition), and
its own `log`, `health` and `metrics` carry every component inside it, as a native box's do (CONTRACT §15 A21).
"""

from __future__ import annotations

import itertools
import sys
from dataclasses import dataclass, field
from typing import Any, TypeVar

from like_keras.component import (
    FACILITY_PORTS,
    Component,
    PortRef,
    _blocks,
    as_ref,
    call_wires,
    message_type,
    path,
    site,
    snake,
    type_name,
    user_site,
)

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
    "bind_key": "bind key {key} (Deployment at {at}) is neither a node path nor a Component class. Fix: key by a role class, bind={{Arm: ...}}, or by a node path, bind={{\"arm\": ...}}",
    "bind_path": "bind[{key!r}] (Deployment at {at}) names no node of {system}; its nodes: {nodes}. Fix: use one of them",
    "bind_class": "bind[{key}] (Deployment at {at}) matches no node of {system}. Fix: drop it, or key by a class the System uses",
    "bind_value": "{how} = a {got} (Deployment at {at}) is not a component. Fix: bind a component instance (a template: each Deployment builds its own)",
    "bind_parent": "{how} = System {got!r} (Deployment at {at}) is already in {parent}. Fix: bind a System of its own (build one per Deployment)",
    "bind_ports": "{how} = {impl} (Deployment at {at}) does not fit {node}: {wrong}. Fix: bind an implementation with {node}'s ports",
    "unbound_role": "{child} is a role ({role}: ports and no forward; added at {at}) and nothing binds it. Fix: bind an implementation, Deployment(..., bind={{{role}: <an implementation>}}) or bind={{\"{node}\": ...}}, or define forward",
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
    "exposed_facility": "{port} is exposed by {system} (at {at}), but a System's {name} carries every component inside it and v0 exposes one leaf's port. Fix: expose each child's {name} under its own name, or wire {port} into a component",
}


def problem(kind: str, **fields: Any) -> str:
    return PROBLEMS[kind].format(**fields)


@dataclass(frozen=True)
class Binding:
    dst: PortRef  # an input of a child (or of a nested System's boundary)
    src: PortRef  # an Out of a sibling, or an input this System exposes
    at: str  # file:line of the add/wire call
    order: int


@dataclass
class _Block:
    """An open `with lk.System(...)`: who joins it, and the calls and exposes to apply, in order, when it closes."""

    members: dict[int, list] = field(default_factory=dict)  # id -> [component, explicit name or None, file:line]
    wires: list[tuple] = field(default_factory=list)  # (component, inputs, file:line, order)
    exposes: list[tuple] = field(default_factory=list)  # (ports, renamed, file:line)


def is_role(component: Component) -> bool:
    """A role (P5): a Python leaf class with ports and no forward, the datasheet without the device. A Deployment's
    `bind=` chooses what runs in its place; unbound, it is refused before start."""
    return (
        not isinstance(component, System)
        and component._lk_language == "python"
        and type(component).forward is Component.forward
        and bool(type(component)._lk_ins or type(component)._lk_outs)
    )


class System(Component):
    """A tree of components and the edges between them, wired from outside. It has no forward: its children run
    in the order the build derives from the edges (CONTRACT §5.3)."""

    def __init__(self, name: str) -> None:
        super().__init__()  # made inside a parent's block (a builder called there), it joins that parent
        _identifier(name, site())
        self._lk.name = name
        self._children: dict[str, Component] = {}
        self._bindings: list[Binding] = []
        self._exposed: dict[str, tuple[PortRef, str]] = {}  # boundary name -> (the child's port, file:line)
        self._block: _Block | None = None

    def add(self, component: C, name: str | None = None, **inputs: PortRef) -> C:
        """Add a child and bind its inputs: `input name = the Out that feeds it`. Returns the child. Inside this
        System's open block it names the child explicitly (a for loop's `arm_3`), bound when the block closes."""
        at = site()
        if not isinstance(component, Component):
            raise TypeError(problem("not_component", got=type(component).__name__, at=at))
        if component._lk.parent is not None:
            parent = path(component._lk.parent)
            raise ValueError(
                problem("has_parent", child=path(component), parent=parent, first=component._lk.site, at=at)
            )
        if self._block is not None:
            pending = component._lk.block
            if pending not in (None, self):  # an explicit add() wins over the block it was made in
                del pending._block.members[id(component)]
                component._lk.block = None
            self._lk_join(component, at, name)
            self._lk_call(component, inputs, at)
            return component
        self._adopt(component, name, at)
        self._bind(component, inputs, at)
        return component

    def wire(self, component: Component, **inputs: PortRef) -> None:
        """Bind inputs of a child added earlier: for cycles and forward references."""
        at = site()
        if self._block is not None:
            self._lk_call(component, inputs, at)
            return
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
        if self._block is not None:
            for port in [*ports, *renamed.values()]:
                self._lk_join(as_ref(port).owner, at)
            self._block.exposes.append((ports, renamed, at))
            return
        self._expose(ports, renamed, at)

    # ------------------------------------------------------------ P1: the call form, inside a `with` block

    def __enter__(self) -> System:
        if self._block is not None:
            raise RuntimeError(f"{path(self)} is open already (at {user_site()}). Fix: one `with` per System at a time")
        self._block = _Block()
        _blocks.__dict__.setdefault("stack", []).append(self)
        return self

    def __exit__(self, kind, error, trace) -> None:
        block, self._block = self._block, None
        _blocks.stack.remove(self)
        for component, _, _ in block.members.values():
            component._lk.block = None
        if kind is None:  # WHY the caller's locals: a child is named after the variable that holds it (P1 §3)
            self._close(block, sys._getframe(1).f_locals)

    def __call__(self, *args: Any, **inputs: Any) -> Any:
        """Inside a parent's block, ports wire this System like any component. Outside, plain values run one step of
        every child in the derived order (one group, now() = 0.0), and the exposed Outs come back as forward's do."""
        if call_wires(self, args, inputs):
            return self
        return self._lk_step(inputs)

    def _lk_join(self, component: Component, at: str, name: str | None = None) -> None:
        """`component` becomes a child when this block closes, unless it has a parent or an open block already."""
        if component is self or component._lk.parent is not None or component._lk.block not in (None, self):
            return
        component._lk.block = self
        entry = self._block.members.setdefault(id(component), [component, None, at])
        entry[1] = name or entry[1]

    def _lk_call(self, component: Component, inputs: dict[str, Any], at: str) -> None:
        """Record `component(**inputs)` for the close; refuse at the call what the two ports alone show wrong."""
        self._lk_join(component, at)
        for name, source in inputs.items():
            dst, src = PortRef(component, name), as_ref(source)
            dst_port = port_of(dst, at)
            if dst_port.output:
                raise ValueError(problem("not_input", port=dst, at=at))
            if src.owner is not self:  # a boundary input of this block is exposed when it closes
                self._lk_join(src.owner, at)
                src_port = port_of(src, at)
                if src_port.output:
                    _check_types(dst, dst_port, src, src_port, at)
        self._block.wires.append((component, dict(inputs), at, next(_ORDER)))

    def _close(self, block: _Block, local_names: dict[str, Any]) -> None:
        named = {}
        for variable, value in local_names.items():
            if isinstance(value, Component) and id(value) in block.members:
                named.setdefault(id(value), variable)
        for component, name, at in block.members.values():
            if component._lk.parent is None:  # else an explicit add() in another System claimed it meanwhile
                self._adopt(component, name or named.get(id(component)), at)
        for ports, renamed, at in block.exposes:
            self._expose(ports, renamed, at)
        for component, inputs, at, order in block.wires:
            if component._lk.parent is not self:
                port, parent = f"{path(component)}.{next(iter(inputs), '')}", path(component._lk.parent or component)
                raise ValueError(
                    problem("not_child", port=port, child=path(component), system=path(self), parent=parent, at=at)
                )
            self._bind(component, inputs, at, order)

    def _lk_step(self, inputs: dict[str, Any]) -> Any:
        """One eager step: every leaf in the order one group derives, lagged inputs from the previous call, each
        leaf activated at its first step. WHY a convenience, not a runtime: a composite tested alone, like a leaf's
        eager call; nothing here touches a backend or a thread."""
        from like_keras.deployment import Group, _derive_order

        found = graph(self)
        group = Group(list(found.leaves), hz=1.0, at="one eager step")  # a rate: a lagged cycle is legal, as in hz
        failures = [*found.failures, *_derive_order(group, found.edges)]
        unknown = sorted(set(inputs) - set(found.exposed_in))
        if failures or unknown:
            known = sorted(found.exposed_in)
            failures += [f"{path(self)} exposes no input {name!r}; its inputs: {known}" for name in unknown]
            raise ValueError(f"{path(self)}: cannot step:\n" + "\n".join(f"- {failure}" for failure in failures))
        state = self.__dict__.setdefault("_lk_eager", {"latest": {}, "active": set()})
        latest, before = state["latest"], dict(state["latest"])
        feeds = {(id(edge.dst), edge.dst_port): (id(edge.src), edge.src_port) for edge in found.edges}
        boundary = {(id(ref.owner), ref.name): name for name, refs in found.exposed_in.items() for ref in refs}
        for leaf in group.order:
            leaf._lk.now = 0.0
            if id(leaf) not in state["active"]:
                leaf.activate()
                state["active"].add(id(leaf))
            values = {}
            for name in type(leaf)._lk_ins:
                key = (id(leaf), name)
                seen = before if type(leaf)._lk_ports[name].lagged else latest  # lagged: the previous step's value
                values[name] = inputs.get(boundary[key]) if key in boundary else seen.get(feeds.get(key))
            result, outs = leaf.forward(**values), type(leaf)._lk_outs
            for port, value in zip(outs, (result,) if len(outs) == 1 else result or (None,) * len(outs)):
                if value is not None:
                    latest[(id(leaf), port)] = value
        exposed = [latest.get((id(ref.owner), ref.name)) for ref in found.exposed_out.values()]
        return exposed[0] if len(exposed) == 1 else tuple(exposed) if exposed else None

    def summary(self) -> str:
        """Print and return this System's children, what feeds each input, the exposed ports, and the order one
        group steps the leaves in (CONTRACT §5.3): the wiring an agent reads instead of the builder's code."""
        from like_keras.deployment import Group, _derive_order

        def source(ref: PortRef) -> str:
            return f"[{ref.name}]" if ref.owner is self else f"{ref.owner._lk.name}.{ref.name}"

        lines, width = [], max((len(name) for name in self._children), default=0)
        for name, child in self._children.items():
            count = len(child._children) if isinstance(child, System) else 0
            kind = f"System of {count}" if isinstance(child, System) else type(child).__qualname__
            kind += " (role: the Deployment binds it)" if is_role(child) else ""
            feeds = [
                f"{b.dst.name} <- {source(b.src)}" + (" (lagged)" if port_of(b.dst).lagged else "")
                for b in self._bindings
                if b.dst.owner is child
            ]
            lines.append(f"  {name:<{width}}  {kind}" + (f"   {'; '.join(feeds)}" if feeds else ""))
        for name, (ref, _) in self._exposed.items():
            arrow = "<-" if port_of(ref).output else "->"
            lines.append(f"  exposed: {name} {arrow} {ref.owner._lk.name}.{ref.name}")
        found = graph(self)
        group = Group(list(found.leaves), hz=1.0, at="summary()")
        roles = tuple(f"{path(leaf)} is a role (" for leaf in found.leaves if is_role(leaf))
        problems = [f for f in found.failures if not f.startswith(roles or ("\0",))] + _derive_order(group, found.edges)
        top = path(self)
        order = ", ".join(path(leaf)[len(top) + 1 :] for leaf in group.order)
        head = f"{top}: {len(self._children)} children, {len(found.leaves)} leaves; one group steps: {order}"
        text = "\n".join([head, *lines, *(f"  problem: {problem}" for problem in problems)])
        print(text)
        return text

    def _adopt(self, component: Component, name: str | None, at: str) -> None:
        name = name or (component._lk.name if isinstance(component, System) else snake(type(component).__name__))
        self._claim(name, at, component)
        component._lk.name, component._lk.parent, component._lk.site = name, self, at
        self._children[name] = component

    def _expose(self, ports: tuple, renamed: dict[str, PortRef], at: str) -> None:
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
            block = self.__dict__.get("_block")
            if block is not None:  # an open block: a child named by add(), a port expose() has promised
                for component, given, _ in block.members.values():
                    if given == name:
                        return component
                for ports, renamed, _ in block.exposes:
                    if name in renamed or any(as_ref(port).name == name for port in ports):
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
        if name in FACILITY_PORTS:  # a child or exposed port of that name replaces the System's own (A21): the
            self.__dict__[name] = value  # instance attribute must win over the Component class's port descriptor
        elif hasattr(System, name) or name == "params":
            raise ValueError(problem("member", port=f"{path(self)}.{name}", at=at))

    def _bind(self, component: Component, inputs: dict[str, Any], at: str, order: int | None = None) -> None:
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
            self._bindings.append(Binding(dst, src, at, next(_ORDER) if order is None else order))


def _identifier(name: str, at: str) -> None:
    if not isinstance(name, str) or not name.isidentifier():
        raise ValueError(problem("name", name=name, at=at))


def port_of(ref: PortRef, at: str = ""):
    """The declared port behind a PortRef, through a System's exposed ports; a System's own facility port is the
    Component's (CONTRACT §15 A21)."""
    owner = ref.owner
    if isinstance(owner, System) and ref.name in owner._exposed:
        return port_of(owner._exposed[ref.name][0], at)
    if isinstance(owner, System) and ref.name in FACILITY_PORTS:
        return vars(Component)[ref.name]
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


def out_leaves(ref: PortRef) -> list[PortRef]:
    """The leaf Outs behind an Out, down through exposed Outs. A System's own log, health or metrics is that port of
    every leaf inside it (CONTRACT §15 A21). WHY: a Python <-> C++ swap must not change how a chain's facilities are
    read; a native box's facility ports carry every inner component's records, so a Python System's do too. A
    boundary input met on the way (an exposed In used as a source) is returned as it is."""
    while isinstance(ref.owner, System):
        exposed = ref.owner._exposed.get(ref.name)
        if exposed is None:
            return [PortRef(leaf, ref.name) for leaf in leaves_of(ref.owner)]
        if not port_of(exposed[0]).output:
            return [ref]
        ref = exposed[0]
    return [ref]


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

    def leaf_outs(ref: PortRef) -> list[PortRef]:
        """Down through exposed Outs (and up through boundary inputs used as sources) to the leaf Outs: one, or
        every leaf's for a System's own facility port; none when an attach client writes the top's boundary input."""
        while len(refs := out_leaves(ref)) == 1 and isinstance(refs[0].owner, System):
            binding, ref = find_writer(refs[0])
            if binding is None:
                return []  # fed by a boundary input of the top System: an attach client writes it
            ref = binding.src
        return refs

    leaves, edges, failures = leaves_of(top), [], []
    exposed_in: dict[str, list[PortRef]] = {}
    for leaf in leaves:
        if is_role(leaf):
            role, node = type(leaf).__qualname__, path(leaf).partition(".")[2]
            failures.append(problem("unbound_role", child=path(leaf), role=role, node=node, at=leaf._lk.site))
        elif leaf._lk_language == "python" and type(leaf).forward is Component.forward:
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
                if not type(leaf)._lk_ports[name].optional:  # an optional In (A11, A32) stays unwired: no slot, None
                    failures.append(problem("unbound", port=ref, name=name, at=leaf._lk.site))
            elif not (sources := leaf_outs(binding.src)):
                exposed_in.setdefault(find_writer(binding.src)[1].name, []).append(ref)
            else:  # several sources only for a System's own facility port, fanned in to this one input (A21)
                edges += [Edge(src.owner, src.name, leaf, name, binding.at, binding.order) for src in sources]
    exposed_out = {}
    for name, (inner, at) in top._exposed.items():
        if port_of(inner).output:
            sources = leaf_outs(PortRef(top, name))
            if len(sources) == 1:
                exposed_out[name] = sources[0]
            else:  # WHY refused, not degraded: one exposed name is one leaf's port on the attach socket in v0
                failures.append(problem("exposed_facility", port=inner, system=path(top), at=at, name=inner.name))
    return Graph(leaves, edges, exposed_out, exposed_in, failures)
