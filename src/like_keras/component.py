"""Component, In, Out: a black box with typed ports (CONTRACT §4.1); `native`, a C++ box read from its .so (§4.5).

A component names its inputs and outputs and nothing else: never a peer, a transport, a thread or a host. Who
feeds it is a System's business, where it runs a Deployment's. Importing this module imports no robot_msgs (lean
core): port types may be robot_msgs registry names, resolved when a plan is built.
"""

from __future__ import annotations

import dataclasses
import logging
import re
import sys
from typing import Any, Generic, TypeVar

T = TypeVar("T")
ABI = 1
STANDARD_METRICS = frozenset({"steps", "step_p50_s", "step_p99_s", "step_max_s", "period_error_p99_s", "overruns"})
PER_PORT_METRICS = ("input_age_s.", "drops.", "refused_writes.")
FACILITY_PORTS = ("log", "health", "metrics")
MEMBERS = frozenset({"forward", "Params", "params", "activate", "deactivate", "now", "period", "metric"})


def site(depth: int = 2) -> str:
    """`file.py:line` of the user's call: every wiring failure names the call that caused it (CONTRACT §4.3)."""
    frame = sys._getframe(depth)
    return f"{frame.f_code.co_filename.rsplit('/', 1)[-1]}:{frame.f_lineno}"


def snake(name: str) -> str:
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])", "_", name).lower()


def message_type(kind):
    """The class behind a port type: robot_msgs registry names ("control/ServoState") resolve here, lazily."""
    if isinstance(kind, str):
        import robot_msgs

        return robot_msgs.message_types()[kind]
    return kind


def type_name(kind) -> str:
    """`control/ServoState` for a robot_msgs type, `python:module.Name` for any other Python object."""
    if isinstance(kind, str):
        return kind
    return getattr(kind, "type_name", None) or f"python:{kind.__module__}.{kind.__qualname__}"


def is_message(kind) -> bool:
    return isinstance(kind, str) or hasattr(kind, "type_name") and kind.__module__.startswith("robot_msgs")


class _Port(Generic[T]):
    """A port declared as a class attribute; on an instance it reads back as a PortRef, used only for wiring."""

    def __init__(
        self,
        kind,
        output: bool,
        queue: int | None = None,
        stale: float | None = None,
        lagged: bool = False,
        fresh: bool = False,
    ) -> None:
        self.type, self.output, self.queue, self.stale, self.lagged = kind, output, queue, stale, lagged
        self.fresh = fresh
        self.name, self.hash = "", None  # hash: what a native .so declared (CONTRACT §6.3), checked against ours
        # WHY optional (CONTRACT §15 A11): an attach proxy mirrors every exposed In before anyone knows whether this
        # client writes; such an input may stay unwired, gets no slot, and forward gets None for it (A32).
        self.optional = False

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name

    def __get__(self, instance, owner=None):
        return self if instance is None else PortRef(instance, self.name)


class In(_Port[T]):
    def __init__(
        self,
        type: type[T],
        *,
        queue: int | None = None,
        stale: float | None = None,
        lagged=False,
        fresh=False,
        optional=False,
    ) -> None:
        if queue is not None and (isinstance(queue, bool) or not isinstance(queue, int) or queue < 1):
            raise ValueError(f"In(queue={queue!r}): a queue holds n >= 1 messages; queue=None is latest")
        if stale is not None and not stale > 0:
            raise ValueError(f"In(stale={stale!r}): stale is an age in seconds, > 0")
        if lagged and queue is not None:
            raise ValueError("In(lagged=True, queue=...): a lagged input reads the previous step's latest value")
        # WHY fresh (CONTRACT §15 A25): forward gets None unless a value arrived since its last step, as C++ take()
        # says, so a plan sent again counts again and a swap counts the same; a queue's messages are each new already
        if fresh and queue is not None:
            raise ValueError("In(fresh=True, queue=...): every queued message is new already; fresh is for a latest input")
        super().__init__(type, False, queue, stale, lagged, bool(fresh))
        # WHY optional= (CONTRACT §15 A32, consumer-015): A23's meaning in Python, so the Python peer of a C++ box with
        # an optional input swaps in one line: it may stay unwired, and forward then gets None for it
        self.optional = bool(optional)


class Out(_Port[T]):
    def __init__(self, type: type[T]) -> None:
        super().__init__(type, True)


class PortRef:
    """`planner.state`: a port of one component instance, named for wiring (never read for data)."""

    __slots__ = ("owner", "name")

    def __init__(self, owner: Component, name: str) -> None:
        self.owner, self.name = owner, name

    def __repr__(self) -> str:
        return f"{path(self.owner)}.{self.name}"

    def __eq__(self, other: object) -> bool:
        return isinstance(other, PortRef) and other.owner is self.owner and other.name == self.name

    def __hash__(self) -> int:
        return hash((id(self.owner), self.name))


HEALTH_LEVELS = ("ok", "warn", "error")
HEALTH_CODE = re.compile(r"[A-Z][A-Za-z0-9]*(\.[A-Za-z0-9_]+)*")  # robot_msgs DiagnosticRecord.code's rule


class _Health(PortRef):
    """`self.health`: the `health` port, for wiring; called, the component's own level and code, held until it is
    called again (CONTRACT §15 A22). Each round health reports the worse of it and the facility's own level; on a tie
    the component's code, which says more; an Exception in forward always wins."""

    __slots__ = ()

    def __call__(self, level: str, code: str) -> None:
        # WHY checked here: a bad level or code would otherwise raise on the facilities thread, a second later
        if level not in HEALTH_LEVELS:
            raise ValueError(f"health({level!r}, {code!r}): level is one of {', '.join(HEALTH_LEVELS)}")
        if (code or level != "ok") and not HEALTH_CODE.fullmatch(code):
            raise ValueError(f"health({level!r}, {code!r}): code is a PascalCase reason (e.g. 'Refused'), "
                             "required unless the level is ok")
        self.owner._lk.own = (level, code)


def as_ref(port) -> PortRef:
    """A PortRef from `planner.state`, or from `component.log` (the logger is also the `log` port)."""
    ref = port if isinstance(port, PortRef) else getattr(port, "_lk_ref", None)
    if ref is None:
        raise TypeError(f"{port!r} is not a port: wire a port of a component, e.g. arm.state")
    return ref


def path(component: Component) -> str:
    names = []
    while component is not None:
        names.append(component._lk.name)
        component = component._lk.parent
    return ".".join(reversed(names))


class ComponentState:
    """Where a component sits in the tree, and what its facilities report (CONTRACT §8). The runtime writes it;
    health and metrics are read from it once a second, off the step."""

    def __init__(self, name: str) -> None:
        self.name, self.parent, self.site = name, None, ""
        self.reset()

    def reset(self) -> None:
        self.mode, self.now, self.steps, self.started = "initialization", 0.0, 0, 0.0
        self.period = 0.0  # the group's, set before activate (CONTRACT §15 A20)
        self.durations: list[float] = []
        self.period_errors: list[float] = []
        self.overruns = self.overruns_reported = 0
        self.stale: set[str] = set()
        self.ages: dict[str, float] = {}
        self.warning: str | None = None  # the last warning logged this period (health: WarningLogged)
        self.logged_error: str | None = None  # the last error logged this period (health: ErrorLogged, §15 A22)
        self.error: str | None = None  # why forward raised (health: Exception, for good)
        self.own: tuple[str, str] | None = None  # the component's own health(level, code), held (§15 A22)
        self.custom: dict[str, float] = {}
        self.log_sinks: tuple = ()


class Component:
    """A black box: typed `In`/`Out` class attributes, a `forward`, frozen `Params`. Every component also has the
    facility ports `log` (LogRecord), `health` (DiagnosticRecord) and `metrics` (Metrics), never declared."""

    @dataclasses.dataclass(frozen=True)
    class Params:
        """No parameters. A component with parameters declares a @dataclass(frozen=True) class Params."""

    log = Out("std/LogRecord")  # the class's port; on an instance `self.log` is the logger that feeds it
    health = Out("std/DiagnosticRecord")
    metrics = Out("std/Metrics")
    _lk_language = "python"
    _lk_ports: dict[str, _Port] = {}
    _lk_ins: tuple[str, ...] = ()
    _lk_outs: tuple[str, ...] = ()

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        ports: dict[str, _Port] = {}
        for klass in reversed(cls.__mro__):
            ports.update({name: value for name, value in vars(klass).items() if isinstance(value, _Port)})
        clashes = [
            name
            for name, port in ports.items()
            if name in MEMBERS or name in FACILITY_PORTS and port is not vars(Component)[name]
        ]
        if clashes:
            raise TypeError(f"{cls.__qualname__}: port {clashes[0]!r} collides with a Component member; rename it")
        cls._lk_ports = ports
        cls._lk_ins = tuple(name for name, port in ports.items() if not port.output)
        cls._lk_outs = tuple(name for name, port in ports.items() if port.output and name not in FACILITY_PORTS)
        if not (dataclasses.is_dataclass(cls.Params) and cls.Params.__dataclass_params__.frozen):
            raise TypeError(f"{cls.__qualname__}.Params must be a @dataclasses.dataclass(frozen=True)")
        forward = cls.forward
        if forward is not Component.forward and cls._lk_language == "python":
            code = forward.__code__
            names = set(code.co_varnames[1 : code.co_argcount + code.co_kwonlyargcount])
            if not code.co_flags & 0x08 and names != set(cls._lk_ins):  # 0x08: **kwargs
                raise TypeError(
                    f"{cls.__qualname__}.forward takes ({', '.join(sorted(names))}) but its inputs are "
                    f"({', '.join(cls._lk_ins)}): name forward's arguments after the In ports"
                )

    def __init__(self, **params: Any) -> None:
        """Final: `self.params = type(self).Params(**params)`. Subclasses create state in `activate`, not here."""
        self.params = type(self).Params(**params)
        self._lk = ComponentState(snake(type(self).__name__))
        self.log = _Log(self)
        self.health = _Health(self, "health")

    def forward(self, **inputs: Any) -> Any:
        """One step: inputs by name in; one Out returns its value, several a tuple; None is not sent."""
        raise NotImplementedError(f"{type(self).__qualname__} defines no forward()")

    def activate(self) -> None:
        """Create state, open devices. Raise to refuse: the group never starts (CONTRACT §5.7)."""

    def deactivate(self) -> None:
        """Release devices. Must not raise."""

    def now(self) -> float:
        """This step's scheduled time in seconds: epoch in real time, logical under lockstep, 0.0 when eager."""
        return self._lk.now

    def period(self) -> float:
        """The group's period in seconds (1 / hz), 0.0 in a rate-less group or eager; valid from activate() on.
        WHY (CONTRACT §15 A20): a component refuses in activate a placement it cannot serve (a bus exchange
        longer than the tick), before anything moves, instead of showing it as overruns afterwards."""
        return self._lk.period

    def metric(self, name: str, value: float) -> None:
        """A custom metric, published on `metrics` at 1 Hz with the standard ones (which it may not shadow)."""
        if name in STANDARD_METRICS or name.startswith(PER_PORT_METRICS):
            raise ValueError(f"metric {name!r} is a standard metric (CONTRACT §8); give yours another name")
        self._lk.custom[name] = float(value)

    def __call__(self, **inputs: Any) -> Any:
        """Eager: forward on plain values, now() = 0.0, no backend, no thread (a component tested alone)."""
        self._lk.now = 0.0
        return self.forward(**inputs)

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)
        ports = ", ".join(type(self)._lk_ports) or "none"
        raise AttributeError(f"{path(self)} has no port or member {name!r}; its ports: {ports}")

    def _lk_params(self) -> dict[str, Any]:
        return dataclasses.asdict(self.params)


class _Log(logging.Logger):
    """`self.log`: a stdlib logger named after the component path; every record also goes out on the `log` port.

    WHY no propagation: info records at 100 Hz belong on the port, not on a console. Warnings and errors are handed
    to the `like_keras` logger, so they reach the user's handlers (or stderr) as well."""

    def __init__(self, component: Component) -> None:
        super().__init__(path(component), logging.INFO)
        self.propagate = False
        self._lk_ref = PortRef(component, "log")
        self.addHandler(_LogPort(component._lk))


class _LogPort(logging.Handler):
    def __init__(self, state: ComponentState) -> None:
        super().__init__()
        self.state = state

    def emit(self, record: logging.LogRecord) -> None:
        state, message = self.state, record.getMessage()
        if record.levelno >= logging.ERROR:
            state.logged_error = message  # health goes error for this period, with the text (CONTRACT §15 A22)
        elif record.levelno >= logging.WARNING:
            state.warning = message  # health goes warn for this period, with the text (CONTRACT §8)
        if record.levelno >= logging.WARNING:
            logging.getLogger("like_keras").handle(record)
        if state.log_sinks:
            from robot_msgs.std import Header, LogRecord

            value = LogRecord(
                header=Header(stamp=record.created),
                path=record.name,
                tick=state.steps,
                level=record.levelno,
                message=message,
            )
            for sink in state.log_sinks:
                sink(value, state.now)


# ---------------------------------------------------------------- native (CONTRACT §4.5, §6.3)


def native(path: str | Any, config: Any = None) -> Component:
    """A C++ System built into a .so, as a component: its ports are the System's exposed ports, read from
    `lk_describe()` (no construction, no I/O). `config` is a TOML path or a mapping, given to `lk_create` as JSON."""
    import ctypes
    import json
    import os

    library = ctypes.CDLL(os.fspath(path))
    library.lk_abi_version.restype = ctypes.c_uint32
    library.lk_describe.restype = ctypes.c_char_p
    description = json.loads(library.lk_describe())
    abi = library.lk_abi_version()
    if abi != ABI or description.get("abi") != ABI:
        raise ValueError(
            f"{os.fspath(path)}: abi {abi} (lk_describe says {description.get('abi')}), this like_keras "
            f"{description.get('like_keras', '?')!r} speaks abi {ABI}: rebuild the .so against it"
        )
    ports = {}
    for name, spec in description["ports"].items():
        port = (
            In(spec["type"], queue=spec["queue"], stale=spec["stale"], lagged=spec["lagged"])
            if spec["dir"] == "in"
            else Out(spec["type"])
        )
        port.hash = int(spec["hash"], 16)
        port.optional = bool(spec.get("optional"))  # a C++ InOpts{.optional = true} may stay unwired (§15 A23)
        ports[name] = port
    if config is not None and not hasattr(config, "keys"):
        import tomllib

        with open(config, "rb") as file:
            config = tomllib.load(file)
    cls = type(
        description["system"],
        (_Native,),
        {**ports, "__module__": __name__, "_lk_impl": os.fspath(path), "_lk_description": description},
    )
    box = cls()
    # WHY the defaults beneath (CONTRACT §15 A41, consumer-020): the datasheet shows every field the box runs with, as
    # a Python component's does, and lk_create gets that same mapping in both hosts
    box._lk_config = _overlaid(description.get("params", {}), config or {})
    return box


def _overlaid(defaults: dict[str, Any], given: Any) -> dict[str, Any]:
    """`given` laid over `defaults` as LK_PARAMS fills the box: a nested aggregate given in part keeps its other
    defaults; any other value given (a number, a row table) replaces its default whole."""
    shown = dict(defaults)
    for key, value in given.items():
        nested = isinstance(shown.get(key), dict) and hasattr(value, "keys")
        shown[key] = _overlaid(shown[key], value) if nested else value
    return shown


class _Native(Component):
    """A C++ box: its steps run on its own RT thread inside the .so, never in Python (CONTRACT §5.1)."""

    _lk_language = "native"

    def _lk_params(self) -> dict[str, Any]:
        return self._lk_config
