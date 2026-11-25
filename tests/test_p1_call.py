"""P1, the call form (proposals/api/P1-keras-call.md): inside `with lk.System(...)` calling a component with ports is
the wiring; outside a block the same call runs it. Additive: `add`/`wire`/`expose` and every check stay as they are,
and a block builds exactly the graph the same `add`/`wire` lines build.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_p1_call.py
"""

from __future__ import annotations

import inspect

import pytest

import like_keras as lk
from like_keras.record import read
from robot_msgs.std import Header


class Clock(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        return Header(stamp=self.now())


class Echo(lk.Component):
    value = lk.In(Header)
    out = lk.Out(Header)

    def forward(self, value):
        return value


class Sum(lk.Component):
    """out = a + b + the previous out (b is lagged: the loop closes on one thread)."""

    a = lk.In(float)
    b = lk.In(float, lagged=True)
    out = lk.Out(float)

    def activate(self):
        self.activations = getattr(self, "activations", 0) + 1

    def forward(self, a, b):
        return (a or 0.0) + (b or 0.0)


class Constant(lk.Component):
    value = lk.Out(float)

    def forward(self):
        return 1.0


def chain_v0() -> lk.System:
    s = lk.System("chain")
    clock = s.add(Clock(), "clock")
    first = s.add(Echo(), "first", value=clock.value)
    second = s.add(Echo(), "second")
    s.wire(second, value=first.out)
    s.expose(second.out)
    return s


def chain_p1() -> lk.System:
    with lk.System("chain") as s:
        clock, first, second = Clock(), Echo(), Echo()
        first(value=clock.value)
        second(value=first.out)
        s.expose(second.out)
    return s


def plan_without_sites(dep: lk.Deployment) -> dict:
    sheet = dep.describe()
    for edge in sheet["edges"]:
        edge.pop("at")
    sheet.pop("hash")
    return sheet


def test_a_block_builds_the_graph_add_and_wire_build():
    sheets = []
    for build in (chain_v0, chain_p1):
        cell = build()
        dep = lk.Deployment(cell, realtime=False)
        dep.place(cell.clock, hz=10)
        dep.place(cell.first, cell.second, hz=10)
        sheets.append(plan_without_sites(dep))
    assert sheets[0] == sheets[1]


def test_a_block_runs_what_add_and_wire_run(tmp_path):
    recorded = []
    for build in (chain_v0, chain_p1):
        cell = build()
        dep = lk.Deployment(cell, realtime=False)
        dep.place(cell.clock, hz=10)
        dep.place(cell.first, cell.second, hz=10)
        dep.record(tmp_path / f"{build.__name__}.mcap")
        dep.start(until=1.0)
        dep.stop()
        recorded.append(read(tmp_path / f"{build.__name__}.mcap"))
    assert recorded[0] == recorded[1] and len(recorded[0][1]) >= 10


def test_children_are_named_after_their_variables_else_as_add_names_them():
    with lk.System("cell") as s:
        planner = Echo()
        Clock()  # nobody holds it: snake_case of its class, as add() names it
        ticker = Clock()
        planner(value=ticker.value)
        rack = s.add(Echo(), "arm_3")  # add() inside a block names a child explicitly
        rack(value=ticker.value)
    assert sorted(s._children) == ["arm_3", "clock", "planner", "ticker"]
    assert s.planner is planner and s.arm_3 is rack


def test_a_component_passed_in_joins_where_it_is_first_called_or_named():
    io = Sum()  # made outside any block, as a builder's argument

    def box(io: lk.Component) -> lk.System:
        with lk.System("box") as s:
            echo = Sum()
            echo(a=io.out)  # io joins: named as a source here
            io(b=echo.out)  # define, then wire: the loop is legal because b is lagged
            s.expose(io.a, echo.out)
        return s

    built = box(io)
    assert built.io is io and built.echo._lk.parent is built


def test_a_builder_called_in_a_block_makes_a_subsystem_and_a_system_is_called_like_a_component():
    def box() -> lk.System:
        with lk.System("box") as s:
            total = Sum()
            total(b=total.out)
            s.expose(total.a, total.out)
        return s

    with lk.System("cell") as cell:
        clock, inner, echo = Clock(), box(), Echo()
        echo(value=clock.value)
        inner(a=Constant().value)
    assert cell.inner._lk.parent is cell and cell.inner.total._lk.parent is cell.inner
    assert sorted(cell._children) == ["clock", "constant", "echo", "inner"]


def test_the_call_returns_the_component_so_outputs_read_as_attributes():
    with lk.System("cell") as s:
        clock = Clock()
        last = Echo()(value=Echo()(value=clock.value).out)
        s.expose(last.out)
    assert s._exposed["out"][0].owner is last


def test_one_call_two_meanings_never_switch_silently():
    here = inspect.currentframe().f_lineno
    with pytest.raises(TypeError, match=rf"passes a port outside a System \(at test_p1_call.py:{here + 2}\).*with lk.System"):
        Echo()(value=Clock().value)
    with pytest.raises(TypeError, match=r"passes a value inside `with lk.System\(...\)`.*Fix: pass a port"):
        with lk.System("cell"):
            Echo()(value=Header(stamp=0.0))
    with pytest.raises(TypeError, match=r"positional argument.*Fix: name each input, e.g. echo\(value=...\)"):
        Echo()(Header(stamp=0.0))


def test_eager_outside_a_block_is_v0s_call():
    assert Echo()(value=Header(stamp=3.0)).stamp == 3.0


def test_a_wrong_port_is_refused_at_the_call_with_its_line():
    with pytest.raises(ValueError) as caught:
        with lk.System("cell"):
            clock, echo = Clock(), Echo()
            echo(valu=clock.value)  # <- the call
    assert "echo.valu" in str(caught.value) and "its inputs: value" in str(caught.value)
    line = next(number for number, text in enumerate(open(__file__), 1) if "# <- the call" in text)
    assert f"test_p1_call.py:{line}" in str(caught.value)


def test_a_same_group_cycle_without_a_lag_is_refused_naming_the_call():
    class Plain(lk.Component):
        value = lk.In(float)
        out = lk.Out(float)

        def forward(self, value):
            return value

    with lk.System("loop") as s:
        a, b = Plain(), Plain()
        a(value=b.out)
        b(value=a.out)  # closes the cycle
    dep = lk.Deployment(s)
    dep.place(s.a, s.b, hz=10)
    line = next(number for number, text in enumerate(open(__file__), 1) if "# closes the cycle" in text)
    with pytest.raises(ValueError, match=rf"loop\.b\.value closes a same-group cycle \(bound at test_p1_call.py:{line}"):
        dep.describe()


def test_a_port_of_another_systems_child_is_refused_at_the_close():
    with lk.System("other") as other:
        far = Clock()
    with pytest.raises(ValueError, match=r"cell.echo.value <- other.far.value.*a writer is an Out of a sibling"):
        with lk.System("cell"):
            Echo()(value=far.value)
    assert other.far is far


def test_a_system_called_with_values_runs_one_step_of_its_children():
    with lk.System("acc") as acc:
        total = Sum()
        total(b=total.out)  # a lagged loop: the running sum
        acc.expose(total.a, total.out)
    assert [acc(a=1.0) for _ in range(4)] == [1.0, 2.0, 3.0, 4.0]
    assert acc.total.activations == 1, "each leaf is activated once, at its first eager step"
    with pytest.raises(ValueError, match="exposes no input 'b'"):
        acc(b=1.0)


def test_summary_prints_children_feeds_exposed_ports_and_the_step_order(capsys):
    with lk.System("cell") as s:
        clock, first, second = Clock(), Echo(), Echo()
        second(value=first.out)
        first(value=clock.value)
        s.expose(second.out)
    text = s.summary()
    assert capsys.readouterr().out.strip() == text
    assert text.splitlines()[0] == "cell: 3 children, 3 leaves; one group steps: clock, first, second"
    assert "  second  Echo   value <- first.out" in text and "  exposed: out <- second.out" in text


def test_a_block_that_raises_leaves_nothing_open():
    with pytest.raises(RuntimeError, match="boom"):
        with lk.System("cell"):
            Clock()
            raise RuntimeError("boom")
    assert Echo()(value=Header(stamp=1.0)).stamp == 1.0  # no block is left open: the call runs
