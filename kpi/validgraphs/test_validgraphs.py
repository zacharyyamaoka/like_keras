"""Criterion 12's other half: frozen valid graphs that describe() must accept (any false positive scores 0)."""

from __future__ import annotations

import json

import pytest

from kpi.support.lk import BUILD, like_keras


def fake_ok() -> str:
    path = BUILD / "libfake_native_ok.so"
    if not path.exists():
        pytest.skip(f"{path} not built")
    return str(path)


def graph_linear(lk, parts):
    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    through = s.add(parts.Through(), "through", value=src.value)
    s.add(parts.Snk(), "sink", value=through.out)
    dep = lk.Deployment(s)
    dep.place(s.src, s.through, s.sink, hz=10)
    return dep


def graph_fan_out(lk, parts):
    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "left", value=src.value)
    s.add(parts.Snk(), "right", value=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, s.left, s.right, hz=10)
    return dep


def graph_lagged_cycle(lk, parts):
    s = lk.System("v")
    lag = s.add(parts.LagThrough(), "lag")
    through = s.add(parts.Through(), "through", value=lag.out)
    s.wire(lag, value=through.out)
    dep = lk.Deployment(s)
    dep.place(s.lag, s.through, hz=10)
    return dep


def graph_cross_group_cycle(lk, parts):
    """A cycle across groups needs no lag: every crossing edge is already delayed (§5.4)."""
    s = lk.System("v")
    a = s.add(parts.Through(), "a")
    b = s.add(parts.Through(), "b", value=a.out)
    s.wire(a, value=b.out)
    dep = lk.Deployment(s)
    dep.place(s.a, hz=10)
    dep.place(s.b, hz=20)
    return dep


def graph_rateless_queue(lk, parts):
    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    through = s.add(parts.QThrough(), "through", value=src.value)
    s.add(parts.Snk(), "sink", value=through.out)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.through, s.sink)
    return dep


def graph_nested_system(lk, parts):
    inner = lk.System("inner")
    through = inner.add(parts.Through(), "through")
    inner.expose(through.value, through.out)
    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    s.add(inner, "inner", value=src.value)
    s.add(parts.Snk(), "sink", value=inner.out)
    dep = lk.Deployment(s)
    dep.place(s.src, s.inner, s.sink, hz=10)
    return dep


def graph_no_place(lk, parts):
    """With no place() at all every leaf runs in one group in process main; a graph with no source needs no hz."""
    s = lk.System("v")
    through = s.add(parts.QThrough(), "through")
    s.add(parts.Snk(), "sink", value=through.out)
    s.expose(through.value)
    return lk.Deployment(s)


def graph_exposed_input_unwired(lk, parts):
    """An exposed input of the top-level System is the attach clients' to write, so it is left unwired inside."""
    s = lk.System("v")
    sink = s.add(parts.QSnk(), "sink")
    s.expose(sink.value)
    dep = lk.Deployment(s)
    dep.place(s.sink)
    return dep


def graph_fifo_on_python_ignored(lk, parts):
    """fifo/cpus on a Python group are not applied and are listed under `ignored` (D7), never refused."""
    s = lk.System("v")
    s.add(parts.Src(), "src")
    dep = lk.Deployment(s)
    dep.place(s.src, hz=100, cpus=[0], fifo=80)
    return dep


def graph_python_object_in_process(lk, parts):
    s = lk.System("v")
    src = s.add(parts.ObjSrc(), "src")
    s.add(parts.ObjSnk(), "sink", value=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.sink)
    return dep


def graph_multi_process_dora(lk, parts):
    from like_keras.backends import dora

    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    s.add(parts.QSnk(), "sink", value=src.value)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.src, process="writer", hz=10)
    dep.place(s.sink, process="reader")
    return dep


def graph_native_group(lk, parts):
    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    box = s.add(lk.native(fake_ok()), "box", input=src.value)
    s.add(parts.QSnk(), "sink", value=box.output)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.box, process="controller", hz=100, cpus=[0], fifo=80)
    dep.place(s.sink)
    return dep


def graph_native_own_process_dora(lk, parts):
    from like_keras.backends import dora

    s = lk.System("v")
    src = s.add(parts.Src(), "src")
    box = s.add(lk.native(fake_ok()), "box", input=src.value)
    s.add(parts.QSnk(), "sink", value=box.output)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.src, hz=10)
    dep.place(s.box, process="controller", hz=100)
    dep.place(s.sink)
    return dep


def graph_gather(lk, parts):
    s = lk.System("v")
    first = s.add(parts.Src(), "first")
    second = s.add(parts.Src(), "second")
    s.add(parts.Pair(), "pair", left=first.value, right=second.value)
    dep = lk.Deployment(s)
    dep.place(s.first, hz=10)
    dep.place(s.second, hz=5)
    dep.place(s.pair)
    return dep


GRAPHS = {name.removeprefix("graph_"): function for name, function in globals().items() if name.startswith("graph_")}


@pytest.mark.parametrize("name", sorted(GRAPHS))
def test_valid_graph_accepted(name):
    lk = like_keras()
    from kpi.miswire import parts

    plan = GRAPHS[name](lk, parts).describe()
    json.dumps(plan)
    if name == "fifo_on_python_ignored":
        assert plan["ignored"], "fifo/cpus on a Python group must be listed under ignored"


def test_reference_cell_describes():
    """The reference consumer's fallback deployment describes; its derived order is bus -> jtc -> guard."""
    like_keras()
    import tempfile
    from pathlib import Path

    from kpi.support import cells

    with tempfile.TemporaryDirectory() as scratch:
        plan = cells.deployment("fallback", Path(scratch), lockstep=True).describe()
    orders = [group["order"] for group in plan["groups"] if any("arm" in path for path in group["order"])]
    assert orders and [path.rsplit(".", 1)[-1] for path in orders[0]] == ["bus", "jtc", "guard"], orders
