"""The frozen mis-wirings (CONTRACT §4.3, criterion 12). Each must be rejected before any process starts.

A case builds a System and a Deployment and returns the Deployment; the line that causes the error carries the marker
`# <- caught here`. The rejection (a ValueError from describe(), or an error raised by that very call) must name the
expected `path.port` (or component path), the fix (EXPECT's keyword), and, when it comes from describe(), the
`cases.py:<line>` of the marked call.
"""

from __future__ import annotations

import os

from kpi.support.lk import BUILD

# case name -> (what the message must name, a word of the fix)
EXPECT: dict[str, tuple[str, str]] = {}


def expect(path: str, fix: str):
    def register(case):
        EXPECT[case.__name__.removeprefix("case_")] = (path, fix)
        return case

    return register


def fake(variant: str) -> str:
    return str(BUILD / f"libfake_native_{variant}.so")


@expect("m.sink.valu", "value")
def case_unknown_input(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink", valu=src.value)  # <- caught here
    return lk.Deployment(s)


@expect("valeu", "value")
def case_unknown_output(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink", value=src.valeu)  # <- caught here
    return lk.Deployment(s)


@expect("m.sink.value", "merge")
def case_input_bound_twice(lk, parts):
    s = lk.System("m")
    first = s.add(parts.Src(), "first")
    second = s.add(parts.Src(), "second")
    sink = s.add(parts.Snk(), "sink", value=first.value)
    s.wire(sink, value=second.value)  # <- caught here
    return lk.Deployment(s)


@expect("m.sink.value", "expose")
def case_input_unbound(lk, parts):
    s = lk.System("m")
    s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink")  # <- caught here
    return lk.Deployment(s)


@expect("m.sink.value", "ServoState")
def case_type_mismatch(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.ServoSnk(), "sink", value=src.value)  # <- caught here
    return lk.Deployment(s)


@expect("m.box.input", "0123456789abcdef")
def case_schema_hash_mismatch(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("skew")), "box", input=src.value)  # <- caught here
    dep = lk.Deployment(s)  # (the .so's hash is read by lk.native; the mismatch is the wire above)
    dep.place(s.src, hz=10)
    dep.place(s.box, hz=10)
    return dep


@expect("abi", "2")
def case_wrong_abi(lk, parts):
    s = lk.System("m")
    s.add(lk.native(fake("abi2")), "box")  # <- caught here
    return lk.Deployment(s)


@expect("m.a.value", "lagged")
def case_same_group_cycle(lk, parts):
    s = lk.System("m")
    a = s.add(parts.Through(), "a")
    b = s.add(parts.Through(), "b", value=a.out)
    s.wire(a, value=b.out)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.a, s.b, hz=10)  # <- caught here
    return dep


@expect("m.lag.value", "group")
def case_lagged_across_groups(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.LagThrough(), "lag", value=src.value)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.lag, hz=10)  # <- caught here
    return dep


@expect("m.join|m.back", "hz")
def case_cycle_in_rateless_group(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    join = s.add(parts.Join(), "join", trigger=src.value)
    back = s.add(parts.Through(), "back", value=join.out)
    s.wire(join, back=back.out)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.join, s.back)  # <- caught here
    return dep


@expect("m.sink", "place")
def case_leaf_never_placed(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink", value=src.value)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    return dep


@expect("m.sink", "once")
def case_leaf_placed_twice(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink", value=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, s.sink, hz=10)
    dep.place(s.sink, hz=10)  # <- caught here
    return dep


@expect("m.box", "language")
def case_mixed_language_group(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("ok")), "box", input=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, s.box, hz=10)  # <- caught here
    return dep


@expect("m.box", "process")
def case_native_and_python_one_process_dora(lk, parts):
    from like_keras.backends import dora

    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("ok")), "box", input=src.value)
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.src, process="shared", hz=10)  # <- caught here
    dep.place(s.box, process="shared", hz=10)  # <- caught here
    return dep


@expect("m.src", "hz")
def case_source_without_hz(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.QSnk(), "sink", value=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src)  # <- caught here
    dep.place(s.sink)
    return dep


@expect("m.sink", "queue")
def case_never_steps(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(parts.Snk(), "sink", value=src.value)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.sink)  # <- caught here
    return dep


@expect("arm controller", "a-zA-Z0-9_.-")
def case_process_name_invalid(lk, parts):
    s = lk.System("m")
    s.add(parts.Src(), "src")
    dep = lk.Deployment(s)
    dep.place(s.src, process="arm controller", hz=10)  # <- caught here
    return dep


@expect("dora", "process")
def case_process_named_dora(lk, parts):
    s = lk.System("m")
    s.add(parts.Src(), "src")
    dep = lk.Deployment(s)
    dep.place(s.src, process="dora", hz=10)  # <- caught here
    return dep


@expect("fifo", "99")
def case_fifo_out_of_range(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("ok")), "box", input=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.box, hz=100, fifo=100)  # <- caught here
    return dep


@expect("22", "cpus")
def case_reserved_cpu(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("ok")), "box", input=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.box, hz=100, cpus=[22])  # <- caught here
    return dep


@expect(str((os.cpu_count() or 1) + 4), "cpus")
def case_cpu_beyond_machine(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(lk.native(fake("ok")), "box", input=src.value)
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.box, hz=100, cpus=[(os.cpu_count() or 1) + 4])  # <- caught here
    return dep


@expect("host", "inproc")
def case_host_on_inproc(lk, parts):
    s = lk.System("m")
    s.add(parts.Src(), "src")
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10, host="rack")  # <- caught here
    return dep


@expect("m.sink.value", "robot_msgs")
def case_python_object_across_processes(lk, parts):
    from like_keras.backends import dora

    s = lk.System("m")
    src = s.add(parts.ObjSrc(), "src")
    s.add(parts.ObjSnk(), "sink", value=src.value)  # <- caught here
    dep = lk.Deployment(s, backend=dora)
    dep.place(s.src, process="writer", hz=10)
    dep.place(s.sink, process="reader")  # <- caught here
    return dep


@expect("m.sink.value", "attach")
def case_exposed_input_also_wired(lk, parts):
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    sink = s.add(parts.QSnk(), "sink", value=src.value)  # <- caught here
    s.expose(sink.value)  # <- caught here
    dep = lk.Deployment(s)
    dep.place(s.src, hz=10)
    dep.place(s.sink)
    return dep


@expect("snk", "name")
def case_duplicate_child_name(lk, parts):
    s = lk.System("m")
    s.add(parts.Snk())
    s.add(parts.Snk())  # <- caught here
    return lk.Deployment(s)


@expect("m.inner.sink.value", "expose")
def case_reach_into_nested_system(lk, parts):
    inner = lk.System("inner")
    inner_src = inner.add(parts.Src(), "src")
    inner.add(parts.Snk(), "sink", value=inner_src.value)
    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(inner, "inner")
    s.wire(inner.sink, value=src.value)  # <- caught here
    return lk.Deployment(s)


@expect("m.src", "hz")
def case_rate_not_honourable(lk, parts):
    s = lk.System("m")
    s.add(parts.Src(), "src")
    dep = lk.Deployment(s)
    dep.place(s.src, hz=0.0)  # <- caught here
    return dep


@expect("queue", "1")
def case_queue_policy_not_honourable(lk, parts):
    """queue=0 is a policy no backend honours (queue=n needs n >= 1, CONTRACT §4.1)."""
    from robot_msgs.sensor import JointState

    class ZeroQueue(lk.Component):
        value = lk.In(JointState, queue=0)  # <- caught here

        def forward(self, value):
            return None

    s = lk.System("m")
    src = s.add(parts.Src(), "src")
    s.add(ZeroQueue(), "sink", value=src.value)  # <- caught here
    return lk.Deployment(s)


CASES = {name.removeprefix("case_"): function for name, function in globals().items() if name.startswith("case_")}
NEEDS_DORA = {"native_and_python_one_process_dora", "python_object_across_processes"}
NEEDS_FAKE_NATIVE = {"schema_hash_mismatch", "wrong_abi", "mixed_language_group", "native_and_python_one_process_dora",
                     "fifo_out_of_range", "reserved_cpu", "cpu_beyond_machine"}
