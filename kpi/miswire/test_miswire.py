"""Criterion 12: every frozen mis-wiring in cases.py is rejected before any process starts, naming `path.port`,
the fix and the `file:line` of the call that caused it (CONTRACT §4.3). Valid graphs: kpi/validgraphs."""

from __future__ import annotations

import inspect
import threading

import pytest

from kpi.miswire import cases
from kpi.support.lk import like_keras


def marked_lines(case) -> set[int]:
    lines, first = inspect.getsourcelines(case)
    return {first + index for index, line in enumerate(lines) if "# <- caught here" in line}


@pytest.mark.parametrize("name", sorted(cases.CASES))
def test_miswire_rejected(name, request):
    lk = like_keras()
    if name in cases.NEEDS_DORA:
        pytest.importorskip("like_keras.backends.dora")
    if name in cases.NEEDS_FAKE_NATIVE and not (cases.BUILD / "libfake_native_ok.so").exists():
        pytest.skip("build/kpi/libfake_native_*.so not built (cmake --build /home/bam/like_keras/build)")
    from kpi.miswire import parts

    threads_before = threading.active_count()
    case = cases.CASES[name]
    path_alternatives, fix = cases.EXPECT[name]
    raised_at_call = True
    try:
        dep = case(lk, parts)
        raised_at_call = False
        dep.describe()
    except (ValueError, TypeError, AttributeError) as error:
        message = str(error)
    else:
        pytest.fail(f"{name}: describe() accepted a mis-wiring")
    if not raised_at_call:  # start() runs the same checks and refuses before anything starts
        try:
            dep.start()
        except ValueError:
            pass
        else:
            dep.stop()
            pytest.fail(f"{name}: start() accepted a graph describe() rejected")
    assert threading.active_count() == threads_before, "a thread was started for a rejected graph"
    assert any(path in message for path in path_alternatives.split("|")), f"{name}: {message!r} names none of {path_alternatives}"
    assert fix.lower() in message.lower(), f"{name}: {message!r} does not name the fix ({fix!r})"
    if not raised_at_call:
        where = {f"cases.py:{line}" for line in marked_lines(case)}
        assert any(spot in message for spot in where), f"{name}: {message!r} names none of {sorted(where)}"
