"""The acceptance suite's one backend switch (ruler v2, idea I-002): which like_keras backends a kpi run tests.

    python kpi/run.py conformance --backend inproc        (or LK_BACKEND=inproc; a comma list, or the flag repeated)

WHY (Zach, 2026-11-25): "the acceptance tests should be written with the like keras API, so that way if you add a
new backend you can just run the exact same tests". Every backend-parametrized test takes its backends from here, so
`--backend <new>` runs the same tests on a new backend with zero test edits.

- **Default:** the ruler's backends, RULER: exactly ruler v1's test ids, so kpi/score.py's frozen denominators are
  unchanged. The scoreboard always runs the default (kpi/score.py refuses LK_BACKEND).
- **What a test is about** is a `backend(name)` mark, on the test or on one parametrization. The helpers below put
  it there: `names()` (the selected backends themselves), `combinations()` (`python-<backend>` and the like, for
  every selected backend), `cases()` (fixed `<language>-<backend>[-...]` cases), `placements()` (cells.NEEDS_DORA:
  `target` runs on dora, every other placement on inproc) and `imported_by()` (a parametrization runs on the
  backends its own source imports: a swap variant, a task, a graph). A test that is about one backend whatever its
  parameters carries the mark itself: `@pytest.mark.backend("dora")`.
- **The rule** (kpi/conftest.py): a test that names backends runs only when every backend it names is selected. A
  test that names none runs on every selection: it is static, or it runs only inproc as the reference the other
  backends are compared with (the fallback twin, the hand-stepped schedule, a default `lk.Deployment`). Add
  `-m backend` to keep only the tests that name one (`--backend dora -m backend`: dora's own ids).
- **An unknown name** fails the run at once and names the known backends (RULER and every module of
  like_keras.backends), so a typo never quietly collects nothing.
- **A new backend's own setup** is not here: test bodies that branch on `backend_name == "dora"` (the private dora,
  one process per node) take their other branch for it, as for inproc.
"""

from __future__ import annotations

import functools
import importlib.util
import os
import pkgutil
import re
from collections.abc import Callable, Iterable

import pytest

RULER = ("inproc", "dora")  # the backends ruler v1 and v2 score, in v1's parametrization order
ENVIRONMENT = "LK_BACKEND"
IMPORT = re.compile(r"from like_keras\.backends import (\w+)")

_selected: tuple[str, ...] | None = None


@functools.cache
def known() -> tuple[str, ...]:
    """RULER, then every other backend module like_keras ships (src/like_keras/backends/<name>). WHY found on disk,
    not imported: collection must not run like_keras (an empty repo has none, and kpi/support/lk.py then skips)."""
    spec = importlib.util.find_spec("like_keras")
    folders = [os.path.join(folder, "backends") for folder in (spec.submodule_search_locations or [])] if spec else []
    shipped = sorted(module.name for module in pkgutil.iter_modules(folders) if not module.name.startswith("_"))
    return RULER + tuple(name for name in shipped if name not in RULER)


def parse(values: Iterable[str]) -> list[str]:
    """`--backend` values (each a comma list, or the flag repeated) as names, in order, without repeats."""
    names: list[str] = []
    for value in values:
        for name in str(value).split(","):
            if name.strip() and name.strip() not in names:
                names.append(name.strip())
    return names


def select(values: Iterable[str]) -> tuple[str, ...]:
    """Choose the backends of this run (kpi/conftest.py, from `--backend` else LK_BACKEND); none named = RULER.
    Raises ValueError naming the known backends when a name is not one of them."""
    global _selected
    names, choices = parse(values), known()
    unknown = [name for name in names if name not in choices]
    if unknown:
        raise ValueError(f"unknown backend {', '.join(map(repr, unknown))}; the known backends are "
                         f"{', '.join(choices)} (the ruler's, then every other module of like_keras.backends)")
    _selected = tuple(name for name in choices if name in names) if names else RULER  # known() order: RULER first
    return _selected


def selected() -> tuple[str, ...]:
    """The backends this run tests: select()'s choice, else LK_BACKEND's, else RULER."""
    if _selected is None:
        return select(parse([os.environ.get(ENVIRONMENT, "")]))
    return _selected


def mark(*backends: str) -> pytest.MarkDecorator:
    return pytest.mark.backend(*backends)


def names() -> list:
    """The selected backends as parameters (`backend`, `backend_name`): a new backend gets its own test ids."""
    return [pytest.param(name, marks=mark(name)) for name in selected()]


def combinations(*prefixes: str) -> list:
    """`<prefix>-<backend>` for each prefix and each selected backend, prefix first (v1's order: python-inproc,
    python-dora, cpp-inproc, cpp-dora)."""
    return [pytest.param(f"{prefix}-{name}", marks=mark(name)) for prefix in prefixes for name in selected()]


def cases(values: Iterable[str]) -> list:
    """Fixed cases written `<x>-<backend>[-...]` (e.g. py-inproc-SIGTERM), each marked with the backend it names; a
    case that names none (e.g. before_activate, the C ABI alone) stays unmarked."""
    choices = known()
    found = []
    for value in values:
        named = [token for token in value.split("-") if token in choices]
        found.append(pytest.param(value, marks=mark(*named)) if named else value)
    return found


def placement_backend(placement: str) -> str:
    """The backend a placement of the reference cell runs on, as the suite already says it (cells.NEEDS_DORA:
    target on dora; same_box, fallback and the attached planner on inproc)."""
    from kpi.support import cells

    return "dora" if placement in cells.NEEDS_DORA else "inproc"


def placements(*values: str) -> list:
    return [pytest.param(value, marks=mark(placement_backend(value))) for value in values]


def imported_by(values: Iterable[str], source: Callable[[str], str]) -> list:
    """Each value marked with the backends its own source imports (`from like_keras.backends import <name>`); one
    that imports none runs on the default inproc and stays unmarked."""
    choices = known()
    found = []
    for value in values:
        named = sorted({name for name in IMPORT.findall(source(value)) if name in choices})
        found.append(pytest.param(value, marks=mark(*named)) if named else value)
    return found


def names_of(item: pytest.Item) -> set[str]:
    """The backends a collected test names (its own mark and its parametrization's)."""
    return {name for marker in item.iter_markers("backend") for name in marker.args}
