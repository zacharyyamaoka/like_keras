"""Lane core's own tests of `like_keras.backends` (not the ruler: kpi/ is read-only to builders).

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests

A backend is resolved by name on first use (`from like_keras.backends import <name>`), lazily, THROUGH the import
system: an import hook or blocker in sys.meta_path must see `like_keras.backends.<name>` like any other import.
WHY (fix round 1, M1): the lookup called PathFinder directly, so kpi/blackbox's Blocker never saw the backend it
blocks; the from-import succeeded and the first attribute access ran the blocked module.

Each case runs in a fresh interpreter, because it changes sys.meta_path and sys.modules.
"""

from __future__ import annotations

import os
import subprocess
import sys
import textwrap
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def run_fresh(script: str) -> str:
    environment = dict(os.environ, PYTHONPATH=str(REPO / "src"))
    result = subprocess.run([sys.executable, "-c", textwrap.dedent(script)], cwd=REPO, env=environment,
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr[-3000:]
    return result.stdout.strip()


def test_a_blocked_backend_is_refused_at_the_from_import():
    """Every backend on disk not yet loaded, blocked in sys.meta_path: the from-import raises and nothing runs."""
    output = run_fresh('''
        import importlib.abc, pkgutil, sys
        import like_keras.backends as backends
        unloaded = sorted(module.name for module in pkgutil.iter_modules(backends.__path__)
                          if f"like_keras.backends.{module.name}" not in sys.modules)
        assert unloaded, "no unloaded backend on disk to block"

        class Blocker(importlib.abc.MetaPathFinder):
            def find_spec(self, name, path=None, target=None):
                if name.startswith("like_keras.backends.") and name.rpartition(".")[2] in unloaded:
                    raise ImportError(f"{name} is blocked")
                return None

        sys.meta_path.insert(0, Blocker())
        for backend in unloaded:
            try:
                exec(f"from like_keras.backends import {backend}")
            except ImportError as error:
                assert "is blocked" in str(error), error
            else:
                raise AssertionError(f"from like_keras.backends import {backend} bypassed sys.meta_path")
            assert f"like_keras.backends.{backend}" not in sys.modules, f"blocked {backend} was registered anyway"
            assert backend not in vars(backends), f"blocked {backend} was bound on the package anyway"
        print("refused", unloaded)
    ''')
    assert output.startswith("refused")


def test_a_backend_supplied_through_meta_path_resolves_and_stays_lazy():
    """A finder in sys.meta_path can supply a backend; its body runs at first attribute access, not at the import."""
    output = run_fresh('''
        import importlib.abc, importlib.machinery, sys
        executed = []

        class ProbeLoader(importlib.abc.Loader):
            def create_module(self, spec):
                return None

            def exec_module(self, module):
                executed.append(module.__name__)
                module.NAME = "probe"

        class ProbeFinder(importlib.abc.MetaPathFinder):
            def find_spec(self, name, path=None, target=None):
                if name == "like_keras.backends.probe":
                    return importlib.machinery.ModuleSpec(name, ProbeLoader())
                return None

        sys.meta_path.insert(0, ProbeFinder())
        from like_keras.backends import probe
        assert executed == [], "the backend body ran at the import, not at first use (the lean core needs lazy)"
        assert probe.NAME == "probe" and executed == ["like_keras.backends.probe"], executed
        import like_keras.backends as backends
        assert backends.probe is probe is sys.modules["like_keras.backends.probe"]
        print("ok")
    ''')
    assert output == "ok"


def test_an_unknown_backend_is_an_attribute_error_naming_it():
    output = run_fresh('''
        import like_keras.backends as backends
        try:
            backends.no_such_backend
        except AttributeError as error:
            assert "no backend 'no_such_backend'" in str(error), error
        else:
            raise AssertionError("an unknown backend resolved")
        try:
            from like_keras.backends import no_such_backend  # noqa: F401
        except ImportError:
            pass
        else:
            raise AssertionError("an unknown backend imported")
        print("ok")
    ''')
    assert output == "ok"


def test_a_backend_already_imported_is_the_same_module_object():
    """import_module first, then the lazy lookup: one module, never a second copy of a backend's state."""
    output = run_fresh('''
        import importlib, pkgutil, sys
        import like_keras.backends as backends
        unloaded = sorted(module.name for module in pkgutil.iter_modules(backends.__path__)
                          if f"like_keras.backends.{module.name}" not in sys.modules)
        for backend in unloaded:
            imported = importlib.import_module(f"like_keras.backends.{backend}")
            vars(backends).pop(backend, None)  # as if only sys.modules knew it
            assert getattr(backends, backend) is imported
        print("ok")
    ''')
    assert output == "ok"
