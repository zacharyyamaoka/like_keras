"""Criteria 4, 7 and 8 part 5: the API surface, the workflow size, the codebase size, the C++ boilerplate.

    python3 /home/bam/like_keras/kpi/surface.py --api --tasks --size
    python3 /home/bam/like_keras/kpi/surface.py --cpp-boilerplate kpi/tasks/cpp_gain

JSON on stdout. stdlib only (tokenize, ast); ruff / clang-format format a temporary copy first when present
(defeating line golf); `formatted` says whether they ran.

--api   public names, counted as CONTRACT §3 counts them: like_keras.__all__; the public members each class in it
        defines (Component's implicit ports health/metrics/log included); the backend modules; and the C++ public
        method and macro names of lk.hpp that have no same-named Python counterpart (stdlib Logger's info/warning/
        error/debug count as Python's, reached through `log`). Types, option structs and constants are not names.
        Also the widest public signature (`**inputs` port-name kwargs of add/wire excepted, as §3 says), the number
        of **kwargs option bags and of string-enum (Literal) parameters.
--tasks code tokens of each frozen task solution against its baseline (raw dora for a cross-process task, plain
        Python for an in-process one).
--size  first-party SLOC per part (CONTRACT §11 criterion 7), generated views counted, kpi/ and tests excluded;
        the facility SLOC share (criterion 9: <= 30 %), found by name (heuristic, reported).
--cpp-boilerplate DIR  a C++ component's files and its lines that name the framework (lk::, LK_, like_keras/,
        lk_component); the message types (rm::) are the domain, not boilerplate.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import tokenize

REPO = pathlib.Path(__file__).resolve().parents[1]
SRC = REPO / "src" / "like_keras"
CPP = REPO / "cpp"
TASKS = REPO / "kpi" / "tasks"
FRAMEWORK = re.compile(r"\blk::|\bLK_[A-Z]+|like_keras|\blk_component\b")
FACILITY = re.compile(r"log|metric|health|diagnos|facilit|record|replay|describe|percentile|stats|stale", re.I)
LOGGER_METHODS = {"info", "warning", "error", "debug", "critical", "exception"}


# ---------------------------------------------------------------- formatting and counting


def scratch_dir() -> pathlib.Path:
    """Temporary copies live under build/kpi/tmp: kpi writes nothing outside the repo."""
    path = REPO / "build" / "kpi" / "tmp"
    path.mkdir(parents=True, exist_ok=True)
    return path


def ruff() -> list[str] | None:
    local = pathlib.Path(sys.executable).with_name("ruff")  # locked in the test extra
    if local.exists():
        return [str(local)]
    found = shutil.which("ruff")
    if found:
        return [found]
    if shutil.which("uvx"):
        probe = subprocess.run(["uvx", "ruff", "--version"], capture_output=True, text=True, timeout=120)
        if probe.returncode == 0:
            return ["uvx", "ruff"]
    return None


def formatted_copy(paths: list[pathlib.Path]) -> tuple[list[pathlib.Path], bool, tempfile.TemporaryDirectory]:
    """Copies of `paths` run through ruff format / clang-format (when present), in a temporary directory."""
    scratch = tempfile.TemporaryDirectory(prefix="surface-", dir=scratch_dir())
    copies, ran = [], True
    python_tool = ruff()
    for index, path in enumerate(paths):
        copy = pathlib.Path(scratch.name) / f"{index}_{path.name}"
        shutil.copyfile(path, copy)
        copies.append(copy)
    python_files = [str(copy) for copy in copies if copy.suffix == ".py"]
    if python_files:
        if python_tool:
            subprocess.run([*python_tool, "format", "--quiet", "--line-length", "120", *python_files],
                           capture_output=True, timeout=300)
        else:
            ran = False
    cpp_files = [str(copy) for copy in copies if copy.suffix in (".cpp", ".hpp", ".h", ".cc")]
    if cpp_files:
        if shutil.which("clang-format"):
            subprocess.run(["clang-format", "-i", "--style={BasedOnStyle: Google, ColumnLimit: 120}", *cpp_files],
                           capture_output=True, timeout=300)
        else:
            ran = False
    return copies, ran, scratch


def python_lines(path: pathlib.Path) -> set[int]:
    """Lines holding code: no blank lines, comments or docstrings."""
    lines: set[int] = set()
    with open(path, "rb") as file:
        previous = None
        for token in tokenize.tokenize(file.readline):
            if token.type in (tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT,
                              tokenize.ENCODING, tokenize.ENDMARKER):
                previous = token if token.type in (tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT) else previous
                continue
            is_docstring = token.type == tokenize.STRING and (previous is None or previous.type in (
                tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT)) and token.string.lstrip("rbuRBU").startswith(('"""', "'''", '"', "'"))
            previous = token
            if is_docstring:
                continue
            lines.update(range(token.start[0], token.end[0] + 1))
    return lines


def strip_cpp(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def cpp_lines(path: pathlib.Path) -> list[str]:
    return [line for line in strip_cpp(path.read_text(encoding="utf-8", errors="replace")).splitlines() if line.strip()]


def sloc(paths: list[pathlib.Path]) -> tuple[int, bool]:
    if not paths:
        return 0, True
    copies, ran, scratch = formatted_copy(paths)
    with scratch:
        total = sum(len(python_lines(copy)) if copy.suffix == ".py" else len(cpp_lines(copy)) for copy in copies)
    return total, ran


def code_tokens(path: pathlib.Path) -> int:
    if path.suffix == ".py":
        count = 0
        with open(path, "rb") as file:
            previous = None
            for token in tokenize.tokenize(file.readline):
                if token.type in (tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT,
                                  tokenize.ENCODING, tokenize.ENDMARKER):
                    previous = token if token.type != tokenize.COMMENT else previous
                    continue
                if token.type == tokenize.STRING and (previous is None or previous.type in (tokenize.NEWLINE, tokenize.INDENT)):
                    previous = token
                    continue  # a docstring is not code
                previous = token
                count += 1
        return count
    text = path.read_text(encoding="utf-8")
    if path.suffix in (".yml", ".yaml", ".toml", ".txt", ".json"):
        text = re.sub(r"#[^\n]*", "", text)
        return len(re.findall(r"[A-Za-z0-9_.]+|[^\sA-Za-z0-9_.]", text))
    return len(re.findall(r"[A-Za-z0-9_]+|[^\sA-Za-z0-9_]", strip_cpp(text)))


# ---------------------------------------------------------------- --api

API_PROBE = r"""
import inspect, json, sys, typing
import like_keras as lk
top = list(getattr(lk, "__all__", []))
names, signatures = set(top), []
def public(cls):
    own = {}
    for klass in reversed(cls.__mro__):
        if klass is object or klass.__module__.split(".")[0] != "like_keras":
            continue
        own.update({k: v for k, v in vars(klass).items() if not k.startswith("_")})
    return own
for name in top:
    obj = getattr(lk, name)
    members = public(obj) if inspect.isclass(obj) else {}
    if inspect.isclass(obj) and name == "Component":
        members.update({port: None for port in ("health", "metrics", "log")})
    for member, value in members.items():
        names.add(member)
        target = value.__func__ if isinstance(value, (staticmethod, classmethod)) else value
        if callable(target) and not inspect.isclass(target):
            try:
                signatures.append((f"{name}.{member}", inspect.signature(target)))
            except (TypeError, ValueError):
                pass
    if callable(obj):
        try:
            signatures.append((name, inspect.signature(obj.__init__ if inspect.isclass(obj) else obj)))
        except (TypeError, ValueError):
            pass
backends = []
try:
    import like_keras.backends as bk, pkgutil
    backends = sorted(m.name for m in pkgutil.iter_modules(bk.__path__) if not m.name.startswith("_"))
except Exception:
    pass
widest, kwargs_bags, string_enums = ("", 0), 0, 0
for where, signature in signatures:
    params = [p for p in signature.parameters.values() if p.name not in ("self", "cls")]
    if len(params) > widest[1]:
        widest = (where, len(params))
    for p in params:
        if p.kind is p.VAR_KEYWORD and not where.endswith((".add", ".wire", ".expose", ".__init__", ".__call__", ".forward")):
            kwargs_bags += 1
        if "Literal" in str(p.annotation):
            string_enums += 1
print(json.dumps({"top": top, "names": sorted(names), "backends": backends, "widest": widest,
                  "kwargs_bags": kwargs_bags, "string_enums": string_enums}))
"""


def cpp_public_names() -> set[str]:
    header = CPP / "include" / "like_keras" / "lk.hpp"
    if not header.exists():
        return set()
    text = strip_cpp(header.read_text(encoding="utf-8"))
    names = set(re.findall(r"#\s*define\s+(LK_[A-Z_]+)", text))
    for match in re.finditer(r"(?:class|struct)\s+(\w+)[^;{]*\{", text):
        cls = match.group(1)
        start, depth, index = match.end(), 1, match.end()
        while depth and index < len(text):
            depth += {"{": 1, "}": -1}.get(text[index], 0)
            index += 1
        body = text[start:index]
        public = body if match.group(0).lstrip().startswith("struct") else ""
        for section in re.split(r"\b(public|private|protected)\s*:", body)[1:]:
            pass
        sections = re.split(r"\b(public|private|protected)\s*:", body)
        access = "public" if match.group(0).lstrip().startswith("struct") else "private"
        chunks = [(access, sections[0])] + list(zip(sections[1::2], sections[2::2]))
        public = "".join(chunk for kind, chunk in chunks if kind == "public")
        for method in re.findall(r"\b([a-z_][a-z0-9_]*)\s*\(", public):
            if method not in (cls, "operator", "if", "for", "while", "return", "sizeof", "static_cast", "decltype",
                              "noexcept", "alignas", "requires", "explicit", "const"):
                names.add(method)
    return names


def api() -> dict:
    env = dict(os.environ, PYTHONPATH=str(REPO / "src"))
    probe = subprocess.run([sys.executable, "-c", API_PROBE], env=env, capture_output=True, text=True, timeout=120)
    if probe.returncode != 0:
        return {"available": False, "reason": (probe.stderr.strip().splitlines() or ["import failed"])[-1]}
    found = json.loads(probe.stdout)
    python_names = set(found["names"]) | set(found["backends"])
    cpp_only = sorted(cpp_public_names() - python_names - LOGGER_METHODS)
    total = len(found["names"]) + len(found["backends"]) + len(cpp_only)
    return {"available": True, "top_level": len(found["top"]), "total": total, "python_names": found["names"],
            "backends": found["backends"], "cpp_only": cpp_only, "widest": {"callable": found["widest"][0],
            "params": found["widest"][1]}, "kwargs_bags": found["kwargs_bags"], "string_enums": found["string_enums"]}


# ---------------------------------------------------------------- --tasks

WORKFLOWS = {
    # task: (kind, the like_keras solution, the baseline it is measured against)
    "relay": ("cross_process", ["relay/solution/relay.py"],
              ["relay/raw_dora/dataflow.yml", "relay/raw_dora/codec.py", "relay/raw_dora/count.py",
               "relay/raw_dora/double.py", "relay/raw_dora/save.py"]),
    "pipeline": ("in_process", ["pipeline/solution/pipeline.py"], ["pipeline/plain/pipeline.py"]),
}


def tasks() -> dict:
    out = {}
    for task, (kind, solution, baseline) in WORKFLOWS.items():
        solution_tokens = sum(code_tokens(TASKS / path) for path in solution)
        baseline_tokens = sum(code_tokens(TASKS / path) for path in baseline)
        out[task] = {"kind": kind, "solution_tokens": solution_tokens, "baseline_tokens": baseline_tokens,
                     "ratio": round(solution_tokens / baseline_tokens, 4)}
    return out


# ---------------------------------------------------------------- --size


def files(*patterns: str, root: pathlib.Path = REPO) -> list[pathlib.Path]:
    found: list[pathlib.Path] = []
    for pattern in patterns:
        found += [path for path in sorted(root.glob(pattern)) if path.is_file() and "__pycache__" not in path.parts]
    return found


PARTS = {
    "core": (650, ["src/like_keras/__init__.py", "src/like_keras/component.py", "src/like_keras/system.py",
                   "src/like_keras/deployment.py", "src/like_keras/codec.py", "src/like_keras/record.py"]),
    "backend_inproc": (300, ["src/like_keras/backends/__init__.py", "src/like_keras/backends/inproc.py"]),
    "backend_dora": (300, ["src/like_keras/backends/dora/*.py"]),
    "attach_cli": (250, ["src/like_keras/attach.py", "src/like_keras/cli.py"]),
    "cpp_runtime": (850, ["cpp/include/like_keras/lk.hpp", "cpp/include/like_keras/abi.h", "cpp/src/*.cpp",
                          "cpp/hosts/*.cpp"]),
    "generated_views": (100, ["cpp/include/like_keras/msgs/*.hpp"]),
}
TOTAL_TARGET = 2450


def facility_sloc(paths: list[pathlib.Path]) -> int:
    """Lines of record.py, plus Python functions/classes and C++ functions whose names say facility (heuristic)."""
    total = 0
    for path in paths:
        if path.name == "record.py":
            total += len(python_lines(path))
            continue
        if path.suffix == ".py":
            tree = ast.parse(path.read_text(encoding="utf-8"))
            code = python_lines(path)
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and FACILITY.search(node.name):
                    total += len(code & set(range(node.lineno, (node.end_lineno or node.lineno) + 1)))
        elif path.suffix in (".cpp", ".hpp", ".h"):
            text = strip_cpp(path.read_text(encoding="utf-8"))
            for match in re.finditer(r"\b(\w+)\s*\([^;{)]*\)[^;{]*\{", text):
                if FACILITY.search(match.group(1)):
                    depth, index = 1, match.end()
                    while depth and index < len(text):
                        depth += {"{": 1, "}": -1}.get(text[index], 0)
                        index += 1
                    total += sum(1 for line in text[match.start():index].splitlines() if line.strip())
    return total


def size() -> dict:
    out: dict = {"parts": {}}
    every: list[pathlib.Path] = []
    formatted = True
    for part, (target, patterns) in PARTS.items():
        paths = files(*patterns)
        every += paths
        count, ran = sloc(paths)
        formatted &= ran
        out["parts"][part] = {"sloc": count, "target": target, "files": [str(p.relative_to(REPO)) for p in paths]}
    others = [p for p in files("src/like_keras/**/*.py", "cpp/**/*.cpp", "cpp/**/*.hpp", "cpp/**/*.h") if p not in every]
    count, ran = sloc(others)
    out["parts"]["other"] = {"sloc": count, "target": None, "files": [str(p.relative_to(REPO)) for p in others]}
    out["total"] = {"sloc": sum(part["sloc"] for part in out["parts"].values()), "target": TOTAL_TARGET}
    facility = facility_sloc(every + others)
    out["facility"] = {"sloc": facility, "share": round(facility / out["total"]["sloc"], 4) if out["total"]["sloc"] else None}
    out["formatted"] = formatted and ran
    return out


# ---------------------------------------------------------------- --cpp-boilerplate


def cpp_boilerplate(directory: pathlib.Path) -> dict:
    sources = [p for p in sorted(directory.iterdir()) if p.suffix in (".cpp", ".hpp", ".h", ".cc") or p.name == "CMakeLists.txt"]
    copies, ran, scratch = formatted_copy(sources)
    with scratch:
        framework = []
        for original, copy in zip(sources, copies, strict=True):
            lines = cpp_lines(copy) if copy.suffix != ".txt" else [l for l in re.sub(r"#[^\n]*", "", copy.read_text()).splitlines() if l.strip()]
            framework += [f"{original.name}: {line.strip()}" for line in lines if FRAMEWORK.search(line)]
    return {"directory": str(directory), "files": len(sources), "framework_lines": len(framework), "lines": framework,
            "formatted": ran}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="API surface, workflow size, SLOC, C++ boilerplate (JSON)")
    parser.add_argument("--api", action="store_true")
    parser.add_argument("--tasks", action="store_true")
    parser.add_argument("--size", action="store_true")
    parser.add_argument("--cpp-boilerplate", type=pathlib.Path)
    arguments = parser.parse_args(argv)
    if arguments.cpp_boilerplate:
        print(json.dumps(cpp_boilerplate(arguments.cpp_boilerplate.resolve())))
        return 0
    report = {}
    if arguments.api:
        report["api"] = api()
    if arguments.tasks:
        report["tasks"] = tasks()
    if arguments.size:
        report["size"] = size()
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
