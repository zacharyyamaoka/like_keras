"""Criterion 3's static half: black-box rules 1, 2, 4 and 5 on every frozen reference component, and rule 5 on the
library itself. JSON on stdout; exit 0 only when everything is clean.

    python3 /home/bam/like_keras/kpi/blackbox.py

Per component (a Python class deriving from Component, a C++ struct deriving from lk::Component, under kpi/tasks):
  1 names no peer, address, transport or backend, and no System or Deployment, inside the component
  2 authors no edge inside the component (add / wire / expose)
  4 makes no request/response call (await, futures, asyncio, a .call/.request)
  5 reads no environment variable
Rule 3 (the step runs with dora, pyarrow and the dora backend blocked) is dynamic: kpi/blackbox/test_blackbox.py.
The library (src/like_keras outside backends/) must read no environment variable either (rule 5).
Also reported, for criterion 10's counter-metric: wall-clock sleeps in kpi/twin.
"""

from __future__ import annotations

import ast
import json
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
TASKS = REPO / "kpi" / "tasks"

FORBIDDEN_NAMES = {"System", "Deployment", "native", "attach", "replay", "backends", "dora", "pyarrow", "zmq", "zenoh",
                   "socket"}
FORBIDDEN_MODULES = ("dora", "pyarrow", "zmq", "zenoh", "socket", "like_keras.backends", "asyncio", "concurrent")
ADDRESS = re.compile(r"(unix:|tcp://|udp://|localhost|127\.0\.0\.1|\.sock\b|/dev/tty|:\d{4,5}\b)")
EDGE_CALLS = {"add", "wire", "expose"}
REQUEST_CALLS = {"call", "request", "result", "send_request", "wait_for"}
STD_HEADERS = {"algorithm", "array", "atomic", "cassert", "chrono", "cmath", "cstddef", "cstdint", "cstdio", "cstdlib",
               "cstring", "functional", "limits", "memory", "numeric", "optional", "span", "stdexcept", "string",
               "string_view", "type_traits", "utility", "vector"}
POSIX_HEADERS = {"fcntl.h", "sys/file.h", "sys/mman.h", "time.h", "unistd.h"}  # device emulation (the fake bus)


def is_component(node: ast.ClassDef) -> bool:
    return any((isinstance(base, ast.Attribute) and base.attr == "Component")
               or (isinstance(base, ast.Name) and base.id in ("Component", "PickPlanner")) for base in node.bases)


def forbidden_bindings(module: ast.Module) -> set[str]:
    """Names the module binds from a transport or backend import (`from dora import Node` binds Node)."""
    bound = set()
    for node in ast.walk(module):
        if isinstance(node, ast.Import):
            bound |= {(alias.asname or alias.name).split(".")[0] for alias in node.names if alias.name.startswith(FORBIDDEN_MODULES)}
        elif isinstance(node, ast.ImportFrom) and (node.module or "").startswith(FORBIDDEN_MODULES):
            bound |= {alias.asname or alias.name for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module == "like_keras.backends":
            bound |= {alias.asname or alias.name for alias in node.names}
    return bound


def python_component_violations(module: ast.Module, cls: ast.ClassDef, source: str) -> list[str]:
    """Rules 1, 2, 4, 5 inside the class. A module may import a backend for its own deployment function (a one-file
    task does); what counts is whether the component's own code uses it."""
    found = []
    bound = forbidden_bindings(module)
    for node in ast.walk(cls):
        if isinstance(node, ast.Name) and node.id in bound:
            found.append(f"rule 1: uses {node.id}, imported from a transport or backend (line {node.lineno})")
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module or ""]
            found += [f"rule 1/4: imports {name} (line {node.lineno})" for name in names if name.startswith(FORBIDDEN_MODULES)]
        if isinstance(node, ast.Name) and node.id in FORBIDDEN_NAMES:
            found.append(f"rule 1: names {node.id} (line {node.lineno})")
        if isinstance(node, ast.Attribute) and node.attr in FORBIDDEN_NAMES and not (
                isinstance(node.value, ast.Name) and node.value.id == "self"):
            found.append(f"rule 1: names .{node.attr} (line {node.lineno})")
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and ADDRESS.search(node.value):
            found.append(f"rule 1: an address string {node.value!r} (line {node.lineno})")
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr in EDGE_CALLS and not (isinstance(node.func.value, ast.Name) and node.func.value.id == "self"):
                found.append(f"rule 2: authors an edge with .{node.func.attr}() (line {node.lineno})")
            if node.func.attr in REQUEST_CALLS:
                found.append(f"rule 4: a request/response call .{node.func.attr}() (line {node.lineno})")
        if isinstance(node, (ast.Await, ast.AsyncFunctionDef, ast.AsyncFor, ast.AsyncWith)):
            found.append(f"rule 4: async/await (line {node.lineno})")
        if isinstance(node, ast.Attribute) and node.attr in ("environ", "getenv", "putenv"):
            found.append(f"rule 5: reads the environment (line {node.lineno})")
    return sorted(set(found))


def python_components(path: pathlib.Path) -> list[dict]:
    source = path.read_text(encoding="utf-8")
    module = ast.parse(source)
    return [{"name": cls.name, "file": str(path.relative_to(REPO)), "language": "python",
             "task": path.relative_to(TASKS).parts[0], "violations": python_component_violations(module, cls, source)}
            for cls in module.body if isinstance(cls, ast.ClassDef) and is_component(cls)]


def strip_cpp_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def cpp_include_violations(path: pathlib.Path, device_header: bool) -> list[str]:
    found = []
    for match in re.finditer(r'#include\s*([<"])([^>"]+)[>"]', strip_cpp_comments(path.read_text(encoding="utf-8"))):
        bracket, header = match.groups()
        if bracket == '"':
            if "/" in header or not (path.parent / header).exists():
                found.append(f"rule 1: includes {header!r}, not one of its own headers")
            continue
        if header == "like_keras/lk.hpp" or re.fullmatch(r"like_keras/msgs/\w+\.hpp", header) or header in STD_HEADERS:
            continue
        if device_header and header in POSIX_HEADERS:
            continue
        found.append(f"rule 1: includes <{header}>")
    return found


def cpp_components(path: pathlib.Path) -> list[dict]:
    text = strip_cpp_comments(path.read_text(encoding="utf-8"))
    out = []
    headers = [path] + [path.parent / name for name in re.findall(r'#include\s*"([^"]+)"', text) if (path.parent / name).exists()]
    shared = []
    for header in headers:
        shared += cpp_include_violations(header, device_header=header.name.startswith("fake_"))
        body = strip_cpp_comments(header.read_text(encoding="utf-8"))
        if re.search(r"\bdora\b|arrow/|zmq|zenoh|<sys/socket\.h>", body):
            shared.append(f"rule 1: {header.name} names a transport")
        if re.search(r"\bgetenv\b|\benviron\b|secure_getenv", body):
            shared.append(f"rule 5: {header.name} reads the environment")
        if re.search(r"std::(future|promise|async)\b", body):
            shared.append(f"rule 4: {header.name} uses a future")
    for match in re.finditer(r"struct\s+(\w+)\s*:\s*lk::Component\s*\{", text):
        start = match.end()
        depth, index = 1, start
        while depth and index < len(text):
            depth += {"{": 1, "}": -1}.get(text[index], 0)
            index += 1
        body = text[start:index]
        violations = list(shared)
        if re.search(r"\.\s*(wire|expose)\s*\(|\badd\s*<", body):
            violations.append("rule 2: authors an edge inside the component")
        if re.search(r"lk::System|Deployment", body):
            violations.append("rule 1: names a System inside the component")
        if ADDRESS.search(body):
            violations.append("rule 1: an address string")
        out.append({"name": match.group(1), "file": str(path.relative_to(REPO)), "language": "cpp",
                    "task": path.relative_to(TASKS).parts[0], "violations": sorted(set(violations))})
    for header in headers[1:]:  # components defined in the task's own headers (guard.hpp)
        for match in re.finditer(r"struct\s+(\w+)\s*:\s*lk::Component", strip_cpp_comments(header.read_text())):
            out.append({"name": match.group(1), "file": str(header.relative_to(REPO)), "language": "cpp",
                        "task": path.relative_to(TASKS).parts[0], "violations": sorted(set(shared))})
    return out


def library_env_reads() -> list[str]:
    source = REPO / "src" / "like_keras"
    found = []
    for path in sorted(source.rglob("*.py")) if source.exists() else []:
        if "backends" in path.relative_to(source).parts:
            continue
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if re.search(r"os\.environ|getenv\(", line):
                found.append(f"{path.relative_to(REPO)}:{number}")
    return found


def twin_sleeps() -> list[str]:
    found = []
    for path in sorted((REPO / "kpi" / "twin").glob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "sleep":
                found.append(f"{path.relative_to(REPO)}:{node.lineno}")
    return found


def scan() -> dict:
    components = []
    for path in sorted(TASKS.rglob("*.py")):
        if "swaps" not in path.parts:
            components += python_components(path)
    for path in sorted(TASKS.rglob("*.cpp")):
        components += cpp_components(path)
    unique = {(c["file"], c["name"]): c for c in components}
    components = sorted(unique.values(), key=lambda c: (c["task"], c["file"], c["name"]))
    return {"components": components, "clean": sum(not c["violations"] for c in components), "total": len(components),
            "library_env_reads": library_env_reads(), "twin_sleeps": twin_sleeps()}


if __name__ == "__main__":
    report = scan()
    print(json.dumps(report, indent=1))
    sys.exit(0 if report["clean"] == report["total"] and not report["library_env_reads"] and not report["twin_sleeps"] else 1)
