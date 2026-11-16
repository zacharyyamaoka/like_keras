"""The plan as a dora 0.5.0 dataflow (CONTRACT §9.2): build/like_keras/<system>/{dataflow.yml, plan.json}.

Emission rules: nodes only, never operators; `path`, `args` and `env` at node level, never `custom:`;
`restart_policy: always` on every node; `queue_size` = the input's queue (1 for latest); ids are the dotted component
path and port; `_unstable_deploy: {machine: <host>}` only when host= is set (the 0.5 spelling: a 1.x rename lands in
this file only); `lk_hello` wired to every peer; a gateway node when the System exposes ports or records.

One node per process (its id is the process name) and the gateway. An output `<leaf path>.<port>` goes once to
each node that reads it; a node fans it out to its own readers. plan.json says, per node, what it builds, reads and
sends: the hosts never re-derive the graph. A native process also gets `<process>.host`, the same facts as plain
lines, so lk-dora-host needs no JSON parser.
"""

from __future__ import annotations

import json
import os
import pathlib
import sys
import time

from like_keras.component import FACILITY_PORTS, message_type, path

GATEWAY = "lk_gateway"
HELLO = "lk_hello"
# WHY so deep under lockstep: a frame dora drops is a step the receiver never sees, and a sender with no inputs runs
# to `until` before its readers start. In real time a latest input keeps 1: the newest wins (CONTRACT §5.4).
LOCKSTEP_QUEUE = 1_000_000
RECORD_QUEUE = 1000  # the gateway's recorded inputs: a recording loses nothing
FACILITY_QUEUE = 64  # one record per component of a native box arrives per round


def directory_of(system: str) -> pathlib.Path:
    return pathlib.Path(os.getcwd()) / "build" / "like_keras" / system


def write(plan, until: float | None) -> tuple[pathlib.Path, dict]:
    """Write the dataflow, plan.json and the native processes' .host files; returns (directory, plan document)."""
    system = plan.datasheet["system"]
    if not plan.realtime and any(group.language == "native" and not group.hz for group in plan.groups):
        raise ValueError("dora lockstep steps a native group on its hz ticks: give the native group hz=...")
    natives = [group.process for group in plan.groups if group.language == "native"]
    if len(natives) != len(set(natives)):
        raise ValueError("under dora a process runs one native .so (lk-dora-host): give each its own process=")
    directory = directory_of(system)
    import shutil
    import uuid

    for folder in ("markers", "logs"):  # this run's markers and node logs only
        shutil.rmtree(directory / folder, ignore_errors=True)
        (directory / folder).mkdir(parents=True)
    document = plan_document(plan, until, directory)
    document["markers"] = str(directory / "markers" / uuid.uuid4().hex[:12])
    os.mkdir(document["markers"])
    if GATEWAY in {group.process for group in plan.groups}:
        raise ValueError(f"process {GATEWAY!r} is the dora backend's gateway node: name the process something else")
    plan_path = directory / "plan.json"
    plan_path.write_text(json.dumps(document, indent=1, default=str))
    nodes = []
    for name, node in document["nodes"].items():
        entry = {"id": name, "restart_policy": "always", "outputs": [HELLO, *node["outputs"]]}
        if node["role"] == "native":
            entry["path"], entry["args"] = native_host(), f"{plan_path} {name}"
            (directory / f"{name}.host").write_text(host_lines(document, name))
        else:  # WHY /usr/bin/env: dora resolves `path` through symlinks, and the venv's python is one (no venv then)
            module = "gateway" if node["role"] == "gateway" else "host"
            entry["path"] = "/usr/bin/env"
            entry["args"] = f"{sys.executable} -m like_keras.backends.dora.{module} {plan_path}" + (
                "" if module == "gateway" else f" {name}"
            )
            entry["env"] = {"PYTHONPATH": os.pathsep.join(python_path())}
        if node.get("host"):
            entry["_unstable_deploy"] = {"machine": node["host"]}
        queue = (lambda size: size) if plan.realtime else (lambda size: LOCKSTEP_QUEUE)
        entry["inputs"] = {
            **{f"{HELLO}.{peer}": {"source": f"{peer}/{HELLO}", "queue_size": 16} for peer in node["peers"]},
            **{
                name: {"source": spec["source"], "queue_size": queue(spec["queue"])}
                for name, spec in node["inputs"].items()
            },
        }
        nodes.append(entry)
    # WHY JSON: it is YAML 1.2, so dora reads it, and json writes it exactly (no YAML library in like_keras)
    (directory / "dataflow.yml").write_text(json.dumps({"nodes": nodes}, indent=1) + "\n")
    return directory, document


def plan_document(plan, until: float | None, directory: pathlib.Path) -> dict:
    from like_keras import codec
    from like_keras.backends import dora

    sheet, system = plan.datasheet, plan.datasheet["system"]
    process_of = {path(leaf): group.process for group in plan.groups for leaf in group.leaves}
    nodes: dict[str, dict] = {}
    for group in plan.groups:
        node = nodes.setdefault(
            group.process,
            {"role": group.language, "host": group.host, "groups": [], "inputs": {}, "outputs": {}, "peers": set()},
        )
        node["groups"].append(
            {"hz": group.hz, "cpus": group.cpus, "fifo": group.fifo, "order": [path(leaf) for leaf in group.order]}
        )
    gateway = bool(plan.graph.exposed_out or plan.graph.exposed_in or plan.recordings)
    if gateway:
        nodes[GATEWAY] = {"role": "gateway", "host": None, "groups": [], "inputs": {}, "outputs": {}, "peers": set()}

    def port_type(leaf: str, port: str) -> str:
        return sheet["components"][leaf]["ports"][port]["type"]

    def connect(leaf: str, port: str, reader: str, deliver=None, queue: int = 1) -> str:
        """Route leaf.port to node `reader` (once), delivering to (leaf, port) there; returns the output id."""
        output, writer = f"{leaf}.{port}", process_of[leaf]
        nodes[writer]["outputs"].setdefault(output, port_type(leaf, port))
        spec = nodes[reader]["inputs"].setdefault(
            output,
            {
                "source": f"{writer}/{output}",
                "type": port_type(leaf, port),
                "deliver": [],
                "queue": 1,
                # a native box names its inner components; the reader prefixes the box's own path (abi.h)
                "prefix": leaf if port in FACILITY_PORTS and sheet["components"][leaf]["language"] == "native" else "",
            },
        )
        spec["queue"] = max(spec["queue"], queue)
        if deliver is not None:
            spec["deliver"].append(deliver)
        nodes[writer]["peers"].add(reader)
        nodes[reader]["peers"].add(writer)
        return output

    edges = []
    for edge in plan.graph.edges:
        source, target = path(edge.src), path(edge.dst)
        edges.append([source, edge.src_port, target, edge.dst_port])
        if process_of[source] != process_of[target]:
            queue = type(edge.dst)._lk_ports[edge.dst_port].queue or 1
            connect(source, edge.src_port, process_of[target], [target, edge.dst_port], queue)
    exposed = {}
    for name, ref in plan.graph.exposed_out.items():
        exposed[name] = {
            "dir": "out",
            "source": connect(path(ref.owner), ref.name, GATEWAY),
            "type": port_type(path(ref.owner), ref.name),
        }
    for name, refs in plan.graph.exposed_in.items():
        output = f"{system}.{name}"
        nodes[GATEWAY]["outputs"][output] = port_type(path(refs[0].owner), refs[0].name)
        exposed[name] = {"dir": "in", "source": output, "type": nodes[GATEWAY]["outputs"][output]}
        for ref in refs:
            reader, leaf = process_of[path(ref.owner)], path(ref.owner)
            spec = nodes[reader]["inputs"].setdefault(
                output,
                {
                    "source": f"{GATEWAY}/{output}",
                    "type": exposed[name]["type"],
                    "deliver": [],
                    "queue": 1,
                    "prefix": "",
                },
            )
            spec["deliver"].append([leaf, ref.name])
            spec["queue"] = max(spec["queue"], type(ref.owner)._lk_ports[ref.name].queue or 1)
            nodes[reader]["peers"].add(GATEWAY)
            nodes[GATEWAY]["peers"].add(reader)
    if gateway:  # lk top reads every component's health and metrics (CONTRACT §9.3)
        for leaf in sheet["components"]:
            for port in ("health", "metrics"):
                connect(leaf, port, GATEWAY, queue=FACILITY_QUEUE)
    recordings = []
    for file, channels in plan.recordings:
        rows = [
            {
                "input": connect(path(leaf), port, GATEWAY, queue=RECORD_QUEUE),
                "topic": topic,
                "type": port_type(path(leaf), port),
            }
            for leaf, port, topic in channels
        ]
        os.makedirs(os.path.dirname(os.path.abspath(file)), exist_ok=True)  # the gateway writes it, elsewhere
        recordings.append({"file": os.path.abspath(file), "channels": rows})
    for node in nodes.values():
        node["peers"] = sorted(node["peers"])
    types = {kind for node in nodes.values() for kind in node["outputs"].values()}
    types |= {spec["type"] for node in nodes.values() for spec in node["inputs"].values()}
    components = {}
    for leaf, entry in sheet["components"].items():
        impl = entry["impl"] if entry["language"] == "python" else os.path.abspath(entry["impl"])
        components[leaf] = {
            "impl": impl,
            "language": entry["language"],
            "params": entry["params"],
            "process": process_of[leaf],
        }
    return {
        "system": system,
        "directory": str(directory),
        "realtime": plan.realtime,
        "until": until,
        "versions": {"like_keras": sheet["like_keras"], "abi": sheet["abi"], "dora": dora.VERSION},
        "started": time.time(),
        "nodes": nodes,
        "components": components,
        "edges": edges,
        "exposed": exposed,
        "recordings": recordings,
        "schemas": {kind: schema_line(message_type(kind), codec) for kind in sorted(types)},
        "datasheet": sheet,
    }


def schema_line(cls, codec) -> str:
    """A type's Arrow tree as `name:format:nullable:children;` depth first (lk-dora-host builds ArrowSchemas)."""
    parts = []

    def walk(node) -> None:
        parts.append(f"{node.name}:{node.format}:{int(node.nullable)}:{len(node.children)};")
        for child in node.children:
            walk(child)

    walk(codec.Node("", "+s", children=codec.schema(cls), message=cls))
    return "".join(parts)


def host_lines(document: dict, process: str) -> str:
    """The facts lk-dora-host needs, one per line: `key value...` (plan.json stays the source of truth)."""
    node = document["nodes"][process]
    (group,) = node["groups"]
    (leaf,) = group["order"]
    component, sheet = document["components"][leaf], document["datasheet"]["components"][leaf]
    ins = [name for name, port in sheet["ports"].items() if port["dir"] == "in"]
    outs = [name for name, port in sheet["ports"].items() if port["dir"] == "out" and name not in FACILITY_PORTS]
    lines = [
        f"so {component['impl']}",
        f"path {leaf}",
        (
            f"place {group['hz'] or 0} {group['fifo'] or 0} {0 if document['realtime'] else 1} "
            f"{','.join(map(str, group['cpus'] or [])) or '-'}"
        ),
        f"until {document['until'] if document['until'] is not None else '-'}",
        f"versions {document['versions']['like_keras']} {document['versions']['dora']}",
        f"markers {document['markers']}",
        f"log {document['directory']}/logs/{process}.log",
        f"params {json.dumps(component['params'])}",
    ]
    lines += [f"peer {peer}" for peer in node["peers"]]
    for output, kind in node["outputs"].items():
        port = output[len(leaf) + 1 :]
        index = (outs + list(FACILITY_PORTS)).index(port)
        lines.append(f"out {index} {output} {sheet['ports'][port]['hash']} {document['schemas'][kind]}")
    for name, spec in node["inputs"].items():
        for target, port in spec["deliver"]:
            if len(spec["deliver"]) > 1:
                raise ValueError(f"{name} feeds {len(spec['deliver'])} inputs of the native box {leaf}: wire one")
            port_sheet = sheet["ports"][port]
            lines.append(
                f"in {ins.index(port)} {name} {spec['source'].split('/')[0]} {port_sheet['hash']} "
                f"{port_sheet['queue'] or 0} {document['schemas'][spec['type']]}"
            )
    return "\n".join(lines) + "\n"


def native_host() -> str:
    """lk-dora-host: on PATH, else where cpp/hosts/build_dora_cxx.sh builds it."""
    import shutil

    found = shutil.which("lk-dora-host")
    return found or str(pathlib.Path(__file__).resolve().parents[4] / "build" / "hosts" / "lk-dora-host")


def python_path() -> list[str]:
    """The launcher's import path minus the interpreter's own: a node rebuilds components from the same modules."""
    own = {os.path.realpath(prefix) for prefix in (sys.prefix, sys.base_prefix)}
    entries = [os.path.abspath(entry or os.getcwd()) for entry in sys.path]
    return list(
        dict.fromkeys(
            entry
            for entry in entries
            if os.path.isdir(entry) and not any(os.path.realpath(entry).startswith(prefix) for prefix in own)
        )
    )


def stop_natives(document: dict, timeout_s: float) -> None:
    """Real time, before `dora stop`: SIGTERM each native node of this run on this machine (lk-dora-host, its pid in
    markers/started.<node>) and wait until it has deactivated its box and sent the last records (markers/drained.<node>).
    WHY first (fix round 1 after the first consumer, item 2; CONTRACT §15 A28): `dora stop` stops every node at once,
    so the gateway and the Python readers were gone before lk-dora-host's final drain reached them: no hand-back line
    in the recording, and health left `operational`. The node then waits for dora's Stop, so it is never restarted."""
    import signal

    markers = pathlib.Path(document["markers"])
    plan = str(pathlib.Path(document["directory"]) / "plan.json")
    natives = [name for name, node in document["nodes"].items() if node["role"] == "native" and not node["host"]]
    signalled = []
    for name in natives:
        try:
            pid = int((markers / f"started.{name}").read_text())
            argv = pathlib.Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")
            if argv[1:3] == [plan.encode(), name.encode()]:  # a pid is reused once its process is gone: only ours
                os.kill(pid, signal.SIGTERM)
                signalled.append(name)
        except (OSError, ValueError, IndexError):  # it never started, or it has exited: `dora stop` covers it
            continue
    # WHY only those signalled: a node that never started (one holding a refusal, A34) waits for `dora stop` itself
    wait_markers(document["markers"], "drained|exit", signalled, timeout_s, quiet=True, failures_raise=False)


def refusal(markers: pathlib.Path, node: str) -> str:
    """What a node said when it refused (its failed marker, else its activate's refused marker), or ''."""
    for kind in ("failed", "refused"):
        if (markers / f"{kind}.{node}").exists():
            return (markers / f"{kind}.{node}").read_text() or f"{node}: refused"
    return ""


def wait_markers(
    markers: str, kind: str, nodes: list[str], timeout_s: float, quiet: bool = False, failures_raise: bool = True
) -> None:
    """Wait for markers/<kind>.<node> of every node (`kind` may be "started|refused"); raise naming the missing
    ones (unless quiet), or a node's refusal: marked failed, or refused in activate (final, CONTRACT §15 A34)."""
    kinds, deadline, markers = kind.split("|"), time.monotonic() + timeout_s, pathlib.Path(markers)
    directory = markers.parents[1]
    while True:
        missing = [node for node in nodes if not any((markers / f"{one}.{node}").exists() for one in kinds)]
        failures = [text for node in nodes if (text := refusal(markers, node))]
        if failures and failures_raise:
            raise RuntimeError("a dora node refused:\n" + "\n".join(failures))
        if not missing or time.monotonic() > deadline:
            break
        time.sleep(0.05)
    if missing and not quiet:
        raise TimeoutError(
            f"dora nodes {missing} did not reach {kind} within {timeout_s:.0f} s (their output: {directory / 'out'})"
        )
