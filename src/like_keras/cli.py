"""`lk`: look at a running deployment through its attach socket (CONTRACT §9.3).

    lk ls [ADDRESS]     the exposed ports: name, direction, type, schema hash, policy; the backend and its version
    lk top ADDRESS      every component's health and metrics, live; STALE when a record is overdue

ADDRESS is the explicit `unix:<path>` that describe() prints. `lk ls` without one lists the sockets under
$XDG_RUNTIME_DIR/like_keras as a convenience, never as the source of truth. `lk top` prints plain text, one block a
second, when its output is not a terminal (Ctrl-C ends it).
"""

from __future__ import annotations

import argparse
import importlib
import math
import os
import select
import signal
import struct
import sys
import time

attach = importlib.import_module("like_keras.attach")  # the module: the package attribute `attach` is lk.attach


def ls(address: str | None) -> int:
    if address is None:
        directory = os.path.join(os.environ.get("XDG_RUNTIME_DIR") or "/tmp", "like_keras")
        found = sorted(name for name in os.listdir(directory) if name.endswith(".sock")) if os.path.isdir(directory) else []
        for name in found:
            print(f"unix:{os.path.join(directory, name)}")
        return 0
    hello, sheet, connection = attach.request(address)
    connection.close()
    backend = sheet["backend"]
    print(f"{sheet['system']}  {address}  like_keras {sheet['like_keras']}  abi {sheet['abi']}  "
          f"backend {backend['name']} {backend['version']}  describe {sheet['hash'][:16]}")
    for name, port in attach.exposed_ports(sheet).items():
        policy = f"queue({port['queue']})" if port["queue"] else "latest"
        stale = f" stale={port['stale']}" if port["stale"] else ""
        print(f"  {name:<20} {port['dir']:<4} {port['type']:<28} {port['hash']}  {policy}{stale}  "
              f"<- {sheet['exposed'][name]}")
    return 0


def _overdue(health, now: float) -> bool:
    """STALE (io-diagnostics record v0): no record for max(3 x expected_period_s, 5 s)."""
    record, heard = health
    return now - heard > max(3.0 * record.expected_period_s, 5.0)


def _render(sheet: dict, health: dict, metrics: dict, started: float) -> str:
    now = time.monotonic()
    natives = {leaf for leaf, component in sheet["components"].items() if component["language"] == "native"}
    paths = sorted(set(health) | set(metrics) | set(sheet["components"]))
    lines = [f"{sheet['system']}  {time.strftime('%H:%M:%S')}  backend {sheet['backend']['name']} "
             f"{sheet['backend']['version']}"]
    for name in paths:
        if name in health:
            record = health[name][0]
            level = "STALE" if _overdue(health[name], now) else record.level
            status = f"{level:<7} {record.code or '-':<14} mode={record.mode}"
            if record.stale_inputs:
                status += f" stale_inputs={','.join(record.stale_inputs)}"
            if record.message:
                status += f"  {record.message}"
        elif name in natives or name not in sheet["components"]:
            status = "-"  # a native box's own record carries metrics, never health (CONTRACT §15 A9)
        else:
            status = "STALE   no record yet" if now - started > 5.0 else "unknown waiting for the first record"
        lines.append(f"{name}  {status}")
        if name in metrics:
            values = metrics[name][0]
            lines.append("    " + "  ".join(
                f"{key}={value:.6g}" if math.isfinite(value) else f"{key}={value}"
                for key, value in zip(values.names, (float(number) for number in values.values))
            ))
    return "\n".join(lines)


def top(address: str, once: bool = False) -> int:
    from like_keras import codec
    from like_keras.component import message_type

    _, sheet, connection = attach.request(address)
    connection.sendall(attach.frame(attach.SUB, b'["' + attach.FACILITIES.encode() + b'"]'))
    connection.setblocking(False)
    kinds = {codec.schema_hash(message_type(name)): message_type(name) for name in ("std/DiagnosticRecord", "std/Metrics")}
    health, metrics, buffer, started = {}, {}, bytearray(), time.monotonic()
    interactive, next_print = sys.stdout.isatty(), time.monotonic() + 1.0
    stopping = []
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    while not stopping:
        readable, _, _ = select.select([connection], [], [], max(0.0, next_print - time.monotonic()))
        if readable:
            chunk = connection.recv(1 << 20)
            if not chunk:
                print(f"{address} closed", file=sys.stderr)
                break
            buffer += chunk
            for kind, payload in attach.frames(buffer):
                if kind != attach.DATA or struct.unpack_from("<H", payload)[0] != attach.FACILITY_PORT:
                    continue
                cls = kinds.get(struct.unpack_from("<Q", payload, 2 + 8)[0])
                if cls is None:
                    continue
                record = codec.decode(payload[2:], cls)
                (health if cls.type_name == "std/DiagnosticRecord" else metrics)[record.path] = (record, time.monotonic())
        if time.monotonic() >= next_print:
            next_print += 1.0
            text = _render(sheet, health, metrics, started)
            print(("\x1b[H\x1b[2J" if interactive else "") + text + ("" if interactive else "\n"), flush=True)
            if once:
                break
    connection.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="lk", description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("ls", help="list a deployment's exposed ports")
    listing.add_argument("address", nargs="?", help="unix:<path>, as describe() prints it")
    watching = commands.add_parser("top", help="every component's health and metrics, live")
    watching.add_argument("address", help="unix:<path>, as describe() prints it")
    watching.add_argument("--once", action="store_true", help="print one block and exit")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "ls":
            return ls(arguments.address)
        return top(arguments.address, arguments.once)
    except (OSError, ValueError) as error:
        print(f"lk {arguments.command}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
