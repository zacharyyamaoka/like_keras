"""A byte pipe between two stream sockets, stdlib only (it runs on the Windows camera box and on a cloud VM too).

    python relay.py --listen tcp:0.0.0.0:47012 --to tcp:192.168.1.65:47011 --lifetime 600
    python relay.py --listen unix:/run/user/1000/like_keras/lan.sock --to tcp:127.0.0.1:47032

WHY it exists: like_keras's attach socket is unix-only (attach.py refuses any other address), so a core on another host is
reached by carrying its unix socket's bytes over TCP. It understands nothing of the protocol: the handshake, the fingerprint
check and the lease are like_keras's own, end to end. (socat does the same where it is installed; the box and the VM have none.)
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time


def _address(text: str):
    kind, _, rest = text.partition(":")
    if kind == "unix":
        return socket.AF_UNIX, rest  # WHY only looked up here: Windows' socket module has no AF_UNIX
    if kind == "tcp":
        host, _, port = rest.rpartition(":")
        return socket.AF_INET, (host, int(port))
    raise SystemExit(f"address {text!r}: use unix:<path> or tcp:<host>:<port>")


def _pump(src: socket.socket, dst: socket.socket) -> None:
    try:
        while data := src.recv(65536):
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--listen", required=True)
    parser.add_argument("--to", required=True)
    parser.add_argument("--lifetime", type=float, default=0.0, help="exit after this many seconds (0: never)")
    arguments = parser.parse_args(argv)
    family, where = _address(arguments.listen)
    to_family, to = _address(arguments.to)
    server = socket.socket(family, socket.SOCK_STREAM)
    unix = family != socket.AF_INET
    if unix:
        if os.path.exists(where):
            os.unlink(where)
    else:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(where)
    server.listen()
    server.settimeout(1.0)
    print(f"relay {arguments.listen} -> {arguments.to}", flush=True)
    end = time.monotonic() + arguments.lifetime if arguments.lifetime else None
    while end is None or time.monotonic() < end:
        try:
            client, _ = server.accept()
        except TimeoutError:
            continue
        upstream = socket.socket(to_family, socket.SOCK_STREAM)
        upstream.connect(to)
        for s in (client, upstream):
            if s.family == socket.AF_INET:
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # WHY: small frames; Nagle would add ~40 ms
        threading.Thread(target=_pump, args=(client, upstream), daemon=True).start()
        threading.Thread(target=_pump, args=(upstream, client), daemon=True).start()
    server.close()
    if unix and os.path.exists(where):
        os.unlink(where)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
