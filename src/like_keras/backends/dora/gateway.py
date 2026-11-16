"""The gateway node of a dora deployment (CONTRACT §9.2, §9.3): `python -m like_keras.backends.dora.gateway <plan.json>`.

A node wired to every exposed port, every recorded port and every component's health and metrics. It serves the
attach socket for the whole dataflow (the OBSBOT gateway pattern: it sidesteps dora 0.5's unanswered late-joining
dynamic nodes) and writes the recordings. An attach client's write goes out on `<system>.<input>`; a retraction is
an empty frame, read by the receiving process as `None` (CONTRACT §5.4).
"""

from __future__ import annotations

import collections
import json
import pathlib
import sys
import time
import traceback
import types

from like_keras.backends.dora.emit import GATEWAY
from like_keras.backends.dora.host import FOREVER, Link, decoder, fail, log_to, mark


class Gateway:
    def __init__(self, document: dict, link: Link) -> None:
        from like_keras.component import message_type
        from like_keras.record import Recorder

        self.document, self.link, self.lockstep = document, link, not document["realtime"]
        node = document["nodes"][GATEWAY]
        self.routes: dict[str, list] = collections.defaultdict(list)  # input id -> what each value goes to
        self.decoders = {input_id: decoder(spec) for input_id, spec in node["inputs"].items()}
        self.open = set(node["inputs"])  # lockstep: inputs whose sender has not yet promised it is done
        self.recorders = []
        for recording in document["recordings"]:
            recorder = Recorder(recording["file"])
            self.recorders.append(recorder)
            for channel in recording["channels"]:
                self.routes[channel["input"]].append(recorder.channel(channel["topic"], message_type(channel["type"])))
        self.server = None
        if document["exposed"]:
            from like_keras.attach import Server

            inputs = {
                name: [types.SimpleNamespace(write=self._writer(spec["source"]))]
                for name, spec in document["exposed"].items()
                if spec["dir"] == "in"
            }
            for slot in (slot for slots in inputs.values() for slot in slots):
                slot.push = slot.write
            self.server = Server(document["datasheet"], inputs, clock=time.time, writable=not self.lockstep)
            for name, spec in document["exposed"].items():
                if spec["dir"] == "out":
                    self.routes[spec["source"]].append(lambda value, sent, name=name: self.server.publish(name, value))
            for input_id in node["inputs"]:
                if input_id.rsplit(".", 1)[1] in ("health", "metrics"):
                    self.routes[input_id].append(lambda value, sent: self.server.facility(value))
        link.receive_with(self._frame)

    def _writer(self, output: str):
        def write(value, sent) -> None:  # the attach thread: the latest written value (None: retracted)
            self.link.send(output, value)

        return write

    def _frame(self, input_id: str, array, metadata: dict) -> None:
        if input_id not in self.decoders:
            return
        if self.lockstep and max(metadata.get("lk_t", 0), metadata.get("lk_h", 0)) >= FOREVER:
            self.open.discard(input_id)
            if not self.open:  # every sender stepped to `until`: what they sent is all here
                mark(self.document, "done", GATEWAY)  # WHY no flush here: one thread writes a recording (run())
        if not len(array):
            return  # under lockstep an empty frame only carries a promise
        try:
            value = self.decoders[input_id](array)
        except (ValueError, TypeError) as refused:
            print(f"like_keras {GATEWAY}: {input_id}: refused a frame: {refused}", file=sys.stderr, flush=True)
            return
        for route in self.routes[input_id]:
            route(value, 0.0)

    def run(self) -> int:
        if self.lockstep:  # an attach client never writes into a lockstep run: promise its readers nothing, ever
            for output in self.document["nodes"][GATEWAY]["outputs"]:
                self.link.send(output, None, {"lk_t": FOREVER})
            if not self.open:
                mark(self.document, "done", GATEWAY)
        mark(self.document, "started", GATEWAY)
        while not self.link.stopping.wait(1.0):
            for recorder in self.recorders:
                recorder.flush()
        if self.server is not None:
            self.server.close()
        self.link.rx.join(timeout=2.0)  # what the RX thread decoded last goes into the recordings
        for recorder in self.recorders:
            recorder.close()
        return 0


def main(argv: list[str]) -> int:
    document = json.loads(pathlib.Path(argv[1]).read_text())
    log_to(document, GATEWAY)
    try:
        link = Link(document, GATEWAY)
        try:
            gateway = Gateway(document, link)
        except Exception as error:  # noqa: BLE001 - it cannot serve or record here: say so once, never loop silently
            traceback.print_exc()
            fail(document, GATEWAY, f"cannot serve or record: {type(error).__name__}: {error}", 1)
        link.wait_ready()
        return gateway.run()
    finally:
        mark(document, "exit", GATEWAY)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
