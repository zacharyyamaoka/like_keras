"""The raw dora 0.5.0 baseline of criterion 2: a sender, a forwarder and a receiver written straight on dora's
Python API, the way a user without like_keras would (kpi/bench_overhead.py runs it A/B against like_keras).

    python raw_dora_node.py sender <bytes> <samples> | forwarder | receiver <out.npy>

The sender is open loop at 1 kHz from a deadline (its scheduled time rides in the metadata); the receiver stores
receive wall time minus that scheduled time, and whether it could read the payload with zero copies.
"""

import sys
import time

import numpy as np
import pyarrow as pa
from dora import Node


def sender(size: int, samples: int) -> None:
    node = Node()
    payload = pa.array(np.random.default_rng(size).integers(0, 255, size, dtype=np.uint8))
    start = time.time() + 2.0  # let the forwarder and the receiver come up
    for index in range(samples):
        scheduled = start + index / 1000.0
        delay = scheduled - time.time()
        if delay > 0:
            time.sleep(delay)
        node.send_output("data", payload, {"scheduled": scheduled})
    node.send_output("done", pa.array([1], type=pa.uint8()))


def forwarder() -> None:
    node = Node()
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "data":
            node.send_output("data", event["value"], event["metadata"])
        elif event["type"] == "INPUT" and event["id"] == "done":
            node.send_output("done", event["value"])


def receiver(out: str) -> None:
    node = Node()
    latencies, zero_copy = [], True
    for event in node:
        if event["type"] != "INPUT":
            continue
        if event["id"] == "done":
            break
        now = time.time()
        latencies.append(now - event["metadata"]["scheduled"])
        try:
            event["value"].to_numpy(zero_copy_only=True)
        except Exception:
            zero_copy = False
    np.save(out, np.array(latencies))
    with open(out + ".copies", "w") as file:
        file.write("0" if zero_copy else "1")


if __name__ == "__main__":
    role = sys.argv[1]
    if role == "sender":
        sender(int(sys.argv[2]), int(sys.argv[3]))
    elif role == "forwarder":
        forwarder()
    else:
        receiver(sys.argv[2])
