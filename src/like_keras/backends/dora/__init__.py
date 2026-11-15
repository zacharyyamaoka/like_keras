"""The dora backend (CONTRACT §9.2): a STAND-IN written by lane core at M1, replaced by lane dora at M3.

It declares the backend's identity and capabilities as data, so describe() and the §4.3 checks run against a dora
deployment (kpi/validgraphs, kpi/miswire), and the reference cell's `from like_keras.backends import dora` resolves.
start() refuses. See docs/friction/contract-003.md.
"""

from like_keras.backends import address  # noqa: F401  (the attach socket describe() prints)

NAME = "dora"
VERSION = "0.5.0"  # CONTRACT §2.2: the one place the dora version is written
# processes: every process is a dora node of its own, so every component is rebuilt there as module:qualname(**params);
# no mixed_process: a native .so runs in lk-dora-host, a process of its own (CONTRACT §4.3 check 3).
CAPABILITIES = ("latest", "queue", "lagged", "host", "lockstep", "processes")


def start(plan, until):
    raise NotImplementedError(
        "the dora backend is a stand-in until lane dora lands it (CONTRACT §12, M3); "
        "deploy with the default inproc backend meanwhile"
    )
