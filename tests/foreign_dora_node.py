"""A foreign dora node for tests/test_dora_boundary.py's shared-daemon case (CONTRACT §15 A35; the library verifier's
probe, credit): plain dora, not like_keras. It only reads a timer, so a daemon that runs it beside a like_keras
dataflow is SHARED, and lk-dora-host must leave that daemon alone."""

from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
