"""The golden instances as robot_msgs objects, and a component that emits one (CONTRACT §7.2; T10).

WHY a module of its own: the tests that block pyarrow (kpi/churn's wire test) import `emitter` from here. Beside
the Arrow oracle (kpi/golden/test_golden.py imports kpi/golden/arrow_oracle.py, which imports pyarrow) the import
itself would fail, so blocking pyarrow would test the ruler, not like_keras (ruler-002 §3). numpy, robot_msgs and
the ruler's own codec only.
"""

from __future__ import annotations

from kpi.golden import lkf, make_golden


def build(type_name: str, message: dict):
    """The robot_msgs object for an instance dict."""
    import robot_msgs

    cls = robot_msgs.message_types()[type_name]
    values = {}
    for field in lkf.SCHEMAS[type_name]:
        value = message[field.name]
        if field.kind == "struct" and value is not None:
            value = build(field.of, value)
        values[field.name] = value
    return cls(**values)


def emitter(lk, type_name: str):
    """A source component class that sends the golden instance of `type_name` once, on its first step."""
    cls = __import__("robot_msgs").message_types()[type_name]
    instance = build(type_name, make_golden.INSTANCES[type_name])

    class Emit(lk.Component):
        value = lk.Out(cls)

        def forward(self):
            sent = getattr(self, "_sent", False)
            self._sent = True
            return None if sent else instance

    return Emit
