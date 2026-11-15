"""Sinks for the facility ports every component has (CONTRACT §8): log, health and metrics, written to JSON lines."""

from __future__ import annotations

import dataclasses
import json

import like_keras as lk
from robot_msgs.std import DiagnosticRecord, LogRecord, Metrics


@dataclasses.dataclass(frozen=True)
class FacilityParams:
    path: str


class _Rows:
    def _write(self, row: dict) -> None:
        if not hasattr(self, "_file"):
            self._file = open(self.params.path, "a", encoding="utf-8")
        row["now"] = self.now()
        self._file.write(json.dumps(row) + "\n")
        self._file.flush()


class LogLog(_Rows, lk.Component):
    value = lk.In(LogRecord, queue=1000)
    Params = FacilityParams

    def forward(self, value):
        self._write({"stamp": value.header.stamp, "path": value.path, "tick": value.tick, "level": value.level,
                     "message": value.message, "tick_type": type(value.tick).__name__})


class HealthLog(_Rows, lk.Component):
    value = lk.In(DiagnosticRecord, queue=1000)
    Params = FacilityParams

    def forward(self, value):
        self._write({"stamp": value.header.stamp, "path": value.path, "level": value.level, "code": value.code,
                     "message": value.message, "mode": value.mode, "expected_period_s": value.expected_period_s,
                     "uptime_s": value.uptime_s, "stale_inputs": list(value.stale_inputs)})


class MetricsLog(_Rows, lk.Component):
    value = lk.In(Metrics, queue=1000)
    Params = FacilityParams

    def forward(self, value):
        self._write({"stamp": value.header.stamp, "path": value.path,
                     "values": {name: float(number) for name, number in zip(value.names, value.values)}})
