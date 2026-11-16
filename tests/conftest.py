"""The lane tests' shared fixtures. WHY one place: a session fixture imported by two test modules is two fixtures,
and the second private dora refuses to start while the first answers on 127.0.0.1:6012 (kpi/support/dora_private)."""

from kpi.support.dora_private import private_dora  # noqa: F401  (the dora cases' session fixture)
