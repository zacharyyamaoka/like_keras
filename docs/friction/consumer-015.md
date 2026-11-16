# consumer-015: a Python `lk.In` cannot be optional, so the Python peer of a C++ component with an optional input sets a private attribute (controllers lane, re-port onto A16-A28, 2026-11-16)

**The ask.** Give the Python `lk.In` the option the C++ `InOpts{.optional = true}` has (A23): `lk.In(T, optional=True)`. It is an option, like `fresh=` (A25), so no new public name.

## What I tried

A23 let me port the raw node's guard deadman and its stop and brake requests. The C++ chain has a `verdict` input, `lk::In<rm::DiagnosticRecord> verdict{this, "verdict", {.optional = true}}`, and the twin cell leaves it unwired, so no guard is configured. The all-Python fallback `ArmControllerPy` must expose the same ports, so that swapping it in stays the one-line change of §10 (criterion 5, T1). Its `MoteusChainPy.verdict` must therefore also be allowed to stay unwired.

## Contract sections

§15 A23 (C++ optional input), §15 A11 (the attach proxy's inputs are optional, internally), §4.1 (`In` options), §10 T1 and criterion 5 (the swap is one line).

## Repro

Python's `lk.In` signature (`src/like_keras/component.py:81`):

```
def __init__(self, type, *, queue=None, stale=None, lagged=False, fresh=False)
```

Without the option (the attribute put back to `False`), the fallback cell fails `describe()`:

```
$SBX ~/like_keras/.venv/bin/python -c "import arm_controller_py as a, cell; \
  a.MoteusChainPy.__dict__['verdict'].optional = False; cell.fallback(realtime=False).describe()"
ValueError: arm_cell: 1 problem(s) found before start (CONTRACT §4.3):
- arm_cell.arm.chain.verdict is not bound (added at arm_controller_py.py:504). Fix: bind it with add(..., verdict=<an Out>) or wire(...), or expose it
```

Exposing it is no way out: `ArmControllerPy` already exposes `verdict` on the arm System, and the error above is with that. A nested System's exposed input is a boundary its parent must bind, and only the top-level System's may stay unbound (§4.3 check 1). The C++ `lk.native` arm in the same cell needs nothing, because its port is optional.

## What I did instead

`lk/arm_controller_py.py`:

```python
def _optional(port: lk.In) -> lk.In:
    port.optional = True  # WHY the private attribute: Python's lk.In has no optional= ...
    return port

class MoteusChainPy(lk.Component):
    verdict = _optional(lk.In(DiagnosticRecord, fresh=True))
    def forward(self, trajectory, verdict=None): ...
```

It works. The attribute `lk.native()` and `attach` already set is honoured by `system.py:315`. An unwired input is then left out of `forward`'s arguments, hence `verdict=None`. But it is private API, and a refactor of `_Port` would silently break the swap.

## Proposal

- **Library fix.** `lk.In(T, *, ..., optional=False)`, with A23's meaning: unwired is legal, `forward` gets `None` for it, and `describe()` lists it.
- **New ruler test.** A Python component with an optional input that stays unwired passes `describe()`, and `forward` gets `None`; the same component wired gets the values.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A31-A36); its frozen test comes in ruler M0.5 (A38).
