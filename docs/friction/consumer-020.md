# consumer-020: a native box's `describe()` lists only the params its config gave, never the .so's defaults, while a Python box lists every default (controllers lane, adapting to A31-A37, 2026-11-16)

**The ask.** In the datasheet, give a native box the params a Python box gets: every key `lk_describe()` returned, with the config's value where one was given and the default otherwise. A Python ↔ C++ swap would then not change what `describe()` says the component runs with.

## What I tried

A33 let `guard_timeout_s` default to NaN (off) again, as the raw node's INI does. I wanted a test that the shipped `.so`'s own default is NaN and reads back through `lk.native()`. I first read the default from the deployment's datasheet, which is what an operator or `lk ls` would read. The key was not there.

## Contract sections

§4.4 / §4.5 (`describe()`, `native` reads `lk_describe()`), §15 A6 (`lk_describe()` builds from the default params), §15 A24 ("the datasheet must show" the per-joint table), §10 criterion 5 (the swap is one line).

## Repro

`lk/friction_repro/describe_defaults.py` (bam_ws): the controller as a native box and as its Python peer, each given only its joint table (`{"joints": [...]}`), like_keras adbac3b:

```
$SBX ~/like_keras/.venv/bin/python $PKG/lk/friction_repro/describe_defaults.py
same_box: describe() lists 1 params; guard_timeout_s: <absent>
fallback: describe() lists 24 params; guard_timeout_s: nan
lk_describe()'s defaults, kept privately by lk.native: 24 params; guard_timeout_s: nan
```

The same arm, the same config: the Python chain's datasheet has all 24 keys, the native box's has one. The defaults are known to like_keras (`lk.native()` parsed them from `lk_describe()` into the private `_lk_description`), so this is a gap in the datasheet, not missing information. `src/like_keras/component.py` `native()`: `box._lk_config = dict(config or {})`.

## What I did instead

`lk/tests/test_features.py::test_guard_timeout_nan_or_zero_is_off` reads the `.so`'s defaults through the C ABI with ctypes (`lk_describe()`, then `json.loads`), as `lk.native()` does. Every placement in the cell passes the whole params mapping (`node_ini.params()`), so the cell's own datasheets are complete; only a partial config shows the gap.

## Proposal

- **Library fix.** `native()` keeps `lk_describe()`'s `params` and the datasheet shows them overlaid with the config (nested rows as given: a vector's default is empty). One dictionary merge; no new name.
- **New ruler test.** For a box with a Python twin (the reference cell's), `describe()` with the same partial config lists the same keys and values in both languages, NaN included.
