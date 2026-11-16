# consumer-016: a NaN or infinite LK_PARAMS default breaks `lk.native()`, because `lk_describe()` prints it as `nan`, which no JSON reader takes (controllers lane, re-port onto A24, 2026-11-16)

**The ask.** `lk_describe()` must print every params default as JSON that `lk.native()`'s `json.loads` reads back. Today a `NaN` or `±inf` default comes out as `nan` or `inf`. Either print `NaN` / `Infinity` / `-Infinity` (Python's spelling, which `lk_create`'s `strtod` already parses), or `null`, or refuse such a default at build time with a message.

## What I tried

A24 moved the controller's config from the node's INI into `LK_PARAMS`. In that config, NaN means "off" in three places, all from the raw node's `mcn/config.hpp`:

- `guard_timeout_s` (the deadman);
- `hold_kp_scale` / `hold_kd_scale` ("the run scale");
- `path_tolerance_rad` ("no path guard").

The position limits default to `±inf`. With `guard_timeout_s = NaN` as the default, `lk.native()` failed before anything ran.

## Contract sections

§6.1 (`LK_PARAMS`: "JSON → Type"), §15 A24 (vector params, shown in `describe()`), §15 A6 (`lk_describe()` builds from the default params), §4.5 (`native` reads `lk_describe()`).

## Repro

`lk/friction_repro/nan_default.cpp`: one param, `double guard_timeout_s = NaN`. Build it with `cmake -S lk/friction_repro -B <build> -DLIKE_KERAS_ROOT=/home/bam/like_keras`, then:

```
$SBX ~/like_keras/.venv/bin/python -c "import ctypes, like_keras as lk; \
  lib = ctypes.CDLL('<build>/libnan_default.so'); lib.lk_describe.restype = ctypes.c_char_p; \
  print(lib.lk_describe().decode()); lk.native('<build>/libnan_default.so')"
{"like_keras": "0.1.0", "abi": 1, "system": "nan_default", ... "params": {"guard_timeout_s": nan}}
json.decoder.JSONDecodeError: Expecting value: line 1 column 275 (char 274)
```

Cause: `cpp/src/abi.cpp` `number()` uses `std::to_chars`, which writes `nan` and `inf`. The other direction already works. A config that carries `NaN` / `Infinity` (Python's `json.dumps`) reaches `lk_create`, and `strtod` reads it, which is how every joint row of the twin passes its NaN hold scales and infinite limits today.

A related reading of A6 that also cost a workaround: `lk_describe()` constructs the System from the **default** params. So a constructor that refuses an empty joint table, the obvious place to refuse it, makes `lk_describe()` return no ports, with `"error": "params.joints: the chain drives at least one joint"`. The check had to move to `activate()`.

## What I did instead

- Every top-level default is finite. `guard_timeout_s` defaults to `0.0`, meaning off; `0` or `NaN` from a config is also off. The joint rows keep their NaN and inf defaults, because an empty default table never prints a row.
- The "at least one joint" check moved from `lk_create` to `activate()`. Both changes carry a WHY comment in `lk/arm_controller.cpp`.

## Proposal

- **Library fix.** In `abi.cpp`'s `number()`, write `NaN`, `Infinity` and `-Infinity` (what `strtod` and Python both read), or `null` for NaN. Then a component's natural "NaN = off" default round-trips.
- **New ruler test.** A component with a NaN and an infinite default passes `lk.native()`, and `describe()` shows both.
- **Contract (A6).** State that defaults must construct, and that a component should refuse a missing table in `activate()`, not in its constructor. Better: let `lk_describe()` skip running a factory that throws on its defaults and list the ports from a second, documented `describe` path.

## Resolution (2026-11-16, dispatcher)

Decided in CONTRACT.md §15 (A31-A36); its frozen test comes in ruler M0.5 (A38).

## Verified by the consumer (2026-11-16, adapting to A31-A37)

Resolved by A33. `guard_timeout_s` defaults to NaN (off) again in `ChainParams`, in the Python peer's `Params` and in `node_ini.params()`, which no longer turns the INI's NaN into 0; 0 is still off.

- The repro of this note now loads (built against adbac3b):
  ```
  "params": {"guard_timeout_s": NaN}}
  lk.native ok; default guard_timeout_s = nan
  ```
- `lk/tests/test_features.py::test_guard_timeout_nan_or_zero_is_off`: the `.so`'s own default reads back through `lk_describe()` and `lk.native()` as NaN, and with `guard_timeout_s` NaN or 0 a guard that goes silent mid-plan stops nothing in either placement (the plan runs to its end, the same commands). Every cell placement now passes NaN through JSON to `lk_create` and to `lk-dora-host` (T1 target).
- Not addressed by A33, and still how the port works: a factory that refuses an empty joint table makes `lk_describe()` list no ports, so that check stays in `activate()`. Found meanwhile and filed as consumer-020: a native box's `describe()` shows only the params its config gave, so the test reads the defaults through `lk_describe()` itself.
- bam_ws `claude/cpp-controller-nodes-f9a521` @ `ed5b76c96`, `src/core/mdp/agent/actuators/moteus_controller_node/lk`, built and run against a `git archive` of like_keras `adbac3b` (the live checkout was being edited by the A35 lane); `lk/tests` green 3 of 3 back to back in the lane's sandbox, 32 passed and 1 skipped (a timing-tier test) each, rack powered, cores 22-23, load 14-37, not the timing tier.
