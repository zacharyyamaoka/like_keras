# ruler-007: the dora CLI's uv/pip lookup reaches two places in kpi/ (lane dora, M3 fix round 1, 2026-11-16)

dora 0.5.0 builds its `--version` string every time it parses its arguments (`binaries/cli/src/lib.rs`:
`#[clap(version = get_version_info())]`). So every `dora` call (`--version`, `list`, `check`, `start`, `stop`,
`coordinator`, `daemon`) first runs `uv pip show dora-rs`, then `pip show dora-rs`, off PATH. From a directory with
no `.venv` above it (a scratch copy, or any user's project), `uv` fails and `/usr/bin/pip` starts. Measured in the
sandbox at load 25-33 (rack powered, cores 22-23, not the timing tier):

- `dora list`, as is, took 0.53-0.95 s. With `uv` and `pip` shadowed by `false`, it took 0.01-0.04 s.
- `check_version()` (150 calls) had a median of 0.66-0.88 s, a max of 1.82 s and 2 refusals. With the fix, the
  median was 0.03-0.06 s, the max 0.26 s, with 0 refusals.

`src/like_keras/backends/dora/__init__.py` now runs every CLI call that way (fix round 1). The ruler has the same
exposure in two places. Neither is wrong today, so this note is for the dispatcher to decide.

## 1. `kpi/support/dora_private.py::PrivateDora._daemon_connected` polls `dora check` with a 3 s timeout

- It inherits PATH, so each poll pays the lookup: 0.4 s idle, up to about 2.6 s on the powered rig.
- It retries until 60 s, so a slow poll only delays the fixture; it cannot fail it.
- The coordinator and daemon pay the lookup once each, at start.
- **Proposal (optional):** run the fixture's CLI with `uv` and `pip` shadowed, as the backend does, or leave it.

## 2. T11's hanging-CLI case cannot tell a hang from a fast failure

- `kpi/churn/test_churn.py::test_T11_cli_skew_refused_within_2s[hangs]` asserts only that `0.5.0` is in the
  message and that the refusal came within 2.5 s.
- Suppose the backend ran the CLI with PATH set to an empty directory (the first fix that was suggested). Then the
  fake's `sleep 30` is not found, and the CLI exits at once printing `.../dora: 2: sleep: not found`.
- The backend then refuses it as a CLI of version `.../dora: 2: sleep: not found`. That message names `0.5.0`, so
  T11 passes without ever exercising the timeout.
- Measured with a scratch mutant of `call()` (PATH = only the hidden helpers): T11 passed 2/2 on it, the `hangs`
  case in under 5 ms. Lane dora's own
  `tests/test_dora_cli.py::test_a_hanging_cli_is_still_refused_as_a_hang_within_2s` failed on it, as intended.
- **Proposal:** in the `hangs` case, also assert `"did not answer" in message`, or `elapsed >= 1.0`. Until then,
  lane dora's test covers it.

## Resolution (2026-11-16, ruler lane)

See CONTRACT.md §15, amendment A27. §2 is done in the M0.3 commit ("kpi: M0.3 — ruler amendment A27"): `test_T11_cli_skew_refused_within_2s[hangs]` also asserts `"did not answer"` in the refusal. With the backend's CLI run on a PATH of only the hidden helpers (the mutant above), the M0.2 test passed and the M0.3 test fails, naming the `sleep: not found` refusal; on the real backend both pass. §1 is left as it is: a slow `dora check` delays the private-dora fixture but cannot fail it.
