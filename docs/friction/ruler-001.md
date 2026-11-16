# ruler-001: criterion 2's cross-process A/B cannot score from `kpi/score.py` on bam-GPU (M0 verification, 2026-11-15)

From the M0 verification lane, measured against the ruler at `b76d5b2`. **This does not block M0:** outside the timing tier the A/B is skipped by design, and the empty repo scores 0 either way. It matters at M3, when the dora lane's A/B first runs in the timing tier. The ruler lane owns the fix, because `kpi/` is read-only to everyone else.

## 1. The A/B runs in the shared network namespace, where the private dora refuses (criterion 2, cross-process part)

- `kpi/score.py:356` calls `kpi/bench_overhead.py` directly, through `run_json`. It does not go through `kpi/run.py`'s `bwrap --unshare-net --unshare-pid`, which the 12 pytest suites use.
- In the timing tier, `bench_overhead.cross_process()` starts `kpi.support.dora_private.PrivateDora`. That refuses whenever 127.0.0.1:6012 answers, and returns `{"error": "no private dora: ..."}`.
- On bam-GPU the production coordinator and daemon listen all day: `ss -ltn` showed `0.0.0.0:6012`, `0.0.0.0:53290` and `127.0.0.1:53291` at 10:45. So even with `KPI_TIER=timing` and the rack unpowered, every payload row is empty, and the cross-process part of criterion 2 (a quarter of its 13 weight) scores 0. Only a host without the production dora, such as the cloud VM, could score it.
- **Suggested fix:** run `bench_overhead.py` through the same private-namespace wrapper as the suites (the `bwrap` line of `kpi/run.py:command`).

## 2. `score.py` does not pin the A/B to cores 20-21

CONTRACT §11, criterion 2, says to pin the A/B to cores 20-21 (`kpi/bench_overhead.py --pin 20,21`). `score.py` passes no `--pin`, so in the timing tier the A/B runs on whatever cores the scheduler picks. The fix is to pass `--pin 20,21` when `KPI_TIER=timing`.

## 3. A skipped valid graph never counts as a false positive (criterion 12, low risk)

- `score.py:wiring_checks` takes false positives only from `failed` outcomes in `validgraphs.xml`. It needs at least one pass, but it never requires every valid graph to pass.
- Two graphs (`test_validgraphs.py:138`, `:152`) skip when `build/kpi/libfake_native_ok.so` is not built. If they skip, criterion 12 can still score 1.0 with those graphs never checked.
- cmake always builds that `.so`, so in practice the risk is small. But it contradicts the README's line that "a test that skips, errors or is missing counts as failed".
- **Suggested fix:** count a valid graph that does not pass as a false positive, or freeze the list of `validgraphs` ids in `score.py`.

## Resolution (2026-11-15, dispatcher)

See CONTRACT.md §15 (amendments A1-A7). Decided: A7, the ruler lane fixes it in the M0.1 commit.
