# like_keras KPIs: what an AI build loop can measure and push (research, 2026-11-15)

**Verdict**

- **Use 5 hard gates and 10 soft KPIs.** A gate is pass/fail and is never traded away. A KPI is scored inside the gates (Zach's rule, vault note `C - Hard Gates vs Soft KPIs`, as in `robot_controllers`).
- **Speed is the latency like_keras adds over raw dora**, at p99, measured in the same run, with no extra copies or allocations. In-process, it is the cost per hop over a direct call.
- **Every KPI has a counter-metric, and every score stops rising at its target**, so the loop never gives up one KPI to over-push another (extremal Goodhart).
- **The ruler is frozen and the builder cannot touch it.** Consumer friction does not enter the score. It becomes new held-out tasks.

## 1. What can be measured automatically

| Source | Measurable idea we take |
|---|---|
| Green & Petre, cognitive dimensions ([doi](https://doi.org/10.1006/jvlc.1996.0009)); Clarke's API version ([DDJ 2004](https://drdobbs.com/windows/measuring-api-usability/184405654)) | **Viscosity** is the effort a small change takes (here, the swap cost). **Hidden dependencies** are links you cannot see in the code (here, env or magic placement). **Diffuseness** is how much text it takes to say something (here, tokens). |
| Rama & Kak 2013 ([pdf](https://engineering.purdue.edu/RVL/Publications/RamaKakAPIQ_SPE.pdf)) | Structural API metrics computed straight from signatures, such as parameter-list complexity (APXI) and overload consistency (AMNOI) |
| Scheller & Kühn 2015 ([IST](https://doi.org/10.1016/j.infsof.2015.01.009), [thesis](https://repositum.tuwien.at/handle/20.500.12708/3858)) | Automated "concept complexity" scores, validated against user studies, and applied to **middleware** APIs |
| Myers & Stylos, CACM 2016 ([acm](https://cacmb4.acm.org/magazines/2016/6/202645-improving-api-usability)) | Factories were significantly slower to use than constructors (Ellis et al. 2007) |
| Keras API design guidelines ([github](https://github.com/keras-team/governance/blob/master/keras_api_design_guidelines.md)) | At most 2–3 mental models, at most 6–7 arguments per signature, every docstring has an example, and every error says what happened, what was expected and how to fix it |
| Ousterhout's "deep modules"; Normand's critique ([link](https://ericnormand.me/issues/purelyfunctional-tv-newsletter-412-module-depth-is-bogus)) | Interface is cost, functionality is benefit. Functionality is hard to measure, so **we measure it as a frozen task suite** |
| ADK Arena ([arXiv 2606.05548](https://arxiv.org/abs/2606.05548)); OSS-UAgent ([2505.23239](https://arxiv.org/abs/2505.23239)) | A fixed coding agent learns the API from its docs; its effort is a usability proxy (5.6× cost spread over 51 frameworks) |
| Apex `performance_test` ([docs](https://docs.ros.org/en/jazzy/p/performance_test/)); wrk2 ([repo](https://github.com/giltene/wrk2)); pyperf ([tuning](https://pyperf.readthedocs.io/en/latest/system.html)) | Latency, jitter, lost samples, CPU and RSS together. Open-loop, timed from the scheduled send. Isolated cores |
| ros2_control ([writing a controller](https://control.ros.org/iron/doc/ros2_controllers/doc/writing_new_controller.html)); PickNik `EXPECT_NO_MALLOC` ([guide](https://docs.picknik.ai/api/pages/md-controllers-2-c-l-a-u-d-e/)) | No memory allocation in `update()`, enforced by a test |
| dora-node-api 0.5.0 ([source](https://docs.rs/crate/dora-node-api/0.5.0/source/src/node/mod.rs)) | `ZERO_COPY_THRESHOLD = 4096`: smaller messages go over TCP, larger ones over shared memory, so 4 KiB is a natural payload to benchmark |
| Google mutation testing ([ICSE-SEIP 2018](https://research.google/pubs/pub46584/)) | Mutants on the diff only, surfaced in code review |
| Manheim & Garrabrant ([arXiv 1803.04585](https://arxiv.org/abs/1803.04585)) | Four ways a proxy metric breaks: regressional, extremal, causal and adversarial |

## 2. Baseline, measured today on the tracer bullet `lk_dora`

Measured with a stdlib probe (prototype of `kpi/surface.py`) and `python -B -X importtime`, Python 3.11, dora-rs 0.5.0:

| Metric | `lk_dora` |
|---|---|
| Top-level public names / all public names | 11 / 43 |
| Most parameters on one callable | 6 (`Deployment.process`) |
| Core SLOC / code tokens (docstrings and comments excluded) | 784 / 5,778 |
| `import lk_dora`, cumulative | ≈ 97 ms (n = 3); `yaml` was 86 ms of one trace (eager backend import) |
| `import dora` / `import pyarrow` | 54 ms / 117 ms |
| Direct runtime dependencies | 4 (`dora-rs`, `dora-rs-cli`, `pyarrow`, `pyyaml`) |

## 3. Hard gates (any red → score 0, not landed)

| Gate | Check | Command |
|---|---|---|
| G1 Contract and equivalence | Frozen contract tests pass; each reference task gives **identical output streams** in-process, cross-process and with C++ replacements, on recorded inputs | `pytest kpi/contract` |
| G2 Black-box boundary | Components import only ports and `robot_msgs`, never `System`, `Deployment` or a backend; core never imports `dora`; no `os.environ` outside backends | `uvx --from import-linter lint-imports` plus a check of `sys.modules` after `import like_keras` |
| G3 No silent API break | A public-API break requires a version bump | `uvx griffe check like_keras -a <last tag>` |
| G4 Lean core | Core runtime dependencies ≤ 1 (`robot_msgs`). Backends are extras (`like_keras[dora]`) | `kpi/deps.py` (the `importlib.metadata.requires` closure) |
| G5 Real-time hot path | 0 heap allocations per tick, counted by a `malloc`/`new` counter over 100k steady-state ticks of the C++ in-process chain. Payload copies ≥ 4 KiB no more than raw dora's | `kpi/cpp/alloc_probe` |

## 4. Soft KPIs: the scoreboard

"Zero point" is the value that scores 0. "Target" is the value that scores 1, and scores stop rising there.

| # | KPI (weight) | Exact definition | Measured by | Zero point → target |
|---|---|---|---|---|
| K1 | **Added latency** (20) | Cross-process: like_keras p99 minus raw dora p99, same payload and placement, for payloads of 64 B, 4 KiB and 1 MiB, sent open-loop at 1 kHz, 100k samples, A/B interleaved. In-process: ns per hop over a direct call | `kpi/bench_overhead.py --pin 20,21` | +100% of raw → ≤ max(10 µs, 5%). C++ ≤ 100 ns per hop, Python ≤ 1 µs per hop |
| K2 | **Backend-swap cost** (15) | Lines changed, over **every** file (py, yml, toml, env), to move each reference task in-process↔cross-process, Python↔C++ (the C++ body itself excluded) and dora↔in-process thread backend | `kpi/swap_cost.py` (`ruff format`, then `git diff --no-index --numstat`) | 20 lines → 0 outside `Deployment`, ≤ 1 per swap inside it |
| K3 | **Cold-agent success** (15) | A fresh pinned agent (`claude-sonnet-5-5`) sees only README, DATASHEET and `help()`; pass@1 on held-out task tests, median repair rounds as tiebreaker | `kpi/agent_tasks/run.sh` (drives `claude -p`; n = 3 seeds) | 50% → ≥ 90% with ≤ 2 repair rounds |
| K4 | **Concept count** (10) | Public names in Python ∪ C++. A C++ name with no same-named Python counterpart counts again | `kpi/surface.py` and `griffe dump` | 43 → ≤ 8 top-level, ≤ 30 total |
| K5 | **Workflow size** (10) | Code tokens of the reference solutions to the frozen tasks, divided by the tokens of the frozen raw-dora solutions (dataflow.yml plus node scripts). For in-process tasks, divided by plain Python function composition instead | `kpi/surface.py --tasks` | ratio 1.0 → ≤ 0.4 of raw dora, ≤ 1.2 of plain Python |
| K6 | **Core size** (8) | Code tokens of the core plus each backend, generated and vendored code included, counted after `ruff format`. C++ is counted on comment-stripped tokens | `kpi/surface.py` | 5,778 → ≤ 4,000 core + ≤ 2,000 per backend |
| K7 | **Loop timing** (7) | Deadline misses per 1e6 cycles and p99.9 period error, for a 3-component C++ chain at 1 kHz on a moteus stub, with `system_load` running alongside | `kpi/bench_loop` | any miss → 0 misses, ≤ 50 µs |
| K8 | **Build-time error catch rate** (5) | Share of ~20 frozen mis-wirings (wrong port, type mismatch, undelayed cycle, dangling input, bad placement) raised **at build time**, naming `path.port`, the expectation and the fix | `pytest kpi/miswire` | 50% → 100% |
| K9 | **Startup** (5) | `import like_keras` cumulative time (median of 10), and time to first message, from launch to first delivery, against raw dora | `kpi/startup.py` | 97 ms → ≤ 20 ms. Time to first message ≤ raw dora + 10% |
| K10 | **Mutation score** (5) | Share of mutants killed in core modules (mull for C++) | `uvx mutmut run` (nightly), diff-only on PRs | 50% → ≥ 85% |

## 5. Goodhart risks and their counter-metrics

| KPI | How an optimizer games it | Counter-metric |
|---|---|---|
| K1 | Special-case benchmark payloads. Report the mean. Measure closed-loop, which hides stalls. Drop messages | Timed from the scheduled send. Sent = received (G1). Random payloads. Throughput and CPU % reported |
| K2 | Hide placement in env, YAML or auto-detection so the diff reads 0 (a hidden dependency) | Diff counts every file; G2 bans `os.environ`; `Deployment.describe()` prints all placement |
| K3 | Docs that leak solutions; tuning to visible tasks | 50% held-out tasks, rotated monthly; README + DATASHEET ≤ 4k tokens; pinned model |
| K4 | A god object with string options or `**kwargs` | ≤ 6 params per callable; 0 `**kwargs`/string-enum options (APXI-style); K3 |
| K5 | Magic defaults that are wrong for tasks nobody has seen | Held-out tasks, G1 and K8 |
| K6 | Code golf; complexity pushed into deps or user code | G4, generated code counted, K5, K3 |
| K7 | Busy-spin a core. Measure only when idle | CPU % per component. Run under load (the dora daemon here has stalled up to 4 s) |
| K8 | Rejecting legal graphs | A frozen *valid*-graph suite must all pass (0 false positives) |
| K9 | A lazy import that just moves the cost to the first message | Time to first message is in the same KPI |
| K10 | Exclusion pragmas; tests pinned to internals | Exclusions reported; tests import only public `like_keras` |

## 6. Design score

```text
s_i   = clamp((zero_i - x_i) / (zero_i - target_i), 0, 1)        # mirrored when higher is better
score = 100 * Σ w_i s_i / Σ w_i   if G1..G5 all green   else 0
```

Always publish the 10-number vector beside the scalar, which hides trades.

## 7. How the build loop uses it (a ratchet that never regresses)

- **Ledger.** `kpi/run.py --tier fast|full|nightly` writes `kpi/results/<sha>.json` and appends to `kpi/ledger.jsonl` (keyed by host); `kpi/baseline.json` is committed, as in `robot_controllers`.
- **Land rule.** A change lands only when all gates are green, the score is at or above the baseline, and **no single KPI falls below its baseline by more than its noise band**. The one exception is a commit trailer `KPI-tradeoff: K6 4100→4300 because …` together with a score rise of at least 2 points. Accepting a regression rewrites the baseline explicitly, like `betterer --update`.
- **Noise.** Timing KPIs run as ≥ 5 interleaved A/B pairs and ratchet on the conservative edge of a bootstrap 95% CI. The winner is re-measured fresh before the baseline moves (regressional Goodhart). Hosts are never compared.
- **Frozen ruler.** `kpi/` (probes, tasks, raw-dora references, the mis-wiring and valid-graph suites, benchmarks) is committed before dispatch and read-only to the builder. Only the orchestrator changes it, re-scoring both old and new head.
- **Cadence.** *fast* per commit (< 60 s): G1–G4, K2, K4–K6, K8, K9. *full* per merge (~10 min): K1, K7, G5 on isolated cores, never while the rack is powered, or on one fixed cloud VM. *nightly*: K3, K10.
- **Consumer friction becomes ruler, not score.** The controllers lane files `docs/friction/NNN.md` (task, blocker, workaround lines); the orchestrator turns each into a held-out task or mis-wiring case. The consumer sets the spec without being able to game the score.

**Not measured:** role-expressiveness and closeness of mapping, which need a judge. Those are for Zach's taste pass on the five proposals.
