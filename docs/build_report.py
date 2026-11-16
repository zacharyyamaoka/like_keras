#!/usr/bin/env python3
"""Build the like_keras v0 report: one self-contained HTML page, from real runs and real files only.

    python3 /home/bam/like_keras/docs/build_report.py extract   # scoreboard + evidence files under docs/scoreboard/
    python3 /home/bam/like_keras/docs/build_report.py capture   # the consumer's cell, real runs on its twin (~1 min)
    python3 /home/bam/like_keras/docs/build_report.py render    # the hero video (MP4 + GIF) from the capture
    python3 /home/bam/like_keras/docs/build_report.py page      # reports/media/like-keras-v0-<date>.html
    python3 /home/bam/like_keras/docs/build_report.py all       # render + page (the default)

Stages and what each reads:
- extract: build/kpi/scoreboard-*.json (the milestone history, gitignored), the A35 A/B JSONs in the lanes' scratch
  folders, and `kpi/surface.py --api`. It writes docs/scoreboard/<date>.json (a byte copy of the latest scoreboard),
  docs/scoreboard/<date>.md (each criterion's raw number vs target vs zero point) and docs/scoreboard/<date>-evidence.json
  (the ratchet history, the A/B rows and the API names), so `page` rebuilds after the scratch folders are gone.
- capture: builds the consumer's controller .so against this checkout, then runs the consumer's own demo
  (lk/twin_demo.py) in `target` under script(1) with `lk ls` / `lk top` on its attach socket under a second
  script(1), and `same_box` and `fallback` plainly. Everything runs in the consumer's device-free sandbox
  (lk/sandbox.sh: bwrap with no devices and its own network, BAM_LANE_NO_DEVICES=1, nice 19, cores 22-23) with a
  private XDG_RUNTIME_DIR, so its private dora and attach socket never meet a peer's. Writes build/report/cap/.
- render: replays the two script(1) logs at their recorded times into 10 fps frames (stdlib + PIL), then ffmpeg.
- page: docs/CONTRACT.md (§3, §10, §14, §15), docs/design/2026-11-15/judging.md, docs/friction/*.md, git log, the
  scoreboard files above, the capture and the hero media; the consumer's cell.py, planner.py and arm_controller.cpp.

Runs on the system python3 (stdlib; PIL for `render` only). Every python/test command it starts goes through a
device-free sandbox: the robot arm on this host is powered all day.
"""

from __future__ import annotations

import argparse
import base64
import collections
import difflib
import html
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import textwrap
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
DATE = "2026-11-16"
WORK = REPO / "build" / "report"
CAPTURE = WORK / "cap"
SCOREBOARDS = REPO / "build" / "kpi"
LATEST_SCOREBOARD = SCOREBOARDS / "scoreboard-lib4.json"
EVIDENCE = REPO / "docs" / "scoreboard" / f"{DATE}-evidence.json"
PAGE = REPO / "reports" / "media" / f"like-keras-v0-{DATE}.html"
VENV_PYTHON = REPO / ".venv" / "bin" / "python"
# WHY a short private runtime dir inside the repo: dora's daemon sockets and the attach socket live under it, and a
# unix socket path holds at most 107 bytes (kpi/run.py measured it); private, so a peer's arm_cell.sock never collides.
RUNTIME_DIR = REPO / "build" / "rx"
CONSUMER_CANDIDATES = (
    pathlib.Path("/home/bam/bam_ws/.claude/worktrees/cpp-controller-nodes-f9a521/src/core/mdp/agent/actuators/"
                 "moteus_controller_node"),
    pathlib.Path.home() / "bam_ws" / "src" / "core" / "mdp" / "agent" / "actuators" / "moteus_controller_node",
)
TIER = "rack powered, cores 22-23, not the timing tier"

# The A35 A/B evidence of 2026-11-16 (lanes' scratch folders; `extract` folds what the page needs into EVIDENCE).
_SCRATCH = pathlib.Path("/tmp/claude-1000/-home-bam-bam-ws--claude-worktrees-cpp-controller-nodes-f9a521")
AB_SOURCES = {
    "kpi_before": _SCRATCH / "8d63efe5-dad3-48eb-87e4-ce978b961621/scratchpad/lk-build/lib2-b/a35_ab_before_m05_ruler.json",
    "consumer_first_set": _SCRATCH / "8d63efe5-dad3-48eb-87e4-ce978b961621/scratchpad/lk-build/lib2-b/ab_5x30.json",
    "kpi_head": _SCRATCH / "fe9045e3-39d0-43e6-952b-a6b60a112c96/scratchpad/a35_ab_head.json",
    "consumer_final": _SCRATCH / "fe9045e3-39d0-43e6-952b-a6b60a112c96/scratchpad/ab_final.json",
}

# Each scoreboard of the build, in the order it was taken: what had just happened (from git log of build/v0).
RATCHET_LABELS = {
    "scoreboard-m0-verify.json": ("M0", "ruler: the empty repo scores 0", "ruler"),
    "scoreboard-m1.json": ("M1", "core: Python API, inproc, codec, lockstep", "library"),
    "scoreboard-m0.1.json": ("M2 · M0.1", "C++ runtime; ruler amendment A7", "library"),
    "scoreboard.json": ("M3", "dora backend, attach, lk ls / lk top", "library"),
    "scoreboard-m0.2.json": ("M0.2", "ruler A14: the guard recovers", "ruler"),
    "scoreboard-m0.3-before.json": ("A16-A27", "consumer notes 001-011 decided", "library"),
    "scoreboard-m0.3-after.json": ("M0.3", "11 consumer tests frozen, failing first", "ruler"),
    "scoreboard-lane-cpp.json": ("cpp lane", "A17, A19, A20, A22-A24, A28", "library"),
    "scoreboard-lane-core-dbff127.json": ("core lane", "A16, A20-A22, A25, A26", "library"),
    "scoreboard-lane-core.json": ("core fix", "A18 signals", "library"),
    "scoreboard-fix-lib-r1.json": ("fix round 1", "the library verifier's defects", "library"),
    "scoreboard-m0.4-after.json": ("M0.4", "ruler: fix round 1's defects, A29", "ruler"),
    "scoreboard-m0.5-after.json": ("M0.5", "tests for A31-A37 frozen, failing first", "ruler"),
    "scoreboard-lib2a.json": ("A31-A37", "consumer round 2 lands", "library"),
    "scoreboard-lib3.json": ("A35 A41 A42", "dora boundary RT, datasheet, refusals", "library"),
    "scoreboard-lib4.json": ("A44 · HEAD", "partly given rows (ruler M0.8)", "library"),
}

CRITERIA = [  # WHY these numbers: the ruler's own (criteria.md / judging.md table, kpi/score.py), quoted, never re-set
    ("1_consumer_fit", "Consumer fit", "9/9 consumer tests (T1-T8, T12)", "4/9"),
    ("2_speed", "Speed", "≤ max(10 µs, 5 %) over raw dora p99; C++ ≤ 100 ns, Python ≤ 1 µs per edge; 0 extra copies; "
     "import ≤ 20 ms", "+100 % p99; 1 µs / 10 µs per edge; 1 extra copy; 97 ms"),
    ("3_black_box", "Black box", "100 % of reference components clean", "80 %, or any controller violation"),
    ("4_succinctness", "Succinctness", "≤ 8 top-level, ≤ 30 names; ≤ 6 params; ≤ 0.4× raw dora, ≤ 1.2× plain Python",
     "11 / 43 names; 10 params; 1.0× / 2.0×"),
    ("5_swap_cost", "Swap cost", "0 component lines, ≤ 1 Deployment line per swap", "20 lines per swap"),
    ("6_dora_insulation", "dora insulation", "5/5 (dora named only in its backend; skew refused ≤ 2 s; …)", "0/5"),
    ("7_compactness", "Compactness", "total ≤ 2,450 SLOC (core 650, backend 300, attach+CLI 250, C++ 850, views 100)",
     "2× each target (total 4,900)"),
    ("8_reach", "Python + C++ reach", "5/5", "0/5"),
    ("9_facilities", "Facilities", "7/7, facility SLOC ≤ 30 %", "3/7"),
    ("10_testability", "Testability", "4/4 at ≥ 10× real time", "0/4"),
    ("11_attach", "Network attach", "4/4, same host", "0/4"),
    ("12_wiring_checks", "Wiring checks", "100 % caught, 0 false positives", "50 %"),
]

# The 46 §15 amendments by theme (each in exactly one; `amendment_themes` checks it). Highlighted: worth a look.
THEMES = [
    ("Ruler amendments: friction becomes frozen tests, never a changed score",
     [7, 14, 15, 27, 29, 38, 40, 43, 45]),
    ("Milestones and judging", [1, 3, 5, 8]),
    ("Readings of gaps the contract left, ratified", [2, 6, 9, 10, 12, 13, 30, 39]),
    ("Attach, signals and lifecycle", [11, 18, 28, 34, 36, 37]),
    ("C++ features the consumer needed", [4, 16, 17, 19, 23, 24, 31, 33, 41, 42, 44]),
    ("Two new public names", [20, 22]),
    ("Python parity with C++", [21, 25, 26, 32]),
    ("The dora boundary in real time", [35, 46]),
]
OVERTURN = {
    14: "the reference guard recovers after a runaway hold instead of latching (your 11-13 rule, applied to the ruler)",
    20: "period(): a new public name in both languages",
    22: "health(level, code): a new public name in both languages",
    5: "compactness targets kept as written; the code is {ratio}× over them and M4 is deferred",
    46: "consumer-022 carried: an attached guard's deadman stays 1.0 s, not the raw node's 0.25 s",
    13: "the dora init watchdog stays 2 s, so exit 4 means 'slow peer or skewed daemon'",
}


# ── small helpers ─────────────────────────────────────────────────────────────────────────────────────────────────

def git(*args: str, cwd: pathlib.Path = REPO) -> str:
    return subprocess.run(["git", "-C", str(cwd), *args], capture_output=True, text=True, check=True).stdout


def consumer_root() -> pathlib.Path:
    for candidate in CONSUMER_CANDIDATES:
        if (candidate / "lk" / "cell.py").is_file():
            return candidate
    raise SystemExit("the consumer's port (moteus_controller_node/lk) is not found at " +
                     " or ".join(map(str, CONSUMER_CANDIDATES)))


def esc(text: object) -> str:
    return html.escape(str(text), quote=True)


def inline_code(text: str) -> str:
    """Markdown `code` spans as <code>, everything else escaped (the friction notes' own spelling, untouched)."""
    parts = re.split(r"(`[^`]+`)", text)
    return "".join(f"<code>{esc(part[1:-1])}</code>" if part.startswith("`") and part.endswith("`") else esc(part)
                   for part in parts)


def sandboxed(argv: list[str]) -> list[str]:
    """The device-free sandbox every python command of this script runs in (the arm on this host is powered)."""
    return ["nice", "-n", "19", "ionice", "-c3", "taskset", "-c", "22-23",
            "bwrap", "--dev-bind", "/", "/", "--ro-bind", "/archive", "/archive", "--dev", "/dev",
            "--bind", "/dev/shm", "/dev/shm", "--ro-bind", "/dev/dri", "/dev/dri", "--bind", "/run/user/1000",
            "/run/user/1000", "--", "env", "BAM_LANE_NO_DEVICES=1", "OMP_NUM_THREADS=1", "OPENBLAS_NUM_THREADS=1",
            "MKL_NUM_THREADS=1", f"PYTHONPATH={REPO / 'src'}", *argv]


# ── extract ───────────────────────────────────────────────────────────────────────────────────────────────────────

def _ab_rows_kpi(path: pathlib.Path) -> dict:
    data = json.loads(path.read_text())
    lost, handoff = data["states_lost_fraction"], data["handoff_p99_s"]
    loads = [run["load"][0] for side in ("like_keras", "raw") for run in data["runs"].get(side, [])]
    return {"lk_lost_pct": [100 * x for x in lost["like_keras"]], "raw_lost_pct": [100 * x for x in lost["raw"]],
            "lk_p99_ms": [1e3 * x for x in handoff["like_keras"]], "raw_p99_ms": [1e3 * x for x in handoff["raw"]],
            "load": [min(loads), max(loads)] if loads else None,
            "within": {"states": lost["within"], "handoff": handoff["within"],
                       "states_upper_pct": 100 * lost["upper_95_of_mean_difference"], "states_band_pct": 100 * lost["band"],
                       "handoff_upper_ms": 1e3 * handoff["upper_95_of_mean_difference"],
                       "handoff_band_ms": 1e3 * handoff["band"]}}


def _ab_rows_consumer(runs_lk: list, runs_raw: list, within: dict | None) -> dict:
    loads = [run["load_before"][0] for run in runs_lk + runs_raw]
    row = {"lk_lost_pct": [100 * run["subscriber"]["lost_fraction"] for run in runs_lk],
           "raw_lost_pct": [100 * run["subscriber"]["lost_fraction"] for run in runs_raw],
           "lk_p99_ms": [run["handoff_ms"]["p99"] for run in runs_lk],
           "raw_p99_ms": [run["handoff_ms"]["p99"] for run in runs_raw],
           "load": [min(loads), max(loads)], "within": None}
    if within:
        lost, handoff = within["states_lost_fraction"], within["handoff_p99_s"]
        row["within"] = {"states": lost["within"], "handoff": handoff["within"],
                         "states_upper_pct": 100 * lost["upper_95_of_mean_difference"],
                         "states_band_pct": 100 * lost["band"],
                         "handoff_upper_ms": 1e3 * handoff["upper_95_of_mean_difference"],
                         "handoff_band_ms": 1e3 * handoff["band"]}
    return row


def extract() -> None:
    out = REPO / "docs" / "scoreboard"
    out.mkdir(parents=True, exist_ok=True)
    latest = json.loads(LATEST_SCOREBOARD.read_text())
    shutil.copyfile(LATEST_SCOREBOARD, out / f"{DATE}.json")
    (out / f"{DATE}.md").write_text(scoreboard_markdown(latest))

    ratchet = []
    for path in SCOREBOARDS.glob("scoreboard*.json"):
        if path.name not in RATCHET_LABELS:
            raise SystemExit(f"{path.name}: no label in RATCHET_LABELS (say what happened before charting it)")
        board = json.loads(path.read_text())
        label, what, kind = RATCHET_LABELS[path.name]
        ratchet.append({"file": path.name, "label": label, "what": what, "kind": kind,
                        "generated": board["generated"], "commit": board["commit"],
                        "weighted_if_gates_green": board["weighted_if_gates_green"], "total": board["total"],
                        "vector": board["vector"], "gates": {k: v["pass"] for k, v in board["gates"].items()}})
    ratchet.sort(key=lambda row: row["generated"])

    first_set = json.loads(AB_SOURCES["consumer_first_set"].read_text())["results"]
    final = json.loads(AB_SOURCES["consumer_final"].read_text())
    a35 = [
        {"label": "kpi A35 test · before A35", "detail": "library cede79b, ruler M0.5 (the test fails first)",
         "phase": "before", **_ab_rows_kpi(AB_SOURCES["kpi_before"])},
        {"label": "consumer A/B · before A35", "detail": "first set: .so on pre-A35 like_keras, 5 × 30 s, 3 sides",
         "phase": "before", **_ab_rows_consumer(first_set["lk_before"], first_set["raw"], None)},
        {"label": "kpi A35 test · after", "detail": "library bda76d9 (A35 in), 5 pairs × 20 s",
         "phase": "after", **_ab_rows_kpi(AB_SOURCES["kpi_head"])},
        {"label": "consumer A/B · final", "detail": "like_keras 41ce82d, consumer a99338289, 5 pairs × 30 s",
         "phase": "after", **_ab_rows_consumer(final["like_keras_target"], final["raw_dora"], final["within_noise"])},
    ]
    surface = subprocess.run(sandboxed([str(VENV_PYTHON), str(REPO / "kpi" / "surface.py"), "--api"]),
                             capture_output=True, text=True, check=True, cwd=REPO)
    evidence = {"generated": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "tier": TIER,
                "note": "extracted by docs/build_report.py extract; every number from a real run on bam-GPU",
                "ratchet": ratchet, "a35_ab": a35, "surface": json.loads(surface.stdout)["api"],
                "sources": {name: str(path) for name, path in AB_SOURCES.items()}}
    EVIDENCE.write_text(json.dumps(evidence, indent=1) + "\n")
    print(f"wrote {out / f'{DATE}.json'}, {out / f'{DATE}.md'}, {EVIDENCE}")


def criterion_measured(name: str, row: dict) -> str:
    """The raw number(s) behind one criterion's score, in words."""
    measured = row.get("measured")
    if name == "1_consumer_fit":
        skipped = [test.removeprefix("test_") for test, outcomes in row["outcomes"].items()
                   if test.startswith("test_T") and all("skipped" in outcome for outcome in outcomes)]
        return f"{measured} (skipped off the timing tier: {', '.join(skipped)})"
    if name == "2_speed":
        return (f"import {measured['import_ms']:.1f} ms; C++ {measured['cxx_per_edge_ns']:.0f} ns and Python "
                f"{measured['python_per_edge_ns']:.0f} ns per edge; the cross-process A/B is timing-tier only")
    if name == "4_succinctness":
        tasks = measured["tasks"]
        return (f"{measured['top_level']} top-level / {measured['total']} names; widest {measured['widest']['callable']} "
                f"({measured['widest']['params']} params); relay {tasks['relay']['ratio']:.2f}× raw dora, "
                f"pipeline {tasks['pipeline']['ratio']:.2f}× plain Python")
    if name == "5_swap_cost":
        return "; ".join(f"{swap}: {cell['deployment_lines']} Deployment line, {cell['component_lines']} component"
                         for swap, cell in measured.items())
    if name == "7_compactness":
        return (f"total {measured['total']:,} SLOC: core {measured['core']:,}, inproc {measured['backend_inproc']:,}, "
                f"dora {measured['backend_dora']:,}, attach+CLI {measured['attach_cli']:,}, C++ "
                f"{measured['cpp_runtime']:,}, views {measured['generated_views']:,}, other {measured['other']:,}")
    if name == "9_facilities":
        return f"{measured}; facility SLOC share {100 * row['facility_sloc_share']:.1f} %"
    if "parts" in row:
        parts = row["parts"]
        good = [key for key, value in parts.items() if value is True or value == 1.0]
        bad = [key for key, value in parts.items() if not (value is True or value == 1.0)]
        return f"{len(good)}/{len(parts)}" + (f" (not passing: {', '.join(bad)})" if bad else "")
    return str(measured)


def scoreboard_markdown(board: dict) -> str:
    gates = board["gates"]
    lines = [f"# like_keras scoreboard, {DATE} (build/v0 @ `{board['commit']}`)", "",
             f"Generated {board['generated']} on {board['host']} by `kpi/score.py` (the frozen ruler, M0.8 `73bb8e2`). "
             f"Tier: **{board['tier']}**. The JSON beside this file is a byte copy of "
             f"`build/kpi/{LATEST_SCOREBOARD.name}`.", "",
             f"**Design score {board['weighted_if_gates_green']:.1f} / 100 if every gate were green. Land total "
             f"{board['total']:.0f}**: one gate (fallback untouched) is red because its suite was not run here.", "",
             "## Gates (any red sets the total to 0)", "", "| Gate | Pass | Evidence |", "|---|---|---|"]
    gate_notes = {
        "equivalence": ", ".join(f"{key} {'✓' if value else '✗'}" for key, value in gates["equivalence"]["checks"].items()),
        "fallback_untouched": (f"files changed {gates['fallback_untouched']['files_changed']}, like_keras required "
                               f"{gates['fallback_untouched']['like_keras_required']}; bam_rack_twin suite "
                               f"**{gates['fallback_untouched']['suite']}** (memory cap locally, no cloud workers today)"),
        "realtime_hot_path": f"build/kpi/alloc_probe (0 heap allocations per tick): {gates['realtime_hot_path']['tail'][-1].strip()}",
        "lean_core": (f"runtime dependencies {gates['lean_core']['dependencies']}, forbidden modules loaded "
                      f"{gates['lean_core']['loaded_forbidden']}, import {gates['lean_core']['import_ms']:.1f} ms"),
    }
    for gate, row in gates.items():
        lines.append(f"| {gate} | {'green' if row['pass'] else '**red**'} | {gate_notes.get(gate, '')} |")
    lines += ["", "## Criteria: raw number vs target vs zero point", "",
              "Each criterion scores `clamp((zero − x) / (zero − target), 0, 1)`; the design score is Σ wᵢ sᵢ.", "",
              "| # | Criterion (weight) | Measured | Target (score 1) | Zero point (score 0) | Score | Weighted |",
              "|---|---|---|---|---|---|---|"]
    for name, title, target, zero in CRITERIA:
        row = board["criteria"][name]
        lines.append(f"| {name.split('_')[0]} | {title} ({row['weight']}) | {criterion_measured(name, row)} | {target} | "
                     f"{zero} | {row['score']:.3f} | {row['weight'] * row['score']:.2f} |")
    lines += ["", f"Vector: `{board['vector']}`. Weighted if gates green: **{board['weighted_if_gates_green']}**. "
              f"Lands: {'yes' if board['land']['ok'] else 'no (' + '; '.join(board['land']['reasons']) + ')'}.", "",
              "## Not judged here, and why", "",
              "- **The timing tier** (criterion 2 Speed; T2 send cost, T3 isolation, T4 boundary stall; A1 and A2; the "
              "A35 within-noise check): the rack is powered all day, so the lanes share cores 22-23 at nice 19 and every "
              "timing number here is labelled and reported, never scored. Criterion 2 is scored 0 by rule off the tier "
              f"(its parts would be per-edge {board['criteria']['2_speed']['parts']['per_edge']:.2f}, import "
              f"{board['criteria']['2_speed']['parts']['import']:.2f}, cross-process and copies/CPU 0: not run).",
              "- **The fallback gate's suite** (`bam_rack_twin` pytest): not run, for the memory cap locally and no cloud "
              "workers today. Its other two checks pass: no fallback file changed, and `bam_rack_twin` never names "
              "like_keras (a scoped grep, 2026-11-16).",
              "- **Network attach `test_clients_cannot_hurt_the_node`** failed in this run (it passed at `41ce82d`): a "
              "load-sensitive test (CONTRACT §15 A15).",
              "- **Real-arm runs**: out of v0's acceptance (CONTRACT §10); the consumer refuses a real bus in `activate()`.",
              ""]
    return "\n".join(lines)


# ── capture ───────────────────────────────────────────────────────────────────────────────────────────────────────

HERO_DRIVER = '''"""The consumer's own demo (lk/twin_demo.py) with its .so built against this like_keras checkout.

The only change from `lk/twin_demo.sh <placement>`: cell.PLACEMENTS' native placements get library=<this build>,
because cell.py binds its LIBRARY default at import time to lk/build (the consumer's own build).
"""
import functools
import pathlib
import sys

LK = pathlib.Path(sys.argv[1])
LIBRARY = pathlib.Path(sys.argv[2])
sys.path[:0] = [str(LK), str(LK / "tests"), str(pathlib.Path.home() / "robot_controllers" / "src")]

import cell  # noqa: E402
import twin_demo  # noqa: E402

for placement in ("same_box", "target"):
    cell.PLACEMENTS[placement] = functools.partial(getattr(cell, placement), library=LIBRARY)
print(f"(.so: {LIBRARY.name}, built against like_keras HEAD)", flush=True)
raise SystemExit(twin_demo.main(sys.argv[3:]))
'''

RECORD_TARGET = r'''#!/usr/bin/env bash
# Inside the consumer's sandbox: `target` under script(1), and lk ls + lk top on its attach socket under a second one.
set -uo pipefail
cap="$1"; lk="$2"; library="$3"; seconds="$4"
py="$HOME/like_keras/.venv/bin/python"
sock="$XDG_RUNTIME_DIR/like_keras/arm_cell.sock"
rm -f "$sock"
date +%s.%N > "$cap/left.t0"
script -q -f -T "$cap/left.tim" -O "$cap/left.out" -c "$py $cap/hero_driver.py $lk $library target $seconds" &
left=$!
for _ in $(seq 1 900); do [[ -S "$sock" ]] && break; sleep 0.1; done
sleep 0.3
date +%s.%N > "$cap/right.t0"
script -q -f -T "$cap/right.tim" -O "$cap/right.out" \
  -c "$py -m like_keras.cli ls unix:$sock; sleep 1.5; $py -m like_keras.cli top unix:$sock" &
right=$!
wait "$left"; status=$?
sleep 1
kill -INT "$right" 2>/dev/null
wait "$right"
echo "target exit $status"
'''


def capture(seconds: float) -> None:
    consumer = consumer_root()
    lk_dir = consumer / "lk"
    sandbox = lk_dir / "sandbox.sh"
    CAPTURE.mkdir(parents=True, exist_ok=True)
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    RUNTIME_DIR.chmod(0o700)
    build = WORK / "so"
    env = dict(os.environ, XDG_RUNTIME_DIR=str(RUNTIME_DIR), LIKE_KERAS_ROOT=str(REPO))
    # WHY -j 1 and only the controller target: two shared cores on a host whose arm is powered
    subprocess.run([str(sandbox), "bash", "-c",
                    f"cmake -S {lk_dir} -B {build} -DCMAKE_BUILD_TYPE=Release -DLIKE_KERAS_ROOT={REPO} >/dev/null && "
                    f"cmake --build {build} -j 1 --target mcn_arm_controller"], env=env, check=True)
    library = build / "libmcn_arm_controller.so"
    (CAPTURE / "hero_driver.py").write_text(HERO_DRIVER)
    (CAPTURE / "record_target.sh").write_text(RECORD_TARGET)
    meta = {"like_keras": git("rev-parse", "--short", "HEAD").strip(),
            "consumer": git("rev-parse", "--short", "HEAD", cwd=consumer).strip(),
            "consumer_branch": git("rev-parse", "--abbrev-ref", "HEAD", cwd=consumer).strip(),
            "seconds": seconds, "runs": {}}
    try:
        for placement in ("target", "same_box", "fallback"):
            load = pathlib.Path("/proc/loadavg").read_text().split()[:3]
            started = time.strftime("%Y-%m-%d %H:%M:%S %Z")
            if placement == "target":
                result = subprocess.run([str(sandbox), "bash", str(CAPTURE / "record_target.sh"), str(CAPTURE),
                                         str(lk_dir), str(library), f"{seconds:g}"], env=env, capture_output=True,
                                        text=True, timeout=600)
            else:
                result = subprocess.run([str(sandbox), "python", str(CAPTURE / "hero_driver.py"), str(lk_dir),
                                         str(library), placement, f"{seconds:g}"], env=env, capture_output=True,
                                        text=True, timeout=600)
                (CAPTURE / f"{placement}.txt").write_text(result.stdout + result.stderr)
            meta["runs"][placement] = {"started": started, "load_1_5_15": load, "exit": result.returncode}
            print(f"{placement}: exit {result.returncode} (load {' '.join(load)})")
    finally:
        shutil.rmtree(RUNTIME_DIR, ignore_errors=True)
        shutil.rmtree(build, ignore_errors=True)  # WHY: the disk is 98 % full; the .so is rebuilt in ~1 min
    (CAPTURE / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")


# ── render: the hero video ────────────────────────────────────────────────────────────────────────────────────────

def read_script_log(side: str) -> tuple[float, list[tuple[float, str]]]:
    """script(1)'s output log and classic timing file → (start epoch, [(epoch, text chunk)])."""
    raw = (CAPTURE / f"{side}.out").read_bytes()
    body = raw.split(b"\n", 1)[1]  # the "Script started on ..." header is not in the timing file
    trailer = body.rfind(b"\nScript done on")
    body = body[:trailer] if trailer >= 0 else body
    start = float((CAPTURE / f"{side}.t0").read_text())
    chunks, offset, clock = [], 0, start
    for line in (CAPTURE / f"{side}.tim").read_text().split("\n"):
        if not line.strip():
            continue
        delay, size = line.split()
        clock += float(delay)
        chunks.append((clock, body[offset:offset + int(size)].decode("utf-8", "replace")))
        offset += int(size)
    return start, chunks


class Screen:
    """Just enough of a terminal for these two programs: text, CR, LF, cursor home and erase-display."""

    def __init__(self) -> None:
        self.rows: list[list[str]] = [[]]
        self.row = self.col = 0
        self.clears = 0

    def feed(self, text: str) -> None:
        index = 0
        while index < len(text):
            match = re.match(r"\x1b\[([0-9;?]*)([A-Za-z])", text[index:])
            if match:
                if match.group(2) == "H":
                    self.row = self.col = 0
                elif match.group(2) == "J" and match.group(1) in ("2", "3"):
                    self.rows, self.clears = [[]], self.clears + 1
                index += match.end()
                continue
            char = text[index]
            index += 1
            if char == "\n":
                self.row += 1
                self.col = 0
                while len(self.rows) <= self.row:
                    self.rows.append([])
            elif char == "\r":
                self.col = 0
            elif char == "\x1b":
                continue
            else:
                line = self.rows[self.row]
                while len(line) < self.col:
                    line.append(" ")
                if self.col < len(line):
                    line[self.col] = char
                else:
                    line.append(char)
                self.col += 1

    def lines(self) -> list[str]:
        lines = ["".join(row) for row in self.rows]
        while lines and not lines[-1].strip():
            lines.pop()
        return lines


TERM = {"bg": (13, 17, 23), "bar": (22, 27, 34), "rule": (48, 54, 61), "text": (201, 209, 217), "dim": (125, 133, 144),
        "bright": (240, 246, 252), "ok": (63, 185, 80), "warn": (210, 153, 34), "error": (248, 81, 73),
        "changed": (121, 192, 255), "changed_bg": (31, 52, 84)}


def render() -> None:
    from PIL import Image, ImageDraw, ImageFont

    meta = json.loads((CAPTURE / "meta.json").read_text())
    font_dir = pathlib.Path("/usr/share/fonts/truetype/dejavu")
    font = ImageFont.truetype(str(font_dir / "DejaVuSansMono.ttf"), 13)
    bold = ImageFont.truetype(str(font_dir / "DejaVuSansMono-Bold.ttf"), 13)
    title_font = ImageFont.truetype(str(font_dir / "DejaVuSansMono-Bold.ttf"), 15)
    width, height, fps = 1280, 600, 10
    char_w = font.getlength("M")
    line_h = 17
    left_x, split_x, top_y, bottom_y = 12, 452, 74, height - 30
    left_cols = int((split_x - left_x - 10) // char_w)
    right_cols = int((width - split_x - 22) // char_w)

    start_left, left_chunks = read_script_log("left")
    start_right, right_chunks = read_script_log("right")
    epoch0 = start_left
    end = max(left_chunks[-1][0], right_chunks[-1][0]) + 1.5
    frames = int(math.ceil((end - epoch0) * fps))
    started_at = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(epoch0))

    def wrap(line: str, cols: int) -> list[str]:
        """Word-wrapped at the pane's width; continuation rows indented two past the line's own indent."""
        if len(line) <= cols:
            return [line]
        indent = " " * (len(line) - len(line.lstrip()) + 2)
        return textwrap.wrap(line, cols, subsequent_indent=indent, break_on_hyphens=False) or [line]

    def values_of(lines: list[str]) -> dict:
        found, path = {}, ""
        for line in lines:
            if line and not line.startswith(" "):
                path = line.split()[0]
            for key, value in re.findall(r"(\S+?)=(\S+)", line):
                found[(path, key)] = value
        return found

    process = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                "-s", f"{width}x{height}", "-r", str(fps), "-i", "-", "-c:v", "libx264",
                                "-pix_fmt", "yuv420p", "-crf", "30", "-preset", "slow", "-tune", "stillimage",
                                "-movflags", "+faststart", str(WORK / "hero.mp4")], stdin=subprocess.PIPE)
    left, right = Screen(), Screen()
    left_index = right_index = 0
    previous_values, changed, shown_clears = {}, set(), 0
    for frame in range(frames):
        now = epoch0 + frame / fps
        while left_index < len(left_chunks) and left_chunks[left_index][0] <= now:
            left.feed(left_chunks[left_index][1])
            left_index += 1
        while right_index < len(right_chunks) and right_chunks[right_index][0] <= now:
            right.feed(right_chunks[right_index][1])
            right_index += 1
        if right.clears != shown_clears:  # a new lk top screen: highlight what changed since the last one
            current = values_of(right.lines())
            changed = {key for key, value in current.items() if key in previous_values and previous_values[key] != value}
            previous_values, shown_clears = current, right.clears

        image = Image.new("RGB", (width, height), TERM["bg"])
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, width, 32), fill=TERM["bar"])
        draw.text((12, 8), f"like_keras v0 · the consumer's arm cell in `target` · real run {started_at} · "
                  f"like_keras {meta['like_keras']}", font=title_font, fill=TERM["bright"])
        clock = f"t = {now - epoch0:5.1f} s  (real time, 1×)"
        draw.text((width - 12 - title_font.getlength(clock), 8), clock, font=title_font, fill=TERM["changed"])
        draw.line((split_x, 36, split_x, bottom_y), fill=TERM["rule"], width=1)
        draw.text((left_x, 44), "$ lk/twin_demo.py target 14", font=bold, fill=TERM["ok"])
        draw.text((left_x, 44 + line_h - 2), "  planner: Python node · chain: .so under lk-dora-host", font=font,
                  fill=TERM["dim"])
        draw.text((split_x + 12, 44), "$ lk ls <socket>; lk top <socket>", font=bold, fill=TERM["ok"])
        draw.text((split_x + 12, 44 + line_h - 2), f"  {RUNTIME_DIR.name}/like_keras/arm_cell.sock · refreshed "
                  "each second · changed values highlighted", font=font, fill=TERM["dim"])

        y = top_y + 8
        for line in left.lines():
            for piece in wrap(line, left_cols):
                if y > bottom_y - line_h:
                    break
                colour = TERM["bright"] if re.match(r"^\w+: plans adopted", piece) else TERM["text"]
                draw.text((left_x, y), piece, font=font, fill=colour)
                y += line_h

        y, path = top_y + 8, ""
        for line in right.lines():
            if line and not line.startswith(" "):
                path = line.split()[0]
            for piece in wrap(line, right_cols):
                if y > bottom_y - line_h:
                    break
                x = split_x + 12
                for token in re.split(r"(\s+)", piece):
                    if not token:
                        continue
                    colour = TERM["text"]
                    match = re.match(r"(\S+?)=(\S+)$", token)
                    if token in ("ok",):
                        colour = TERM["ok"]
                    elif token in ("warn", "STALE"):
                        colour = TERM["warn"]
                    elif token == "error":
                        colour = TERM["error"]
                    elif token.startswith("arm_cell") and x == split_x + 12:
                        colour = TERM["bright"]
                    if match and (path, match.group(1)) in changed:
                        key_width = font.getlength(match.group(1) + "=")
                        draw.text((x, y), match.group(1) + "=", font=font, fill=TERM["dim"])
                        value_width = font.getlength(match.group(2))
                        draw.rectangle((x + key_width - 1, y, x + key_width + value_width + 1, y + line_h - 2),
                                       fill=TERM["changed_bg"])
                        draw.text((x + key_width, y), match.group(2), font=font, fill=TERM["changed"])
                    elif match:
                        draw.text((x, y), match.group(1) + "=", font=font, fill=TERM["dim"])
                        draw.text((x + font.getlength(match.group(1) + "="), y), match.group(2), font=font, fill=colour)
                    else:
                        draw.text((x, y), token, font=font, fill=colour)
                    x += font.getlength(token)
                y += line_h

        draw.rectangle((0, height - 26, width, height), fill=TERM["bar"])
        draw.text((12, height - 21), f"device-free sandbox · private dora 0.5.0 · twin bus · nice 19 · {TIER} · "
                  "recorded with script(1)", font=font, fill=TERM["dim"])
        process.stdin.write(image.tobytes())
        if frame == frames - 1:
            image.save(WORK / "hero-last.png")
        if abs(now - epoch0 - 9.0) < 0.05:
            image.save(WORK / "hero-mid.png")
    process.stdin.close()
    if process.wait() != 0:
        raise SystemExit("ffmpeg failed on the hero video")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(WORK / "hero.mp4"), "-vf",
                    "fps=4,scale=640:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=24:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=none:diff_mode=rectangle", str(WORK / "hero.gif")], check=True)
    (WORK / "hero.json").write_text(json.dumps({"frames": frames, "fps": fps, "seconds": round(frames / fps, 1),
                                                "started": started_at}) + "\n")
    print(f"hero: {frames / fps:.1f} s, mp4 {(WORK / 'hero.mp4').stat().st_size // 1024} KiB, gif "
          f"{(WORK / 'hero.gif').stat().st_size // 1024} KiB")


# ── page: the parsed sources ──────────────────────────────────────────────────────────────────────────────────────

def amendments() -> dict[int, dict]:
    text = (REPO / "docs" / "CONTRACT.md").read_text()
    found = {}
    for line in text.split("\n"):
        match = re.match(r"^\| A(\d+) \| (.*?) \| (.*?) \| (.*?) \| (.*?) \|$", line)
        if match:
            number = int(match.group(1))
            found[number] = {"amends": match.group(2), "decision": match.group(3), "why": match.group(4),
                             "undo": match.group(5)}
    themed = sorted(number for _, numbers in THEMES for number in numbers)
    if themed != sorted(found):
        raise SystemExit(f"THEMES must hold each §15 amendment once: themed {themed}, contract {sorted(found)}")
    return found


def decisions() -> list[dict]:
    text = (REPO / "docs" / "CONTRACT.md").read_text()
    rows = []
    for line in text.split("\n"):
        match = re.match(r"^\| (D\d+) \| (.*?) \| (.*?) \| (.*?) \|$", line)
        if match:
            rows.append({"id": match.group(1), "decision": match.group(2), "why": match.group(3),
                         "undo": match.group(4)})
    return rows


def library_log() -> list[tuple[str, str, str]]:
    rows = []
    for line in git("log", "--format=%h\t%ad\t%s", "--date=format:%m-%d %H:%M", "main..HEAD").splitlines():
        sha, when, subject = line.split("\t", 2)
        rows.append((sha, when, subject))
    return rows


def fix_commits(log: list[tuple[str, str, str]]) -> dict[int, list[tuple[str, str]]]:
    """Library commits naming a decision in their subject, `area (A17): ...`; contract, ruler, friction and
    test-only commits excluded."""
    found = collections.defaultdict(list)
    for sha, _, subject in reversed(log):
        if re.match(r"^(contract|kpi:|friction|docs:|tests )", subject):
            continue
        match = re.match(r"^[^()]{1,40}\(([^)]*)\)", subject)
        if not match:
            continue
        for number in re.findall(r"A(\d+)", match.group(1)):
            found[int(number)].append((sha, subject))
    return found


def ruler_tests(log: list[tuple[str, str, str]]) -> dict[int, list[tuple[str, str]]]:
    """Each ruler amendment commit's 'New tests' lines: decision → [(milestone, suite::test)]."""
    found = collections.defaultdict(list)
    for sha, _, subject in log:
        milestone = re.match(r"^kpi: (M0\.\d+)", subject)
        if not milestone:
            continue
        for line in git("show", "-s", "--format=%B", sha).splitlines():
            if not line.startswith("- A"):
                continue
            test = re.search(r"(\w+::\w+)", line)
            if test:
                for number in re.findall(r"A(\d+)", line[:test.start()]):
                    found[int(number)].append((milestone.group(1), test.group(1)))
    # WHY by hand: M0.6's one test answers A34 through A40 (its body names the test after "CONTRACT §15 A40:")
    found[34].append(("M0.6", "conformance::test_activate_raising_sends_nothing"))
    return found


# WHY: A28 decided two notes at once; 013 (exports) was fixed only by the commits that say so, the rest are 012's drain
FIX_FILTER = {"consumer-013": r"export|C ABI"}


def friction_notes(log) -> list[dict]:
    fixes, tests = fix_commits(log), ruler_tests(log)
    notes = []
    for path in sorted((REPO / "docs" / "friction").glob("consumer-*.md")):
        text = path.read_text()
        title = text.split("\n", 1)[0].removeprefix("# ")
        title = re.sub(r"^consumer-\d+: ", "", title)
        title = re.sub(r" \((controllers lane|the controllers lane)[^)]*\)\s*$", "", title)
        sections = {}
        for block in re.split(r"\n## ", text)[1:]:
            heading, _, body = block.partition("\n")
            sections[heading.split(" (")[0]] = body.strip()
        verified = sections.get("Verified by the consumer", "")
        resolution = sections.get("Resolution", "")
        sentences = re.split(r"(?<=\.)\s+", verified.split("\n\n")[0].replace("\n", " ")) if verified else [""]
        first = sentences[0]
        if (len(first) < 70 and len(sentences) > 1 and len(first) + len(sentences[1]) < 330
                and not sentences[1].endswith(":")):
            first = f"{first} {sentences[1]}"  # WHY: "Resolved by A16." alone says nothing about what was checked
        numbers = [int(n) for n in re.findall(r"A(\d+)", sentences[0])] or [int(n) for n in re.findall(r"A(\d+)", resolution)]
        # WHY >= 16: A1-A3 are also the acceptance tests' names (§10), and the consumer's decisions start at A16
        numbers = [n for n in dict.fromkeys(numbers)
                   if n >= 16 and not re.search(rf"ruler M0\.\d+ \(A{n}\)", resolution)]
        if not verified:
            status = "carried"
        elif "half met" in verified:
            status = "half met"
        elif first.startswith("Measured"):
            status = "measured"
        else:
            status = "verified"
        note_tests = [test for n in numbers for test in tests.get(n, [])]
        if path.stem == "consumer-013":  # WHY: A28's frozen test (M0.4) covers the drain half (012), not exports
            note_tests = []
        notes.append({"id": path.stem.removeprefix("consumer-"), "title": title, "decisions": numbers,
                      "tests": sorted(dict.fromkeys(note_tests), key=lambda test: float(test[0][1:])),
                      "fixes": list(dict.fromkeys(sha for n in numbers for sha, subject in fixes.get(n, [])
                                                  if re.search(FIX_FILTER.get(path.stem, ""), subject))),
                      "status": status, "verified": first or resolution.split("\n")[0]})
    return notes


def function_lines(path: pathlib.Path, name: str) -> list[tuple[int, str]]:
    lines = path.read_text().split("\n")
    start = next(i for i, line in enumerate(lines) if line.startswith(f"def {name}("))
    body = [(start + 1, lines[start])]
    for index in range(start + 1, len(lines)):
        if lines[index] and not lines[index].startswith((" ", ")")):
            break
        body.append((index + 1, lines[index]))
    while body and not body[-1][1].strip():
        body.pop()
    return body


def find_lines(path: pathlib.Path, wanted: list[str]) -> list[tuple[int, str]]:
    """The given lines exactly as the file has them, with their line numbers (a snippet that cannot drift)."""
    lines = path.read_text().split("\n")
    found = []
    for text in wanted:
        index = next((i for i, line in enumerate(lines) if line.strip().startswith(text)), None)
        if index is None:
            raise SystemExit(f"{path}: no line starting {text!r}")
        found.append((index + 1, lines[index]))
    return found


def placement_summary(placement: str) -> list[str]:
    if placement == "target":
        _, chunks = read_script_log("left")
        text = "".join(chunk for _, chunk in chunks).replace("\r", "")
    else:
        text = (CAPTURE / f"{placement}.txt").read_text()
    lines = text.split("\n")
    start = next(i for i, line in enumerate(lines) if line.startswith(f"{placement}: plans adopted"))
    end = next(i for i in range(start, len(lines)) if lines[i].startswith("  log:"))
    return [line for line in lines[start:end + 1] if line.strip()]  # WHY to "log:": stderr follows in the file


def summary_facts(lines: list[str]) -> dict:
    """The numbers twin_demo printed (the arm's own recording, read back), so the prose never states a stale one."""
    text = "\n".join(lines)

    def find(pattern: str, default: str = "?") -> str:
        match = re.search(pattern, text)
        return match.group(1) if match else default

    return {"adopted": find(r"plans adopted (\d+)"), "refused": find(r"refused (\d+)"),
            "commands": find(r"; (\d+) commands"), "ended": find(r"at its last metrics round: (\S+)"),
            "errors": find(r"(\d+) at error"), "rt": find(r"RT thread (SCHED_\w+ \d+)", "none (all Python)"),
            "wake_ms": find(r"worst wake period ([\d.]+) ms"), "overruns": find(r"overruns (\d+)"),
            "io": find(r"I/O threads (SCHED_\w+ \d+)", "-"), "never_sent": find(r"states never sent (\d+)", "-")}


# ── page: charts (inline SVG, theme-aware through CSS custom properties) ──────────────────────────────────────────

def svg_vector(board: dict) -> str:
    rows = len(CRITERIA)
    row_h, left, bar_w, top = 30, 196, 420, 26
    height = top + rows * row_h + 10
    parts = [f'<svg viewBox="0 0 860 {height}" class="chart" style="max-width:860px" role="img" '
             f'aria-label="Latest KPI vector against targets">',
             f'<text x="{left}" y="14" class="axis-label">0</text>'
             f'<text x="{left + bar_w}" y="14" class="axis-label" text-anchor="end">1 = target</text>']
    for index, (name, title, target, zero) in enumerate(CRITERIA):
        row = board["criteria"][name]
        y = top + index * row_h
        score, weight = row["score"], row["weight"]
        unscored = name == "2_speed" and not row.get("scored", True)
        parts.append(f'<text x="{left - 10}" y="{y + 15}" class="label" text-anchor="end">{index + 1} {esc(title)} '
                     f'<tspan class="muted">w{weight}</tspan></text>')
        parts.append(f'<rect x="{left}" y="{y + 4}" width="{bar_w}" height="14" rx="4" class="track"/>')
        if unscored:
            parts.append(f'<rect x="{left}" y="{y + 4}" width="{bar_w}" height="14" rx="4" fill="url(#hatch)"/>')
        elif score > 0:
            parts.append(f'<rect x="{left}" y="{y + 4}" width="{max(8, bar_w * score):.1f}" height="14" rx="4" '
                         f'class="bar-s1"><title>{esc(title)}: {score:.3f} × {weight} = {score * weight:.2f}\n'
                         f'measured: {esc(criterion_measured(name, row))}\ntarget: {esc(target)}\nzero: {esc(zero)}'
                         f'</title></rect>')
        note = ("timing tier: reported, scored 0" if unscored else
                f"{score:.2f} · {score * weight:.1f} of {weight}")
        parts.append(f'<text x="{left + bar_w + 10}" y="{y + 15}" class="value">{esc(note)}</text>')
    parts.append('<defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" '
                 'patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="6" class="hatch"/></pattern></defs>')
    parts.append("</svg>")
    return "".join(parts)


def svg_ratchet(ratchet: list[dict]) -> str:
    width, height, left, right, top, bottom = 980, 320, 44, 24, 16, 96
    plot_w, plot_h = width - left - right, height - top - bottom
    step = (plot_w - 28) / (len(ratchet) - 1)

    def point(index: int, value: float) -> tuple[float, float]:
        return left + 14 + index * step, top + plot_h * (1 - value / 100)

    parts = [f'<svg viewBox="0 0 {width} {height}" class="chart" style="max-width:{width}px" role="img" '
             f'aria-label="Design score over the build">']
    for value in (0, 25, 50, 75, 100):
        _, y = point(0, value)
        parts.append(f'<line x1="{left}" x2="{width - right}" y1="{y:.1f}" y2="{y:.1f}" class="grid"/>'
                     f'<text x="{left - 8}" y="{y + 4:.1f}" class="axis-label" text-anchor="end">{value}</text>')
    path = " ".join(f"{'M' if i == 0 else 'L'}{point(i, row['weighted_if_gates_green'])[0]:.1f},"
                    f"{point(i, row['weighted_if_gates_green'])[1]:.1f}" for i, row in enumerate(ratchet))
    parts.append(f'<path d="{path}" class="line-s1"/>')
    for index, row in enumerate(ratchet):
        x, y = point(index, row["weighted_if_gates_green"])
        red = [gate for gate, ok in row["gates"].items() if not ok]
        tip = (f"{row['label']} — {row['what']}\n{row['commit']} · {row['generated'][:16].replace('T', ' ')} UTC\n"
               f"design score {row['weighted_if_gates_green']:.1f} if gates green; land total {row['total']:.0f}"
               f"\nred gates: {', '.join(red) or 'none'}")
        shape = (f'<rect x="{x - 5:.1f}" y="{y - 5:.1f}" width="10" height="10" class="mark-s2" transform="rotate(45 '
                 f'{x:.1f} {y:.1f})"/>' if row["kind"] == "ruler" else
                 f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" class="mark-s1"/>')
        parts.append(f'<g class="hit"><title>{esc(tip)}</title><rect x="{x - step / 2:.1f}" y="{top}" '
                     f'width="{step:.1f}" height="{plot_h}" fill="transparent"/>{shape}</g>')
        parts.append(f'<text x="{x:.1f}" y="{top + plot_h + 14}" class="tick" text-anchor="end" '
                     f'transform="rotate(-40 {x:.1f} {top + plot_h + 14})">{esc(row["label"])}</text>')
    last = ratchet[-1]
    x, y = point(len(ratchet) - 1, last["weighted_if_gates_green"])
    parts.append(f'<text x="{x - 8:.1f}" y="{y - 10:.1f}" class="value" text-anchor="end">'
                 f'{last["weighted_if_gates_green"]:.1f}</text>')
    parts.append("</svg>")
    return "".join(parts)


def svg_heatmap(ratchet: list[dict]) -> str:
    cell_w, cell_h, left, top = 34, 20, 168, 8
    width = left + cell_w * len(ratchet) + 8
    height = top + cell_h * len(CRITERIA) + 70
    parts = [f'<svg viewBox="0 0 {width} {height}" class="chart" style="max-width:{width}px" role="img" '
             f'aria-label="The 12-criterion vector at every scoreboard">']
    for row_index, (name, title, _, _) in enumerate(CRITERIA):
        y = top + row_index * cell_h
        parts.append(f'<text x="{left - 8}" y="{y + 14}" class="label" text-anchor="end">{row_index + 1} '
                     f'{esc(title)}</text>')
        for col, board in enumerate(ratchet):
            value = board["vector"][row_index]
            x = left + col * cell_w
            share = round(8 + 88 * value)
            parts.append(f'<rect x="{x + 1}" y="{y + 1}" width="{cell_w - 2}" height="{cell_h - 2}" rx="2" '
                         f'style="fill:color-mix(in oklab, var(--s1) {share}%, var(--surface))"><title>{esc(title)} '
                         f'at {esc(board["label"])} ({board["commit"]}): {value:.2f}</title></rect>')
    for col, board in enumerate(ratchet):
        x = left + col * cell_w + cell_w / 2
        y = top + cell_h * len(CRITERIA) + 8
        parts.append(f'<text x="{x:.1f}" y="{y}" class="tick" text-anchor="end" transform="rotate(-50 {x:.1f} {y})">'
                     f'{esc(board["label"])}</text>')
    parts.append("</svg>")
    return "".join(parts)


def svg_ab(rows: list[dict]) -> str:
    panels = [("States lost at the subscriber (%)", "lost_pct", 7.0, [0, 2, 4, 6], "states"),
              ("Hand-off p99 (ms)", "p99_ms", 180.0, [0, 50, 100, 150], "handoff")]
    width, left, right, row_h, panel_gap = 1000, 230, 300, 34, 50
    plot_w = width - left - right
    height = 20 + len(panels) * (len(rows) * row_h + panel_gap) + 10
    parts = [f'<svg viewBox="0 0 {width} {height}" class="chart" style="max-width:{width}px" role="img" '
             f'aria-label="A35 before and after">']
    y0 = 20
    for title, key, maximum, ticks, verdict_key in panels:
        parts.append(f'<text x="{left}" y="{y0 - 6}" class="label strong">{esc(title)}</text>')
        bottom = y0 + len(rows) * row_h
        for tick in ticks:
            x = left + plot_w * tick / maximum
            parts.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{y0}" y2="{bottom}" class="grid"/>'
                         f'<text x="{x:.1f}" y="{bottom + 14}" class="axis-label" text-anchor="middle">{tick}</text>')
        for index, row in enumerate(rows):
            y = y0 + index * row_h + row_h / 2
            if index and row["phase"] != rows[index - 1]["phase"]:
                parts.append(f'<line x1="8" x2="{width - 8}" y1="{y - row_h / 2:.1f}" y2="{y - row_h / 2:.1f}" '
                             f'class="axis"/>')
            parts.append(f'<text x="{left - 10}" y="{y - 2:.1f}" class="label" text-anchor="end">{esc(row["label"])}'
                         f'</text><text x="{left - 10}" y="{y + 11:.1f}" class="tick" text-anchor="end">load '
                         f'{row["load"][0]:.0f}-{row["load"][1]:.0f}</text>')
            for side, cls, offset in (("raw", "mark-s2", 5), ("lk", "mark-s1", -5)):
                for pair, value in enumerate(row[f"{side}_{key}"]):
                    x = left + plot_w * min(value, maximum) / maximum
                    name = "raw dora" if side == "raw" else "like_keras"
                    tip = f"{row['label']} ({row['detail']}), pair {pair}: {name} {value:.2f}"
                    if side == "raw":
                        parts.append(f'<rect x="{x - 4:.1f}" y="{y + offset - 4:.1f}" width="8" height="8" '
                                     f'class="{cls}" transform="rotate(45 {x:.1f} {y + offset:.1f})"><title>{esc(tip)}'
                                     f'</title></rect>')
                    else:
                        parts.append(f'<circle cx="{x:.1f}" cy="{y + offset:.1f}" r="4.5" class="{cls}"><title>'
                                     f'{esc(tip)}</title></circle>')
            within = row.get("within")
            if within is None:
                verdict = "no noise test (3 sides)"
            else:
                ok = within[verdict_key]
                upper = within["states_upper_pct" if verdict_key == "states" else "handoff_upper_ms"]
                band = within["states_band_pct" if verdict_key == "states" else "handoff_band_ms"]
                unit = " %" if verdict_key == "states" else " ms"
                verdict = (f"{'✓ within' if ok else '✗ not within'} noise: CI edge {upper:.2f} "
                           f"{'≤' if ok else '>'} band {band:.2f}{unit}")
            parts.append(f'<text x="{left + plot_w + 10}" y="{y + 4:.1f}" class="value">{esc(verdict)}</text>')
        y0 = bottom + panel_gap
    parts.append("</svg>")
    return "".join(parts)


def svg_architecture() -> str:
    """The seam, drawn from CONTRACT §4-§9 and §15 A19/A35/A36: who calls whom in `target`."""
    return """<svg viewBox="0 0 1080 640" class="chart arch" style="max-width:1080px" role="img" aria-label="like_keras architecture">
<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
<path d="M0,0 L10,5 L0,10 z" class="arrowhead"/></marker></defs>
<text x="20" y="22" class="label strong">What an author writes (the stable API)</text>
<rect x="20" y="34" width="320" height="132" rx="8" class="box"/>
<text x="34" y="56" class="label strong">Component</text>
<text x="34" y="78" class="code">target = lk.In(JointState, queue=8)</text>
<text x="34" y="96" class="code">command = lk.In(ServoCommand, stale=0.1)</text>
<text x="34" y="114" class="code">trajectory = lk.Out(JointTrajectory)</text>
<text x="34" y="132" class="code">def forward(self, target, command)</text>
<text x="34" y="154" class="muted">names no peer, transport, thread or host</text>
<rect x="380" y="34" width="320" height="132" rx="8" class="box"/>
<text x="394" y="56" class="label strong">System — wired from outside</text>
<text x="394" y="78" class="code">s.add(ArmPlanner(), target=…, command=…)</text>
<text x="394" y="96" class="code">s.wire(arm, trajectory=planner.trajectory)</text>
<text x="394" y="114" class="code">s.expose(arm.state, arm.command)</text>
<text x="394" y="136" class="muted">subscribe direction (RobMoSys ports);</text>
<text x="394" y="154" class="muted">a System is itself a component</text>
<rect x="740" y="34" width="320" height="132" rx="8" class="box"/>
<text x="754" y="56" class="label strong">Deployment — placement only</text>
<text x="754" y="78" class="code">lk.Deployment(cell, backend=dora)</text>
<text x="754" y="96" class="code">dep.place(cell.arm, process=…,</text>
<text x="754" y="114" class="code">          hz=100, cpus=[23], fifo=80)</text>
<text x="754" y="136" class="muted">describe() = the datasheet; every wiring</text>
<text x="754" y="154" class="muted">check runs before any process starts</text>
<line x1="340" y1="100" x2="378" y2="100" class="edge" marker-end="url(#arrow)"/>
<line x1="700" y1="100" x2="738" y2="100" class="edge" marker-end="url(#arrow)"/>
<rect x="560" y="196" width="240" height="62" rx="8" class="box alt"/>
<text x="574" y="218" class="label strong">backends.inproc</text>
<text x="574" y="236" class="muted">one process, a thread per group;</text>
<text x="574" y="252" class="muted">lock-free slots; natives via PyDLL</text>
<rect x="820" y="196" width="240" height="62" rx="8" class="box alt"/>
<text x="834" y="218" class="label strong">backends.dora (dora-rs 0.5.0)</text>
<text x="834" y="236" class="muted">emit.py → dataflow (nodes only);</text>
<text x="834" y="252" class="muted">the only module that names dora</text>
<line x1="900" y1="166" x2="690" y2="194" class="edge" marker-end="url(#arrow)"/>
<line x1="900" y1="166" x2="940" y2="194" class="edge" marker-end="url(#arrow)"/>
<text x="20" y="214" class="muted">one conformance suite runs every rule of §5 on both backends;</text>
<text x="20" y="232" class="muted">moving a component is one edit of process= or backend=</text>
<text x="20" y="300" class="label strong">What `target` runs (the hero video)</text>
<rect x="20" y="312" width="300" height="200" rx="8" class="box"/>
<text x="34" y="334" class="label strong">process "main" · Python dora node</text>
<text x="34" y="356" class="code">targets  (hz=2)</text>
<text x="34" y="376" class="code">planner  (rate-less, one step per target)</text>
<text x="34" y="400" class="muted">python -m like_keras.backends.dora.host</text>
<text x="34" y="420" class="muted">RX thread: node.next(timeout=0.1)</text>
<text x="34" y="440" class="muted">TX thread: no group thread ever calls</text>
<text x="34" y="458" class="muted">into a transport (§5.4)</text>
<rect x="390" y="350" width="220" height="110" rx="8" class="box dora"/>
<text x="404" y="372" class="label strong">private dora daemon</text>
<text x="404" y="392" class="muted">Arrow C Data frames, the</text>
<text x="404" y="410" class="muted">layout like_keras owns;</text>
<text x="404" y="428" class="muted">schema hash once (lk_hello)</text>
<text x="404" y="446" class="muted">raised to RR 5 only while needed</text>
<rect x="680" y="300" width="380" height="226" rx="8" class="box"/>
<text x="694" y="322" class="label strong">process "arm-controller" · lk-dora-host</text>
<text x="694" y="344" class="muted">main: 2 s init watchdog, dlopen, lk_create, hello</text>
<text x="694" y="364" class="muted">RX: event_as_arrow_input → lk_push (rtkit RR 10, A35)</text>
<text x="694" y="384" class="muted">TX: lk_wait → lk_take → send_arrow_output (RR 10)</text>
<rect x="694" y="398" width="352" height="116" rx="6" class="box inner"/>
<text x="708" y="418" class="label strong">libmcn_arm_controller.so (lk.hpp, C ABI only)</text>
<text x="708" y="438" class="code">RT: SCHED_FIFO → rtkit RR 20 → OTHER (A19)</text>
<text x="708" y="456" class="code">chain: trajectory, verdict → state, command</text>
<text x="708" y="474" class="muted">0 allocations per tick; plans converted on the</text>
<text x="708" y="492" class="muted">pushing thread (A17), retired off the RT thread</text>
<line x1="320" y1="380" x2="388" y2="390" class="edge" marker-end="url(#arrow)"/>
<line x1="612" y1="392" x2="678" y2="366" class="edge" marker-end="url(#arrow)"/>
<line x1="678" y1="386" x2="612" y2="420" class="edge" marker-end="url(#arrow)"/>
<line x1="388" y1="430" x2="322" y2="440" class="edge" marker-end="url(#arrow)"/>
<text x="330" y="372" class="tick">trajectory</text>
<text x="330" y="458" class="tick">command</text>
<rect x="390" y="548" width="220" height="70" rx="8" class="box alt"/>
<text x="404" y="570" class="label strong">gateway node</text>
<text x="404" y="588" class="muted">every exposed port; serves</text>
<text x="404" y="606" class="code">unix:…/arm_cell.sock</text>
<line x1="500" y1="462" x2="500" y2="546" class="edge" marker-end="url(#arrow)"/>
<rect x="680" y="548" width="380" height="70" rx="8" class="box alt"/>
<text x="694" y="570" class="label strong">attach clients (any process, same host)</text>
<text x="694" y="588" class="code">lk ls · lk top · lk.attach(address)</text>
<text x="694" y="606" class="muted">one writer per exposed input: a lease (A36)</text>
<line x1="612" y1="584" x2="678" y2="584" class="edge" marker-end="url(#arrow)"/>
</svg>"""


# ── page: assembly ────────────────────────────────────────────────────────────────────────────────────────────────

CSS = """
:root{color-scheme:light;--bg:#f9f9f7;--surface:#fcfcfb;--ink:#0b0b0b;--ink-2:#52514e;--muted:#898781;--grid:#e1e0d9;
--axis:#c3c2b7;--border:rgba(11,11,11,.10);--s1:#2a78d6;--s2:#eb6834;--good:#0ca30c;--good-ink:#006300;--warn:#fab219;
--crit:#d03b3b;--code-bg:#f0efec;--hi:#fff3c4;--tag:#e8f1fc}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--bg:#0d0d0d;--surface:#1a1a19;
--ink:#f0efec;--ink-2:#c3c2b7;--muted:#898781;--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);--s1:#3987e5;
--s2:#d95926;--good-ink:#0ca30c;--code-bg:#22221f;--hi:#3d3415;--tag:#1c2a3d}}
:root[data-theme="dark"]{color-scheme:dark;--bg:#0d0d0d;--surface:#1a1a19;--ink:#f0efec;--ink-2:#c3c2b7;--muted:#898781;
--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);--s1:#3987e5;--s2:#d95926;--good-ink:#0ca30c;
--code-bg:#22221f;--hi:#3d3415;--tag:#1c2a3d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,-apple-system,"Segoe UI",sans-serif}
main{max-width:1120px;margin:0 auto;padding:24px 16px 64px}
h1{font-size:28px;margin:0 0 4px}h2{font-size:21px;margin:44px 0 10px;padding-top:12px;border-top:1px solid var(--grid)}
h3{font-size:16px;margin:22px 0 8px}p,li{max-width:860px}a{color:var(--s1)}
.sub{color:var(--ink-2);margin:0 0 18px}.muted{color:var(--muted)}
.verdict{font-size:18px;line-height:1.5;background:var(--surface);border:1px solid var(--border);border-left:4px solid
var(--s1);border-radius:8px;padding:14px 18px;margin:18px 0}
.verdict ul{margin:8px 0 0;padding-left:20px;font-size:15px}
code{font:13px/1.45 ui-monospace,"DejaVu Sans Mono",monospace;background:var(--code-bg);padding:1px 4px;border-radius:4px}
pre{font:12.5px/1.5 ui-monospace,"DejaVu Sans Mono",monospace;background:var(--code-bg);padding:10px 12px;border-radius:8px;
overflow-x:auto;margin:6px 0;white-space:pre-wrap;overflow-wrap:anywhere}pre code{background:none;padding:0}
td code{overflow-wrap:anywhere}table.loop{table-layout:fixed;min-width:900px}.wide{min-width:640px}
.wider{min-width:860px}table.parts{min-width:760px}td:first-child code{white-space:nowrap;overflow-wrap:normal}
.hi{background:var(--hi);display:inline-block;min-width:100%}.ln{color:var(--muted);user-select:none}
.card{background:var(--surface);border:1px solid var(--border);border-radius:10px;padding:14px 16px}
.grid3{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.grid2{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:14px}
.scroll{overflow-x:auto;max-width:100%}
table{border-collapse:collapse;width:100%;font-size:13.5px;background:var(--surface)}
th,td{border-bottom:1px solid var(--grid);padding:6px 8px;text-align:left;vertical-align:top}
th{color:var(--ink-2);font-weight:600;position:sticky;top:0;background:var(--surface)}
.pill{display:inline-block;border-radius:999px;padding:2px 10px;font-size:13px;margin:2px 4px 2px 0;
border:1px solid var(--border);background:var(--surface)}
.pill.ok::before{content:"✓ ";color:var(--good-ink);font-weight:700}.pill.bad::before{content:"✗ ";color:var(--crit);
font-weight:700}.pill.half::before{content:"◐ ";color:var(--warn)}.pill.carry::before{content:"→ ";color:var(--muted)}
.tag{display:inline-block;background:var(--tag);border-radius:4px;padding:0 5px;margin:1px 2px;font-size:12.5px}
.status{white-space:nowrap;font-weight:600}.status.verified{color:var(--good-ink)}.status.measured,.status.half{color:#b07800}
.status.carried{color:var(--muted)}
figure{margin:12px 0}figcaption{color:var(--ink-2);font-size:13.5px;margin-top:6px}
video,img.hero{width:100%;height:auto;border-radius:8px;border:1px solid var(--border);background:#0d1117}
svg.chart{width:100%;height:auto;display:block}
svg .label{font:12.5px system-ui,sans-serif;fill:var(--ink)}svg .strong{font-weight:650}
svg .muted{font:12px system-ui,sans-serif;fill:var(--ink-2)}svg .code{font:11.5px ui-monospace,"DejaVu Sans Mono",monospace;
fill:var(--ink)}svg .tick,svg .axis-label{font:11px system-ui,sans-serif;fill:var(--muted)}
svg .value{font:12px system-ui,sans-serif;fill:var(--ink-2)}
svg .grid{stroke:var(--grid);stroke-width:1}svg .axis{stroke:var(--axis);stroke-width:1}
svg .track{fill:var(--grid)}svg .bar-s1{fill:var(--s1)}svg .hatch{stroke:var(--muted);stroke-width:2}
svg .line-s1{fill:none;stroke:var(--s1);stroke-width:2}
svg .mark-s1{fill:var(--s1);stroke:var(--surface);stroke-width:2}svg .mark-s2{fill:var(--s2);stroke:var(--surface);
stroke-width:2}
svg .box{fill:var(--surface);stroke:var(--axis);stroke-width:1.2}svg .box.alt{fill:var(--code-bg)}
svg .box.inner{fill:var(--tag)}svg .box.dora{fill:var(--code-bg);stroke-dasharray:4 3}
svg .edge{stroke:var(--ink-2);stroke-width:1.4;fill:none}svg .arrowhead{fill:var(--ink-2)}
.arch-wrap{min-width:720px}.hi{padding-left:2px}
.legend{font-size:13px;color:var(--ink-2)}.legend .dot{display:inline-block;width:10px;height:10px;border-radius:50%;
background:var(--s1);margin:0 4px 0 10px}.legend .dia{display:inline-block;width:9px;height:9px;background:var(--s2);
transform:rotate(45deg);margin:0 5px 0 12px}
details{margin:8px 0}summary{cursor:pointer;font-weight:600}
.k{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.04em;margin:0}
.big{font-size:30px;font-weight:650;margin:0}
.flag{border-left:3px solid var(--warn);padding-left:8px}
"""


def code_block(lines: list[tuple[int, str]], highlight: set[int] = frozenset(), label: str = "") -> str:
    rows = []
    for number, text in lines:
        content = f'<span class="ln">{number:>3} </span>{esc(text)}'
        rows.append(f'<span class="hi">{content}</span>' if number in highlight else content)
    head = f'<p class="k">{esc(label)}</p>' if label else ""
    return f'{head}<pre><code>{chr(10).join(rows)}</code></pre>'


def placements_section(consumer: pathlib.Path) -> str:
    reference = REPO / "kpi" / "tasks" / "arm_cell" / "cell.py"
    mine = consumer / "lk" / "cell.py"

    def differing(path: pathlib.Path, name: str, base: str) -> tuple[list[tuple[int, str]], set[int]]:
        body, base_body = function_lines(path, name), function_lines(path, base)

        def essence(text: str) -> str:
            return re.sub(rf"\b{name}\b|\b{base}\b", "_", text.split("#")[0]).strip()

        base_set = {essence(text) for _, text in base_body}
        changed = {number for number, text in body[1:] if essence(text) and essence(text) not in base_set}
        return body, changed

    columns = []
    titles = {"target": ("target", "the .so in its own process under lk-dora-host; the planner in a Python node; "
                         "dora 0.5.0 between"),
              "same_box": ("same_box", "the same .so on its own RT thread inside the Python process (inproc)"),
              "fallback": ("fallback", "all Python: the same exposed ports on the Python chain")}
    for placement in ("target", "same_box", "fallback"):
        ref_body, ref_changed = differing(reference, placement, "same_box")
        own_body, own_changed = differing(mine, placement, "same_box")
        summary = placement_summary(placement)
        heading, what = titles[placement]
        if placement == "same_box":
            differs = '<p class="muted">The base the other two are compared against: no line differs.</p>'
        else:
            differs = (code_block([(n, text.strip()) for n, text in ref_body if n in ref_changed], ref_changed,
                                  "differs from same_box — frozen reference, kpi/tasks/arm_cell/cell.py")
                       + code_block([(n, text.strip()) for n, text in own_body if n in own_changed], own_changed,
                                    "differs from same_box — the consumer, lk/cell.py"))
        columns.append(
            f'<div class="card"><h3><code>{heading}</code></h3><p class="muted">{esc(what)}</p>{differs}'
            f'<details><summary>the whole function, reference and consumer</summary>'
            f'{code_block(ref_body, ref_changed, "kpi/tasks/arm_cell/cell.py")}'
            f'{code_block(own_body, own_changed, "lk/cell.py")}</details>'
            f'<p class="k">what it did on the twin today ({esc(placement)}, 14 s)</p>'
            f'<pre><code>{esc(chr(10).join(summary))}</code></pre></div>')
    return "".join(columns)


def page() -> None:
    consumer = consumer_root()
    board = json.loads((REPO / "docs" / "scoreboard" / f"{DATE}.json").read_text())
    evidence = json.loads(EVIDENCE.read_text())
    meta = json.loads((CAPTURE / "meta.json").read_text())
    hero = json.loads((WORK / "hero.json").read_text())
    log = library_log()
    head = git("rev-parse", "--short", "HEAD").strip()
    contract_amendments = amendments()
    notes = friction_notes(log)
    ratchet = evidence["ratchet"]
    surface = evidence["surface"]
    criteria = board["criteria"]
    compact = criteria["7_compactness"]["measured"]
    compact_ratio = compact["total"] / 2450
    statuses = collections.Counter(note["status"] for note in notes)
    mp4 = base64.b64encode((WORK / "hero.mp4").read_bytes()).decode()
    gif = base64.b64encode((WORK / "hero.gif").read_bytes()).decode()
    target_summary = placement_summary("target")
    same_box_summary, fallback_summary = placement_summary("same_box"), placement_summary("fallback")
    run_load = meta["runs"]["target"]["load_1_5_15"][0]

    facts = {name: summary_facts(lines) for name, lines in
             (("target", target_summary), ("same_box", same_box_summary), ("fallback", fallback_summary))}
    target_facts = facts["target"]

    gates = board["gates"]
    gate_pills = "".join(
        f'<span class="pill {"ok" if row["pass"] else "bad"}">{esc(name.replace("_", " "))}'
        f'{"" if row["pass"] else " — suite not run"}</span>' for name, row in gates.items())

    friction_rows = []
    for note in notes:
        decisions_html = " ".join(f'<span class="tag">A{n}</span>' for n in note["decisions"])
        tests_html = "<br>".join(f"{esc(m)} <code>{esc(t)}</code>" for m, t in note["tests"]) or (
            '<span class="muted">consumer\'s own test only</span>' if note["status"] != "carried" else
            '<span class="muted">none yet (next track)</span>')
        fixes_html = " ".join(f"<code>{esc(sha)}</code>" for sha in note["fixes"]) or '<span class="muted">—</span>'
        friction_rows.append(
            f'<tr><td><code>{note["id"]}</code></td><td>{inline_code(note["title"])}</td><td>{decisions_html}</td>'
            f'<td>{tests_html}</td><td>{fixes_html}</td><td><span class="status {note["status"].split()[0]}">'
            f'{esc(note["status"])}</span> {inline_code(note["verified"])}</td></tr>')

    theme_blocks = []
    for theme, numbers in THEMES:
        rows = "".join(
            f'<tr{" class=flagrow" if n in OVERTURN else ""}><td><span class="tag">A{n}</span></td>'
            f'<td>{inline_code(contract_amendments[n]["decision"])}'
            f'{f"<p class=flag><b>Worth a look:</b> {esc(OVERTURN[n].format(ratio=f"{compact_ratio:.1f}"))}</p>" if n in OVERTURN else ""}</td>'
            f'<td>{inline_code(contract_amendments[n]["undo"])}</td></tr>' for n in numbers)
        flagged = [n for n in numbers if n in OVERTURN]
        theme_blocks.append(
            f'<details><summary>{esc(theme)} — {len(numbers)}: '
            f'{", ".join(f"A{n}" for n in numbers)}</summary><div class="scroll"><table><thead><tr><th>#</th>'
            f'<th>Decision</th><th>Undo</th></tr></thead><tbody>{rows}</tbody></table></div></details>')

    decision_rows = "".join(f'<tr><td><span class="tag">{d["id"]}</span></td><td>{inline_code(d["decision"])}</td>'
                            f'<td>{inline_code(d["undo"])}</td></tr>' for d in decisions())

    planner = consumer / "lk" / "planner.py"
    cell = consumer / "lk" / "cell.py"
    usage = (code_block(find_lines(planner, ["class ArmPlanner(lk.Component):", "target = lk.In(", "command = lk.In(",
                                              "trajectory = lk.Out("]), label="lk/planner.py — a component: ports only")
             + code_block(find_lines(cell, ["s = lk.System(name)", "source = s.add(TargetFile(", 's.add(arm, "arm")',
                                            "planner = s.add(ArmPlanner()", "s.wire(arm, trajectory=",
                                            "s.expose(arm.state, arm.command)"]),
                          label="lk/cell.py — the System, wired from outside")
             + code_block([(number, text) for number, text in function_lines(cell, "target")
                           if text.strip().startswith(("cell = arm_cell(", "dep = lk.Deployment(", "dep.place("))],
                          label="lk/cell.py — the deployment (target)"))
    cpp_usage = code_block(find_lines(consumer / "lk" / "arm_controller.cpp", [
        "struct MoteusChain : lk::Component {", "lk::In<rm::JointTrajectory, PlanIn> trajectory",
        "lk::In<rm::DiagnosticRecord> verdict", "lk::Out<rm::ServoState> state", "lk::Out<rm::ServoCommand> command",
        "lk::System moteus_arm(", 'auto& chain = s.add<MoteusChain>("chain"', "s.expose(chain.trajectory",
        "LK_EXPORT(moteus_arm)"]), label="lk/arm_controller.cpp — the C++ side of the same ports")

    names = surface
    top_level = ["Component", "In", "Out", "System", "Deployment", "native", "attach", "replay"]
    others = [name for name in names["python_names"] if name not in top_level]
    surface_html = (
        f'<p><b>{names["top_level"]} top-level names, {names["total"]} in all</b> (target ≤ 8 / ≤ 30; the ratchet may '
        f'only lower it). Widest signature: <code>{esc(names["widest"]["callable"])}</code> with '
        f'{names["widest"]["params"]} parameters; {names["kwargs_bags"]} option bags, {names["string_enums"]} string '
        f'enums. Counted by the frozen ruler (<code>kpi/surface.py --api</code>).</p>'
        f'<p class="k">top level — <code>like_keras.__all__</code></p><p>'
        + "".join(f'<span class="pill">{esc(n)}</span>' for n in top_level)
        + f'</p><p class="k">members (Component, System, Deployment, ports)</p><p>'
        + "".join(f'<span class="tag">{esc(n)}</span>' for n in others)
        + f'</p><p class="k">backends</p><p>' + "".join(f'<span class="tag">{esc(n)}</span>' for n in names["backends"])
        + f'</p><p class="k">C++ only (no same-named Python symbol)</p><p>'
        + "".join(f'<span class="tag">{esc(n)}</span>' for n in names["cpp_only"])
        + '</p><p class="muted">The C++ classes share the Python names: <code>lk::Component</code> (<code>activate</code>, '
          '<code>forward</code>, <code>deactivate</code>, <code>now</code>, <code>period</code>, <code>metric</code>, '
          '<code>log</code>, <code>health</code>), <code>lk::In&lt;T&gt;</code> (<code>get</code>, <code>take</code>), '
          '<code>lk::Out&lt;T&gt;</code> (<code>write</code>), <code>lk::System</code> (<code>add</code>, '
          '<code>wire</code>, <code>expose</code>), and the C ABI of <code>abi.h</code> for hosts, which is not public.</p>')

    rows_vector = []
    for index, (name, title, target, zero) in enumerate(CRITERIA):
        row = criteria[name]
        rows_vector.append(f"<tr><td>{index + 1}</td><td>{esc(title)} ({row['weight']})</td>"
                           f"<td>{esc(criterion_measured(name, row))}</td><td>{esc(target)}</td><td>{esc(zero)}</td>"
                           f"<td>{row['score']:.2f}</td></tr>")

    kpi_after, consumer_final = evidence["a35_ab"][2], evidence["a35_ab"][3]
    kpi_before = evidence["a35_ab"][0]

    body = f"""
<main>
<h1>like_keras v0 — the communication API</h1>
<p class="sub">Build report, {DATE}. Library <code>build/v0</code> @ <code>{head}</code> ({len(log)} commits over
<code>main</code>); ruler M0.8 <code>73bb8e2</code>; consumer <code>{esc(meta["consumer_branch"])}</code> @
<code>{esc(meta["consumer"])}</code>. Every number below comes from a run on this host; timing numbers are labelled
<i>{TIER}</i> and none is judged.</p>

<div class="verdict"><b>like_keras v0 is one thing now: black-box components with typed ports, Systems wired from
outside, and Deployments that place the same files in one process, across processes over dora 0.5.0, or all in
Python — and the consumer's real C++ arm controller runs on it in all three placements on its twin.</b>
It is on <code>build/v0</code> @ <code>{head}</code>, not merged (the dispatcher lands).
<ul>
<li><b>Design score {board["weighted_if_gates_green"]:.1f} / 100 if the gates were green; land total
{board["total"]:.0f}</b>, because the fallback gate's suite was not run (memory cap locally, no cloud workers today).
<code>bam_rack_twin</code> is untouched and never names like_keras.</li>
<li><b>Timing tier not judged here</b>: the rack is powered, so speed, isolation and the boundary stall are reported,
never scored.</li>
<li><b>Compactness far over target</b>: {compact["total"]:,} SLOC against 2,450 (M4, compaction, not done).</li>
<li><b>Never driven the real arm</b>: the consumer refuses a real bus in <code>activate()</code>; the raw-dora node
still holds the arm.</li>
</ul></div>

<h2>Hero: the consumer's arm cell in <code>target</code>, with <code>lk top</code> beside it</h2>
<figure>
<video controls autoplay muted loop playsinline width="1280" height="600"
 aria-label="Terminal recording: the consumer's cell in the target placement and lk top">
<source src="data:video/mp4;base64,{mp4}" type="video/mp4"
 onerror="this.parentNode.outerHTML=document.getElementById('hero-gif').innerHTML">
<img class="hero" alt="GIF of the same recording" src="data:image/gif;base64,{gif}">
</video>
<template id="hero-gif"><img class="hero" alt="GIF of the same recording" src="data:image/gif;base64,{gif}"></template>
<figcaption>A real run, {esc(hero["started"])}, {hero["seconds"]:g} s at real time (1×), load {esc(run_load)}.
<b>Left:</b> the consumer's own demo (<code>lk/twin_demo.py target 14</code>): a private dora 0.5.0 coordinator and
daemon, the planner and the target file in a Python node, the C++ chain (<code>libmcn_arm_controller.so</code>, built
against <code>{esc(meta["like_keras"])}</code>) under <code>lk-dora-host</code>, on the twin bus; its summary at the
end is read back from the arm's own recording. <b>Right:</b> <code>lk ls</code>, then <code>lk top</code> on the
deployment's attach socket, refreshed each second; values that changed since the last refresh are highlighted (the
only thing added to the observed text). Captured with <code>script(1) --log-timing</code> in the device-free sandbox
(bwrap: no devices, its own network; nice 19, cores 22-23). At this load: {target_facts["overruns"]} overrun(s) in
{target_facts["commands"]} ticks, the RT thread's worst wake {target_facts["wake_ms"]} ms — {TIER}.</figcaption>
</figure>
<pre><code>{esc(chr(10).join(target_summary))}</code></pre>

<h2>The architecture: what is stable, what churns</h2>
<p>Authors write the top row and nothing else. A Deployment picks threads, processes and a backend; the backends
underneath can churn (dora 1.x, zenoh) without a component or System line changing — criterion 6 measures it: dora is
named only in its backend and <code>cpp/hosts</code>. The bottom half is the <code>target</code> placement the video
shows.</p>
<div class="scroll"><div class="arch-wrap">{svg_architecture()}</div></div>

<h2>Three placements of the same files, one line apart</h2>
<p>Each column shows the lines of its deployment function that differ from <code>same_box</code> (a line diff of the
files as written, the placement's own name ignored); the whole function is one click away. <code>target</code> also
imports the backend inside the function (A1: importing <code>cell.py</code> must not import dora). The ruler's swap-cost count: <b>1 Deployment line, 0 component
lines</b> for each of process, language, backend and host. Below each: what that placement did on the twin today, read
back from the arm's own recording.</p>
<div class="grid3">{placements_section(consumer)}</div>

<h2>The KPI scoreboard</h2>
<p>{gate_pills}</p>
<div class="grid2">
<div class="card"><p class="k">design score if the gates were green</p><p class="big">{board["weighted_if_gates_green"]:.1f}
<span class="muted" style="font-size:16px">/ 100</span></p><p class="muted">land total {board["total"]:.0f} — one gate
red because its suite was not run · {TIER}</p></div>
<div class="card"><p class="k">consumer friction, 22 notes</p><p class="big">{statuses["verified"]} verified</p>
<p class="muted">{statuses["half met"]} half met (A35's hand-off p99), {statuses["measured"]} measured for the timing
tier (A26), {statuses["carried"]} carried to the next track (A46)</p></div>
</div>
<h3>Latest vector against targets ({esc(board["commit"])}, {esc(board["generated"][:16].replace("T", " "))} UTC)</h3>
<p class="muted">Each bar is one criterion's score (1 = its v0 target, 0 = its zero point); hover a bar for the raw
number. {TIER}.</p>
<div class="scroll"><div class="wide">{svg_vector(board)}</div></div>
<details><summary>The raw numbers: measured vs target vs zero point</summary><div class="scroll"><table><thead><tr>
<th>#</th><th>Criterion (w)</th><th>Measured</th><th>Target (1)</th><th>Zero (0)</th><th>Score</th></tr></thead>
<tbody>{"".join(rows_vector)}</tbody></table></div>
<p class="muted">Also committed as <code>docs/scoreboard/{DATE}.md</code> and <code>.json</code>.</p></details>
<h3>The ratchet: design score at every scoreboard of the build</h3>
<p class="legend"><span class="dot"></span>library landed <span class="dia"></span>ruler amendment (new frozen tests
that fail first, so the score dips, then the library climbs back) · {TIER}</p>
<div class="scroll"><div class="wide">{svg_ratchet(ratchet)}</div></div>
<p class="muted">The land total stayed 0 at every point: before M3 the equivalence or hot-path gate was red, and since
then the fallback gate, whose suite has not been run on this host.</p>
<h3>The 12-criterion vector at each scoreboard</h3>
<p class="muted">Each cell is that criterion's score at that scoreboard: pale = 0, solid blue = 1 (hover for the value).</p>
<div class="scroll"><div class="wide">{svg_heatmap(ratchet)}</div></div>

<h2>The consumer loop: friction → decision → frozen test → fix → verified</h2>
<p>The controllers lane ported its C++ arm controller onto like_keras while the library was being built, and filed
what got in its way. Each note became a contract decision (§15), then a frozen ruler test written to fail first, then
a library fix, then the consumer's own verification on its port. "The consumer can help set the spec."</p>
<div class="scroll"><table class="loop"><colgroup><col style="width:5%"><col style="width:22%"><col style="width:7%">
<col style="width:24%"><col style="width:8%"><col style="width:34%"></colgroup><thead><tr><th>Note</th><th>Friction</th>
<th>Decision</th><th>Frozen ruler test</th><th>Fix</th><th>Verified by the consumer</th></tr></thead><tbody>{"".join(friction_rows)}</tbody></table></div>

<h3>A35 before and after: the dora boundary in real time</h3>
<p>Before A35 only the RT thread was raised, and <code>target</code> lost {min(kpi_before["lk_lost_pct"]):.1f}-{max(kpi_before["lk_lost_pct"]):.1f} %
of states with a hand-off tail up to {max(kpi_before["lk_p99_ms"]):.0f} ms. A35 gave <code>lk-dora-host</code>'s I/O
threads and its private daemon the raw node's rtkit treatment. After: states lost within noise of raw dora in both
A/Bs; hand-off p99 within noise in the kpi test ({kpi_after["within"]["handoff_upper_ms"]:.1f} ≤
{kpi_after["within"]["handoff_band_ms"]:.1f} ms) and narrowly not in the consumer's final set
({consumer_final["within"]["handoff_upper_ms"]:.1f} &gt; {consumer_final["within"]["handoff_band_ms"]:.1f} ms).</p>
<p class="legend"><span class="dot"></span>like_keras <code>target</code> <span class="dia"></span>raw dora 0.5.0 node
· one mark per interleaved pair · {TIER}</p>
<div class="scroll"><div class="wider">{svg_ab(evidence["a35_ab"])}</div></div>

<h2>The public surface</h2>
{surface_html}
<h3>Usage, from the reference consumer: 15 lines of Python, and the C++ side of the same ports</h3>
<p class="muted">Unchanged lines with their line numbers in the consumer's files (<code>{esc(meta["consumer"])}</code>).</p>
<div class="grid2"><div>{usage}</div><div>{cpp_usage}</div></div>

<h2>Stock parts: what like_keras builds on instead of building</h2>
<div class="scroll"><table class="parts"><thead><tr><th>Element</th><th>Today</th><th>Off-the-shelf part</th><th>Must survive</th></tr>
</thead><tbody>
<tr><td>Transport between processes and hosts</td><td><code>backends/dora</code> emits a nodes-only dataflow;
<code>lk-dora-host</code> is the only C++ that links dora (5 allowed cxx calls, listed in its source)</td>
<td><b>dora-rs 0.5.0</b>: the cxx node API for C++, dora's own Python package for Python hosts; its coordinator,
daemon and <code>restart_policy</code></td><td>a dora 1.x upgrade edits only <code>backends/dora</code> and
<code>cpp/hosts</code>; skew refused within 2 s; supervision stays dora's</td></tr>
<tr><td>Wire format</td><td><code>codec.py</code> derives each type's layout from its <code>robot_msgs</code> dataclass;
LKF frames for sockets and recordings</td><td><b>Apache Arrow C Data Interface</b>: two plain C structs declared in
<code>abi.h</code>, no Arrow library linked</td><td>golden bytes bit-exact both ways (T10); schema hash once per
connection; zero extra copies at ≥ 4 KiB</td></tr>
<tr><td>Message types</td><td>like_keras defines none; C++ views generated and committed</td><td><b>robot_msgs</b>
(frozen dataclasses; <code>LogRecord</code>, <code>DiagnosticRecord</code>, <code>Metrics</code> and public
<code>adopt</code> added by its own lane)</td><td>every edge that crosses a process or language carries a
<code>robot_msgs</code> type; the generated-header hash check</td></tr>
<tr><td>Real-time scheduling without root</td><td>the library's ladder (A19, A35): SCHED_FIFO, else rtkit SCHED_RR,
else SCHED_OTHER; RLIMIT_RTTIME with a SIGXCPU demote sentinel lifted from the raw node</td><td><b>rtkit</b>
(<code>org.freedesktop.RealtimeKit1</code>)</td><td>realtime only while needed; no grant outlives stop; the policy
reported on every box (<code>sched_policy</code>, <code>io_sched_policy</code>)</td></tr>
</tbody></table></div>

<h2>The decision surface</h2>
<h3>Done and proved</h3>
<ul>
<li><b>All three placements ran on HEAD today</b> (the consumer's cell, its .so rebuilt against <code>{head}</code>):
{"; ".join(f"<code>{name}</code> adopted {f['adopted']} plans, refused {f['refused']}, {f['errors']} error lines, "
           f"ended {f['ended']}" for name, f in facts.items())}. In <code>target</code> the RT thread ran
{target_facts["rt"]} and the I/O threads {target_facts["io"]} (A19, A35), with {target_facts["never_sent"]} states never
sent.</li>
<li><b>3 of 4 gates green</b> at <code>{esc(board["commit"])}</code>: equivalence (6/6 checks), real-time hot path (0
allocations per tick), lean core (one runtime dependency, <code>robot_msgs</code>).</li>
<li><b>Criteria at target:</b> black box 20/20, swap cost 1 line, dora insulation 5/5, reach 5/5, facilities 7/7,
testability 4/4, wiring checks 28/28 with 0 false positives.</li>
<li><b>{statuses["verified"]} of 22 consumer notes fixed and verified</b> by the consumer on its port, each through a
frozen test written to fail first.</li>
<li><b>A35</b>: states lost now within noise of raw dora (both A/Bs); the hand-off tail fell from up to
{max(kpi_before["lk_p99_ms"]):.0f} ms to within a few ms of raw.</li>
</ul>
<h3>Decided for you (each reversible)</h3>
<ul>
<li><b>The pick: P1, the thin facade</b>, with grafts G1-G14 from the runners-up (judges' mean 76.4 vs P2 75.5, P3
74.9, P4 73.9, P5 67.8; <code>docs/design/2026-11-15/judging.md</code>). Undo: take P3 (the spec compiler) as the base;
§4-§7's port semantics carry over.</li>
<li><b>The contract's own decisions D1-D8</b>:
<details><summary>D1-D8 with their undo</summary><div class="scroll"><table><thead><tr><th>#</th><th>Decision</th>
<th>Undo</th></tr></thead><tbody>{decision_rows}</tbody></table></div></details></li>
<li><b>46 amendments (§15), by theme.</b> The flagged ones are the calls you may want to overturn:
<ul>{"".join(f"<li><span class=tag>A{n}</span> {esc(text.format(ratio=f'{compact_ratio:.1f}'))}</li>" for n, text in OVERTURN.items())}</ul>
{"".join(theme_blocks)}</li>
</ul>
<h3>Left</h3>
<ul>
<li><b>M4, compaction</b> — next: {compact["total"]:,} → ≤ 2,450 SLOC (core {compact["core"]:,}, C++ runtime
{compact["cpp_runtime"]:,}, dora {compact["backend_dora"]:,}, inproc {compact["backend_inproc"]:,}, attach + CLI
{compact["attach_cli"]:,}), behaviour unchanged, with consumer-022 (A46).</li>
<li><b>The timing tier</b> — blocked on a quiet host: criterion 2, T2-T4, A1, A2 and A35's within-noise check, on an
unpowered rack or a cloud VM that grants rtprio.</li>
<li><b>The fallback gate's suite on cloud</b> — next: <code>bam_rack_twin</code>'s pytest with like_keras absent.</li>
<li><b>Real-arm runs through rackq</b> — after the timing tier; today the port refuses a real bus by design.</li>
<li><b>consumer-022</b>: a Python reader of <code>state</code> stalls 60-130 ms every 2 s under <code>target</code>
(the private daemon's main thread); nothing is lost.</li>
<li><code>test_clients_cannot_hurt_the_node</code> failed in the last scoreboard run (passed at <code>41ce82d</code>;
load-sensitive, A15).</li>
</ul>
<h3>Needs you</h3>
<ul>
<li><b>sudo, once:</b> a <code>/etc/security/limits.d</code> rtprio entry for your user, or two cores set aside for
the arm. Recommendation: the limits.d entry (one file, undone by deleting it) — SCHED_FIFO then works and rtkit's
200 ms RLIMIT_RTTIME cap stops being the ceiling.</li>
<li><b>A window with the rack unpowered</b> for the timing tier, or a go for a cloud VM with rtprio instead.
Recommendation: the cloud VM first; the unpowered window only for the real-arm-host numbers.</li>
</ul>
<h3>Deliberately not done</h3>
<ul>
<li>No merge, push or tag: the dispatcher lands <code>build/v0</code>.</li>
<li>No timing claim from this host, and no real-arm run.</li>
<li><code>kpi/</code> and <code>docs/CONTRACT.md</code> untouched (the ruler and the contract are read-only here).</li>
<li>No audit: worth a <code>/llm-judge</code> only if you want a second read of A35's rtkit rules before the real arm.</li>
</ul>

<h2>Sources</h2>
<p class="muted">This page is built by <code>docs/build_report.py</code> from: <code>docs/CONTRACT.md</code> (§3, §10, §14,
§15), <code>docs/design/2026-11-15/judging.md</code>, <code>docs/friction/consumer-*.md</code>, <code>git log
main..build/v0</code>, <code>docs/scoreboard/{DATE}.json</code> and <code>{DATE}-evidence.json</code> (the 16
scoreboards of the build and the A35 A/B runs), the capture in <code>build/report/cap/</code>, and the consumer's
<code>lk/cell.py</code>, <code>lk/planner.py</code>, <code>lk/arm_controller.cpp</code>
(<code>{esc(meta["consumer"])}</code>) and the frozen <code>kpi/tasks/arm_cell/cell.py</code>.</p>
</main>"""
    document = (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" '
                f'content="width=device-width,initial-scale=1"><title>like_keras v0</title>'
                f'<meta name="description" content="like_keras v0 build report, {DATE}"><style>{CSS}</style></head>'
                f'<body>{body}</body></html>\n')
    PAGE.parent.mkdir(parents=True, exist_ok=True)
    PAGE.write_text(document)
    print(f"wrote {PAGE} ({PAGE.stat().st_size / 1e6:.2f} MB)")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("stage", nargs="?", default="all", choices=("extract", "capture", "render", "page", "all"))
    parser.add_argument("--seconds", type=float, default=14.0, help="capture: how long each placement runs")
    arguments = parser.parse_args(argv)
    if arguments.stage == "extract":
        extract()
    elif arguments.stage == "capture":
        capture(arguments.seconds)
    elif arguments.stage == "render":
        render()
    elif arguments.stage == "page":
        page()
    else:
        render()
        page()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
