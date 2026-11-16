"""The scoreboard (CONTRACT §11): every measurement, the four gates, the 12-number vector and the total, as JSON.

    nice -n 19 ionice -c3 env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test python kpi/score.py

Writes kpi/scoreboard.json (gitignored) and compares it with the committed kpi/baseline.json. Each criterion scores
s = clamp((zero - x) / (zero - target), 0, 1) (mirrored when higher is better); the total is sum(w_i s_i) when every
gate is green, else 0; the vector is always published. Every pytest suite runs through kpi/run.py (a private network
and PID namespace). The denominators are frozen lists of test ids below: a test that skips, errors or is missing
counts as failed, so the empty repo scores 0 and a skip can never raise a score.

Timing numbers (criterion 2, and the timing tests of criterion 1) count only in the timing tier (KPI_TIER=timing,
rack unpowered or the fixed cloud VM); elsewhere they are reported, labelled, and scored 0. Criteria 9 and 10 read a
timing-marked twin as well in the timing tier: health ok when unloaded, and >= 10x real time on the wall clock
(the default tier reads warn/Overrun as honest and >= 10x on CPU time; ruler-002 §1, ruler-003). So do criteria 1, 6
and 11 since M0.2 (CONTRACT §15 A14): T5 within 1 s, the two conformance tests' wall-clock bounds, and the attach
node's absolute tick rate with bad clients; criterion 9's health also needs its lockstep half, in every tier.

Since M0.3 (CONTRACT §15 A27) each of the first consumer's eleven decisions (A16-A26) has one frozen test, counted as
a companion of an id or part its criterion already scores (FRICTION_* below): that id or part counts only when its
companions pass as well. Weights, gates, formulas and denominators are unchanged.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import pathlib
import platform
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

REPO = pathlib.Path(__file__).resolve().parents[1]
KPI = REPO / "kpi"
BUILD = REPO / "build" / "kpi"
JUNIT = BUILD / "junit"
sys.path[:0] = [str(REPO)]

SUITES = ["contract", "consumer", "conformance", "churn", "golden", "polyglot", "facilities", "twin", "attach",
          "blackbox", "miswire", "validgraphs"]
WEIGHTS = {"1_consumer_fit": 15, "2_speed": 13, "3_black_box": 11, "4_succinctness": 11, "5_swap_cost": 9,
           "6_dora_insulation": 8, "7_compactness": 8, "8_reach": 7, "9_facilities": 6, "10_testability": 5,
           "11_attach": 4, "12_wiring_checks": 3}

EQUIVALENCE = ["test_one_process_matches_schedule", "test_across_processes", "test_cpp_replacement",
               "test_sent_equals_received", "test_reference_solutions", "test_swap_variant"]
CONSUMER = ["test_T1_swap", "test_T2_send_cost", "test_T3_isolation", "test_T4_boundary_stall", "test_T5_planner_crash",
            "test_T6_node_crash", "test_T7_exclusivity", "test_T8_claim", "test_T12_introspection"]
ACCEPTANCE = ["test_A1_handoff_over_dora", "test_A2_same_planner_in_one_process", "test_A3_fallback_matches_reference"]
FACILITIES = ["log", "metrics", "health", "params", "lifecycle", "recording", "introspection"]
COMBINATIONS = ["python-inproc", "python-dora", "cpp-inproc", "cpp-dora"]
INPROC_COMBINATIONS = ["python-inproc", "cpp-inproc"]
ATTACH = ["test_list_ports_within_1s", "test_attach_receive_and_write", "test_clients_cannot_hurt_the_node",
          "test_attach_needs_an_explicit_address"]
SWAPS = ["process", "language", "backend", "host"]
# Timing-tier twins (CONTRACT §15 A14): a test id that counts in the timing tier only when its twin passes as well.
TIMING_TWINS = {"test_T5_planner_crash": "test_T5_planner_crash_within_1s",
                "test_now_is_scheduled_time": "test_now_is_scheduled_time_wall_clock",
                "test_forward_exception_goes_error": "test_forward_exception_goes_error_wall_clock",
                "test_clients_cannot_hurt_the_node": "test_clients_cannot_hurt_the_node_unloaded"}
# Ruler M0.3 (CONTRACT §15 A27): one frozen test per consumer decision A16-A26 (docs/friction/consumer-001..011), each
# a companion of what its decision serves. Criterion 1: an id of CONSUMER counts only when its companions in the
# consumer suite pass too.
FRICTION_CONSUMER = {
    "test_T1_swap": ["test_fresh_input_counts_arrivals"],  # A25: Python and C++ count the same arrivals
    "test_T2_send_cost": ["test_T2_export_and_push_apart"],  # A26: the send split into export and lk_push (timing)
    "test_T7_exclusivity": ["test_period_in_activate"],  # A20: refused in activate when a tick is too short
}
# Criterion 3: a C++ row is clean only when these pass too (blackbox suite): the component never names its thread.
FRICTION_BLACK_BOX = ["test_rt_ladder_is_the_librarys"]  # A19: the RT ladder is the library's
# Criterion 8: a part counts only when its companions (polyglot suite) pass too.
FRICTION_REACH = {
    "mixed_chain": ["test_converted_input_keeps_a_plan_off_the_rt_thread",  # A17: In<T, Converted>
                    "test_optional_verdict_input"],  # A23: an optional C++ In; a verdict as a DiagnosticRecord
    "codec_bit_exact": ["test_T10_trajectory_joint_names"],  # A16: a JointTrajectory view's joint names
}
# Criterion 9: a facility counts only when its companions (facilities suite) pass too.
FRICTION_FACILITIES = {
    "lifecycle": ["test_sigterm_runs_deactivate"],  # A18: SIGTERM and SIGHUP end a deployment as SIGINT does
    "metrics": ["test_system_facility_ports"],  # A21: a Python System's log/health/metrics, as a native box's
    "health": ["test_health_set_by_the_component"],  # A22: health(level, code); log.error -> ErrorLogged
    "params": ["test_vector_params"],  # A24: LK_PARAMS reflects vectors and vectors of rows
}
VALID_GRAPHS = [("test_valid_graph_accepted", name) for name in (
    "cross_group_cycle", "exposed_input_unwired", "fan_out", "fifo_on_python_ignored", "gather", "lagged_cycle", "linear",
    "multi_process_dora", "native_group", "native_own_process_dora", "nested_system", "no_place",
    "python_object_in_process", "rateless_queue")] + [("test_reference_cell_describes", None)]
BLOCKED_STEPS = {"arm_cell.PickPlanner", "arm_cell.TargetFile", "arm_cell.MoteusBusPy", "arm_cell.JtcPy",
                 "arm_cell.GuardPy", "empty.Empty", "empty.Talker", "relay.Count", "relay.Double", "relay.Save",
                 "pipeline.Count", "pipeline.Double", "pipeline.Save"}


def clamp(value: float) -> float:
    return max(0.0, min(1.0, value))


def lower_is_better(x: float | None, target: float, zero: float) -> float:
    return 0.0 if x is None else clamp((zero - x) / (zero - target))


# ---------------------------------------------------------------- running things


def run_json(command: list[str], timeout: float = 1800.0, env: dict[str, str] | None = None) -> dict:
    env = env or dict(os.environ, PYTHONPATH=f"{REPO / 'src'}:{REPO}")
    try:
        result = subprocess.run(command, cwd=REPO, env=env, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"error": f"timed out after {timeout} s"}
    text = result.stdout.strip()
    if not text:
        return {"error": result.stderr[-800:]}
    for candidate in (text, text.splitlines()[-1]):  # a whole (indented) document, else the last line
        try:
            return json.loads(candidate)
        except ValueError:
            continue
    return {"error": (result.stderr or result.stdout)[-800:]}


def run_suites(suites: list[str], timeout: float) -> dict[str, dict[str, list[str]]]:
    """Each suite through kpi/run.py; {suite: {test id: [outcome per parametrization]}} from its junit file."""
    from kpi import run as runner

    results = {}
    for suite in suites:
        junit = JUNIT / f"{suite}.xml"
        if junit.exists():
            junit.unlink()
        runner.run(suite, [], timeout_s=timeout)
        results[suite] = parse_junit(junit)
    return results


def run_bench(timing: bool) -> dict:
    """kpi/bench_overhead.py through kpi/run.py's private network and PID namespace, the line the suites run under:
    in the shared namespace the production dora answers on 127.0.0.1:6012 and the private dora of the cross-process
    A/B refuses, so that quarter of criterion 2 could never score (ruler-001 §1). The timing tier pins the A/B to
    cores 20-21 (CONTRACT §11 criterion 2; ruler-001 §2)."""
    from kpi import run as runner

    arguments = ["--pin", "20,21"] if timing else ["--no-cross-process"]
    return run_json(runner.namespaced([sys.executable, str(KPI / "bench_overhead.py"), *arguments]), timeout=20000,
                    env=runner.environment())


def parse_junit(path: pathlib.Path) -> dict[str, list[str]]:
    outcomes: dict[str, list[str]] = {}
    if not path.exists():
        return outcomes
    for case in ElementTree.parse(path).iter("testcase"):
        name = case.get("name", "")
        test_id, _, param = name.partition("[")
        tags = {child.tag for child in case}
        outcome = "failed" if tags & {"failure", "error"} else "skipped" if "skipped" in tags else "passed"
        outcomes.setdefault(test_id, []).append(f"{param.rstrip(']')}={outcome}" if param else outcome)
    return outcomes


def passed(results: dict[str, list[str]], test_id: str, param: str | None = None) -> bool:
    outcomes = results.get(test_id, [])
    if param is not None:
        outcomes = [o for o in outcomes if o.startswith(f"{param}=")]
    return bool(outcomes) and all(o.endswith("passed") for o in outcomes)


# ---------------------------------------------------------------- gates


def gates(junit: dict, arguments) -> dict:
    contract = junit.get("contract", {})
    equivalence = {test: passed(contract, test) for test in EQUIVALENCE}
    python_inproc = (passed(contract, "test_one_process_matches_schedule") and passed(contract, "test_sent_equals_received", "inproc")
                     and passed(contract, "test_reference_solutions", "pipeline"))
    fallback = run_json([sys.executable, str(KPI / "fallback.py")] + ([] if arguments.skip_fallback_suite else ["--suite"]),
                        timeout=4000)
    probe = BUILD / "alloc_probe"
    if probe.exists():
        result = subprocess.run([str(probe), "--gtest_brief=1"], capture_output=True, text=True, timeout=900)
        hot_path = {"pass": result.returncode == 0, "tail": result.stdout.strip().splitlines()[-3:]}
    else:
        hot_path = {"pass": False, "reason": f"{probe} is not built"}
    lean = run_json([sys.executable, str(KPI / "deps.py")])
    return {
        "equivalence": {"pass": all(equivalence.values()), "checks": equivalence, "python_inproc": python_inproc},
        "fallback_untouched": {"pass": bool(fallback.get("pass")), **fallback},
        "realtime_hot_path": hot_path,
        "lean_core": {"pass": bool(lean.get("pass")), **lean},
    }


# ---------------------------------------------------------------- criteria


def with_twin(results: dict[str, list[str]], test_id: str, timing: bool) -> bool:
    """passed(), and in the timing tier its timing twin's pass as well (TIMING_TWINS; CONTRACT §15 A14)."""
    twin = TIMING_TWINS.get(test_id)
    return passed(results, test_id) and (not timing or twin is None or passed(results, twin))


def with_companions(results: dict[str, list[str]], companions: list[str]) -> bool:
    """Every companion test of an id or part passes (ruler M0.3, CONTRACT §15 A27)."""
    return all(passed(results, test) for test in companions)


def consumer_fit(junit: dict, timing: bool) -> dict:
    consumer = junit.get("consumer", {})
    passing = [test for test in CONSUMER if with_twin(consumer, test, timing)
               and with_companions(consumer, FRICTION_CONSUMER.get(test, []))]
    companions = [test for tests in FRICTION_CONSUMER.values() for test in tests]
    return {"score": clamp((len(passing) - 4) / (9 - 4)), "measured": f"{len(passing)}/9", "passing": passing,
            "acceptance": {test: passed(consumer, test) for test in ACCEPTANCE},
            "outcomes": {test: consumer.get(test, ["missing"]) for test in CONSUMER + ACCEPTANCE + companions}}


def speed(bench: dict, timing: bool) -> dict:
    cross = bench.get("cross_process") or {}
    payload_scores = []
    for name in ("64B", "4KiB", "1MiB", "7x1000"):
        row = cross.get(name) or {}
        raw, added = row.get("raw_p99_s"), row.get("added_p99_s")
        if raw is None or added is None or not row.get("sent_equals_received"):
            payload_scores.append(0.0)
        else:
            payload_scores.append(lower_is_better(added, max(10e-6, 0.05 * raw), raw))
    cross_score = min(payload_scores)
    edge = (lower_is_better(bench.get("cxx_per_edge_ns"), 100.0, 1000.0)
            + lower_is_better(bench.get("python_per_edge_ns"), 1000.0, 10000.0)) / 2
    pairs = [row for name in ("4KiB", "1MiB", "7x1000") for row in (cross.get(name) or {}).get("pairs", [])]
    extra_copies = [row["lk"].get("copies", 1) - row["raw"].get("copies", 0) for row in pairs if "copies" in row.get("lk", {})]
    copies = 1.0 if extra_copies and max(extra_copies) <= 0 else 0.0
    cpu_rows = [(row["raw"].get("forwarder_cpu_pct"), row["lk"].get("forwarder_cpu_pct")) for row in pairs]
    cpu_rows = [(raw, lk) for raw, lk in cpu_rows if raw and lk is not None]
    cpu = min((lower_is_better((lk - raw) / raw, 0.05, 0.5) for raw, lk in cpu_rows), default=0.0)
    copies_cpu = (copies + cpu) / 2
    imports = lower_is_better(bench.get("import_ms"), 20.0, 97.0)
    parts = {"cross_process": cross_score, "per_edge": edge, "copies_cpu": copies_cpu, "import": imports}
    score = sum(parts.values()) / 4 if timing else 0.0
    return {"score": score, "parts": parts, "scored": timing,
            "note": None if timing else "rack powered, cores 22-23, not the timing tier: reported, scored 0",
            "measured": {key: bench.get(key) for key in ("import_ms", "python_per_edge_ns", "cxx_per_edge_ns", "tier")}}


def black_box(junit: dict, static: dict, linter: dict) -> dict:
    blocked = junit.get("blackbox", {})
    polyglot = junit.get("polyglot", {})
    rows = []
    for component in static.get("components", []):
        if component["language"] == "python":
            key = f"{component['task']}.{component['name']}"
            dynamic = passed(blocked, "test_steps_with_transports_blocked", key) if key in BLOCKED_STEPS else False
        else:
            dynamic = passed(polyglot, "test_cpp_needs_no_python") and with_companions(blocked, FRICTION_BLACK_BOX)
        rows.append({**component, "clean": not component["violations"] and dynamic and linter.get("pass", False)})
    rows.append({"name": "ArmControllerPy", "task": "arm_cell", "language": "python", "violations": [],
                 "clean": passed(blocked, "test_steps_with_transports_blocked", "arm_cell.ArmControllerPy")
                 and linter.get("pass", False)})
    clean = sum(row["clean"] for row in rows)
    share = clean / len(rows) if rows else 0.0
    controller_dirty = [row["name"] for row in rows if row["task"] == "arm_cell" and not row["clean"]]
    library_env = static.get("library_env_reads", [])
    score = 0.0 if controller_dirty or library_env else clamp((share - 0.8) / 0.2)
    return {"score": score, "measured": f"{clean}/{len(rows)}", "controller_violations": controller_dirty,
            "library_env_reads": library_env, "import_linter": linter,
            "friction": {test: blocked.get(test, ["missing"]) for test in FRICTION_BLACK_BOX}}


def succinctness(surface: dict, junit: dict) -> dict:
    api = surface.get("api", {})
    if api.get("available"):
        concepts = (lower_is_better(api["top_level"], 8, 11) + lower_is_better(api["total"], 30, 43)) / 2
        widest = min(lower_is_better(api["widest"]["params"], 6, 10),
                     lower_is_better(api["kwargs_bags"] + api["string_enums"], 0, 3))
    else:
        concepts = widest = 0.0
    contract = junit.get("contract", {})
    workflow = []
    for task, row in (surface.get("tasks") or {}).items():
        if not passed(contract, "test_reference_solutions", task):
            workflow.append(0.0)
        elif row["kind"] == "cross_process":
            workflow.append(lower_is_better(row["ratio"], 0.4, 1.0))
        else:
            workflow.append(lower_is_better(row["ratio"], 1.2, 2.0))
    workflow_score = sum(workflow) / len(workflow) if workflow else 0.0
    return {"score": (concepts + widest + workflow_score) / 3,
            "parts": {"concepts": concepts, "widest_signature": widest, "workflow": workflow_score},
            "measured": {"top_level": api.get("top_level"), "total": api.get("total"), "widest": api.get("widest"),
                         "tasks": surface.get("tasks")}}


def swap_cost(swaps: dict, junit: dict) -> dict:
    contract = junit.get("contract", {})
    per_swap = {}
    for swap in SWAPS:
        row = (swaps.get("swaps") or {}).get(swap)
        runs = passed(contract, "test_swap_variant", f"{swap}-a") and passed(contract, "test_swap_variant", f"{swap}-b")
        if row is None or row["component_lines"] > 0 or not runs:
            per_swap[swap] = 0.0
        else:
            per_swap[swap] = lower_is_better(row["deployment_lines"], 1, 20)
    return {"score": sum(per_swap.values()) / len(SWAPS), "per_swap": per_swap, "measured": swaps.get("swaps")}


def dora_insulation(junit: dict, timing: bool) -> dict:
    churn = junit.get("churn", {})
    conformance = junit.get("conformance", {})
    if not timing:  # WHY: a timing twin skips in the default tier by design; its default-tier half still counts (A14)
        conformance = {test: outcomes for test, outcomes in conformance.items() if test not in TIMING_TWINS.values()}
    parts = {
        "named_only_in_backend": passed(churn, "test_dora_named_only_in_its_backend"),
        "wire_owned": passed(churn, "test_wire_is_owned_by_like_keras"),
        "nodes_only": passed(churn, "test_nodes_only"),
        "skew_refused_2s": passed(churn, "test_T11_cli_skew_refused_within_2s"),
        "conformance_both_backends": bool(conformance) and all(passed(conformance, test) for test in conformance),
    }
    return {"score": sum(parts.values()) / 5, "parts": parts}


def compactness(surface: dict) -> dict:
    size = surface.get("size") or {}
    parts = size.get("parts") or {}
    scores = {}
    for name, row in parts.items():
        if row.get("target") is None:
            continue
        scores[name] = 0.0 if row["sloc"] == 0 else lower_is_better(row["sloc"], row["target"], 2 * row["target"])
    total = (size.get("total") or {}).get("sloc", 0)
    scores["total"] = 0.0 if total == 0 else lower_is_better(total, 2450, 4900)
    return {"score": sum(scores.values()) / len(scores) if scores else 0.0, "parts": scores,
            "measured": {name: row.get("sloc") for name, row in parts.items()} | {"total": total},
            "formatted": size.get("formatted")}


def reach(junit: dict) -> dict:
    polyglot = junit.get("polyglot", {})
    parts = {
        "mixed_chain": passed(polyglot, "test_mixed_chain"),
        "cpp_needs_no_python": passed(polyglot, "test_cpp_needs_no_python"),
        "one_so_three_placements": passed(polyglot, "test_one_so_three_placements"),
        "codec_bit_exact": all(passed(polyglot, test) for test in
                               ("test_T10_codec_roundtrip", "test_T10_trajectory_view", "test_T10_hash_mismatch_refused")),
        "boilerplate": passed(polyglot, "test_cpp_boilerplate"),
    }
    parts = {name: ok and with_companions(polyglot, FRICTION_REACH.get(name, [])) for name, ok in parts.items()}
    friction = {test: polyglot.get(test, ["missing"]) for tests in FRICTION_REACH.values() for test in tests}
    return {"score": sum(parts.values()) / 5, "parts": parts, "friction": friction}


def facilities(junit: dict, surface: dict, timing: bool) -> dict:
    found = junit.get("facilities", {})
    per = {name: all(passed(found, f"test_{name}", combo) for combo in COMBINATIONS) for name in FACILITIES}
    # WHY: health counts only with its lockstep half on both inproc combinations, in every tier: the suite never
    # deployed in lockstep, and a box that sent no health record at all under lockstep scored green (ruler-005, A14)
    per["health"] = per["health"] and all(passed(found, "test_health_lockstep", combo) for combo in INPROC_COMBINATIONS)
    if timing:  # WHY: the default tier's test_health accepts an honest warn/Overrun; unloaded, it must say ok (ruler-003)
        unloaded_ok = all(passed(found, "test_health_ok_when_unloaded", combo) for combo in COMBINATIONS)
        per["health"] = per["health"] and unloaded_ok
    per = {name: ok and with_companions(found, FRICTION_FACILITIES.get(name, [])) for name, ok in per.items()}
    share = ((surface.get("size") or {}).get("facility") or {}).get("share")
    count = sum(per.values())
    score = 0.0 if share is not None and share > 0.30 else clamp((count - 3) / (7 - 3))
    friction = {test: found.get(test, ["missing"]) for tests in FRICTION_FACILITIES.values() for test in tests}
    return {"score": score, "measured": f"{count}/7", "facilities": per, "facility_sloc_share": share,
            "friction": friction}


def testability(junit: dict, timing: bool) -> dict:
    twin = junit.get("twin", {})

    def deterministic_10x(placement: str) -> bool:
        # WHY two ids: ">= 10x real time" reads CPU time in the default tier, where the wall clock measures the
        # scheduler's starvation of a niced process (ruler-002 §1), and the wall clock as well in the timing tier.
        cpu = passed(twin, "test_ten_runs_hash_identical_at_10x_cpu_time", placement)
        if not timing:
            return cpu
        return cpu and passed(twin, "test_ten_runs_hash_identical_at_10x_wall_clock", placement)

    parts = {
        "twin_swap": passed(twin, "test_twin_swap_is_one_config_line"),
        "deterministic_10x": deterministic_10x("fallback"),
        "replay": passed(twin, "test_replay_reproduces_the_run"),
        "isolation": passed(twin, "test_component_alone"),
    }
    no_sleep = passed(junit.get("blackbox", {}), "test_no_wall_clock_sleep_in_twin_tests")
    return {"score": sum(parts.values()) / 4 if no_sleep else 0.0, "parts": parts, "no_wall_clock_sleep": no_sleep,
            "same_box_deterministic": deterministic_10x("same_box")}


def attach(junit: dict, timing: bool) -> dict:
    found = junit.get("attach", {})
    parts = {test: with_twin(found, test, timing) for test in ATTACH}
    return {"score": sum(parts.values()) / 4, "parts": parts}


def wiring_checks(junit: dict) -> dict:
    from kpi.miswire.cases import CASES

    miswire = junit.get("miswire", {})
    caught = sum(passed(miswire, "test_miswire_rejected", name) for name in CASES)
    valid = junit.get("validgraphs", {})
    false_positives = [test for test, outcomes in valid.items() if any(o.endswith("failed") for o in outcomes)]
    # WHY every frozen id: a valid graph that skips, errors or is missing was never shown to be accepted, so it counts
    # as a false positive like a refused one (ruler-001 §3; the README's "a skip can never raise a score").
    false_positives += [f"{test}[{param}]" if param else test for test, param in VALID_GRAPHS
                        if not passed(valid, test, param)]
    accepted_any = any(o.endswith("passed") for outcomes in valid.values() for o in outcomes)
    share = caught / len(CASES)
    score = 0.0 if false_positives or not accepted_any else clamp((share - 0.5) / 0.5)
    return {"score": score, "measured": f"{caught}/{len(CASES)}", "false_positives": false_positives}


# ---------------------------------------------------------------- the board


def git_head() -> str:
    result = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
    return result.stdout.strip()


def import_linter() -> dict:
    """`uvx --from import-linter lint-imports` (criteria.md), from the locked venv when it has it."""
    env = dict(os.environ, PYTHONPATH=f"{REPO / 'src'}:{REPO}")
    local = pathlib.Path(sys.executable).with_name("lint-imports")
    command = [str(local)] if local.exists() else ["uvx", "--from", "import-linter", "lint-imports"]
    try:
        result = subprocess.run([*command, "--config", str(REPO / ".importlinter")],
                                cwd=REPO, env=env, capture_output=True, text=True, timeout=600)
    except (subprocess.TimeoutExpired, FileNotFoundError) as error:
        return {"pass": False, "reason": str(error)}
    return {"pass": result.returncode == 0, "tail": (result.stdout + result.stderr).strip().splitlines()[-4:]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--suites", nargs="*", default=SUITES, help="run only these suites (the rest score 0)")
    parser.add_argument("--suite-timeout", type=float, default=7200.0)
    parser.add_argument("--skip-fallback-suite", action="store_true",
                        help="check the fallback files' bytes only; the gate then reports red (suite not run)")
    parser.add_argument("--out", type=pathlib.Path, default=KPI / "scoreboard.json")
    arguments = parser.parse_args(argv)
    timing = os.environ.get("KPI_TIER") == "timing"
    junit = run_suites(arguments.suites, arguments.suite_timeout)
    static = run_json([sys.executable, str(KPI / "blackbox.py")])
    surface = run_json([sys.executable, str(KPI / "surface.py"), "--api", "--tasks", "--size"])
    swaps = run_json([sys.executable, str(KPI / "swap_cost.py")])
    bench = run_bench(timing)
    linter = import_linter()
    board_gates = gates(junit, arguments)
    criteria = {
        "1_consumer_fit": consumer_fit(junit, timing),
        "2_speed": speed(bench, timing),
        "3_black_box": black_box(junit, static, linter),
        "4_succinctness": succinctness(surface, junit),
        "5_swap_cost": swap_cost(swaps, junit),
        "6_dora_insulation": dora_insulation(junit, timing),
        "7_compactness": compactness(surface),
        "8_reach": reach(junit),
        "9_facilities": facilities(junit, surface, timing),
        "10_testability": testability(junit, timing),
        "11_attach": attach(junit, timing),
        "12_wiring_checks": wiring_checks(junit),
    }
    for name, row in criteria.items():
        row["weight"] = WEIGHTS[name]
    vector = [round(criteria[name]["score"], 4) for name in WEIGHTS]
    weighted = sum(WEIGHTS[name] * criteria[name]["score"] for name in WEIGHTS)
    green = all(gate["pass"] for gate in board_gates.values())
    total = round(weighted if green else 0.0, 3)
    baseline_path = KPI / "baseline.json"
    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {"total": 0.0, "vector": [0.0] * 12}
    drops = [f"{name} {old}->{new}" for name, old, new in zip(WEIGHTS, baseline["vector"], vector) if new < old - 0.02]
    board = {
        "generated": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "commit": git_head(), "host": platform.node(),
        "tier": "timing" if timing else "rack powered, cores 22-23, not the timing tier",
        "suites_run": arguments.suites,
        "gates": board_gates,
        "criteria": criteria,
        "vector": vector,
        "weighted_if_gates_green": round(weighted, 3),
        "total": total,
        "baseline": {"total": baseline["total"], "vector": baseline["vector"]},
        "land": {"ok": green and total >= baseline["total"] and not drops,
                 "reasons": ([] if green else ["a gate is red"]) + ([] if total >= baseline["total"] else ["total below baseline"])
                 + [f"criterion fell beyond its noise band: {drop}" for drop in drops]},
    }
    arguments.out.write_text(json.dumps(board, indent=1, default=str) + "\n")
    print(json.dumps({"total": total, "vector": vector, "gates": {name: gate["pass"] for name, gate in board_gates.items()},
                      "out": str(arguments.out)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
