"""The fallback-untouched gate (CONTRACT §11, as amended by §15 A49): like_keras changes nothing on the robot's existing
Python path. JSON on stdout; exit 0 when green.

    python3 kpi/fallback.py [--runs DIR ...]

Measured at ONE pinned bam_ws commit (fallback_manifest.json), never at bam_ws's moving main, so bam_ws's own drift and
test health are not like_keras's score:

1. bytes: arm_hold, pick_loop_v1 and fivebar_run hash as recorded, in the manifest's root (a clean checkout of the pinned
   commit) and in every shard of the run below;
2. dependency: bam_rack_twin's pyproject requires no like_keras;
3. differential suite: bam_rack_twin's whole test suite at the pinned commit gives the same outcome for every test
   (junit testcase: passed / failure / error / skipped) without like_keras importable and with this checkout's like_keras
   installed into the same venv. A test red in both runs is bam_ws's own, not like_keras's. A test whose outcome differs
   is rerun (kpi/fallback_confirm.py: 3 rounds each way, arms alternating in one venv) and counts as like_keras's unless
   an arm then shows the other arm's outcome. WHY: two runs of the pin without like_keras differed on 2 of 7,795 tests.

Without --runs the gate reports `suite: not run` (red). The suite runs out of memory on the workstation and takes over
3 h on one VM, so it runs as two 8-shard cloud dispatches, one per arm (kpi/fallback_shard.py on each VM), and --runs
names the folders their shards came home to. The recipe, from a bam_ws checkout on a `claude/...` branch at the pinned
commit, with B = the payload branch `--payload` prints:

    python3 kpi/fallback.py --payload <empty dir>        # like_keras src at HEAD + the shard runner, as a git repo to push
    tools/cloud/dispatch.sh --fanout 8 --wait adhoc --package src/dev/bam_rack_twin --extra hardware --extra fivebar \\
        --extra camera --extra record --extra test --name lk-fallback-<arm> -- bash -c '<see PREPARE below>; \\
        P=$(mktemp -d); git -C {repo} fetch -q --depth 1 origin B && git -C {repo} archive FETCH_HEAD | tar -x -C $P \\
        && exec python $P/fallback_shard.py --arm <arm> --out {out} --payload $P'

for <arm> = without and with. If the arms differ, `--payload <new dir> --runs ...` adds the differing tests to a new payload,
and one more dispatch (no --fanout) runs `python $P/fallback_confirm.py --out {out} --payload $P` the same way; pass its
folder in --runs too. A shard that timed out, crashed, or ran on another commit, another split, other fallback bytes
or another like_keras (the scored checkout's `src` tree and `pyproject.toml`) turns the gate red, as does any package other
than like_keras that differs between the arms' venvs. Only the ruler lane re-records the manifest (`--record ROOT`, from a
clean checkout), when the pin moves; a builder never does.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import subprocess
import tomllib
import xml.etree.ElementTree as ElementTree

KPI = pathlib.Path(__file__).resolve().parent
REPO = KPI.parent
MANIFEST = KPI / "fallback_manifest.json"
ARMS = ("without", "with")
# WHY in the ruler: at bam_ws 06424132b code bam_rack_twin's tests reach imports robot_diagnostics, which bam_rack_twin's
# uv.lock lacks; installing one pinned commit of it in both arms keeps those tests running, so more of the path is compared
PREPARE = ("git clone -q https://github.com/zacharyyamaoka/robot_diagnostics.git /home/bam/robot_diagnostics && "
           "git -C /home/bam/robot_diagnostics checkout -q 2ab15cbd0817ae747ce3652acc2ee646dda8de19 && "
           "uv pip install -q --python .venv/bin/python --no-deps -e /home/bam/robot_diagnostics")


def git(*args: str, cwd: pathlib.Path = REPO) -> str:
    result = subprocess.run(["git", "-C", str(cwd), *args], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else ""


def sha256(path: pathlib.Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def identity() -> dict:
    """What the with-arm installs from this checkout: its committed `src` tree and `pyproject.toml`."""
    return {"src_tree": git("rev-parse", "HEAD:src"), "pyproject_blob": git("rev-parse", "HEAD:pyproject.toml"),
            "runner": sha256(KPI / "fallback_shard.py"),  # WHY: a run made by another ruler's runner is not this gate's
            "confirm_runner": sha256(KPI / "fallback_confirm.py")}


def confirmations(runs: list[pathlib.Path], manifest: dict, expected: dict, venv: frozenset | None,
                  problems: list[str]) -> dict[str, dict[str, list[str]]]:
    """test -> {arm: outcomes} over every valid kpi/fallback_confirm.py record under RUNS (one venv, arms alternating)."""
    seen: dict[str, dict[str, list[str]]] = {}
    for folder in runs:
        for path in sorted(folder.rglob("fallback_confirm.json")):
            record, where = json.loads(path.read_text()), f"confirmation ({path.parent})"
            payload = {key: record["payload"].get(key) for key in ("src_tree", "pyproject_blob", "confirm_runner")}
            wrong = [f"bam_ws {record['bam_ws_commit'][:12]}"] * (record["bam_ws_commit"] != manifest["commit"])
            wrong += [f"like_keras or runner {payload}"] * (payload != {key: expected[key] for key in payload})
            wrong += ["a venv other than the arms'"] * (venv is None or frozenset(record["freeze_before"]) != venv)
            for phase in record["phases"]:
                probe, freeze = phase["like_keras"], set(phase["freeze"])
                installed = probe["importable"] and "site-packages" in (probe["origin"] or "")
                if phase["arm"] == "without" and (probe["importable"] or freeze != venv):
                    wrong.append(f"round {phase['round']} without: like_keras importable or the venv not restored")
                if phase["arm"] == "with" and not (installed and len(freeze ^ (venv or set())) == 1):
                    wrong.append(f"round {phase['round']} with: like_keras not installed alone")
                if phase["pytest_exit"] not in (0, 1):
                    wrong.append(f"round {phase['round']} {phase['arm']}: pytest exit {phase['pytest_exit']}")
            if wrong:
                problems.append(f"{where}: {'; '.join(wrong)}")
                continue
            for phase in record["phases"]:
                for test, outcome in outcomes(path.parent / phase["junit"]).items():
                    seen.setdefault(test, {arm: [] for arm in ARMS})[phase["arm"]].extend(outcome)
    return seen


def outcomes(junit: pathlib.Path) -> dict[str, list[str]]:
    """testcase -> its outcomes ('passed', or the sorted failure/error/skipped tags it carries)."""
    found: dict[str, list[str]] = {}
    for case in ElementTree.parse(junit).iter("testcase"):
        tags = sorted({child.tag for child in case} & {"failure", "error", "skipped"})
        found.setdefault(f"{case.get('classname', '')}::{case.get('name', '')}", []).append("+".join(tags) or "passed")
    return found


def differential(runs: list[pathlib.Path], manifest: dict) -> dict:
    records = []
    for folder in runs:
        for path in sorted(folder.rglob("fallback_arm.json")):
            records.append((path.parent, json.loads(path.read_text())))
    problems: list[str] = []
    expected = identity()
    if not expected["src_tree"] or git("status", "--porcelain", "--", "src", "pyproject.toml"):
        problems.append("the scored checkout's src/ or pyproject.toml is uncommitted: the run installed a commit")
    arms: dict[str, dict[int, tuple[pathlib.Path, dict]]] = {arm: {} for arm in ARMS}
    for folder, record in records:
        where = f"{record.get('arm')} shard {record.get('shard')} ({folder})"
        if record.get("schema") != "like_keras/fallback-arm-1" or record.get("arm") not in ARMS:
            problems.append(f"{folder}: not a fallback_shard.py record")
            continue
        if record["shard"] in arms[record["arm"]]:
            problems.append(f"{where}: a second record for this shard (pass each run's folder once)")
        arms[record["arm"]][record["shard"]] = (folder, record)
        if record["bam_ws_commit"] != manifest["commit"]:
            problems.append(f"{where}: ran on bam_ws {record['bam_ws_commit'][:12]}, not the pinned {manifest['commit'][:12]}")
        if record["package"] != manifest["package"]:
            problems.append(f"{where}: ran in {record['package']}, not {manifest['package']}")
        changed = [name for name, digest in manifest["files"].items() if record["fallback_files"].get(name) != digest]
        if changed:
            problems.append(f"{where}: fallback files differ from the manifest: {changed}")
        if record["timed_out"] or record["pytest_exit"] not in (0, 1):
            problems.append(f"{where}: pytest did not finish (exit {record['pytest_exit']}, timed out {record['timed_out']})")
        if record["payload"].get("runner") != expected["runner"]:
            problems.append(f"{where}: run by another kpi/fallback_shard.py than this ruler's")
        probe = record["like_keras"]
        if record["arm"] == "without" and (probe["importable"] or probe["version"]):
            problems.append(f"{where}: like_keras was importable without it ({probe['origin']})")
        if record["arm"] == "with":
            payload = {key: record["payload"].get(key) for key in ("src_tree", "pyproject_blob")}
            if payload != {key: expected[key] for key in payload}:
                problems.append(f"{where}: installed like_keras {payload}, not this checkout's")
            if not (probe["importable"] and probe["version"] and "site-packages" in (probe["origin"] or "")):
                problems.append(f"{where}: like_keras not installed into the venv ({probe})")
            added = sorted(set(record["freeze"]) ^ set(record["freeze_before"]))
            if len(added) != 1 or not added[0].replace("-", "_").startswith("like_keras") or added[0] not in record["freeze"]:
                problems.append(f"{where}: installing like_keras changed more than like_keras: {added}")
    full: set[str] = set()
    for _, record in records:
        full |= set(record.get("all_files", []))
    for arm, shards in arms.items():
        counts = {record["shards"] for _, record in shards.values()}
        if len(counts) != 1 or set(shards) != set(range(next(iter(counts), 0))):
            problems.append(f"{arm} arm: shards {sorted(shards)} of {sorted(counts)} (every shard of one run, once)")
        ran = [name for _, record in shards.values() for name in record["files"]]
        if len(ran) != len(set(ran)) or set(ran) != full or any(set(r["all_files"]) != full for _, r in shards.values()):
            problems.append(f"{arm} arm: its shards do not split the same {len(full)} test files exactly once each")
    venvs = {arm: {frozenset(record["freeze_before" if arm == "with" else "freeze"]) for _, record in shards.values()}
             for arm, shards in arms.items()}
    if len(venvs["without"] | venvs["with"]) > 1:
        problems.append("the arms' venvs differ by more than like_keras (uv pip freeze before the install)")
    tests: dict[str, dict[str, list[str]]] = {arm: {} for arm in ARMS}
    for arm, shards in arms.items():
        for folder, _ in shards.values():
            try:
                for test, seen in outcomes(folder / "junit.xml").items():
                    tests[arm].setdefault(test, []).extend(seen)
            except (OSError, ElementTree.ParseError) as error:
                problems.append(f"{folder}: junit.xml unreadable ({error})")
    names = sorted(set(tests["without"]) | set(tests["with"]))
    differ = [{"test": name, "without": sorted(tests["without"].get(name, ["absent"])),
               "with": sorted(tests["with"].get(name, ["absent"]))} for name in names
              if sorted(tests["without"].get(name, [])) != sorted(tests["with"].get(name, []))]
    # WHY a rerun before a difference counts (A49): two runs of the pin without like_keras differed on 2 of 7,795 tests.
    # A difference is like_keras's unless, rerun >= 3 times each way in one venv, an arm shows the other arm's outcome.
    rerun = confirmations(runs, manifest, expected, next(iter(venvs["without"]), None), problems) if differ else {}
    for difference in differ:
        again = rerun.get(difference["test"], {arm: [] for arm in ARMS})
        difference["rerun"] = again
        if min(len(again["without"]), len(again["with"])) < 3:
            difference["verdict"] = "not confirmed (rerun it: kpi/fallback_confirm.py)"
        elif set(difference["without"]) & set(again["with"]) or set(difference["with"]) & set(again["without"]):
            difference["verdict"] = "noise: an arm reproduced the other's outcome"
        else:
            difference["verdict"] = "like_keras: reproduced, never seen the other way"
    counted = [difference for difference in differ if not difference["verdict"].startswith("noise")]
    tally = {arm: {} for arm in ARMS}
    for arm in ARMS:
        for seen in tests[arm].values():
            for outcome in seen:
                tally[arm][outcome] = tally[arm].get(outcome, 0) + 1
    if not tests["without"]:
        problems.append("no test outcomes at all")
    verdict = "invalid" if problems else "differs" if counted else "identical, after reruns" if differ else "identical"
    return {"suite": verdict, "suite_problems": problems, "tests": len(names), "identical": len(names) - len(differ),
            "different": len(differ), "noise": len(differ) - len(counted), "differences": differ[:50], "outcomes": tally,
            "shards": {arm: len(shards) for arm, shards in arms.items()}}


def record(root: pathlib.Path, manifest: dict) -> dict:
    """Re-record the manifest from ROOT, a clean checkout of the commit to pin (the ruler lane only)."""
    root = root.resolve()
    top = pathlib.Path(git("rev-parse", "--show-toplevel", cwd=root))
    if not top.parts or git("status", "--porcelain", cwd=root):
        raise SystemExit(f"fallback.py: {root} is not a clean checkout")
    commit = git("rev-parse", "HEAD", cwd=root)
    return {"recorded": f"bam_ws {commit[:9]}, from the clean checkout below (ruler lane; CONTRACT §15 A49)",
            "commit": commit, "root": str(root), "package": str(root.relative_to(top)),
            "files": {name: sha256(root / name) for name in manifest["files"]}}


def payload(out: pathlib.Path, manifest: dict, confirm: list[str]) -> dict:
    """OUT: a fresh git repo the VMs fetch: this checkout's like_keras (HEAD), the runners, and the tests to rerun."""
    out.mkdir(parents=True)
    (out / "like_keras").mkdir()
    archive = subprocess.run(["git", "-C", str(REPO), "archive", "HEAD", "pyproject.toml", "README.md", "LICENSE", "src"],
                             capture_output=True, check=True).stdout
    subprocess.run(["tar", "-x", "-C", str(out / "like_keras")], input=archive, check=True)
    for runner in ("fallback_shard.py", "fallback_confirm.py"):
        (out / runner).write_bytes((KPI / runner).read_bytes())
    (out / "confirm.txt").write_text("".join(f"{test}\n" for test in confirm))
    (out / "fallback_manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    commit = git("rev-parse", "HEAD")
    (out / "payload.json").write_text(json.dumps({"like_keras_commit": commit, **identity(),
                                                  "ruler_kpi_tree": git("rev-parse", "HEAD:kpi")}, indent=1) + "\n")
    who = ["-c", "user.name=like_keras kpi", "-c", "user.email=kpi@users.noreply.github.com"]
    for step in (["init", "-q", "-b", "payload"], ["add", "-A"], [*who, "commit", "-q", "-m", f"fallback payload {commit[:9]}"]):
        subprocess.run(["git", "-C", str(out), *step], check=True)
    branch = f"claude/lk-payload/{commit[:9]}-{git('rev-parse', 'HEAD', cwd=out)[:7]}"
    return {"payload": str(out), "branch": branch, "push": f"git -C {out} push <bam_ws origin url> HEAD:refs/heads/{branch}",
            "prepare": PREPARE, "confirm": confirm}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--runs", type=pathlib.Path, nargs="+", default=[],
                        help="folders holding both arms' collected shards (kpi/fallback_shard.py records)")
    parser.add_argument("--suite", action="store_true", help="(kept for older callers; the suite is read from --runs)")
    parser.add_argument("--manifest", type=pathlib.Path, default=MANIFEST, help="(the ruler's mutation checks)")
    parser.add_argument("--payload", type=pathlib.Path,
                        help="write the VMs' payload repo here and exit (with --runs: their differences to rerun)")
    parser.add_argument("--record", type=pathlib.Path, help="re-record the manifest from this clean checkout and exit")
    arguments = parser.parse_args(argv)
    manifest = json.loads(arguments.manifest.read_text())
    if arguments.record or arguments.payload:
        confirm = [row["test"] for row in differential(arguments.runs, manifest)["differences"]] if arguments.runs else []
        result = record(arguments.record, manifest) if arguments.record else payload(arguments.payload, manifest, confirm)
        if arguments.record:
            arguments.manifest.write_text(json.dumps(result, indent=1, ensure_ascii=False) + "\n")
        print(json.dumps(result, indent=1))
        return 0
    root = pathlib.Path(manifest["root"])
    changed = [name for name, digest in manifest["files"].items() if sha256(root / name) != digest]
    if git("rev-parse", "HEAD", cwd=root) != manifest["commit"] or git("status", "--porcelain", "--", ".", cwd=root):
        changed = [f"{root} is not a clean checkout of bam_ws {manifest['commit'][:12]}"] + changed
    try:
        with open(root / "pyproject.toml", "rb") as file:
            project = tomllib.load(file).get("project", {})
    except OSError:
        project = {"dependencies": ["(no pyproject.toml at the manifest's root)"]}
    hard = [spec for spec in project.get("dependencies", []) if spec.replace("-", "_").startswith("like_keras")
            or spec.startswith("(")]
    report = {"files_changed": changed, "like_keras_required": hard, "pinned": manifest["commit"], "suite": "not run"}
    if arguments.runs:
        report |= differential(arguments.runs, manifest)
    report["pass"] = not changed and not hard and report["suite"] in ("identical", "identical, after reruns")
    print(json.dumps(report))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
