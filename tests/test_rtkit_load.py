"""Lane dora's test of tests/rtkit_load.py, the judge the rtkit lane tests (tests/test_dora_boundary.py,
tests/test_io_ladder.py) use so another lane's load on rtkit's shared per-user budget is retried, never failed on and
never passed (CONTRACT §15 A35; library verifier, the round after A31-A42). No rtkit, no dora: the journal lines are
rtkit-daemon's own, verbatim, from the failure the verifier saw on 2026-11-16 at 13:58:50 (its lk-dora-host 3774685
and private daemon 3774484; another lane's lk-dora-host 3774259 and daemon 3774112 granted two seconds earlier).

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_rtkit_load.py
"""

from __future__ import annotations

import datetime

import pytest

from tests import rtkit_load
from tests.rtkit_load import GRANT, judged, refusals_in

RTKIT = "bam-GPU rtkit-daemon[2176414]:"
OTHER_LANE = [f"2026-11-16T13:58:48-07:00 {RTKIT} Successfully made thread {thread} of process {process} owned by "
              f"'1000' RT at priority {priority}."
              for thread, process, priority in ((3774408, 3774259, 20), (3774281, 3774259, 10), (3774130, 3774112, 5))]
OURS_GRANTED = [f"2026-11-16T13:58:50-07:00 {RTKIT} Successfully made thread {thread} of process {process} owned by "
                f"'1000' RT at priority {priority}."
                for thread, process, priority in ((3774834, 3774685, 20), (3774705, 3774685, 10), (3774500, 3774484, 5))]
REFUSED = [f"2026-11-16T13:58:{second}-07:00 {RTKIT} Warning: Reached burst limit for user '1000', denying request."
           for second in (50, 51, 51)]
OURS = {3774685, 3774484}
STARTED_S = datetime.datetime.fromisoformat("2026-11-16T13:58:49-07:00").timestamp()  # the run under judgement


def test_a_refusal_while_another_process_was_granted_is_excused():
    excused = refusals_in(OTHER_LANE + OURS_GRANTED + REFUSED, STARTED_S, OURS)
    assert len(excused) == 3 and "Reached burst limit" in excused[0] and "3774112" in excused[0], excused


def test_a_refusal_only_this_tests_own_requests_explain_is_not_excused():
    assert refusals_in(OURS_GRANTED + REFUSED, STARTED_S, OURS) == []
    assert refusals_in(OTHER_LANE + OURS_GRANTED + REFUSED, STARTED_S, OURS | {3774259, 3774112}) == []


def test_no_refusal_during_the_run_is_no_excuse():
    assert refusals_in(OTHER_LANE + OURS_GRANTED, STARTED_S, OURS) == []
    later_s = datetime.datetime.fromisoformat("2026-11-16T13:58:52-07:00").timestamp()  # refusals before the run
    assert refusals_in(OTHER_LANE + OURS_GRANTED + REFUSED, later_s, OURS) == []


class Attempts:
    """A scenario that returns the given problems, one list per attempt (the last repeats)."""

    def __init__(self, *problems: list[str]) -> None:
        self.problems, self.ran = problems, 0

    def __call__(self, index: int):
        self.ran += 1
        return self.problems[min(index, len(self.problems) - 1)], [1234]


@pytest.fixture
def refused(monkeypatch):
    """rtkit refused grants for another process's load during every attempt."""
    monkeypatch.setattr(rtkit_load, "outside_refusals", lambda since_s, until_s, ours: ["burst limit (other: 99)"])


def test_a_grant_problem_under_outside_refusals_runs_again_and_passes_only_when_the_run_holds(refused):
    scenario = Attempts([f"{GRANT}not raised again"], [])
    assert judged(scenario) == [] and scenario.ran == 2


def test_excused_every_time_is_skipped_never_passed(refused):
    scenario = Attempts([f"{GRANT}not raised again"])
    with pytest.raises(pytest.skip.Exception, match="not judged"):
        judged(scenario)
    assert scenario.ran == rtkit_load.TRIES


def test_a_problem_no_refused_grant_explains_fails_at_once(refused):
    scenario = Attempts([f"{GRANT}not raised again", "a SHARED daemon was raised"])
    assert judged(scenario) == [f"{GRANT}not raised again", "a SHARED daemon was raised"] and scenario.ran == 1


def test_a_grant_problem_without_outside_refusals_fails(monkeypatch):
    monkeypatch.setattr(rtkit_load, "outside_refusals", lambda since_s, until_s, ours: [])
    scenario = Attempts([f"{GRANT}not raised again"])
    assert judged(scenario) == [f"{GRANT}not raised again"] and scenario.ran == 1
