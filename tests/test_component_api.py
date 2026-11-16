"""Lane core's tests of the Python API CONTRACT §15 adds to a component: period() (A20), health(level, code) (A22)
and In(T, fresh=True) (A25), at the edges the frozen M0.3 tests leave alone: eager calls, refusals at class creation,
and the health rules that only show in a combination of facts.

    env -u VIRTUAL_ENV uv run --frozen --directory /home/bam/like_keras --extra test pytest -q tests/test_component_api.py
"""

from __future__ import annotations

import pytest

import like_keras as lk
from robot_msgs.std import Header


class Ticker(lk.Component):
    value = lk.Out(Header)

    def forward(self):
        return Header(stamp=self.now())


# ---------------------------------------------------------------- A20: period()


def test_period_is_zero_when_eager_and_a_port_may_not_take_its_name():
    assert Ticker().period() == 0.0
    with pytest.raises(TypeError, match="'period' collides with a Component member"):

        class Clashing(lk.Component):  # noqa: F841
            period = lk.Out(Header)
