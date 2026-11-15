"""A frozen variant of the reference deployment (criterion 5): this tree and its sibling differ by one line."""

import like_keras as lk
from like_keras.backends import dora

from kpi.tasks.arm_cell.arm_controller_py import ArmControllerPy
from kpi.tasks.arm_cell.cell import CONFIG, LIBRARY, pick_cell


def deployment(config=CONFIG, *, realtime=True, cpus=(20,)) -> lk.Deployment:
    cell = pick_cell(ArmControllerPy(config=config))
    dep = lk.Deployment(cell, realtime=realtime)
    dep.place(cell.arm, process="arm-controller", hz=100, cpus=list(cpus), fifo=80)
    dep.place(cell.targets, hz=2)
    dep.place(cell.planner)
    return dep
