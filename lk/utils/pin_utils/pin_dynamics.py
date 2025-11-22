#!/usr/bin/env python3

# BAM
from typing import TYPE_CHECKING

import numpy as np

# PYTHON
import pinocchio as pin

from lk.utils.pin_utils.pin_robot_model import PinRobotModel

if TYPE_CHECKING:
    from lk.descriptions import RobotDescription


class PinDynamics:
    @classmethod
    def from_robot_description(
        cls, robot_description: "RobotDescription", gravity=[0, 0.0, -9.81]
    ):
        pin_model = PinRobotModel.from_robot_description(robot_description)
        return cls(pin_model, gravity)

    def __init__(self, robot_model: PinRobotModel, gravity=[0, 0.0, -9.81]):
        print("[UNCONFIGURED] PinDynamics")

        self.robot_model = robot_model
        self.model = robot_model.model

        # Set gravity direction (x-axis for example)
        self.model.gravity.linear = np.array(gravity)

        self.data = self.model.createData()
        self.tau = None

        print("[READY] PinDynamics")

    def preprocess(self, val: float | list[float] | np.ndarray) -> np.ndarray:
        arr = None
        if isinstance(val, float):
            arr = np.array([[val]])
        elif isinstance(val, list):
            arr = np.array(val)
        elif isinstance(val, np.ndarray):
            arr = val

        if arr.ndim == 1:
            if arr.shape[0] > 6:
                arr = arr.reshape(-1, 1)  # assumes its many waypoints for single dof
            else:
                arr = arr.reshape(1, -1)  # assumes its a single waypoint for 6 dof

        return arr

    def rnea_i(self, q, qd, qdd):
        q = self.preprocess(q)
        qd = self.preprocess(qd)
        qdd = self.preprocess(qdd)
        self.tau = pin.rnea(self.model, self.data, q, qd, qdd)
        return self.tau

    def rnea(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        q = self.preprocess(q)
        qd = self.preprocess(qd)
        qdd = self.preprocess(qdd)

        tau = np.zeros(q.shape)

        for i in range(q.shape[0]):
            q_i = q[i, :]
            qd_i = qd[i, :]
            qdd_i = qdd[i, :]

            tau_i = pin.rnea(self.model, self.data, q_i, qd_i, qdd_i)
            tau[i, :] = tau_i

        if tau.shape[0] == 1:
            tau = tau[0, :]

        return tau

    def __str__(self):
        self.pin_model.inspect()
        return ""
