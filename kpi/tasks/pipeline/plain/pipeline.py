"""The pipeline as plain Python function composition, the baseline of criterion 4's in-process ratio."""

import time

from robot_msgs.sensor import JointState
from robot_msgs.std import Header


def count(step):
    return JointState(name=("a",), position=[step + 1], header=Header(stamp=time.time()))


def double(value):
    return value.replace(position=2 * value.position)


def save(value, path):
    with open(path, "a") as file:
        file.write(f"{value.position[0]}\n")


def pipeline(path="pipeline.txt", steps=10, hz=10):
    start = time.monotonic()
    for step in range(steps):
        time.sleep(max(0.0, start + step / hz - time.monotonic()))
        save(double(count(step)), path)
