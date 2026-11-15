"""The relay's outputs as plain Python (what the solution must reproduce)."""

from robot_msgs.sensor import JointState


def relay(path="relay.txt", steps=10):
    with open(path, "a") as file:
        for count in range(1, steps + 1):
            doubled = JointState(name=("a",), position=[count]).position * 2
            file.write(f"{doubled[0]}\n")
