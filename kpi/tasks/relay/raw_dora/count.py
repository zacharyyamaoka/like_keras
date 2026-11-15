import time

from codec import to_arrow
from dora import Node
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


def main():
    node = Node()
    count = 0
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            count += 1
            state = JointState(name=("a",), position=[count], header=Header(stamp=time.time()))
            node.send_output("value", to_arrow(state))


if __name__ == "__main__":
    main()
