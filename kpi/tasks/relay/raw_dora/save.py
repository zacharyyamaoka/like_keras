import sys

from codec import from_arrow
from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "value":
            value = from_arrow(event["value"])
            with open(sys.argv[1], "a") as file:
                file.write(f"{value.position[0]}\n")


if __name__ == "__main__":
    main()
