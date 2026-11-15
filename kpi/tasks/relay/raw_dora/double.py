from codec import from_arrow, to_arrow
from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "value":
            value = from_arrow(event["value"])
            node.send_output("doubled", to_arrow(value.replace(position=2 * value.position)))


if __name__ == "__main__":
    main()
