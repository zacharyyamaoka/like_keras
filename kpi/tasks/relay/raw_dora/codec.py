import numpy as np
import pyarrow as pa
from robot_msgs.sensor import JointState
from robot_msgs.std import Header


def to_arrow(state):
    return pa.array([{"name": list(state.name), "position": state.position.tolist(), "stamp": state.header.stamp}])


def from_arrow(array):
    message = array.to_pylist()[0]
    return JointState(name=tuple(message["name"]), position=np.array(message["position"]),
                      header=Header(stamp=message["stamp"]))
