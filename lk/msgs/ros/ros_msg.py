import numpy as np
import copy


class RosMsg:
    def to_ros_msg(self):
        """Convert this py msg to the corresponding ROS2 message instance."""
        # Try to import the ROS2 message class based on this class's name and module
        # E.g., ros_py_msgs.std_msgs.Header -> std_msgs.msg.Header
        py_class = self.__class__
        py_mod = py_class.__module__
        py_name = py_class.__name__
        # Extract ROS package and message name
        # py_mod: ros_py_msgs.std_msgs.Header -> std_msgs.std_msgs.Header
        try:
            parts = py_mod.split('.')
            if len(parts) < 2:
                raise ImportError("Module path too short for ROS msg conversion")
            ros_pkg = parts[1]
            ros_msg_mod = f"{ros_pkg}.msg"
            ros_msg_name = py_name
            ros_msg_module = __import__(ros_msg_mod, fromlist=[ros_msg_name])
            ros_msg_class = getattr(ros_msg_module, ros_msg_name)
        except Exception as e:
            raise ImportError(f"Could not import ROS2 message class for {py_mod}.{py_name}: {e}")

        # Recursively convert fields
        ros_msg = ros_msg_class()
        for field in getattr(self, '__dataclass_fields__', {}):
            value = getattr(self, field)
            # If the value is a GenericRosPyMsg, recursively convert
            if isinstance(value, RosMsg):
                setattr(ros_msg, field, value.to_ros_msg())
            # If it's a list of GenericRosPyMsg, convert each
            elif isinstance(value, list) and value and isinstance(value[0], RosMsg):
                setattr(ros_msg, field, [v.to_ros_msg() for v in value])
            else:
                setattr(ros_msg, field, value)
        return ros_msg

    @classmethod
    def random(cls) -> 'RosMsg':
        ...

    @classmethod
    def random_list(cls, n: int) -> list['RosMsg']: # type: ignore
        return [cls.random() for _ in range(n)]

    @classmethod
    def from_list(cls, list: list[float]) -> 'RosMsg':
        ...

    def to_list(self) -> list[float]:
        ...

    def to_numpy(self) -> np.ndarray:
        return np.array(self.to_list())

    @classmethod
    def from_numpy(cls, numpy: np.ndarray) -> 'RosMsg':
        return cls.from_list(numpy.tolist())


    def copy(self) -> 'RosMsg':
        return copy.deepcopy(self)