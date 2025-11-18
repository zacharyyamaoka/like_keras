from dataclasses import dataclass, field, asdict
import numpy as np

from typing import Any, Literal

from tf_transformations import cartesian_path_analysis, CartesianPathMetrics

@dataclass
class NumpyTrajectory:

    frame_id: str = ""
    header_stamp: float = 0.0
    joint_names: list[str] = None

    t: np.ndarray = None
    q: np.ndarray = None
    qd: np.ndarray = None
    qdd: np.ndarray = None
    tau: np.ndarray = None

    kp: np.ndarray = None
    kd: np.ndarray = None
    max_tau: np.ndarray = None

    q_waypoint: np.ndarray = None
    t_waypoint: np.ndarray = None
    waypoint_name: list[str] = None

    t_path: np.ndarray = None # [t] timestamps for path data (may be sampled if density < 1.0)

    ik_tip_path: np.ndarray = None # [t, 4, 4] T_world_to_ik_tip
    target_link_path: np.ndarray = None # [t, 4, 4] T_world_to_target_link
    
    # Path completion metrics (normalized to [0, 1])
    # Multi-dimensional format: columns represent different dimensions (XYZ, RPY, Vec)
    path: np.ndarray = None # [t, D] normalized cumulative distances [0, 1], D = 2 or 3
    path_d: np.ndarray = None # [t, D] velocity
    path_dd: np.ndarray = None # [t, D] acceleration
    dimension_names: list[str] = None # Names of dimensions e.g., ["XYZ", "RPY", "Vec[1,0,0]"]

    def get_waypoint_time(self, name: str) -> float:
        return self.t_waypoint[self.waypoint_name.index(name)]

    def fill(self, other: Any) -> None:
        """
        Copy all attributes from another NumpyTrajectory instance into self
        for any attributes that exist on both.
        """
        for attr in self.__dataclass_fields__:
            if hasattr(other, attr):
                setattr(self, attr, getattr(other, attr))

    def inspect(self):
        print(f"\n{self.__class__.__name__} Inspection:")
        print(f"  frame_id      : {self.frame_id}")
        print(f"  header_stamp  : {self.header_stamp}")
        print(f"  joint_names   : {len(self.joint_names) if self.joint_names is not None else None}")
        print(f"  t             : {self.t.shape if self.t is not None else None}")
        print(f"  q             : {self.q.shape if self.q is not None else None}")
        print(f"  qd            : {self.qd.shape if self.qd is not None else None}")
        print(f"  qdd           : {self.qdd.shape if self.qdd is not None else None}")
        print(f"  tau           : {self.tau.shape if self.tau is not None else None}")
        print(f"  kp            : {self.kp.shape if self.kp is not None else None}")
        print(f"  kd            : {self.kd.shape if self.kd is not None else None}")
        print(f"  max_tau       : {self.max_tau.shape if self.max_tau is not None else None}")
        print(f"  q_waypoint    : {self.q_waypoint.shape if self.q_waypoint is not None else None}")
        print(f"  t_waypoint    : {self.t_waypoint.shape if self.t_waypoint is not None else None}")
        print(f"  ik_tip_path   : {self.ik_tip_path.shape if self.ik_tip_path is not None else None}")
        print(f"  target_link_path   : {self.target_link_path.shape if self.target_link_path is not None else None}")
        print(f"  t_path        : {self.t_path.shape if self.t_path is not None else None}")
        print(f"  path          : {self.path.shape if self.path is not None else None}")
        print(f"  path_d        : {self.path_d.shape if self.path_d is not None else None}")
        print(f"  path_dd       : {self.path_dd.shape if self.path_dd is not None else None}")
        print(f"  dimension_names : {self.dimension_names}")

    def is_comparable(self, other: 'NumpyTrajectory', verbose: bool = False) -> bool:
        """
            Check if two NumpyTrajectories are comparable (same length and dimensions).
            
            Checks that t, q, qd, and qdd (if present) have matching shapes.
            Returns False if shapes don't match or required fields are missing.
            
            DescriptionArgs:
                other: The other NumpyTrajectory to compare with
                verbose: If True, print detailed inspection when not comparable
        """
        def fail(message: str) -> bool:
            if verbose:
                print(f"Trajectories are not comparable: {message}")
                print("\nReference trajectory:")
                self.inspect()
                print("\nMeasured trajectory:")
                other.inspect()
            return False
        
        # Check that both have time and position data
        if self.t is None or other.t is None:
            return fail("missing time (t) data")
        if self.q is None or other.q is None:
            return fail("missing position (q) data")
        
        # Check time arrays have same length
        if len(self.t) != len(other.t):
            return fail(f"different time lengths ({len(self.t)} vs {len(other.t)})")
        
        # Check position arrays have same shape
        if self.q.shape != other.q.shape:
            return fail(f"different position shapes ({self.q.shape} vs {other.q.shape})")
        
        # Check velocity if both have it
        if (self.qd is not None) and (other.qd is not None):
            if self.qd.shape != other.qd.shape:
                return fail(f"different velocity shapes ({self.qd.shape} vs {other.qd.shape})")
        
        # Check acceleration if both have it
        if (self.qdd is not None) and (other.qdd is not None):
            if self.qdd.shape != other.qdd.shape:
                return fail(f"different acceleration shapes ({self.qdd.shape} vs {other.qdd.shape})")
        
        return True

    def save(self, filepath: str):
        """Save NumpyTrajectory to npz file."""
        data_dict = asdict(self)
        
        # Filter out None values and convert joint_names to object array
        data = {}
        for key, value in data_dict.items():
            if value is not None:
                if key == 'joint_names':
                    data[key] = np.array(value, dtype=object)
                else:
                    data[key] = value
        
        np.savez(filepath, **data)

    @classmethod
    def load(cls, filepath: str) -> 'NumpyTrajectory':
        """Load NumpyTrajectory from npz file."""
        data = np.load(filepath, allow_pickle=True)
        
        # Convert numpy arrays and handle special types
        kwargs = {}
        for key in data.keys():
            if key == 'joint_names':
                kwargs[key] = list(data[key])
            elif key in ['frame_id']:
                kwargs[key] = str(data[key])
            elif key in ['header_stamp']:
                kwargs[key] = float(data[key])
            else:
                kwargs[key] = data[key]
        
        return cls(**kwargs)

    def compute_cartesian_path(self, link_name: Literal["ik_tip", "target_link"] = "ik_tip", vec_dirs: list[np.ndarray] = None, in_place = False) -> CartesianPathMetrics:

        assert self.t_path is not None, "Time array (t_path) is required to compute cartesian path, please make sure to call add_fk_path() first"

        if vec_dirs is None:
            vec_directions = [
                np.array([1.0, 0.0, 0.0]),  # X-axis
                np.array([0.0, 1.0, 0.0]),  # Y-axis
                np.array([0.0, 0.0, 1.0]),  # Z-axis
            ]
        else:
            vec_directions = vec_dirs

        print(f"Analyzing {link_name} path along vec_directions:")
        for i, vec in enumerate(vec_directions):
            print(f"  [{i}]: {vec}")

        if link_name == "ik_tip":
            path = self.ik_tip_path
        elif link_name == "target_link":
            path = self.target_link_path
        else:
            raise ValueError(f"Invalid link name: {link_name}")

        metrics = cartesian_path_analysis(path, self.t_path, vec_directions)

        if in_place:
            self.path = metrics.path
            self.path_d = metrics.path_d
            self.path_dd = metrics.path_dd
            self.dimension_names = metrics.dimension_names
        else:
            return metrics
        return metrics