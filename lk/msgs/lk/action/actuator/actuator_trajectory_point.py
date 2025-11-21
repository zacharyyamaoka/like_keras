from dataclasses import dataclass

from bam.msgs.ros_msgs import JointTrajectoryPoint

"""
We will borrow the ROS msg abstractions.
- If you build standard ROS msgs, they cannot depend on anything else
- Alot of experince built into thoose msgs (PoseStamped, Path, etc..)

(Dependency Flow)
ros_msgs -> bam.msgs -> moteus_msgs, etc...

Q: Should I add tolerances here? or have another msg for that? 
A: hmm.. Right now I want to make another message beacuse Naming wise then I need a ActuatorTrajectoryAction, it starts to get a bit redundant...

If you have just an ActuatorTrajectory, that reminds me of essentially just the JointTrajectory....

Ok I can just borrow the abstractions from moveit and ros2 control, no need to reinvient the wheel here. I would rather follow the abstraction and use a longer name than compress them

It feels a bit silly to just have this very thin layer here...
Is there really something that unique to all of these things? not really I think is the issue. Each Actuator is so unique...

"""


@dataclass
class ActuatorTrajectoryPoint(JointTrajectoryPoint): ...
