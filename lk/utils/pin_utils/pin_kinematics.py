# #!/usr/bin/env python3

# # BAM
# from pin_utils.pin_model import PinModel

# # PYTHON
# import numpy as np
# import pinocchio as pin
# from typing import Optional, TYPE_CHECKING

# if TYPE_CHECKING:
#     from bam.descriptions import RobotDescription, RobotDescription
# """

# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html

#     Notice that this way to compute the damped pseudo-inverse was chosen mostly because of its simplicity of implementation.
#     It is not necessarily the best nor the fastest way, and using a fixed damping factor is not necessarily the best course of action.

# This is having problems with convergence!

# Ok Problem... for example robot data, its fine to use the world base frame. but more generally you may wanat to use other frames....

# One way is to assume that the pose_matrix will always be respect to the world frame, then you can just do

# Case 1: (Simplest)
# - Assume pose_matrix is always given for ik_joint_frame.
# - This is a bad idea! beacuse you want to define the pose_matrix for a meaningful frame (tool0, tcp, etc..)

# Case 2:
#  - Define pose_matrix for arbitrary ik_tip with respect to world frame
#  - T_base_link_to_ik_tip = T_world_ik_tip @ T_ik_tip_ik_joint

# Case 3: (Most general case, what I have implemented)
#  - Define pose_matrix for arbitrary ik_tip with respect to arbitrary base_link
# - T_base_link_to_ik_tip = T_world_base_link @ T_base_link_ik_tip @ T_ik_tip_ik_joint

# """


# class PinKinematics():

#     @classmethod
#     def from_robot_description(cls, robot_description: 'RobotDescription', verbose=False):
#         pin_model = PinRobotModel.from_robot_description(robot_description)

#         return cls(pin_model, robot_description.ik_tip, verbose)

#     def __init__(self, pin_model: PinModel, ik_tip_link: str, verbose=False):

#         self.pin_model = pin_model
#         # Create our own copy of the model and data internally, so we can modify it as needed
#         self.model = pin_model.model
#         self.data =self.model.createData()

#         self.ik_tip_link = ik_tip_link
#         self.ik_tip_idx = self.model.getFrameId(ik_tip_link)

#         self.verbose = verbose

#     def IK(self, T_base_link_to_ik_tip: np.ndarray, seed_q: Optional[np.ndarray] = None) -> tuple[bool, np.ndarray]:
#         """ Inverse Kinematics Notes:
#             There are two examples of Pinnochio Inverse kinematics:

#             1. is just for translation (xyz) - https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics-3d.py
#             2. The other that we will use is for full 6 dof (xyzrpy) - https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/inverse-kinematics.py

#             Even still this takes wayyyy to long to computer for 43K poses for reach analysis....

#         """

#         # BUG: pin doesn't accept raw numpy matrix, needs its own types
#         oMdes = pin.SE3(T_base_link_to_ik_tip[:3, :3], T_base_link_to_ik_tip[:3, 3])

#         # Set starting point for optimization
#         q = seed_q
#         if q is None:
#             q = pin.neutral(self.model)

#         eps = 1e-4
#         IT_MAX = 100
#         DT = 1e-1
#         damp = 1e-6

#         i = 0
#         while True:
#             pin.forwardKinematics(self.model, self.data, q) # set starting position
#             iMd = self.data.oMi[self.ik_tip_idx].actInv(oMdes) # actual T_base_link_to_ik_tip vs target T_base_link_to_ik_tip
#             err = pin.log(iMd).vector  # calculate error (in joint frame)
#             if np.linalg.norm(err) < eps:
#                 success = True
#                 break
#             if i >= IT_MAX:
#                 success = False
#                 break

#             # Servo joints to decrease error
#             J = pin.computeJointJacobian(self.model, self.data, q, self.ik_tip_idx)  # in joint frame
#             J = -np.dot(pin.Jlog6(iMd.inverse()), J)
#             v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
#             q = pin.integrate(self.model, q, v * DT)


#             i += 1

#         if self.verbose:
#             if success:
#                 print("Convergence achieved!")
#             else:
#                 print(
#                     "\n"
#                     "Warning: the iterative algorithm has not reached convergence "
#                     "to the desired precision"
#                 )

#             print(f"\nresult: {q.flatten().tolist()}")
#             print(f"\nfinal error: {err.T}")

#         return success, q


#     def FK(self, q: np.ndarray) -> tuple[bool, np.ndarray]:

#         pin.forwardKinematics(self.model, self.data, q) # set starting position
#         # pin.updateFramePlacements(self.model, self.data) # no need as you read from the joint frames, and then transform with static T
#         T_base_link_to_ik_tip = self.data.oMi[self.ik_tip_idx]
#         return True, T_base_link_to_ik_tip


# if __name__ == "__main__":

#     import example_robot_data

#     # BUG Panda has many possible solutions!!!
#     # robot = example_robot_data.load("panda")
#     # pin_model = PinModel(robot.model, robot.collision_model, robot.visual_model)
#     # K = PinKinematics(pin_model, "panda_link0", "panda_link7", verbose=True)

#     # BUG UR also has many solutions actually... 8
#     robot = example_robot_data.load("ur5")
#     pin_model = PinModel(robot.model, robot.collision_model, robot.visual_model)
#     # K = PinKinematics(pin_model, "base_link", "tool0", verbose=True)
#     # K = PinKinematics(pin_model, "base_link", "ee_link", verbose=True)
#     # K = PinKinematics(pin_model, "base_link", "wrist3_link", verbose=True)
#     K = PinKinematics(pin_model, "world", "tool0", verbose=True)

#     success, pose_matrix = K.FK(pin.neutral(K.model))
#     print(pose_matrix)
#     success, joint_positions = K.IK(pose_matrix)
#     print(success, joint_positions)
#     print("")

#     q_random = pin_model.get_q_random()
#     success, fk_sol = K.FK(q_random)
#     success, ik_sol = K.IK(fk_sol)
#     success, fk_sol_2 = K.FK(ik_sol)

#     print(q_random)
#     print(fk_sol)
#     print(ik_sol)
#     print(fk_sol_2)
#     print(np.round(fk_sol_2 - fk_sol, 3))
#     assert np.allclose(fk_sol, fk_sol_2, atol=1e-1)
