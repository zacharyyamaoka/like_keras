# import numpy as np
# import pinocchio as pin
# # from pin_utils import PinKinematics, PinModel

# # BUG Panda has 7 DOF and has 0 - inf redundant solutions, UR has 6 DOF and 0 - 8 possible solutions


# def run_kinematics_test(K: PinKinematics, n_tests=100, seed=42, atol=1e-1):

#     # check neutral configuration, as neural is seed of IK, should start at error = 0
#     fk_success, fk_sol = K.FK(pin.neutral(K.model))
#     ik_success, ik_sol = K.IK(fk_sol)
#     assert fk_success and ik_success

#     # BUG IK is not well tuned and doesn't converge always...
#     # It's fine for testing, but not for actual use right now!
#     # All I can say is that I am greatful for KDL and closed form IK... what a nightmare to have a optimzation loop in such a critical part of the robot....
#     np.random.seed(seed)  # Set seed for reproducibility

#     failure_count = 0
#     for i in range(n_tests):
#         q_random = K.pin_model.get_q_random()
#         fk_success, fk_sol = K.FK(q_random)
#         ik_success, ik_sol = K.IK(fk_sol)
#         fk_success_2, fk_sol_2 = K.FK(ik_sol)
#         # BUG sometimes ik will fail to converge so you cannot check success
#         # assert fk_success and ik_success and fk_success_2

#         if not np.allclose(fk_sol, fk_sol_2, atol=atol):
#             failure_count += 1

#     print(f"Failure count: {failure_count} / {n_tests}")
#     return failure_count

# def test_kin():

#     model = pin.buildSampleModelManipulator()
#     pin_model = PinModel(model)
#     K = PinKinematics(pin_model, "shoulder1_joint", "effector_body", verbose=False)
#     n_tests = 100
#     failure_count = run_kinematics_test(K, n_tests, seed=42, atol=1e-1)
#     assert failure_count < n_tests * 0.1


# def test_kin_different_links():
#     """ Should support any base_link and ik_tip_link combo

#         wrong! it will not support if base_link is not fixed joint to the world... beacuse it just computes the T once at the start...

#         pin.buildSampleModelManipulator()

#         [0] universe - FIXED_JOINT - 0
#         [1] shoulder1_joint - JOINT - 1
#         [2] shoulder1_body - BODY - 1
#         [3] shoulder2_joint - JOINT - 2
#         [4] shoulder2_body - BODY - 2
#         [5] shoulder3_joint - JOINT - 3
#         [6] shoulder3_body - BODY - 3
#         [7] upperarm_body - BODY - 3
#         [8] elbow_joint - JOINT - 4
#         [9] elbow_body - BODY - 4
#         [10] lowerarm_body - BODY - 4
#         [11] wrist1_joint - JOINT - 5
#         [12] wrist1_body - BODY - 5
#         [13] wrist2_joint - JOINT - 6
#         [14] wrist2_body - BODY - 6
#         [15] effector_body - BODY - 6
#     """

#     model = pin.buildSampleModelManipulator()
#     pin_model = PinModel(model)
#     n_tests = 100

#     K = PinKinematics(pin_model, "shoulder1_joint", "wrist2_body", verbose=False)
#     failure_count = run_kinematics_test(K, n_tests, seed=42, atol=1e-1)
#     assert failure_count < n_tests * 0.1

#     K = PinKinematics(pin_model, "shoulder2_body", "wrist1_joint", verbose=False)
#     failure_count = run_kinematics_test(K, n_tests, seed=42, atol=1e-1)
#     assert failure_count < n_tests * 0.1

# if __name__ == "__main__":
#     test_kin()
#     test_kin_different_links()
#     print("Success!")
    