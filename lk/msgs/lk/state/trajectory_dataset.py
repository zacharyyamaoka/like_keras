

# # TODO change this to use centered differences, like the trapedzoidal calculation!
# # Mabye it actually makes sense to do the same calculation that Pilz is doing internally to validate...
# def path_to_traj_dataset(path: List[PoseStamped]) -> TrajectoryDataset:
#     dataset = TrajectoryDataset()
#     dataset.joint_names = ['translation', 'rotation']
#     n_points = len(path)
#     trans_point = JointTrajectoryPoint()   
#     rot_point = JointTrajectoryPoint()


#     # Precompute total path length
#     total_distance = 0.0
#     accumlated_distance = [0.0]
#     for i in range(1, n_points):
#         p1 = path[i-1].pose.position
#         p2 = path[i].pose.position
#         dx = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
#         dist = np.linalg.norm(dx)
#         total_distance += dist
#         accumlated_distance.append(total_distance)


#     for i in range(n_points):
#         # Time
#         dataset.time_from_start.append(path[i].header.stamp)

#         if i == 0:
#             trans_point.positions.append(0.0)
#             trans_point.velocities.append(0.0)
#             trans_point.accelerations.append(0.0)

#             rot_point.velocities.append(0.0)
#             rot_point.accelerations.append(0.0)
            
#             continue

#         # Δt
#         t1 = path[i-1].header.stamp
#         t2 = path[i].header.stamp
#         dt = (t2.sec - t1.sec) + (t2.nanosec - t1.nanosec) * 1e-9
#         dt = max(dt, 1e-6)

#         # Translation Vel.
#         p1 = path[i-1].pose.position
#         p2 = path[i].pose.position
#         dx = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
#         dist = np.linalg.norm(dx)
#         path_fraction = accumlated_distance[i] / total_distance if total_distance > 0 else 0.0
#         trans_point.positions.append(path_fraction)

#         v =  dist / dt
#         trans_point.velocities.append(v)

#         # Translation Accel.
#         # May be slightly wrong as you assume starting velocity is zero.
#         dv = trans_point.velocities[-1] - trans_point.velocities[-2] # trans_point.velocities[-1] = v, it was just added
#         a = dv / dt

#         trans_point.accelerations.append(a)

#         # Rotational Vel.
#         q1 = path[i-1].pose.orientation
#         q2 = path[i].pose.orientation
#         ax, angle = axangle_diff(q1, q2)
#         w = angle / dt  # angular velocity
#         rot_point.velocities.append(w)

#         # Rotational Accel.
#         # if i > 1:
#         dw = w - rot_point.velocities[-2]
#         alpha = dw / dt

#         rot_point.accelerations.append(alpha)
    
#     dataset.data = [trans_point, rot_point]
#     return dataset
    