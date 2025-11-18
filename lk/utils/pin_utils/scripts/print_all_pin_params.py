import pinocchio as pin
import numpy as np


# Build the model
model = pin.Model()
joint_id = 0
parent_id = 0

# Link lengths and masses
l1, l2 = 1.0, 1.0  # meters
m1, m2, payload = 1.0, 1.0, 1.0  # kg

# Inertia (assuming point masses at the link ends)
interia_l1 = pin.Inertia(m1, np.array([l1/2, 0., 0.]), np.diag([0., m1 * (l1**2) / 12, m1 * (l1**2) / 12]))

interia_point_mass = pin.Inertia(payload, np.array([l1, 0., 0.]), np.zeros((3, 3)))

interia_l1 = interia_l1 + interia_point_mass

# Joint 1 (rotates around Z at origin)
joint_placement1 = pin.SE3.Identity()
joint_id1 = model.addJoint(parent_id, pin.JointModelRZ(), joint_placement1, "joint1")
model.appendBodyToJoint(joint_id1, interia_l1, pin.SE3.Identity())

# Add link 1
X1 = pin.SE3(np.eye(3), np.array([l1, 0., 0.]))
parent_id = joint_id1

# Joint 2 (rotates around Z at end of link 1)
# joint_id2 = model.addJoint(parent_id, pin.JointModelRZ(), X1, "joint2")
# model.appendBodyToJoint(joint_id2, inertia_l2, pin.SE3.Identity())


# Create data
data = model.createData()

print(30*("-"))
print("MODEL")
for name, function in model.__class__.__dict__.items():
    print(' **** %s: %s' % (name, function.__doc__))
print(30*("-"))
print("\n")
print(30*("-"))
print("DATA")
for name, function in data.__class__.__dict__.items():
    print(' **** %s: %s' % (name, function.__doc__))
print(30*("-"))

# Sample state
q = np.array([np.pi/2])    # joint angles
v = np.array([0.5])            # joint velocities
a = np.array([0.1])            # joint accelerations

# Compute Mass Matrix
M = pin.crba(model, data, q)

# Compute Coriolis and centrifugal vector
C = pin.nonLinearEffects(model, data, q, v) - pin.computeGeneralizedGravity(model, data, q)

# Compute gravity vector
g = pin.computeGeneralizedGravity(model, data, q)

# Compute inverse dynamics (torques)
tau = pin.rnea(model, data, q, v, a)

print(30*("-"))
print("MODEL")
print(model)
print(30*("-"))
print("\n")
print(30*("-"))
print("DATA")
print(data)
print(30*("-"))

# Print results
print("Mass matrix M(q):\n", M)
print("Coriolis and centrifugal vector C(q,v):\n", C)
print("Gravity vector g(q):\n", g)
print("Inverse dynamics torques tau:\n", tau)


# Forward kinematics
pin.forwardKinematics(model, data, q, v, a)
oM1 = data.oMi[joint_id1]
# oM2 = data.oMi[joint_id2]

