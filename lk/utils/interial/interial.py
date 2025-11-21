#!/usr/bin/env python3

"""
Inertial Visualization Utilities

Computes equivalent inertia boxes and center-of-mass spheres for visualization,
matching the algorithm used in RViz (ros2/rviz#1527).

The equivalent inertia box represents the principal axes and moments of inertia
as a box with the same mass and inertia tensor.

The COM sphere represents a uniform ball of lead with the same mass as the link.
"""

# PYTHON
import numpy as np
from typing import Tuple, Optional


# Constants
LEAD_DENSITY = 11340.0  # kg/m³


def equivalent_inertia_box(
    mass: float,
    inertia_matrix: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute equivalent inertia box dimensions and orientation.

    Given mass and inertia tensor about the COM, computes the dimensions
    and orientation of a box with equivalent inertia properties.

    Uses the closed-form solution:
        Ix = 1/12 * m * (b² + c²)
        Iy = 1/12 * m * (a² + c²)
        Iz = 1/12 * m * (a² + b²)

    Solving for box dimensions:
        a² = 6/m * (Iy + Iz - Ix)
        b² = 6/m * (Ix + Iz - Iy)
        c² = 6/m * (Ix + Iy - Iz)

    Args:
        mass: Link mass in kg
        inertia_matrix: 3x3 inertia tensor about COM in kg·m²
                       Should be symmetric positive definite

    Returns:
        box_lengths: (3,) array [a, b, c] box side lengths in meters
        R_principal: (3, 3) rotation matrix of principal axes

    Raises:
        ValueError: If inertia is not physically valid (not SPD)
    """
    # Validate inputs
    if mass <= 0:
        raise ValueError(f"Mass must be positive, got {mass}")

    if inertia_matrix.shape != (3, 3):
        raise ValueError(f"Inertia matrix must be 3x3, got {inertia_matrix.shape}")

    # Check symmetry
    if not np.allclose(inertia_matrix, inertia_matrix.T):
        raise ValueError("Inertia matrix must be symmetric")

    # Eigen decomposition to get principal moments and axes
    eigenvalues, eigenvectors = np.linalg.eigh(inertia_matrix)

    # Check positive definiteness
    if np.any(eigenvalues <= 0):
        raise ValueError(
            f"Inertia matrix must be positive definite, got eigenvalues {eigenvalues}"
        )

    if np.linalg.det(inertia_matrix) <= 0:
        raise ValueError(
            f"Inertia matrix determinant must be positive, got {np.linalg.det(inertia_matrix)}"
        )

    # Extract principal moments
    lambda_x, lambda_y, lambda_z = eigenvalues

    # Compute box dimensions using closed-form solution
    # a² = 6/m * (λy + λz - λx)
    # b² = 6/m * (λx + λz - λy)
    # c² = 6/m * (λx + λy - λz)
    box_squared = np.array(
        [
            lambda_y + lambda_z - lambda_x,
            lambda_x + lambda_z - lambda_y,
            lambda_x + lambda_y - lambda_z,
        ]
    )

    # Check that all dimensions are positive
    if np.any(box_squared < 0):
        raise ValueError(
            f"Invalid inertia: box dimensions would be imaginary. "
            f"Got box_squared = {box_squared}"
        )

    box_lengths = np.sqrt(6.0 / mass * box_squared)

    # Principal axes orientation (eigenvectors form rotation matrix)
    R_principal = eigenvectors

    return box_lengths, R_principal


def com_sphere_radius(
    mass: float,
    density: float = LEAD_DENSITY,
) -> float:
    """
    Compute center-of-mass sphere radius.

    The sphere represents a uniform ball of the given density (default: lead)
    with the same mass as the link. This provides intuition about link density:
    if the sphere is as big as your model, your model is as dense as lead.

    Uses: mass = density * (4/3) * π * r³
    Solving for r: r = (3 * mass / (4 * π * density))^(1/3)

    Args:
        mass: Link mass in kg
        density: Reference density in kg/m³ (default: LEAD_DENSITY)

    Returns:
        radius: Sphere radius in meters

    Raises:
        ValueError: If mass or density are not positive
    """
    if mass <= 0:
        raise ValueError(f"Mass must be positive, got {mass}")

    if density <= 0:
        raise ValueError(f"Density must be positive, got {density}")

    radius = np.cbrt(3.0 * mass / (4.0 * np.pi * density))

    return radius


def inertia_visual_pose_in_link(
    mass: float,
    inertia_matrix: np.ndarray,
    com_pos_in_link: np.ndarray,
    com_rot_in_link: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Compute complete inertia visualization in link frame.

    Given URDF inertial properties, computes:
    - Equivalent inertia box dimensions, position, and orientation in link frame
    - COM sphere radius

    The inertia matrix should be expressed about the COM.
    The box will be centered at the COM position in the link frame.

    Args:
        mass: Link mass in kg
        inertia_matrix: 3x3 inertia tensor about COM in kg·m²
        com_pos_in_link: (3,) COM position in link frame [x, y, z] in meters
        com_rot_in_link: (3, 3) rotation from link frame to COM frame
                        If None, assumes identity (no rotation)

    Returns:
        box_lengths: (3,) box side lengths [a, b, c] in meters
        box_pos_in_link: (3,) box center position in link frame (same as COM)
        R_box_in_link: (3, 3) box orientation in link frame
        sphere_radius: COM sphere radius in meters
    """
    if com_rot_in_link is None:
        com_rot_in_link = np.eye(3)

    # Compute equivalent inertia box (in COM frame)
    box_lengths, R_principal = equivalent_inertia_box(mass, inertia_matrix)

    # Box is centered at COM, so position is just COM position
    box_pos_in_link = com_pos_in_link.copy()

    # Box orientation in link frame: first rotate to COM frame, then to principal axes
    R_box_in_link = com_rot_in_link @ R_principal

    # Compute COM sphere
    sphere_radius = com_sphere_radius(mass)

    return box_lengths, box_pos_in_link, R_box_in_link, sphere_radius


def inertia_from_urdf_params(
    ixx: float,
    iyy: float,
    izz: float,
    ixy: float = 0.0,
    ixz: float = 0.0,
    iyz: float = 0.0,
) -> np.ndarray:
    """
    Build 3x3 inertia matrix from URDF inertia parameters.

    Args:
        ixx, iyy, izz: Diagonal moments of inertia in kg·m²
        ixy, ixz, iyz: Off-diagonal products of inertia in kg·m²

    Returns:
        inertia_matrix: 3x3 symmetric inertia tensor
    """
    return np.array(
        [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ]
    )


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert roll-pitch-yaw angles to rotation matrix.

    Uses the convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    Args:
        roll: Rotation about x-axis in radians
        pitch: Rotation about y-axis in radians
        yaw: Rotation about z-axis in radians

    Returns:
        R: (3, 3) rotation matrix
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array(
        [
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ]
    )

    Ry = np.array(
        [
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ]
    )

    Rz = np.array(
        [
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ]
    )

    return Rz @ Ry @ Rx


if __name__ == "__main__":
    # Example: compute inertia box and COM sphere for a simple link

    # Example 1: Uniform solid box (1m x 0.5m x 0.3m, mass 10 kg)
    print("Example 1: Uniform solid box")
    print("-" * 50)

    mass = 10.0  # kg
    a, b, c = 1.0, 0.5, 0.3  # meters

    # Theoretical inertia for solid box about COM:
    # Ix = 1/12 * m * (b² + c²)
    # Iy = 1/12 * m * (a² + c²)
    # Iz = 1/12 * m * (a² + b²)
    Ixx = mass / 12.0 * (b**2 + c**2)
    Iyy = mass / 12.0 * (a**2 + c**2)
    Izz = mass / 12.0 * (a**2 + b**2)

    I = inertia_from_urdf_params(Ixx, Iyy, Izz)
    print(f"Mass: {mass} kg")
    print(f"Original box dimensions: [{a:.3f}, {b:.3f}, {c:.3f}] m")
    print(f"Inertia matrix:\n{I}")

    box_lengths, R = equivalent_inertia_box(mass, I)
    print(
        f"Computed box dimensions: [{box_lengths[0]:.3f}, {box_lengths[1]:.3f}, {box_lengths[2]:.3f}] m"
    )
    print(f"Principal axes rotation:\n{R}")
    print(f"(Should be identity for axis-aligned box)")

    sphere_r = com_sphere_radius(mass)
    print(f"COM sphere radius: {sphere_r:.4f} m")
    print(f"(Lead ball of this radius has mass {mass} kg)")
    print()

    # Example 2: Link with COM offset from link frame
    print("Example 2: Box with offset COM")
    print("-" * 50)

    com_pos = np.array([0.1, 0.2, 0.05])  # COM offset in link frame
    com_rpy = np.array([0.0, 0.0, np.pi / 6])  # 30° rotation about z
    com_rot = rotation_from_rpy(*com_rpy)

    box_lengths, box_pos, R_box, sphere_r = inertia_visual_pose_in_link(
        mass, I, com_pos, com_rot
    )

    print(f"COM position in link frame: {com_pos}")
    print(f"COM rotation (30° about z):\n{com_rot}")
    print(f"Box position in link frame: {box_pos}")
    print(f"Box orientation in link frame:\n{R_box}")
    print(f"Box dimensions: {box_lengths}")
    print(f"COM sphere radius: {sphere_r:.4f} m")
