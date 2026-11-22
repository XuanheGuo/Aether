"""Continuous 6-DOF equations in ENU/FLU, SI units."""
import numpy as np

from .frames import _rotation_matrix_raw, quaternion_product
from .types import Array, BodyWrench, RigidBodyParams, scalar, vector


def dxdt(t_s: float, x: Array, params: RigidBodyParams, wrench: BodyWrench) -> Array:
    """Derivative of [p_W, v_W, q_WB, omega_B], shape (13,), for body wrench.

    t_s is finite nonnegative seconds. Internal q is not normalized: callers
    must supply a valid nondegenerate stage quaternion and normalize accepted steps.
    """
    if scalar(t_s, "t_s") < 0:
        raise ValueError("t_s must be nonnegative")
    x = vector(x, (13,), "x")
    q, omega = x[6:10], x[10:13]
    if np.linalg.norm(q) < 1e-12:
        raise ValueError("q_WB norm must be >= 1e-12")
    j = params.J_B_kg_m2
    acceleration = _rotation_matrix_raw(q) @ wrench.force_B_N / params.mass_kg
    acceleration = acceleration + np.array([0., 0., -params.gravity_m_s2])
    qdot = 0.5 * quaternion_product(q, np.r_[0., omega])
    alpha = np.linalg.solve(j, wrench.torque_B_Nm - np.cross(omega, j @ omega))
    return np.r_[x[3:6], acceleration, qdot, alpha]
