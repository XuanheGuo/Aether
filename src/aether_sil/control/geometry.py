"""SO(3) errors for active W←B rotations."""
import numpy as np
from scipy.spatial.transform import Rotation

from ..frames import rotation_matrix


def attitude_errors(q_WB, q_WD, omega_B, omega_D):
    """Return e_R, e_omega, Psi and geodesic radians; omega_D is in D."""
    r, rd = rotation_matrix(q_WB), rotation_matrix(q_WD)
    re = rd.T @ r
    skew = (re-re.T)/2
    e_r = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    e_w = np.asarray(omega_B) - r.T @ rd @ np.asarray(omega_D)
    psi = float(np.clip((3-np.trace(re))/2, 0, 2))
    theta = float(Rotation.from_matrix(re).magnitude())
    return e_r, e_w, psi, theta


def local_rotation_vector(q_WB, q_WD):
    """Principal log(R_Dᵀ R_B), rad in D; undefined at the pi branch cut."""
    re = rotation_matrix(q_WD).T @ rotation_matrix(q_WB)
    return Rotation.from_matrix(re).as_rotvec()
