"""Hamilton wxyz active rotations from body FLU to world ENU."""
import numpy as np

from .types import Array, vector


def quaternion_product(a: Array, b: Array) -> Array:
    """Hamilton product of finite wxyz quaternions, shape (4,)."""
    a = vector(a, (4,), "a")
    b = vector(b, (4,), "b")
    return np.r_[a[0]*b[0] - np.dot(a[1:], b[1:]),
                 a[0]*b[1:] + b[0]*a[1:] + np.cross(a[1:], b[1:])]


def rotation_matrix(q_WB: Array) -> Array:
    """Return active W←B rotation matrix for unit Hamilton wxyz quaternion."""
    q = vector(q_WB, (4,), "q_WB")
    if not np.isclose(np.linalg.norm(q), 1, rtol=0, atol=1e-10):
        raise ValueError("q_WB must be unit length")
    return _rotation_matrix_raw(q)


def _rotation_matrix_raw(q: Array) -> Array:
    # Polynomial extension to unnormalized ODE stage states; no stage normalization.
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def attitude_distance(q1: Array, q2: Array) -> float:
    """Antipodal-invariant angular distance in radians between unit wxyz quaternions."""
    rotation_matrix(q1)
    rotation_matrix(q2)
    return float(2*np.arccos(np.clip(abs(np.dot(q1, q2)), 0, 1)))
