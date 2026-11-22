"""Synthetic torque membership and translation balance; not EDF allocation."""

from ..frames import rotation_matrix
from ..types import scalar, vector


def hover_residual(q_WB, force_B_N, mass_kg, gravity_m_s2=9.80665):
    """Return world force residual N: R_WB F_B + m g_W."""
    mass, g = scalar(mass_kg, 'mass_kg'), scalar(gravity_m_s2, 'gravity_m_s2')
    if mass <= 0 or g < 0:
        raise ValueError('mass must be positive and gravity nonnegative')
    return rotation_matrix(q_WB) @ vector(force_B_N, (3,), 'force_B_N') + [0, 0, -mass*g]
