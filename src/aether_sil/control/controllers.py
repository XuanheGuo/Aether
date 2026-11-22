"""Fixed-reference geometric PD and local CARE LQR in body torque N m."""
from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from ..types import RigidBodyParams, scalar, vector
from .geometry import attitude_errors, local_rotation_vector
from .interfaces import TorqueCommand


def _validate_tick(t_s, measurement, reference, dt_control_s):
    dt = scalar(dt_control_s, 'dt_control_s')
    t = scalar(t_s, 't_s')
    if t < 0 or dt <= 0 or measurement.t_s != t or reference.t_s != t:
        raise ValueError('invalid control time, sample timestamps or control period')
    if np.any(reference.omega_D != 0):
        raise ValueError('these controllers support fixed-attitude regulation only')


@dataclass(frozen=True)
class GeometricPD:
    """B0: scalar positive gains and exact model inertia; no torque limits."""
    params: RigidBodyParams
    k_R: float
    k_omega: float

    def __post_init__(self):
        for name in ('k_R', 'k_omega'):
            if scalar(getattr(self, name), name) <= 0:
                raise ValueError(f'{name} must be positive')

    def reset(self, *, seed: int):
        """Stateless controller; seed is reserved for reproducible interface."""

    def update(self, t_s, measurement, reference, dt_control_s):
        _validate_tick(t_s, measurement, reference, dt_control_s)
        e_r, e_w, _, _ = attitude_errors(measurement.q_WB, reference.q_WD,
                                         measurement.omega_B, reference.omega_D)
        w = measurement.omega_B
        tau = -self.k_R*e_r - self.k_omega*e_w + np.cross(w, self.params.J_B_kg_m2 @ w)
        return TorqueCommand(t_s, tau)


class LocalLQR:
    """B1 continuous CARE, x=[log(RdᵀR), omega_B], valid only near equilibrium.

    The chart mixes desired-frame rotation vector and body rate; at equilibrium
    these frames coincide to first order. This implementation refuses angles above
    max_angle_rad and has no nonlinear compensation or torque saturation.
    """
    def __init__(self, params, Q, R, max_angle_rad=0.5):
        self.params = params
        self.Q = vector(Q, (6, 6), 'Q')
        self.R = vector(R, (3, 3), 'R')
        self.max_angle_rad = scalar(max_angle_rad, 'max_angle_rad')
        if not 0 < self.max_angle_rad < np.pi:
            raise ValueError('max_angle_rad must lie in (0, pi)')
        if not np.allclose(self.Q, self.Q.T, atol=1e-12, rtol=0) or np.min(np.linalg.eigvalsh(self.Q)) < -1e-12:
            raise ValueError('Q must be symmetric positive semidefinite')
        if not np.allclose(self.R, self.R.T, atol=1e-12, rtol=0):
            raise ValueError('R must be symmetric positive definite')
        try:
            np.linalg.cholesky(self.R)
        except np.linalg.LinAlgError as exc:
            raise ValueError('R must be symmetric positive definite') from exc
        self.A = np.block([[np.zeros((3, 3)), np.eye(3)], [np.zeros((3, 6))]])
        self.B = np.vstack([np.zeros((3, 3)), np.linalg.solve(params.J_B_kg_m2, np.eye(3))])
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.solve(self.R, self.B.T @ self.P)
        if np.max(np.real(np.linalg.eigvals(self.A-self.B @ self.K))) >= 0:
            raise ValueError('CARE closed loop is not Hurwitz')

    def reset(self, *, seed: int):
        """Stateless controller; seed is reserved for reproducible interface."""

    def update(self, t_s, measurement, reference, dt_control_s):
        _validate_tick(t_s, measurement, reference, dt_control_s)
        error = local_rotation_vector(measurement.q_WB, reference.q_WD)
        if np.linalg.norm(error) > self.max_angle_rad:
            raise ValueError('attitude exceeds the declared local LQR domain')
        return TorqueCommand(t_s, -self.K @ np.r_[error, measurement.omega_B])
