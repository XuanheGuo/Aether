"""Synthetic convex torque constraints and first-order actuator dynamics."""
from dataclasses import dataclass

import numpy as np

from ..types import scalar, vector


@dataclass(frozen=True)
class BoxLimit:
    """Symmetric box |tau_i|<=limit_i N m, Euclidean projection by clipping."""
    limits: np.ndarray

    def __post_init__(self):
        limits = vector(self.limits, (3,), 'limits')
        if np.any(limits <= 0):
            raise ValueError('box limits must be positive')
        object.__setattr__(self, 'limits', limits)

    def project(self, requested):
        """Return projected body torque, N m."""
        return np.clip(vector(requested, (3,), 'requested'), -self.limits, self.limits)

    def contains(self, torque, *, atol=1e-12):
        """Check box membership with absolute N m tolerance."""
        return bool(np.all(np.abs(vector(torque, (3,), 'torque')) <= self.limits + atol))


@dataclass(frozen=True)
class BallLimit:
    """Euclidean ball ||tau||<=radius N m, radial Euclidean projection."""
    radius: float

    def __post_init__(self):
        radius = scalar(self.radius, 'radius')
        if radius <= 0:
            raise ValueError('radius must be positive')
        object.__setattr__(self, 'radius', radius)

    def project(self, requested):
        """Return projected body torque, N m."""
        tau = vector(requested, (3,), 'requested')
        norm = np.linalg.norm(tau)
        if not np.isfinite(norm):
            raise ValueError('requested torque norm must be finite')
        return tau if norm <= self.radius else tau*(self.radius/norm)

    def contains(self, torque, *, atol=1e-12):
        """Check Euclidean ball membership with absolute N m tolerance."""
        return bool(np.linalg.norm(vector(torque, (3,), 'torque')) <= self.radius + atol)


@dataclass(frozen=True)
class FirstOrderActuator:
    """Diagonal time constants seconds; actual torque is an external plant state."""
    time_constants_s: np.ndarray

    def __post_init__(self):
        constants = vector(self.time_constants_s, (3,), 'time_constants_s')
        if np.any(constants <= 0):
            raise ValueError('time constants must be positive')
        object.__setattr__(self, 'time_constants_s', constants)

    def derivative(self, actual, projected):
        """Return d(actual torque)/dt for held projected target, N m/s."""
        return (vector(projected, (3,), 'projected') - vector(actual, (3,), 'actual')) / self.time_constants_s
