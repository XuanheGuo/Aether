"""Sampled attitude control interfaces, in S0 SI and W←B conventions."""
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..types import Array, RigidBodyState, scalar, vector


def _time(t_s):
    t = scalar(t_s, 't_s')
    if t < 0:
        raise ValueError('t_s must be nonnegative')
    return t


def _quaternion(q):
    q = vector(q, (4,), 'q_WB')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('quaternion norm must be finite and >= 1e-12')
    return vector(q/norm, (4,), 'q_WB')


@dataclass(frozen=True)
class Reference:
    """Timestamp seconds; desired Hamilton wxyz q_WD and desired-frame rate rad/s."""
    t_s: float
    q_WD: Array
    omega_D: Array

    def __post_init__(self):
        object.__setattr__(self, 't_s', _time(self.t_s))
        object.__setattr__(self, 'q_WD', _quaternion(self.q_WD))
        object.__setattr__(self, 'omega_D', vector(self.omega_D, (3,), 'omega_D'))


@dataclass(frozen=True)
class Measurement:
    """Timestamp seconds, body-to-world q_WB, body rate rad/s; ideal in this batch."""
    t_s: float
    q_WB: Array
    omega_B: Array

    def __post_init__(self):
        object.__setattr__(self, 't_s', _time(self.t_s))
        object.__setattr__(self, 'q_WB', _quaternion(self.q_WB))
        object.__setattr__(self, 'omega_B', vector(self.omega_B, (3,), 'omega_B'))


@dataclass(frozen=True)
class TorqueCommand:
    """Timestamped requested body COM torque, N m; not an actuator command."""
    t_s: float
    torque_B_Nm: Array

    def __post_init__(self):
        object.__setattr__(self, 't_s', _time(self.t_s))
        object.__setattr__(self, 'torque_B_Nm', vector(self.torque_B_Nm, (3,), 'torque_B_Nm'))


class AttitudeReference(Protocol):
    def sample(self, t_s: float) -> Reference: ...


class MeasurementModel(Protocol):
    def sample(self, t_s: float, true_state: RigidBodyState,
               rng: np.random.Generator) -> Measurement: ...


class AttitudeController(Protocol):
    def reset(self, *, seed: int) -> None: ...
    def update(self, t_s: float, measurement: Measurement, reference: Reference,
               dt_control_s: float) -> TorqueCommand: ...


@dataclass(frozen=True)
class ConstantReference:
    """Fixed desired attitude; desired angular rate is zero."""
    q_WD: Array

    def __post_init__(self):
        object.__setattr__(self, 'q_WD', _quaternion(self.q_WD))

    def sample(self, t_s: float) -> Reference:
        return Reference(t_s, self.q_WD, [0, 0, 0])


class IdealMeasurement:
    """Perfect, instantaneous attitude and body-rate observation."""
    def sample(self, t_s: float, true_state: RigidBodyState,
               rng: np.random.Generator) -> Measurement:
        return Measurement(t_s, true_state.q_WB, true_state.omega_B)
