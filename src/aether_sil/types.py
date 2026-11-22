"""Validated SI-valued rigid-body data and extension interfaces."""
from dataclasses import dataclass
from typing import Mapping, Protocol

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]


def vector(value: object, shape: tuple[int, ...], name: str) -> Array:
    """Copy a finite array of the required shape as immutable float64."""
    a = np.array(value, dtype=np.float64, copy=True)
    if a.shape != shape or not np.all(np.isfinite(a)):
        raise ValueError(f"{name} must be finite with shape {shape}")
    a.setflags(write=False)
    return a


def scalar(value: float, name: str) -> float:
    """Validate a finite scalar."""
    a = np.asarray(value, dtype=np.float64)
    if a.shape != () or not np.isfinite(a):
        raise ValueError(f"{name} must be a finite scalar")
    return float(a)


@dataclass(frozen=True)
class RigidBodyState:
    """Position m and velocity m/s in W; Hamilton q_WB; angular rate rad/s in B.

    Nonzero input quaternions are normalized at construction. ODE intermediate
    states use a separate raw representation and are not normalized.
    """
    p_W: Array
    v_W: Array
    q_WB: Array
    omega_B: Array

    def __post_init__(self) -> None:
        for name, shape in (("p_W", (3,)), ("v_W", (3,)), ("omega_B", (3,))):
            object.__setattr__(self, name, vector(getattr(self, name), shape, name))
        q = vector(self.q_WB, (4,), "q_WB")
        norm = np.linalg.norm(q)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("q_WB norm must be finite and >= 1e-12")
        object.__setattr__(self, "q_WB", vector(q / norm, (4,), "q_WB"))


@dataclass(frozen=True)
class RigidBodyParams:
    """Constant mass kg, symmetric positive definite inertia kg m² in B, gravity m/s²."""
    mass_kg: float
    J_B_kg_m2: Array
    gravity_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        mass = scalar(self.mass_kg, "mass_kg")
        g = scalar(self.gravity_m_s2, "gravity_m_s2")
        j = vector(self.J_B_kg_m2, (3, 3), "J_B_kg_m2")
        if mass <= 0 or g < 0:
            raise ValueError("mass must be positive and gravity nonnegative")
        if not np.allclose(j, j.T, rtol=0, atol=1e-12):
            raise ValueError("inertia must be symmetric")
        try:
            np.linalg.cholesky(j)
        except np.linalg.LinAlgError as exc:
            raise ValueError("inertia must be positive definite") from exc
        object.__setattr__(self, "mass_kg", mass)
        object.__setattr__(self, "gravity_m_s2", g)
        object.__setattr__(self, "J_B_kg_m2", j)


@dataclass(frozen=True)
class BodyWrench:
    """Non-gravity force N and torque N m about COM, expressed in B."""
    force_B_N: Array
    torque_B_Nm: Array

    def __post_init__(self) -> None:
        for name in ("force_B_N", "torque_B_Nm"):
            object.__setattr__(self, name, vector(getattr(self, name), (3,), name))


@dataclass(frozen=True)
class StageState:
    """Raw ODE stage (SI, W position/velocity, B rate, W←B wxyz quaternion).

    The quaternion is finite and nondegenerate but may be non-unit. It is never
    modified; use physical_rotation() to evaluate orientation-dependent models.
    """
    p_W: Array
    v_W: Array
    q_WB: Array
    omega_B: Array

    def __post_init__(self) -> None:
        for name, shape in (("p_W", (3,)), ("v_W", (3,)),
                            ("q_WB", (4,)), ("omega_B", (3,))):
            object.__setattr__(self, name, vector(getattr(self, name), shape, name))
        norm = np.linalg.norm(self.q_WB)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("stage quaternion norm must be finite and >= 1e-12")

    @classmethod
    def from_array(cls, x: Array) -> "StageState":
        """Copy a raw 13-element stage vector without normalizing it."""
        x = vector(x, (13,), "stage")
        return cls(x[:3], x[3:6], x[6:10], x[10:])

    def physical_rotation(self) -> Array:
        """W←B rotation obtained from a normalized copy of the stage quaternion."""
        from .frames import rotation_matrix
        return rotation_matrix(self.q_WB / np.linalg.norm(self.q_WB))


class WrenchModel(Protocol):
    """Stateless applied wrench evaluated at each ODE stage."""
    def evaluate(self, t_s: float, state: StageState) -> BodyWrench: ...


class Integrator(Protocol):
    def step(self, t_s: float, state: RigidBodyState, params: RigidBodyParams,
             wrench: WrenchModel, dt_s: float) -> RigidBodyState: ...


class Scenario(Protocol):
    def build(self, config: Mapping) -> tuple[RigidBodyState, RigidBodyParams, WrenchModel]: ...
