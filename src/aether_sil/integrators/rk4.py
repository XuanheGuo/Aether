"""Fixed-step classical RK4, with accepted-step quaternion diagnostics."""
from dataclasses import dataclass

import numpy as np

from ..dynamics import dxdt
from ..types import (Array, BodyWrench, RigidBodyParams, RigidBodyState, StageState,
                     WrenchModel, scalar)


def state_vector(state: RigidBodyState) -> Array:
    """Pack a validated physical state in contract order."""
    if not isinstance(state, RigidBodyState):
        raise ValueError("state must be a RigidBodyState")
    return np.r_[state.p_W, state.v_W, state.q_WB, state.omega_B]


def evaluate_rhs(t_s: float, x: Array, params: RigidBodyParams, wrench: WrenchModel) -> Array:
    """Evaluate stateless wrench at the untouched raw stage and then the ODE."""
    stage = StageState.from_array(x)
    applied = wrench.evaluate(t_s, stage)
    if not isinstance(applied, BodyWrench):
        raise ValueError("wrench model must return BodyWrench")
    return dxdt(t_s, x, params, applied)


@dataclass(frozen=True)
class StepDiagnostics:
    """Quaternion norm and signed norm error before accepted-step correction."""
    quaternion_norm_before: float
    quaternion_norm_error: float


class RK4:
    """Classical RK4; call step_with_diagnostics to retain each step's norm error."""

    def step_with_diagnostics(self, t_s: float, state: RigidBodyState,
                              params: RigidBodyParams, wrench: WrenchModel,
                              dt_s: float) -> tuple[RigidBodyState, StepDiagnostics]:
        """Advance one finite positive step in seconds; normalize only at acceptance."""
        t = scalar(t_s, "t_s")
        dt = scalar(dt_s, "dt_s")
        if t < 0 or dt <= 0 or not np.isfinite(t + dt) or t + dt <= t:
            raise ValueError("time must be nonnegative and dt must advance finite time")
        x = state_vector(state)
        k1 = evaluate_rhs(t, x, params, wrench)
        k2 = evaluate_rhs(t + dt/2, x + dt*k1/2, params, wrench)
        k3 = evaluate_rhs(t + dt/2, x + dt*k2/2, params, wrench)
        k4 = evaluate_rhs(t + dt, x + dt*k3, params, wrench)
        result = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        if not np.all(np.isfinite(result)):
            raise ValueError("non-finite integrated state")
        norm = float(np.linalg.norm(result[6:10]))
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("invalid integrated quaternion norm")
        output = RigidBodyState(result[:3], result[3:6], result[6:10], result[10:])
        return output, StepDiagnostics(norm, norm - 1.0)

    def step(self, t_s: float, state: RigidBodyState, params: RigidBodyParams,
             wrench: WrenchModel, dt_s: float) -> RigidBodyState:
        """Advance one step; use step_with_diagnostics to collect norm errors."""
        return self.step_with_diagnostics(t_s, state, params, wrench, dt_s)[0]
