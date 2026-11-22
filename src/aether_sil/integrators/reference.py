"""Independent adaptive SciPy DOP853 integration for cross-checks."""
import numpy as np
from scipy.integrate import solve_ivp

from ..types import RigidBodyParams, RigidBodyState, WrenchModel, scalar, vector
from .rk4 import evaluate_rhs, state_vector


def solve_reference(state: RigidBodyState, params: RigidBodyParams, wrench: WrenchModel,
                    times_s: np.ndarray, *, rtol: float = 1e-12,
                    atol: float = 1e-13) -> list[RigidBodyState]:
    """Integrate at strictly increasing nonnegative sample times using DOP853.

    First time is initial state time. No internal normalization; return samples
    as physical normalized states. Tolerances are positive finite scalars.
    """
    times = np.asarray(times_s, dtype=np.float64)
    if times.ndim != 1 or len(times) < 2:
        raise ValueError("times_s must be a one-dimensional array with at least two times")
    times = vector(times, times.shape, "times_s")
    if times[0] < 0 or np.any(np.diff(times) <= 0):
        raise ValueError("times_s must be nonnegative and strictly increasing")
    rtol, atol = scalar(rtol, "rtol"), scalar(atol, "atol")
    if rtol < 100*np.finfo(float).eps or atol <= 0:
        raise ValueError("rtol must be >= 100 machine eps and atol must be positive")
    solution = solve_ivp(lambda t, x: evaluate_rhs(t, x, params, wrench),
                         (times[0], times[-1]), state_vector(state), method="DOP853",
                         t_eval=times, rtol=rtol, atol=atol)
    if not solution.success or not np.all(np.isfinite(solution.y)):
        raise RuntimeError(f"DOP853 failed: {solution.message}")
    return [RigidBodyState(y[:3], y[3:6], y[6:10], y[10:]) for y in solution.y.T]
