"""Coupled rigid body / first-order torque actuator RK4 simulation."""
from dataclasses import dataclass

import numpy as np

from ..dynamics import dxdt
from ..integrators.rk4 import state_vector
from ..types import BodyWrench, RigidBodyState, scalar, vector
from .actuators import BallLimit, FirstOrderActuator


def augmented_step(t_s, state, actual, params, actuator, projected, dt_s):
    """Classical RK4 on [S0 state, actual torque]; accepted q normalized once.

    Target is held, no hidden actuator state. h/T_i <= 1 is a conservative
    integration restriction; it is not a closed-loop stability guarantee.
    """
    t, h = scalar(t_s, 't_s'), scalar(dt_s, 'dt_s')
    if t < 0 or h <= 0 or not np.isfinite(t+h) or t+h <= t:
        raise ValueError('invalid time or step')
    if not isinstance(actuator, FirstOrderActuator):
        raise ValueError('expected FirstOrderActuator')
    if np.any(h/actuator.time_constants_s > 1):
        raise ValueError('dt must not exceed smallest actuator time constant')
    target = vector(projected, (3,), 'projected')
    x = np.r_[state_vector(state), vector(actual, (3,), 'actual')]
    def rhs(time, y):
        return np.r_[dxdt(time, y[:13], params, BodyWrench([0, 0, 0], y[13:])),
                     actuator.derivative(y[13:], target)]
    k1 = rhs(t, x)
    k2 = rhs(t+h/2, x+h*k1/2)
    k3 = rhs(t+h/2, x+h*k2/2)
    k4 = rhs(t+h, x+h*k3)
    y = x+h*(k1+2*k2+2*k3+k4)/6
    if not np.all(np.isfinite(y)):
        raise ValueError('nonfinite augmented state')
    return RigidBodyState(y[:3], y[3:6], y[6:10], y[10:13]), y[13:].copy()


@dataclass(frozen=True)
class ActuatedTrace:
    """Plant samples and tick-aligned requested/projected/actual torques in SI."""
    times_s: np.ndarray
    states: tuple
    actual_B_Nm: np.ndarray
    control_times_s: np.ndarray
    requested_B_Nm: np.ndarray
    projected_B_Nm: np.ndarray
    actual_at_control_B_Nm: np.ndarray


def simulate_actuated(initial, params, controller, reference, measurement, limit, *,
                      duration_s, dt_s, dt_control_s, actuator=None, initial_actual=(0, 0, 0), seed=7):
    """Run ideal constrained (actuator=None) or coupled lag, with ZOH target."""
    h, hc, duration = (scalar(dt_s, 'dt_s'), scalar(dt_control_s, 'dt_control_s'),
                       scalar(duration_s, 'duration_s'))
    if h <= 0 or hc <= 0 or duration <= 0 or type(seed) is not int or seed < 0:
        raise ValueError('invalid time or seed')
    ratio, total = hc/h, duration/h
    if not np.isfinite(ratio) or not np.isfinite(total) or max(ratio, total) > 10000000:
        raise ValueError('invalid step ratio')
    interval, steps = round(ratio), round(total)
    if interval < 1 or steps < 1 or abs(ratio-interval) > 1e-9 or abs(total-steps) > 1e-9 or steps % interval:
        raise ValueError('duration and control period must be integer multiples of dt')
    actual = vector(initial_actual, (3,), 'initial_actual')
    if not limit.contains(actual):
        raise ValueError('initial actual torque must lie within feasible set')
    if actuator is not None and (not isinstance(actuator, FirstOrderActuator) or
                                 np.any(h/actuator.time_constants_s > 1)):
        raise ValueError('invalid actuator or unstable integration step')
    if actuator is not None and isinstance(limit, BallLimit) and not np.all(
            actuator.time_constants_s == actuator.time_constants_s[0]):
        raise ValueError('ball constraint requires equal time constants to preserve feasibility')
    controller.reset(seed=seed)
    rng = np.random.default_rng(seed)
    states, actuals = [initial], [actual.copy()]
    ticks, requested, projected, actual_ticks = [], [], [], []
    state = initial
    for k in range(steps):
        t = k*h
        if k % interval == 0:
            obs = measurement.sample(t, state, rng)
            ref = reference.sample(t)
            command = controller.update(t, obs, ref, hc)
            if command.t_s != t:
                raise ValueError('controller returned incorrect timestamp')
            target = limit.project(command.torque_B_Nm)
            if actuator is None:
                actual = target.copy()
                actuals[-1] = actual.copy()
            ticks.append(t)
            requested.append(command.torque_B_Nm)
            projected.append(target)
            actual_ticks.append(actual.copy())
        if actuator is None:
            from ..integrators.rk4 import RK4
            from ..wrenches import ConstantBodyWrench
            state = RK4().step(t, state, params, ConstantBodyWrench(BodyWrench([0]*3, target)), h)
        else:
            state, actual = augmented_step(t, state, actual, params, actuator, target, h)
        states.append(state)
        actuals.append(actual.copy())
    return ActuatedTrace(np.arange(steps+1)*h, tuple(states), np.array(actuals), np.array(ticks),
                         np.array(requested), np.array(projected), np.array(actual_ticks))
