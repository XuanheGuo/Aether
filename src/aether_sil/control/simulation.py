"""Minimal sampled closed-loop harness; ideal torque and no disturbances."""
from dataclasses import dataclass

import numpy as np

from ..integrators.rk4 import RK4
from ..types import BodyWrench, RigidBodyParams, RigidBodyState, scalar
from ..wrenches import ConstantBodyWrench
from .interfaces import AttitudeController, AttitudeReference, MeasurementModel


@dataclass(frozen=True)
class ClosedLoopTrace:
    """Plant samples at dt_s and control samples at dt_control_s, all in SI."""
    times_s: np.ndarray
    states: tuple[RigidBodyState, ...]
    control_times_s: np.ndarray
    commands_B_Nm: np.ndarray
    measured_q_WB: np.ndarray
    measured_omega_B: np.ndarray
    reference_q_WD: np.ndarray
    reference_omega_D: np.ndarray


def simulate(initial: RigidBodyState, params: RigidBodyParams, controller: AttitudeController,
             reference: AttitudeReference, measurement: MeasurementModel, *, duration_s: float,
             dt_s: float, dt_control_s: float, seed: int = 7) -> ClosedLoopTrace:
    """Run fixed ticks and zero-order hold, with no control update at RK4 stages.

    Control at k*dt_control on [t_k,t_(k+1)); final endpoint is a plant sample
    only. Require integer duration and control-to-integration step ratios.
    """
    h, hc, duration = (scalar(dt_s, 'dt_s'), scalar(dt_control_s, 'dt_control_s'),
                       scalar(duration_s, 'duration_s'))
    if h <= 0 or hc <= 0 or duration <= 0:
        raise ValueError('duration and step sizes must be positive')
    if type(seed) is not int or seed < 0:
        raise ValueError('seed must be a nonnegative integer')
    ratio, total = hc/h, duration/h
    if not np.isfinite(ratio) or not np.isfinite(total) or ratio > 10000000 or total > 10000000:
        raise ValueError('invalid step ratio')
    interval, steps = round(ratio), round(total)
    if interval < 1 or steps < 1 or abs(ratio-interval) > 1e-9 or abs(total-steps) > 1e-9 or steps % interval:
        raise ValueError('control period and duration must be integer multiples of dt')
    controller.reset(seed=seed)
    rng = np.random.default_rng(seed)
    state = initial
    states, times, commands, control_times = [state], [0.], [], []
    measurements, rates, refs, ref_rates = [], [], [], []
    for k in range(steps):
        t = k*h
        if k % interval == 0:
            obs = measurement.sample(t, state, rng)
            ref = reference.sample(t)
            cmd = controller.update(t, obs, ref, hc)
            if cmd.t_s != t:
                raise ValueError('controller returned incorrect timestamp')
            held = ConstantBodyWrench(BodyWrench([0, 0, 0], cmd.torque_B_Nm))
            control_times.append(t)
            commands.append(cmd.torque_B_Nm)
            measurements.append(obs.q_WB)
            rates.append(obs.omega_B)
            refs.append(ref.q_WD)
            ref_rates.append(ref.omega_D)
        state = RK4().step(t, state, params, held, h)
        states.append(state)
        times.append((k+1)*h)
    return ClosedLoopTrace(np.array(times), tuple(states), np.array(control_times),
                           np.array(commands), np.array(measurements), np.array(rates),
                           np.array(refs), np.array(ref_rates))
