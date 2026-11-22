"""Strict S0 synthetic scenario loading and validation."""
from pathlib import Path

import numpy as np
import yaml

from .types import BodyWrench, RigidBodyParams, RigidBodyState, scalar
from .wrenches import ConstantBodyWrench, ZeroWrench


def fields(obj, expected, name):
    """Require exactly the named mapping keys."""
    if not isinstance(obj, dict) or set(obj) != set(expected):
        raise ValueError(f"{name}: expected fields {sorted(expected)}")


def load_scenario(path):
    """Load and validate a YAML scenario; return config, state, params, wrench, steps."""
    with Path(path).open(encoding="utf-8") as f:
        c = yaml.safe_load(f)
    fields(c, ['scenario_id', 'contract_version', 'seed', 'simulation', 'world', 'body',
               'initial_state', 'wrench', 'output'], 'scenario')
    if not isinstance(c['scenario_id'], str) or not c['scenario_id']:
        raise ValueError('scenario_id must be a nonempty string')
    if c['contract_version'] != '1.0.0':
        raise ValueError('unsupported contract_version')
    if type(c['seed']) is not int or c['seed'] < 0:
        raise ValueError('seed must be a nonnegative integer')
    sim, world, body, initial, wrench, output = (c[k] for k in
        ('simulation', 'world', 'body', 'initial_state', 'wrench', 'output'))
    fields(sim, ['t0_s', 'duration_s', 'dt_s', 'integrator', 'float_dtype'], 'simulation')
    fields(world, ['frame', 'gravity_m_s2'], 'world')
    fields(body, ['frame', 'mass_kg', 'inertia_B_kg_m2'], 'body')
    fields(initial, ['p_W_m', 'v_W_m_s', 'q_WB_wxyz', 'omega_B_rad_s'], 'initial_state')
    fields(output, ['samples', 'format'], 'output')
    if sim['t0_s'] != 0 or sim['integrator'] != 'rk4' or sim['float_dtype'] != 'float64':
        raise ValueError('expected t0=0, rk4 and float64')
    if world['frame'] != 'ENU' or body['frame'] != 'FLU':
        raise ValueError('expected ENU world and FLU body')
    if output != {'samples': 'every_step', 'format': 'npz'}:
        raise ValueError('expected every_step and npz output')
    dt = scalar(sim['dt_s'], 'dt_s')
    duration = scalar(sim['duration_s'], 'duration_s')
    if dt <= 0 or duration <= 0:
        raise ValueError('dt and duration must be positive')
    ratio = duration / dt
    if not np.isfinite(ratio) or ratio > 10000000 or abs(ratio-round(ratio)) > 1e-9:
        raise ValueError('duration must be an integer number of steps (at most 10000000)')
    steps = round(ratio)
    if steps < 1:
        raise ValueError('at least one step is required')
    state = RigidBodyState(initial['p_W_m'], initial['v_W_m_s'], initial['q_WB_wxyz'],
                           initial['omega_B_rad_s'])
    params = RigidBodyParams(body['mass_kg'], body['inertia_B_kg_m2'], world['gravity_m_s2'])
    if not isinstance(wrench, dict):
        raise ValueError('wrench must be a mapping')
    if wrench.get('model') == 'zero':
        fields(wrench, ['model'], 'wrench')
        model = ZeroWrench()
    elif wrench.get('model') == 'constant':
        fields(wrench, ['model', 'force_B_N', 'torque_B_Nm'], 'wrench')
        model = ConstantBodyWrench(BodyWrench(wrench['force_B_N'], wrench['torque_B_Nm']))
    else:
        raise ValueError('wrench model must be zero or constant')
    return c, state, params, model, steps
