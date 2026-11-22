"""Produce frozen T10 refinement and tighter independent reference evidence."""
import json
import platform
import sys

import numpy as np
import scipy
from scipy.integrate import solve_ivp

from aether_sil.integrators.rk4 import RK4, evaluate_rhs, state_vector
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState
from aether_sil.wrenches import ConstantBodyWrench

state = RigidBodyState([.1, -.2, .3], [.2, -.1, .4], [.8, .2, -.3, .4], [.7, -.4, .9])
params = RigidBodyParams(1.3, np.diag([.7, 1.1, 1.6]), 0)
wrench = ConstantBodyWrench(BodyWrench([.4, -.7, .3], [.2, -.15, .3]))
x0 = state_vector(state)


def error(a, b):
    qa = a[6:10]/np.linalg.norm(a[6:10])
    qb = b[6:10]/np.linalg.norm(b[6:10])
    d = min(np.linalg.norm(qa-qb), np.linalg.norm(qa+qb))
    theta = 4*np.arcsin(min(d/2, 1))
    return float(np.linalg.norm(np.r_[a[:6]-b[:6], theta, a[10:]-b[10:]]))


def reference(rtol, atol, max_step):
    result = solve_ivp(lambda t, x: evaluate_rhs(t, x, params, wrench), (0, 2), x0,
                       method='DOP853', rtol=rtol, atol=atol, max_step=max_step)
    if not result.success:
        raise RuntimeError(result.message)
    return result.y[:, -1]


standard = reference(1e-12, 1e-13, np.inf)
strict = reference(2.3e-14, 2e-15, .025)
levels = []
for n in (10, 20, 40, 80):
    h = 2/n
    current = state
    for k in range(n):
        current = RK4().step(k*h, current, params, wrench, h)
    levels.append({'steps': n, 'dt_s': h, 'error': error(state_vector(current), standard)})
for i in range(1, len(levels)):
    levels[i]['observed_order'] = float(np.log2(levels[i-1]['error']/levels[i]['error']))
ref_delta = error(standard, strict)
ratio = ref_delta/levels[-1]['error']
result = {'fixture': {'state': x0.tolist(), 'mass_kg': 1.3, 'inertia_diag': [.7, 1.1, 1.6],
                      'gravity': 0, 'force_B_N': [.4, -.7, .3],
                      'torque_B_Nm': [.2, -.15, .3], 'duration_s': 2},
          'error_definition': 'L2 of position, velocity, geodesic angle and body rate',
          'standard_reference': {'method': 'DOP853', 'rtol': 1e-12, 'atol': 1e-13},
          'strict_reference': {'method': 'DOP853', 'rtol': 2.3e-14, 'atol': 2e-15,
                               'max_step_s': .025},
          'reference_disagreement': ref_delta, 'reference_disagreement_fraction_of_finest': ratio,
          'reference_check_limit': .01, 'levels': levels,
          'python': sys.version, 'platform': platform.platform(),
          'numpy': np.__version__, 'scipy': scipy.__version__}
print(json.dumps(result, indent=2, allow_nan=False))
if ratio >= .01 or min(v['observed_order'] for v in levels[1:3]) < 3.5:
    raise SystemExit('T10 inconclusive or failed')
