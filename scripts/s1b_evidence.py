"""Reproduce small synthetic Batch B validation fixtures as JSON."""
import json
import platform
import subprocess
import sys

import numpy as np
import scipy

from aether_sil.control.actuated_simulation import augmented_step, simulate_actuated
from aether_sil.control.actuators import BoxLimit, FirstOrderActuator
from aether_sil.control.controllers import GeometricPD
from aether_sil.control.geometry import attitude_errors
from aether_sil.control.interfaces import ConstantReference, IdealMeasurement
from aether_sil.types import RigidBodyParams, RigidBodyState

p = RigidBodyParams(1, np.diag([1., 2., 3.]), 0)
axis = np.array([1., -2., 1.])
axis /= np.linalg.norm(axis)
initial = RigidBodyState([0]*3, [0]*3, np.r_[np.cos(.15), np.sin(.15)*axis], [.02, -.03, .01])


def run(h, hc=.04, lag=True, box=.5):
    return simulate_actuated(initial, p, GeometricPD(p, 4, 3), ConstantReference([1, 0, 0, 0]),
                             IdealMeasurement(), BoxLimit([box]*3), duration_s=2, dt_s=h,
                             dt_control_s=hc, actuator=FirstOrderActuator([.08, .1, .12]) if lag else None)


def end(t):
    s = t.states[-1]
    return np.r_[s.q_WB, s.omega_B, t.actual_B_Nm[-1]]


def angle(t):
    s = t.states[-1]
    return attitude_errors(s.q_WB, [1, 0, 0, 0], s.omega_B, [0]*3)[3]


step_errors = {}
for tau in (.05, .1, .3):
    state, actual = initial, np.array([.2, -.1, .05])
    start = actual.copy()
    target = np.array([.8, -.4, .2])
    for k in range(200):
        state, actual = augmented_step(k*.001, state, actual, p, FirstOrderActuator([tau]*3), target, .001)
    step_errors[str(tau)] = float(np.max(np.abs(actual-(target+(start-target)*np.exp(-.2/tau)))))
traces = [run(h) for h in (.04, .02, .01, .005)]
reference = end(traces[-1])
ideal, limited = run(.01, lag=False, box=10), run(.01, lag=False, box=.001)
sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True))
report = {'git_sha': sha, 'git_dirty': dirty, 'python': sys.version, 'platform': platform.platform(),
          'numpy': np.__version__, 'scipy': scipy.__version__,
          'step_response_max_abs_error_Nm': step_errors,
          'step_response': {'initial': [.2, -.1, .05], 'target': [.8, -.4, .2], 'time_s': .2, 'dt_s': .001},
          'refinement': {'control_period_s': .04, 'dt_s': [.04, .02, .01, .005],
                         'end_state_error_vs_dt_0_005': [float(np.linalg.norm(end(x)-reference)) for x in traces[:-1]],
                         'changed_control_period_s': .01,
                         'control_period_change_end_difference': float(np.linalg.norm(end(run(.005, hc=.01))-reference))},
          'constraint_counterexample': {'duration_s': 2, 'unconstrained_approx_box_Nm': 10,
              'limited_box_Nm': .001, 'ideal_final_angle_rad': angle(ideal),
              'limited_final_angle_rad': angle(limited)},
          'synthetic_fixture': {'inertia_diag_kg_m2': [1, 2, 3], 'initial_angle_rad': .3,
              'initial_axis': [1, -2, 1], 'initial_rate_rad_s': [.02, -.03, .01],
              'k_R': 4, 'k_omega': 3, 'gravity': 0, 'reference': [1, 0, 0, 0]}}
print(json.dumps(report, indent=2, allow_nan=False))
