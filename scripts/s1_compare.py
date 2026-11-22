"""Frozen synthetic Batch A comparison; outputs JSON and uncompressed traces."""
import argparse
import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np
import scipy

from aether_sil.control.controllers import GeometricPD, LocalLQR
from aether_sil.control.geometry import attitude_errors
from aether_sil.control.interfaces import ConstantReference, IdealMeasurement
from aether_sil.control.simulation import simulate
from aether_sil.types import RigidBodyParams, RigidBodyState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    j = np.diag([1., 2., 3.])
    params = RigidBodyParams(1, j, 0)
    axis = np.array([1., -2., 1.])
    axis /= np.linalg.norm(axis)
    initial = RigidBodyState([0]*3, [0]*3, np.r_[np.cos(.15), np.sin(.15)*axis], [.02, -.03, .01])
    qweight, rweight = np.diag([4., 4., 4., 3., 3., 3.]), np.eye(3)
    controllers = {'geometric_pd': GeometricPD(params, 4, 3),
                   'local_lqr': LocalLQR(params, qweight, rweight)}
    try:
        sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True))
    except (OSError, subprocess.CalledProcessError):
        sha, dirty = 'unversioned', None
    result = {'git_sha': sha, 'git_dirty': dirty, 'python': sys.version,
              'platform': platform.platform(), 'numpy': np.__version__, 'scipy': scipy.__version__,
              'fixture': {'mass_kg': 1, 'J_B_kg_m2': j.tolist(), 'gravity_m_s2': 0,
                          'initial_q_WB': initial.q_WB.tolist(), 'initial_omega_B_rad_s': initial.omega_B.tolist(),
                          'reference_q_WD': [1, 0, 0, 0], 'duration_s': 8, 'dt_s': .01,
                          'dt_control_s': .02, 'seed': 7, 'external_force_B_N': [0, 0, 0],
                          'disturbance_B_Nm': [0, 0, 0], 'actuation': 'ideal unconstrained torque',
                          'pd_k_R': 4, 'pd_k_omega': 3, 'lqr_Q': qweight.tolist(),
                          'lqr_R': rweight.tolist(), 'lqr_max_angle_rad': .5}, 'controllers': {}}
    for name, controller in controllers.items():
        trace = simulate(initial, params, controller, ConstantReference([1, 0, 0, 0]), IdealMeasurement(),
                         duration_s=8, dt_s=.01, dt_control_s=.02, seed=7)
        q = np.array([s.q_WB for s in trace.states])
        w = np.array([s.omega_B for s in trace.states])
        angles = np.array([attitude_errors(s.q_WB, [1, 0, 0, 0], s.omega_B, [0]*3)[3] for s in trace.states])
        trace_path = out/(name+'.npz')
        np.savez(trace_path, t_s=trace.times_s, q_WB=q, omega_B_rad_s=w,
                 control_t_s=trace.control_times_s, requested_torque_B_Nm=trace.commands_B_Nm,
                 actual_torque_B_Nm=trace.commands_B_Nm, measurement_q_WB=trace.measured_q_WB,
                 measurement_omega_B_rad_s=trace.measured_omega_B, reference_q_WD=trace.reference_q_WD,
                 reference_omega_D_rad_s=trace.reference_omega_D)
        settled = (angles < .02) & (np.linalg.norm(w, axis=1) < .02)
        last_failure = np.flatnonzero(~settled)
        idx = int(last_failure[-1]+1) if len(last_failure) else 0
        settling = float(trace.times_s[idx]) if idx < len(settled) else None
        result['controllers'][name] = {'final_error_rad': float(angles[-1]),
            'final_rate_rad_s': float(np.linalg.norm(w[-1])), 'max_attitude_error_rad': float(np.max(angles)),
            'settling_time_s': settling, 'settling_thresholds': {'angle_rad': .02, 'rate_rad_s': .02},
            'torque_effort_Nm2_s': float(.02*np.sum(trace.commands_B_Nm**2)),
            'max_torque_Nm': float(np.max(np.linalg.norm(trace.commands_B_Nm, axis=1))),
            'trace_sha256': hashlib.sha256(trace_path.read_bytes()).hexdigest()}
    (out/'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
