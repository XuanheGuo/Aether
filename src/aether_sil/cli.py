"""Scenario runner and reproducible artifact writer."""
import argparse
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml

from .frames import rotation_matrix
from .integrators.reference import solve_reference
from .integrators.rk4 import RK4, state_vector
from .scenarios import load_scenario
from .types import StageState


def _revision():
    try:
        root = Path(__file__).resolve().parents[2]
        sha = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                      stderr=subprocess.DEVNULL, text=True).strip()
        dirty = bool(subprocess.check_output(['git', '-C', str(root), 'status', '--porcelain'],
                                             text=True))
        return sha, dirty
    except (OSError, subprocess.CalledProcessError):
        return 'unversioned', None


def run(config_path, output, seed=None):
    """Run an immutable RK4 experiment; output directory must not already exist."""
    config, state, params, wrench, steps = load_scenario(config_path)
    if seed is not None:
        if seed < 0:
            raise ValueError('seed must be nonnegative')
        config['seed'] = seed
    # Seed is recorded even though the current deterministic wrench uses no randomness.
    np.random.default_rng(config['seed'])
    target = Path(output)
    target.mkdir(parents=True, exist_ok=False)
    dt = config['simulation']['dt_s']
    times = np.arange(steps + 1, dtype=np.float64)*dt
    states = [state]
    norms = [np.nan]
    forces, torques = [], []
    config_json = json.dumps(config, sort_keys=True, separators=(',', ':'), allow_nan=False)
    digest = hashlib.sha256(config_json.encode()).hexdigest()
    (target/'resolved_config.yaml').write_text(yaml.safe_dump(config, sort_keys=True), encoding='utf-8')
    sha, dirty = _revision()
    manifest = {'contract_version': config['contract_version'], 'scenario_id': config['scenario_id'],
                'config_sha256': digest, 'config_digest_encoding': 'canonical JSON UTF-8',
                'git_sha': sha, 'git_dirty': dirty, 'python': sys.version,
                'platform': platform.platform(), 'dependencies': {k: importlib.metadata.version(k)
                  for k in ('numpy', 'scipy', 'PyYAML')}, 'seed': config['seed'], 'dt_s': dt,
                'integrator': 'classical RK4', 'reference': {'method': 'DOP853', 'rtol': 1e-12,
                'atol': 1e-13}, 'generated_utc': datetime.now(timezone.utc).isoformat(),
                'results_path': str(target.resolve()), 'float_dtype': 'float64'}
    (target/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    k, t = 0, times[0]
    try:
        for k, t in enumerate(times):
            applied = wrench.evaluate(float(t), StageState.from_array(state_vector(state)))
            forces.append(applied.force_B_N)
            torques.append(applied.torque_B_Nm)
            if k < steps:
                state, diagnostic = RK4().step_with_diagnostics(float(t), state, params, wrench, dt)
                states.append(state)
                norms.append(diagnostic.quaternion_norm_before)
        x = np.array([state_vector(s) for s in states])
        force = np.array(forces)
        torque = np.array(torques)
        np.savez(target/'trajectory.npz', t_s=times, p_W_m=x[:, :3], v_W_m_s=x[:, 3:6],
                 q_WB_wxyz=x[:, 6:10], omega_B_rad_s=x[:, 10:], F_B_N=force,
                 tau_B_Nm=torque, q_norm_before=np.array(norms), dt_s=np.array(dt),
                 integrator=np.array('rk4'))
        ref = solve_reference(states[0], params, wrench, times)
        y = np.array([state_vector(s) for s in ref])
        # Stable geodesic distance, insensitive to q versus -q.
        qdiff = np.minimum(np.linalg.norm(x[:, 6:10]-y[:, 6:10], axis=1),
                           np.linalg.norm(x[:, 6:10]+y[:, 6:10], axis=1))
        angles = 4*np.arcsin(np.clip(qdiff/2, 0, 1))
        j = params.J_B_kg_m2
        energy = np.array([.5*s.omega_B @ j @ s.omega_B for s in states])
        momentum = np.array([rotation_matrix(s.q_WB) @ j @ s.omega_B for s in states])
        zero_torque = bool(np.all(torque == 0))
        metrics = {'max_quaternion_norm_error': float(np.max(np.abs(np.array(norms[1:])-1))),
                   'rotational_invariants': {
                       'interpretation': 'conservation_drift' if zero_torque else 'change_under_torque',
                       'max_relative_energy_change': float(np.max(np.abs(energy-energy[0])) /
                                                            max(abs(energy[0]), 1e-15)),
                       'max_relative_world_momentum_change': float(np.max(np.linalg.norm(
                           momentum-momentum[0], axis=1))/max(np.linalg.norm(momentum[0]), 1e-15))},
                   'max_reference_position_error_m': float(np.max(np.linalg.norm(x[:, :3]-y[:, :3], axis=1))),
                   'max_reference_attitude_error_rad': float(np.max(angles)),
                   'analytic_error': {'status': 'not_applicable',
                                      'reason': 'no analytic reference defined for this scenario'}}
        (target/'metrics.json').write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
        validation = {'run_status': 'completed', 'gate_s0': 'NOT EVALUATED',
                      'note': 'scenario execution does not establish formal S0 gate'}
        (target/'validation.json').write_text(json.dumps(validation, indent=2)+'\n')
    except Exception as exc:
        # States are appended only after accepted steps, including the initial sample.
        partial = np.array([state_vector(s) for s in states])
        np.savez(target/'partial_trajectory.npz', t_s=times[:len(states)],
                 p_W_m=partial[:, :3], v_W_m_s=partial[:, 3:6],
                 q_WB_wxyz=partial[:, 6:10], omega_B_rad_s=partial[:, 10:],
                 q_norm_before=np.array(norms), dt_s=np.array(dt))
        failure = {'run_status': 'FAIL', 'gate_s0': 'NOT EVALUATED',
                   'last_successful_step': len(states)-1,
                   'last_successful_time_s': float(times[len(states)-1]),
                   'failure_step_start_s': float(t),
                   'failure_time_s': getattr(exc, 'aether_failure_time_s', None),
                   'failure_time_note': 'ODE stage time; null when unavailable',
                   'exception_type': type(exc).__name__, 'exception_message': str(exc),
                   'partial_trajectory': 'partial_trajectory.npz'}
        (target/'validation.json').write_text(json.dumps(failure, indent=2)+'\n')
        raise
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(prog='aether_sil')
    sub = parser.add_subparsers(dest='command', required=True)
    command = sub.add_parser('run')
    command.add_argument('scenario')
    command.add_argument('--seed', type=int)
    command.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    try:
        run(args.scenario, args.output, args.seed)
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.exit(2, f'error: {exc}\n')


if __name__ == '__main__':
    main()
