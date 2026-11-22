"""End-to-end reproducibility, off-manifold rotation, and input validation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

from aether_sil.dynamics import dxdt
from aether_sil.integrators.reference import solve_reference
from aether_sil.integrators.rk4 import RK4
from aether_sil.scenarios import load_scenario
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState, StageState
from aether_sil.wrenches import ConstantBodyWrench


class TestScenario(unittest.TestCase):
    def test_stage_rotation_consistency(self):
        raw = np.r_[[0.]*6, [1.6, .4, -.6, .8], [.3, -.2, .4]]
        stage = StageState.from_array(raw)
        force = np.array([.7, -.3, 1.1])
        deriv = dxdt(0, raw, RigidBodyParams(1, np.eye(3), 0), BodyWrench(force, [0]*3))
        np.testing.assert_allclose(deriv[3:6], stage.physical_rotation() @ force, atol=1e-14)
        np.testing.assert_array_equal(stage.q_WB, raw[6:10])

    def test_trajectory_reference_and_sign(self):
        state = RigidBodyState([1, 2, 3], [.2, -.3, .1], [.8, .2, -.3, .4], [.7, -.4, .9])
        flipped = RigidBodyState(state.p_W, state.v_W, -state.q_WB, state.omega_B)
        params = RigidBodyParams(1.3, np.diag([.7, 1.1, 1.6]), 0)
        wrench = ConstantBodyWrench(BodyWrench([.4, -.7, .3], [.2, -.15, .3]))
        times = np.arange(101)*.02
        ref = solve_reference(state, params, wrench, times)
        for k in range(101):
            np.testing.assert_allclose(state.p_W, ref[k].p_W, atol=1e-5, rtol=0)
            diff = min(np.linalg.norm(state.q_WB-ref[k].q_WB),
                       np.linalg.norm(state.q_WB+ref[k].q_WB))
            self.assertLess(4*np.arcsin(min(diff/2, 1)), 1e-5)
            np.testing.assert_allclose(state.p_W, flipped.p_W, atol=1e-12, rtol=0)
            np.testing.assert_allclose(state.v_W, flipped.v_W, atol=1e-12, rtol=0)
            np.testing.assert_allclose(state.q_WB, -flipped.q_WB, atol=1e-12, rtol=0)
            if k < 100:
                state = RK4().step(times[k], state, params, wrench, .02)
                flipped = RK4().step(times[k], flipped, params, wrench, .02)

    def test_cli_and_reproducibility(self):
        with tempfile.TemporaryDirectory() as d:
            env = dict(os.environ, PYTHONPATH='src')
            for n in ('a', 'b'):
                cmd = [sys.executable, '-m', 'aether_sil', 'run', 'scenarios/free_rotation.yaml',
                       '--seed', '7', '--output', str(Path(d)/n)]
                p = subprocess.run(cmd, env=env, capture_output=True, text=True)
                self.assertEqual(p.returncode, 0, p.stderr)
            a, b = Path(d)/'a', Path(d)/'b'
            for name in ('manifest.json', 'resolved_config.yaml', 'metrics.json',
                         'validation.json', 'trajectory.npz'):
                self.assertTrue((a/name).exists(), name)
            self.assertEqual((a/'trajectory.npz').read_bytes(), (b/'trajectory.npz').read_bytes())
            manifest = json.loads((a/'manifest.json').read_text())
            for key in ('git_sha', 'git_dirty', 'config_sha256', 'contract_version',
                        'platform', 'python', 'dependencies', 'seed', 'integrator', 'dt_s'):
                self.assertIn(key, manifest)
            self.assertEqual(json.loads((a/'validation.json').read_text())['run_status'], 'completed')
            with np.load(a/'trajectory.npz') as trace:
                self.assertEqual(len(trace['t_s']), 201)
                self.assertEqual(len(trace['q_norm_before']), 201)
                self.assertTrue(np.isnan(trace['q_norm_before'][0]))

    def test_bad_configs(self):
        original = yaml.safe_load(Path('scenarios/free_rotation.yaml').read_text())
        for mode in ('missing', 'unknown', 'negative', 'nan'):
            c = yaml.safe_load(yaml.safe_dump(original))
            if mode == 'missing':
                del c['body']['mass_kg']
            elif mode == 'unknown':
                c['body']['typo'] = 1
            elif mode == 'negative':
                c['simulation']['dt_s'] = -1
            else:
                c['initial_state']['p_W_m'][0] = float('nan')
            with tempfile.TemporaryDirectory() as d:
                p = Path(d)/'bad.yaml'
                p.write_text(yaml.safe_dump(c))
                with self.subTest(mode=mode), self.assertRaises(ValueError):
                    load_scenario(p)
