"""Independent analytic, convergence and intermediate-stage checks."""
import unittest

import numpy as np

from aether_sil.dynamics import dxdt
from aether_sil.frames import rotation_matrix
from aether_sil.integrators.reference import solve_reference
from aether_sil.integrators.rk4 import RK4, state_vector
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState, StageState
from aether_sil.wrenches import ConstantBodyWrench, ZeroWrench


def run(state, params, wrench, h, n):
    diagnostics = []
    for k in range(n):
        state, d = RK4().step_with_diagnostics(k*h, state, params, wrench, h)
        diagnostics.append(d.quaternion_norm_error)
    return state, np.array(diagnostics)


class AttitudeWrench:
    def evaluate(self, t_s, state):
        # Fixed world force represented in body axes, plus smooth body torque.
        force = state.physical_rotation().T @ np.array([.4, -.7, .3])
        return BodyWrench(force, [.2*np.cos(t_s), -.15, .3])


class TestIntegrators(unittest.TestCase):
    def setUp(self):
        self.state = RigidBodyState([.1, -.2, .3], [.2, -.1, .4],
                                    [.8, .2, -.3, .4], [.7, -.4, .9])
        self.params = RigidBodyParams(1.3, np.diag([.7, 1.1, 1.6]), 0)

    def test_stage_values_are_unmodified(self):
        class Recorder:
            def __init__(self):
                self.samples = []
            def evaluate(self, t, state):
                self.samples.append((t, state.q_WB.copy()))
                return BodyWrench([0]*3, [0]*3)
        recorder = Recorder()
        h = .5
        start = state_vector(self.state)
        zero = BodyWrench([0]*3, [0]*3)
        k1 = dxdt(0, start, self.params, zero)
        expected = start + h*k1/2
        result, diag = RK4().step_with_diagnostics(0, self.state, self.params, recorder, h)
        k2 = dxdt(h/2, expected, self.params, zero)
        stage3 = start + h*k2/2
        k3 = dxdt(h/2, stage3, self.params, zero)
        stage4 = start + h*k3
        k4 = dxdt(h, stage4, self.params, zero)
        raw = start + h*(k1 + 2*k2 + 2*k3 + k4)/6
        np.testing.assert_array_equal(recorder.samples[1][1], expected[6:10])
        np.testing.assert_array_equal(recorder.samples[2][1], stage3[6:10])
        np.testing.assert_array_equal(recorder.samples[3][1], stage4[6:10])
        np.testing.assert_allclose(state_vector(result)[:6], raw[:6], atol=1e-15)
        np.testing.assert_allclose(result.q_WB, raw[6:10]/np.linalg.norm(raw[6:10]), atol=1e-15)
        self.assertAlmostEqual(diag.quaternion_norm_before, np.linalg.norm(raw[6:10]))
        self.assertGreater(abs(np.linalg.norm(recorder.samples[1][1])-1), 1e-3)
        self.assertEqual([t for t, _ in recorder.samples], [0, .25, .25, .5])
        self.assertAlmostEqual(np.linalg.norm(result.q_WB), 1)
        self.assertNotEqual(diag.quaternion_norm_error, 0)
        s = StageState.from_array(expected)
        before = s.q_WB.copy()
        np.testing.assert_allclose(s.physical_rotation().T @ s.physical_rotation(), np.eye(3),
                                   atol=1e-14)
        np.testing.assert_array_equal(s.q_WB, before)

    def test_free_and_ballistic(self):
        for gravity in (0, 9.80665):
            state = RigidBodyState([1, 2, 3], [2, -1, .5], [1, 0, 0, 0], [0]*3)
            out, diag = run(state, RigidBodyParams(1, np.eye(3), gravity), ZeroWrench(), .01, 500)
            np.testing.assert_allclose(out.p_W, state.p_W + 5*state.v_W +
                                       [0, 0, -.5*gravity*25], atol=1e-10 if gravity == 0 else 1e-9, rtol=0)
            self.assertEqual(len(diag), 500)

    def test_angular_invariants_and_rotation(self):
        # T08: floors 1e-15; sample each step, not just the endpoint.
        j = np.diag([1., 2., 3.])
        params = RigidBodyParams(1, j, 0)
        state = self.state
        e0 = .5*state.omega_B @ j @ state.omega_B
        h0 = rotation_matrix(state.q_WB) @ j @ state.omega_B
        max_e = max_h = 0.
        for k in range(10000):
            state, _ = RK4().step_with_diagnostics(k*.001, state, params, ZeroWrench(), .001)
            e = .5*state.omega_B @ j @ state.omega_B
            hh = rotation_matrix(state.q_WB) @ j @ state.omega_B
            max_e = max(max_e, abs(e-e0)/max(abs(e0), 1e-15))
            max_h = max(max_h, np.linalg.norm(hh-h0)/max(np.linalg.norm(h0), 1e-15))
        self.assertLess(max_e, 1e-6)
        self.assertLess(max_h, 1e-6)
        omega = np.array([.3, -.2, .4])
        state = RigidBodyState([0]*3, [0]*3, [1, 0, 0, 0], omega)
        out, _ = run(state, RigidBodyParams(1, np.eye(3)*2, 0), ZeroWrench(), .01, 200)
        n = np.linalg.norm(omega)
        exact = np.r_[np.cos(n), np.sin(n)*omega/n]
        np.testing.assert_allclose(out.q_WB, exact, atol=1e-10, rtol=0)

    def test_convergence_and_reference(self):
        wrench = ConstantBodyWrench(BodyWrench([.4, -.7, .3], [.2, -.15, .3]))
        reference = solve_reference(self.state, self.params, wrench, [0, 2])[-1]
        errors = []
        for n in (10, 20, 40, 80):
            state, _ = run(self.state, self.params, wrench, 2/n, n)
            # Quaternion sign aligned, Euclidean full-state error avoids acos floor.
            x, ref = state_vector(state), state_vector(reference)
            if np.dot(x[6:10], ref[6:10]) < 0:
                x[6:10] *= -1
            attitude = 4*np.arcsin(min(np.linalg.norm(x[6:10]-ref[6:10])/2, 1))
            errors.append(np.linalg.norm(np.r_[x[:6]-ref[:6], attitude, x[10:]-ref[10:]]))
        orders = np.log2(np.array(errors[:-1])/errors[1:])
        self.assertTrue(np.all(orders[:2] >= 3.5), (errors, orders))
        self.assertLess(np.linalg.norm(state.p_W-reference.p_W), 1e-5)
        self.assertLess(2*np.linalg.norm(state.q_WB-reference.q_WB), 1e-5)

    def test_attitude_dependent_wrench(self):
        wrench = AttitudeWrench()
        reference = solve_reference(self.state, self.params, wrench, [0, 2])[-1]
        out, _ = run(self.state, self.params, wrench, .02, 100)
        np.testing.assert_allclose(out.p_W, reference.p_W, atol=1e-7, rtol=0)
        # The model applies a constant world force despite changing attitude.
        expected = self.state.p_W + 2*self.state.v_W + 2*np.array([.4, -.7, .3])/1.3
        np.testing.assert_allclose(out.p_W, expected, atol=1e-7, rtol=0)

    def test_invalid_inputs(self):
        for dt in (0, -1, np.nan, np.inf, 1e-320):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                RK4().step(1, self.state, self.params, ZeroWrench(), dt)
        for t in (-1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                RK4().step(t, self.state, self.params, ZeroWrench(), .1)
        with self.assertRaises(ValueError):
            StageState.from_array(np.zeros(13))
        with self.assertRaises(ValueError):
            solve_reference(self.state, self.params, ZeroWrench(), [0, 0])
        class Bad:
            def evaluate(self, t, state):
                return BodyWrench([np.nan, 0, 0], [0]*3)
        with self.assertRaises(ValueError):
            RK4().step(0, self.state, self.params, Bad(), .1)
        w = ConstantBodyWrench(BodyWrench([1, 2, 3], [0]*3))
        self.assertEqual(w.evaluate(0, StageState.from_array(state_vector(self.state))).force_B_N[0], 1)
