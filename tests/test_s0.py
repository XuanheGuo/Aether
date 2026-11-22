"""Analytic and invariant checks; SciPy is a test-only independent integrator."""
import unittest

import numpy as np
from scipy.integrate import solve_ivp

from aether_sil.dynamics import dxdt
from aether_sil.frames import attitude_distance, quaternion_product, rotation_matrix
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState
from aether_sil.wrenches import ConstantBodyWrench, ZeroWrench


class TestFrames(unittest.TestCase):
    def test_t01_t03(self):
        q = np.array([np.sqrt(.5), 0, 0, np.sqrt(.5)])
        np.testing.assert_allclose(rotation_matrix([1, 0, 0, 0]), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(rotation_matrix(q) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(rotation_matrix(q), rotation_matrix(-q), atol=1e-12)
        self.assertLess(attitude_distance(q, -q), 1e-7)
        rng = np.random.default_rng(7)
        for _ in range(100):
            u = rng.normal(size=4)
            u /= np.linalg.norm(u)
            v = rng.normal(size=3)
            r = rotation_matrix(u)
            np.testing.assert_allclose(r.T @ r @ v, v, rtol=1e-12, atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(r), 1, places=12)

    def test_hamilton(self):
        np.testing.assert_allclose(quaternion_product([0, 1, 0, 0], [0, 0, 1, 0]),
                                   [0, 0, 0, 1])

    def test_t04_invalid(self):
        for m in (0, -1, np.nan):
            with self.assertRaises(ValueError):
                RigidBodyParams(m, np.eye(3))
        for j in (np.diag([1, 0, 1]), np.diag([1, -1, 1]), np.ones((3, 2)),
                  [[1, 1, 0], [0, 1, 0], [0, 0, 1]]):
            with self.assertRaises(ValueError):
                RigidBodyParams(1, j)
        for q in ([0, 0, 0, 0], [np.nan, 0, 0, 0]):
            with self.assertRaises(ValueError):
                RigidBodyState([0]*3, [0]*3, q, [0]*3)
        self.assertEqual(RigidBodyState([0]*3, [0]*3, [2, 0, 0, 0], [0]*3).q_WB[0], 1)
        with self.assertRaises(ValueError):
            BodyWrench([np.nan, 0, 0], [0]*3)


class TestDynamics(unittest.TestCase):
    def integrate(self, x, params, duration):
        zero = BodyWrench([0]*3, [0]*3)
        result = solve_ivp(lambda t, y: dxdt(t, y, params, zero), (0, duration), x,
                           method='DOP853', rtol=1e-12, atol=1e-13)
        self.assertTrue(result.success)
        return result.y[:, -1]

    def test_t05_t06(self):
        x = np.r_[[1., 2, 3], [2., -1, .5], [1., 0, 0, 0], [0., 0, 0]]
        for g in (0., 9.80665):
            out = self.integrate(x, RigidBodyParams(1, np.eye(3), g), 5)
            np.testing.assert_allclose(out[:3], x[:3] + x[3:6]*5 + [0, 0, -.5*g*25],
                                       atol=1e-9, rtol=0)
            np.testing.assert_allclose(out[3:6], x[3:6] + [0, 0, -g*5], atol=1e-10)

    def test_t07_spherical(self):
        omega = np.array([.3, -.2, .4])
        x = np.r_[[0.]*6, [1., 0, 0, 0], omega]
        out = self.integrate(x, RigidBodyParams(1, np.eye(3)*2, 0), 2)
        n = np.linalg.norm(omega)
        exact = np.r_[np.cos(n), np.sin(n)*omega/n]
        self.assertLess(attitude_distance(out[6:10]/np.linalg.norm(out[6:10]), exact), 1e-7)
        np.testing.assert_allclose(out[10:], omega, atol=1e-12)

    def test_invariants(self):
        j = np.diag([1., 2, 3])
        x = np.r_[[0.]*6, [1., 0, 0, 0], [.3, -.2, .4]]
        out = self.integrate(x, RigidBodyParams(1, j, 0), 10)
        def e(w):
            return .5*w @ j @ w
        self.assertLess(abs(e(out[10:])-e(x[10:]))/e(x[10:]), 1e-6)
        h0 = j @ x[10:]
        h1 = rotation_matrix(out[6:10]/np.linalg.norm(out[6:10])) @ j @ out[10:]
        self.assertLess(np.linalg.norm(h1-h0)/np.linalg.norm(h0), 1e-6)

    def test_t09_and_body_force(self):
        p = RigidBodyParams(2, np.eye(3))
        x = np.r_[[0.]*6, [1., 0, 0, 0], [0.]*3]
        w = BodyWrench([0, 0, 2*9.80665], [0]*3)
        self.assertLess(abs(dxdt(0, x, p, w)[5]), 1e-12)
        x[6:10] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
        np.testing.assert_allclose(dxdt(0, x, RigidBodyParams(1, np.eye(3), 0),
                                      BodyWrench([1, 0, 0], [0]*3))[3:6], [0, 1, 0],
                                   atol=1e-12)

    def test_wrench_models(self):
        state = RigidBodyState([0]*3, [0]*3, [1, 0, 0, 0], [0]*3)
        w = BodyWrench([1, 2, 3], [4, 5, 6])
        np.testing.assert_equal(ConstantBodyWrench(w).evaluate(0, state).force_B_N, [1, 2, 3])
        np.testing.assert_equal(ZeroWrench().evaluate(0, state).torque_B_Nm, [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
