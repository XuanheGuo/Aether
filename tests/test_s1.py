"""S1 regulation, local linearization and scheduler tests."""
import numpy as np
import pytest
from scipy.linalg import solve_continuous_are

from aether_sil.control.controllers import GeometricPD, LocalLQR
from aether_sil.control.geometry import attitude_errors, local_rotation_vector
from aether_sil.control.interfaces import ConstantReference, IdealMeasurement, Measurement, Reference
from aether_sil.control.simulation import simulate
from aether_sil.dynamics import dxdt
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState


def setup_case():
    params = RigidBodyParams(1, np.diag([1., 2., 3.]), 0)
    axis = np.array([1., -2., 1.])
    axis /= np.linalg.norm(axis)
    q = np.r_[np.cos(.15), np.sin(.15)*axis]
    state = RigidBodyState([0]*3, [0]*3, q, [.02, -.03, .01])
    return params, state


def test_geometric_error_and_sign():
    q = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
    e, ew, psi, angle = attitude_errors(q, [1, 0, 0, 0], [0, 0, .2], [0, 0, 0])
    np.testing.assert_allclose(e, [0, 0, 1], atol=1e-12)
    np.testing.assert_allclose(ew, [0, 0, .2], atol=1e-12)
    assert psi == pytest.approx(1)
    assert angle == pytest.approx(np.pi/2)
    np.testing.assert_allclose(attitude_errors(-np.array(q), [-1, 0, 0, 0], [0, 0, .2], [0, 0, 0])[0], e)
    e_pi, _, _, theta = attitude_errors([0, 1, 0, 0], [1, 0, 0, 0], [0]*3, [0]*3)
    assert np.linalg.norm(e_pi) < 1e-14
    assert theta == pytest.approx(np.pi)


def test_pd_formula_and_lqr_care():
    p, state = setup_case()
    obs = Measurement(0, state.q_WB, state.omega_B)
    ref = Reference(0, [1, 0, 0, 0], [0]*3)
    pd = GeometricPD(p, 4, 3)
    e, w, _, _ = attitude_errors(obs.q_WB, ref.q_WD, obs.omega_B, ref.omega_D)
    expected = -4*e - 3*w + np.cross(w, p.J_B_kg_m2 @ w)
    np.testing.assert_allclose(pd.update(0, obs, ref, .02).torque_B_Nm, expected)
    lqr = LocalLQR(p, np.diag([4., 4., 4., 3., 3., 3.]), np.eye(3))
    residual = lqr.A.T@lqr.P + lqr.P@lqr.A - lqr.P@lqr.B@np.linalg.solve(lqr.R, lqr.B.T@lqr.P) + lqr.Q
    np.testing.assert_allclose(residual, 0, atol=1e-10)
    np.testing.assert_allclose(lqr.P, solve_continuous_are(lqr.A, lqr.B, lqr.Q, lqr.R))
    assert np.max(np.linalg.eigvals(lqr.A-lqr.B@lqr.K).real) < 0
    for ctrl in (pd, lqr):
        u = ctrl.update(0, obs, ref, .02).torque_B_Nm
        uflip = ctrl.update(0, Measurement(0, -state.q_WB, state.omega_B),
                            Reference(0, [-1, 0, 0, 0], [0]*3), .02).torque_B_Nm
        np.testing.assert_allclose(u, uflip, atol=1e-12)
    with pytest.raises(ValueError):
        lqr.update(0, Measurement(0, [0, 1, 0, 0], [0]*3), ref, .02)


def test_local_linearization():
    p, _ = setup_case()
    eps = 1e-7
    v = np.array([.3, -.4, .5])
    v /= np.linalg.norm(v)
    q = np.r_[np.cos(eps/2), np.sin(eps/2)*v]
    np.testing.assert_allclose(local_rotation_vector(q, [1, 0, 0, 0])/eps, v, atol=1e-10)
    x = np.r_[[0.]*6, [1., 0, 0, 0], [0.]*3]
    tau = eps*v
    d = dxdt(0, x, p, BodyWrench([0]*3, tau))
    np.testing.assert_allclose(d[10:]/eps, np.linalg.solve(p.J_B_kg_m2, v), atol=1e-12)


def test_closed_loop_and_control_schedule():
    p, initial = setup_case()
    class CountMeasurement(IdealMeasurement):
        calls = []
        def sample(self, t_s, true_state, rng):
            self.calls.append(t_s)
            return super().sample(t_s, true_state, rng)
    class CountingPD:
        def __init__(self):
            self.inner = GeometricPD(p, 4, 3)
            self.times = []
        def reset(self, *, seed):
            self.inner.reset(seed=seed)
        def update(self, t_s, measurement, reference, dt_control_s):
            self.times.append(t_s)
            return self.inner.update(t_s, measurement, reference, dt_control_s)
    controllers = [CountingPD(), LocalLQR(p, np.diag([4., 4., 4., 3., 3., 3.]), np.eye(3))]
    for controller in controllers:
        measure = CountMeasurement()
        measure.calls = []
        trace = simulate(initial, p, controller, ConstantReference([1, 0, 0, 0]), measure,
                         duration_s=8, dt_s=.01, dt_control_s=.02, seed=7)
        assert len(trace.control_times_s) == 400
        np.testing.assert_allclose(trace.control_times_s, np.arange(400)*.02, atol=1e-15)
        np.testing.assert_allclose(measure.calls, trace.control_times_s)
        if isinstance(controller, CountingPD):
            np.testing.assert_allclose(controller.times, trace.control_times_s)
        end = trace.states[-1]
        _, _, _, angle = attitude_errors(end.q_WB, [1, 0, 0, 0], end.omega_B, [0]*3)
        assert angle < .02
        assert np.linalg.norm(end.omega_B) < .02
        assert trace.commands_B_Nm.shape == (400, 3)
    with pytest.raises(ValueError):
        simulate(initial, p, controllers[0], ConstantReference([1, 0, 0, 0]), IdealMeasurement(),
                 duration_s=1, dt_s=.01, dt_control_s=.015)


def test_closed_loop_antipodal_trajectory():
    p, initial = setup_case()
    flipped = RigidBodyState(initial.p_W, initial.v_W, -initial.q_WB, initial.omega_B)
    for controller in (GeometricPD(p, 4, 3),
                       LocalLQR(p, np.diag([4., 4., 4., 3., 3., 3.]), np.eye(3))):
        traces = [simulate(s, p, controller, ConstantReference(q), IdealMeasurement(),
                           duration_s=1, dt_s=.01, dt_control_s=.02)
                  for s, q in ((initial, [1, 0, 0, 0]), (flipped, [-1, 0, 0, 0]))]
        np.testing.assert_allclose(traces[0].commands_B_Nm, traces[1].commands_B_Nm, atol=1e-12)
        for a, b in zip(traces[0].states, traces[1].states, strict=True):
            np.testing.assert_allclose(a.q_WB, -b.q_WB, atol=1e-12)
            np.testing.assert_allclose(a.omega_B, b.omega_B, atol=1e-12)


def test_body_rate_transform_and_invalid_inputs():
    q = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
    _, rate, _, _ = attitude_errors(q, [1, 0, 0, 0], [0]*3, [1, 0, 0])
    np.testing.assert_allclose(rate, [0, 1, 0], atol=1e-12)
    p, _ = setup_case()
    with pytest.raises(ValueError):
        LocalLQR(p, -np.eye(6), np.eye(3))
    with pytest.raises(ValueError):
        LocalLQR(p, np.eye(6), np.zeros((3, 3)))
    with pytest.raises(ValueError):
        Measurement(0, [0]*4, [0]*3)
