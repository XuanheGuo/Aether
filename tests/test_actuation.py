"""Synthetic actuator constraints, dynamics and sampled closed-loop tests."""
import numpy as np
import pytest

from aether_sil.control.actuated_simulation import augmented_step, simulate_actuated
from aether_sil.control.actuators import BallLimit, BoxLimit, FirstOrderActuator
from aether_sil.control.controllers import GeometricPD
from aether_sil.control.feasibility import hover_residual
from aether_sil.control.geometry import attitude_errors
from aether_sil.control.interfaces import ConstantReference, IdealMeasurement
from aether_sil.dynamics import dxdt
from aether_sil.types import BodyWrench, RigidBodyParams, RigidBodyState


def fixture():
    p = RigidBodyParams(1, np.diag([1., 2., 3.]), 0)
    axis = np.array([1., -2., 1.])
    axis /= np.linalg.norm(axis)
    state = RigidBodyState([0]*3, [0]*3, np.r_[np.cos(.15), np.sin(.15)*axis], [.02, -.03, .01])
    return p, state


def test_projection_and_validation():
    box, ball = BoxLimit([1, 2, 3]), BallLimit(2)
    np.testing.assert_allclose(box.project([2, -3, .5]), [1, -2, .5])
    np.testing.assert_allclose(ball.project([3, 4, 0]), [1.2, 1.6, 0])
    assert box.contains([1, 2, 3]) and not box.contains([1.01, 0, 0])
    assert ball.contains([0, 2, 0]) and not ball.contains([0, 2.01, 0])
    np.testing.assert_equal(ball.project([0, 0, 0]), [0, 0, 0])
    for ctor, arg in ((BoxLimit, [1, 0, 1]), (BallLimit, 0), (BallLimit, np.nan),
                      (FirstOrderActuator, [.1, -.1, .2]), (FirstOrderActuator, [.1, np.nan, .2])):
        with pytest.raises(ValueError):
            ctor(arg)


@pytest.mark.parametrize('tau', [.05, .1, .3])
def test_step_response(tau):
    p, state = fixture()
    act = FirstOrderActuator([tau]*3)
    actual = np.array([.2, -.1, .05])
    initial = actual.copy()
    target = np.array([.8, -.4, .2])
    h = .001
    for k in range(200):
        state, actual = augmented_step(k*h, state, actual, p, act, target, h)
    np.testing.assert_allclose(actual, target + (initial-target)*np.exp(-.2/tau), atol=1e-8)
    np.testing.assert_equal(act.derivative([0]*3, [0]*3), [0]*3)
    with pytest.raises(ValueError):
        augmented_step(0, state, actual, p, act, target, tau*1.01)


def run(h, hc=.04, *, lag=True, qsign=1, box=.5, duration=2):
    p, s = fixture()
    s = RigidBodyState(s.p_W, s.v_W, qsign*s.q_WB, s.omega_B)
    act = FirstOrderActuator([.08, .1, .12]) if lag else None
    return simulate_actuated(s, p, GeometricPD(p, 4, 3), ConstantReference([1, 0, 0, 0]),
                             IdealMeasurement(), BoxLimit([box]*3), duration_s=duration,
                             dt_s=h, dt_control_s=hc, actuator=act)


def test_sampled_actuation_and_sign():
    a, b = run(.01), run(.01, qsign=-1)
    assert len(a.control_times_s) == 50
    np.testing.assert_allclose(a.requested_B_Nm, b.requested_B_Nm, atol=1e-12)
    np.testing.assert_allclose(a.actual_B_Nm, b.actual_B_Nm, atol=1e-12)
    np.testing.assert_allclose([s.q_WB for s in a.states], -np.array([s.q_WB for s in b.states]), atol=1e-12)
    assert np.max(np.abs(a.projected_B_Nm)) <= .5
    assert np.max(np.abs(a.actual_B_Nm)) <= .5 + 1e-12
    assert np.any(np.abs(a.requested_B_Nm-a.projected_B_Nm) > 1e-3)
    assert np.any(np.abs(a.actual_at_control_B_Nm-a.projected_B_Nm) > 1e-3)
    ideal = run(.01, lag=False)
    np.testing.assert_allclose(ideal.actual_at_control_B_Nm, ideal.projected_B_Nm)
    assert np.linalg.norm(ideal.states[-1].omega_B-a.states[-1].omega_B) > 1e-4


def test_closed_loop_refinement():
    traces = [run(h) for h in (.04, .02, .01, .005)]
    def end(t):
        s = t.states[-1]
        return np.r_[s.q_WB, s.omega_B, t.actual_B_Nm[-1]]
    errors = [np.linalg.norm(end(t)-end(traces[-1])) for t in traces[:-1]]
    assert errors[1] < errors[0]/5
    assert errors[2] < errors[1]/5
    # Changing control period changes the sampled-data problem, not only numerical accuracy.
    other = run(.005, hc=.01)
    assert np.linalg.norm(end(other)-end(traces[-1])) > 1e-5


def test_equilibrium_and_hover_are_distinct():
    p, s = fixture()
    x = np.r_[[0.]*6, s.q_WB, [0.]*3]
    np.testing.assert_allclose(dxdt(0, x, p, BodyWrench([0]*3, [0]*3))[6:], 0, atol=1e-15)
    np.testing.assert_allclose(hover_residual([1, 0, 0, 0], [0, 0, 19.6133], 2), [0]*3, atol=1e-12)
    assert np.linalg.norm(hover_residual([1, 0, 0, 0], [0]*3, 2)) > 19


def test_constraint_counterexample_and_bad_initial():
    ideal = run(.01, lag=False, box=10, duration=2)
    limited = run(.01, lag=False, box=.001, duration=2)
    def error(t):
        s = t.states[-1]
        return attitude_errors(s.q_WB, [1, 0, 0, 0], s.omega_B, [0]*3)[3]
    assert error(limited) > error(ideal) + .1
    p, s = fixture()
    with pytest.raises(ValueError):
        simulate_actuated(s, p, GeometricPD(p, 4, 3), ConstantReference([1, 0, 0, 0]),
                          IdealMeasurement(), BoxLimit([.1]*3), duration_s=1, dt_s=.01,
                          dt_control_s=.02, actuator=FirstOrderActuator([.1]*3), initial_actual=[1, 0, 0])
    with pytest.raises(ValueError):
        simulate_actuated(s, p, GeometricPD(p, 4, 3), ConstantReference([1, 0, 0, 0]),
                          IdealMeasurement(), BallLimit(1), duration_s=1, dt_s=.01,
                          dt_control_s=.02, actuator=FirstOrderActuator([.1, .2, .3]))


def test_actuated_scheduler_calls_once_per_tick():
    p, s = fixture()
    class Counter:
        def __init__(self):
            self.inner = GeometricPD(p, 4, 3)
            self.calls = []
        def reset(self, *, seed):
            self.inner.reset(seed=seed)
        def update(self, t, obs, ref, dt):
            self.calls.append(t)
            return self.inner.update(t, obs, ref, dt)
    class Measure(IdealMeasurement):
        def __init__(self):
            self.calls = []
        def sample(self, t, state, rng):
            self.calls.append(t)
            return super().sample(t, state, rng)
    controller, measure = Counter(), Measure()
    trace = simulate_actuated(s, p, controller, ConstantReference([1, 0, 0, 0]), measure,
                             BoxLimit([2]*3), duration_s=.4, dt_s=.01, dt_control_s=.04,
                             actuator=FirstOrderActuator([.1]*3))
    assert len(controller.calls) == len(measure.calls) == 10
    np.testing.assert_allclose(controller.calls, trace.control_times_s)
    np.testing.assert_allclose(measure.calls, trace.control_times_s)


def test_ball_lag_remains_feasible():
    p, s = fixture()
    limit = BallLimit(.4)
    trace = simulate_actuated(s, p, GeometricPD(p, 4, 3), ConstantReference([1, 0, 0, 0]),
                              IdealMeasurement(), limit, duration_s=1, dt_s=.01,
                              dt_control_s=.02, actuator=FirstOrderActuator([.1]*3),
                              initial_actual=[.4, 0, 0])
    assert np.all(np.linalg.norm(trace.actual_B_Nm, axis=1) <= .4 + 1e-12)
    assert np.all(np.linalg.norm(trace.projected_B_Nm, axis=1) <= .4 + 1e-12)
