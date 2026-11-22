# S0-02/03 · 6-DOF Rigid Body and Numerical Model

**Tasks:** RMP-S0-02, RMP-S0-03 · **Taxonomy:** L.01.01–L.01.04  
**Status:** mathematical reference, not EDF-calibrated flight model.

## 1. Assumptions

Rigid body, constant mass `m`, constant SPD inertia `J_B` about COM; inertial world ENU; uniform gravity; all non-gravity forces and torques expressed in B; actuators and fluid dynamics represented only by externally supplied wrench. No aerodynamic damping, rotor gyroscopic reaction, fuel/mass-flow, ground contact, flexible-body dynamics, motor electrical dynamics or sensor error by default. Those are **omitted phenomena**, not assumed negligible for actual flight.

State `x = (p_W, v_W, q_WB, omega_B)`; 13 scalar coordinates, with one quaternion norm constraint, hence 12 physical degrees of freedom in phase space.

## 2. Continuous-time equations

Let `R=R(q_WB)`, `F_B` be non-gravity force about COM, `tau_B` be net applied COM torque. Then:

```math
\dot p_W = v_W,\quad
m \dot v_W = R_{WB} F_B + m g_W,
```

```math
\dot q_{WB} = \tfrac12 q_{WB}\otimes(0,\omega_B),\quad
J_B \dot\omega_B = \tau_B-\omega_B\times(J_B\omega_B).
```

Here `g_W=(0,0,-g)`; `||q||=1`; physical angular momentum in world `H_W=R_WB J_B omega_B`. Right-hand-rule rotation convention defined in interface contract.

**Dynamics API:** `dxdt(t, x, params, wrench)` returns derivatives in state field order; the integrator is separate. The implemented numerical path should not construct `J^{-1}` explicitly; solve `J alpha = tau - omega × (J omega)` with a linear solver.

## 3. Rotation and quaternion computation

For `q=(w, r)` and vector `v`:

```math
R(q)v = v + 2w(r\times v)+2[r\times(r\times v)]
```

(only valid as stated for a unit quaternion). Quaternion signs have identical physical meaning. After each accepted fixed step, normalize the quaternion; do not normalize intermediate derivative states silently, because it can alter numerical-order behavior. For tests, compare physical rotations rather than raw quaternion components.

## 4. Reference integrators

- **Initial implementation:** fixed-step classical RK4 of the entire coupled 13-component ODE, `q` normalized after the step; evaluate external wrench at all RK4 stage states/times. Emit diagnostics for quaternion norm before normalization.
- **Independent comparator:** SciPy `solve_ivp` with DOP853 (or RK45), strict documented `rtol`,`atol`, evaluate on same sample times. Do not use an identical copied RK4 code path as the sole comparison.
- **Cross-check:** analytic special cases and step refinement (`h`, `h/2`, `h/4`, `h/8`). Fourth-order convergence only expected in sufficiently smooth, well-conditioned test problems before roundoff dominates; discontinuous forces and stepwise switching invalidate naive fourth-order assertions.
- **Future upgrade:** geometric or Lie-group integrator as a research candidate, not required for Gate S0.

## 5. Illustrative mathematical test configurations

`m=1 kg`, `J=diag(1,2,3) kg·m²` and `q=(1,0,0,0)` are synthetic analytic validation fixtures. They are **not** estimates for the eventual vehicle. Work in unforced dynamics first, then continuous smooth wrench fields; keep the real EDF out until S1/S2.

## 6. Conservation / analytical references

- Zero applied body torque ⇒ rotational kinetic energy `E_rot = 0.5 omegaᵀ J omega` and inertial angular momentum vector `H_W` should remain constant (numerical error bounded).
- Zero applied force with `g=0` ⇒ `v_W` constant and `p_W=p0+v0 t`.
- Zero applied force with gravity ⇒ `v_W=v0+g_W t` and `p_W=p0+v0 t+0.5 g_W t²`.
- Spherical inertia `J=cI`, zero torque, constant `omega_B`: exact attitude `q(t)=q0⊗[cos(||omega||t/2), sin(||omega||t/2) omega/||omega||]`.
- At rest, `q` identity and force `F_B=(0,0,mg)` produces instantaneous zero vertical acceleration (for identity attitude); this is **force balance only**, not a proof of attitude-stable hovering.

## 7. No implicit EDF model

S0 may have a `ConstantBodyWrench` fixture and `ZeroWrench` fixture. A real EDF model requires separate validated parameters and input-to-wrench mapping, including spin reaction, tilt, rotor dynamics, airflow effects and constraints. Those belong to S1/S2 and shall be tagged `placeholder` until measured/validated.
