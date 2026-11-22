# 02 · Interfaces, Sampled Control and Actuators

**Task mapping:** RMP-S1-01, 02 · **Taxonomy:** A.03.02/03, C.01.04, L.02.01/02, L.07.01

## 1. Required interfaces
Keep S0 immutable outside explicit bug fixes, with gate regression. S1 adapters (illustrative names; implementation may differ if documented):

```python
class AttitudeReference(Protocol):
    def sample(self, t_s: float) -> Reference: ...  # R_d/q_d, omega_d, omega_dot_d if needed
class AttitudeController(Protocol):
    def reset(self, *, seed: int) -> None: ...
    def update(self, t_s: float, measurement: Measurement,
               reference: Reference, dt_control_s: float) -> TorqueCommand: ...
class TorqueActuator(Protocol):
    def derivative(self, t_s: float, actual_tau_B: Vec3,
                   held_command_B: Vec3) -> Vec3: ...
class MeasurementModel(Protocol):
    def sample(self, t_s: float, true_state: RigidBodyState, rng: Generator) -> Measurement: ...
```

`TorqueCommand`: body-frame Nm, finite, timestamped, semantic `requested torque`, separate from `actual torque`. `Measurement`: timestamp plus q_WB and omega_B (ideal in E0); clearly identify latency and noise. Disturbance is a separate model, not hidden in the command.

## 2. Execution order
Controllers operate at `t_k=k*dt_control` and update the held command once per tick; integrator uses zero-order-hold command during interval `[t_k,t_{k+1})`. Substages of RK4 **must not rerun** the controller or measurement model. The plant/actuator internal dynamics may be evaluated at every RK4 substage. If `dt_control` differs from `dt_integrator`, require an integer ratio or a formally defined scheduler with no skipped updates. All timestamps derive from integer tick indices, not accumulated float addition.

## 3. Torque actuator model hierarchy
**A0 ideal:** `tau_actual = clip(tau_cmd, box or norm bound)`, with bound disabled for explicitly ideal tests. For per-axis clipping, the admissible set is a box; a Euclidean-ball constraint is different and must not be confused.

**A1 first-order:** `T dot(tau_actual) = tau_target - tau_actual` with `T=diag(Tx,Ty,Tz)` positive. `tau_target` is command-projected into a declared feasible torque set. `tau_actual` is a **dynamical state** integrated jointly with the rigid body, not mutated inside a stateless `WrenchModel`. Specify optional output saturation and rate limits; do not claim exact dynamics if rate and saturation order is unspecified.

**A2 work-domain constraints:** add torque-rate limit, actuator effectiveness matrix and mismatch, explicitly labeled synthetic. Defer actual EDF shaft response, vanes, nozzle and cross-coupling to S2.

## 4. Important architectural constraint
The S0 `WrenchModel` is stateless. A1 adds at least three actual torque states, so use an *augmented plant* and a coupled ODE whose RK4 stages include both body and actuator. Do not hide actuator state in `evaluate()` or update a state variable on RK4 substage calls. Retain old S0 `dxdt` and RK4 regression tests unchanged as compatibility targets.

## 5. Optional measurement disturbances
Ideal full-state feedback for first baseline. Later Gaussian IMU-like noise must be sampled at control ticks from an explicit RNG, not independently at RK4 substeps. Specify distribution, units, covariance, bias and sample-and-hold; do not label this an inertial navigation solution. Any filtering or state estimation added in S1 must be a transparent placeholder and separately evaluated.

## 6. Logging
At each control tick: `R/q`, `omega`, reference, measured state, requested torque, projected/limited torque, actual torque, feedback timestamp, saturation mask. At plant samples: rigid-body state and actuator states; store both control and integration rates and distinguish modeled disturbance from actuator output. Preserve S0 manifest schema through versioned additions; add contract version only if a breaking change is made.
