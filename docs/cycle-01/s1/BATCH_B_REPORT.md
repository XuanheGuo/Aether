# Cycle-01 S1 Batch B — S1-T04/T05

Tested code commit `64700d8c0fadb59c6afbe05d3f1488500d4ea9f5`, based on merged Batch A commit `6709bb0a848f39278ff9ee01836af0a4d7981c0d`. This report-only commit follows it. S0 and Batch A numerical code remain unchanged. This is not a Gate S1 decision.

## Models and boundaries

- `BoxLimit` is the symmetric box `|tau_i|<=L_i`, projected by componentwise clipping. `BallLimit` is the Euclidean ball `||tau||₂<=r`, projected radially. Membership and boundary cases are tested. Limits are strictly positive and finite.
- A0 sets actual torque equal to the held projected target at each control tick. A1 uses `T_i dot(tau_i)=target_i-tau_i`; requested and projected torque are distinct from actual torque. `ActuatedTrace` retains requested, projected, actual-at-control and actual-at-plant-sample torque with their respective timestamps.
- A1 torque is part of a 16-element rigid body plus torque state. Classical RK4 evaluates rigid-body and torque derivatives together at each stage; the controller and measurement execute only once per control tick. No state is hidden in S0's stateless wrench. Only the final accepted quaternion is normalized.
- The initial actual torque must be feasible. For lagged ball limits, the three time constants must be equal; with anisotropic lag, componentwise evolution can exit a ball even with feasible endpoints. The RK4 step is limited to `dt <= min(T_i)`, a conservative numerical restriction, not a proof of sampled closed-loop stability. There is no actuator rate limit or post-lag saturation; the tested box and equal-time-constant ball constructions retain feasible actual states under this restriction.
- The synthetic harness applies no non-gravity force and no disturbance. Constraint limits and time constants are mathematical fixtures, not measured EDF characteristics.

## Feasibility and equilibrium

Requested torque is feasible exactly when the relevant box or ball membership condition holds; projection gives the nearest Euclidean admissible vector. For zero applied torque, zero angular rate is an equilibrium at any fixed attitude; this says nothing about translating equilibrium. `hover_residual` independently evaluates `R_WB F_B + m(0,0,-g)`: for identity orientation, m=2 kg and `F_B=[0,0,19.6133]` N, residual is zero, while zero force produces a 19.6133 N residual. This is a synthetic force-balance check, not an EDF feasibility claim.

A numerical counterexample uses the same 2 s synthetic PD scenario with ideal projected actuation: a 10 N m box yields terminal angle **0.0155518 rad**, whereas a 0.001 N m box yields **0.371897 rad**. This is a finite-horizon constrained performance failure, not a global uncontrollability proof.

## Tests and numerical evidence

Clean checkout of the tested SHA, Linux x86_64, CPython 3.12.14, uv 0.13.9, NumPy 2.3.5, SciPy 1.17.0:

```sh
UV_CACHE_DIR=/tmp/aether-s1b-clean-cache uv sync --locked --extra test --python 3.12.14
.venv/bin/ruff check .
.venv/bin/pytest -q
.venv/bin/python scripts/s1b_evidence.py > /tmp/aether-s1b-clean-evidence.json
```

Ruff passed; **40 tests passed in 27.40 s**, including all S0 and Batch A tests and ten Batch B cases. The [code commit CI](https://github.com/XuanheGuo/Aether/actions/runs/44993782266) passed. Tests cover box/ball projection and boundary, invalid times/constants/initial state, zero derivative, analytical step response at three time constants, equal-tau ball feasibility, ideal versus lag, control update counts, quaternion sign, and coupled closed-loop step refinement.

For target [0.8,-0.4,0.2] N m, initial [0.2,-0.1,0.05] N m, t=0.2 s and dt=0.001 s, maximum absolute differences from the exact exponential are **5.960e-11**, **1.365e-11**, and **2.118e-13 N m** for time constants 0.05, 0.1 and 0.3 s respectively.

With control period fixed at 0.04 s and dt 0.04, 0.02, 0.01 s, terminal augmented-state differences against dt 0.005 s are **4.221e-6**, **2.176e-7**, **1.169e-8**. Changing control period from 0.04 to 0.01 s at fixed dt 0.005 changes the terminal state by **0.01424**, illustrating a different sampled system rather than an integration refinement. The error norm mixes quaternion, rad/s and N m components and is an engineering comparison, not a dimensionless universal tolerance.

At dt=0.01, control period 0.04, 0.5 N m box, terminal body rates for ideal and lagged actuator were respectively [-0.00954,0.09673,-0.05000] and [-0.00516,0.11561,-0.05799] rad/s. Maximum requested–projected component difference was 0.5548 N m; lagged actual–projected difference reached 0.5 N m. Full fixture and numerical results are recorded in [batch_b_evidence.json](batch_b_evidence.json).

## Limitations

No real EDF, nozzle, vane, rate limit, actuator output saturation, noise, disturbance, or E1–E5 large-scale sweep is modeled. The harness does not yet persist full trace files or provenance; it returns inspectable in-memory arrays and the small reproducible evidence script emits JSON. Failure/restart handling for this new harness is not an S1 gate artifact. The coupled solver and the constraint policy require independent review; no claim is made about the closed-loop stability for arbitrary time constants, limits, initial conditions or control sample rates. Build-tool hashes retain the S0 gate report limitations.
