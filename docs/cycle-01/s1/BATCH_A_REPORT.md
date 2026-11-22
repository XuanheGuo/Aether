# Cycle-01 S1 Batch A — implementation report

Tested code commit: `a2068d1e8c8e2f2c3f62ec737b90937fa41b2028`. The report-only commit follows that revision. S0 accepted code and tests are unchanged. This is not a Gate S1 decision.

## Scope and interfaces

S1-T01–T03 implement `Reference`, `Measurement`, `TorqueCommand` and stateless sampled controller/reference/measurement protocols, SO(3) errors, geometric PD, local continuous CARE LQR, and a minimal sampled harness. Quaternion conventions and gravity follow S0. Desired angular rates are expressed in desired frame D; the two implemented controllers reject nonzero desired rates and support fixed-attitude regulation only. Requested body torque is passed directly to the synthetic plant as ideal, unlimited actual COM torque; zero non-gravity force, zero disturbance and ideal measurements are used. No EDF allocation or actuator dynamics are modeled.

`e_R = vee(RdᵀR - RᵀRd)/2` and `e_omega = omega_B - RᵀRd omega_D`. Geodesic error uses a rotation-matrix based principal angle and remains meaningful at pi, where `e_R` vanishes. B0 uses `tau=-4 e_R-3 omega+omega×Jomega`, assuming exact known J and ideal torque. For this ideal continuous model, `V=omegaᵀJomega/2+4 Psi` has `Vdot=-3||omega||²`; this alone does not establish global stability. The pi critical set and discrete sampling prevent a global claim.

B1 uses local `x=[log(RdᵀR), omega_B]` (rotation vector in D, body rate in B); frames coincide at equilibrium to first order. `A=[[0,I],[0,0]]`, `B=[[0],[solve(J,I)]]`, Q=diag(4,4,4,3,3,3), R=I. `P` solves `AᵀP+PA-PBR⁻¹BᵀP+Q=0` and `K=solve(R,BᵀP)`; CARE residual and Hurwitz eigenvalues are tested. The nonlinear controller rejects attitude errors greater than 0.5 rad; this is a declared local chart bound, not a proven region of attraction. No feedforward, actuator limit, tracking or uncertainty is claimed.

The simulation requires integer control-to-RK4 and duration-to-step ratios. Controller, reference and measurement run once per control tick; the resultant torque is held constant across all RK4 substages and integration substeps. Plant and control samples use integer tick products. The final plant endpoint is not an extra controller tick.

## Reproduction and tests

Clean checkout at the tested SHA, Linux x86_64, Python 3.12.14, uv 0.13.9, NumPy 2.3.5, SciPy 1.17.0:

```sh
UV_CACHE_DIR=/tmp/aether-s1-check-cache uv sync --locked --extra test --python 3.12.14
.venv/bin/ruff check .
.venv/bin/pytest -q
.venv/bin/python scripts/s1_compare.py --output /tmp/aether-s1-clean
```

Ruff passed; **30 tests passed in 31.47 s**, including all 24 existing S0 tests and six S1 tests. Tests cover SO(3) signs and pi, desired-rate frame transformation, PD equation, CARE residual/eigenvalues, small-angle rotation vector and torque linearization, closed-loop sign equivalence, scheduler update count and invalid ratio/domain. [CI for tested commit](https://github.com/XuanheGuo/Aether/actions/runs/44990183922) passed. The environment inherits S0's documented build-tool hash limitation.

## Synthetic common-case comparison

The same initial attitude (0.3 rad around normalized [1,-2,1]), initial rate [0.02,-0.03,0.01] rad/s, inertia diag(1,2,3) kg m², initial translation, zero gravity, duration 8 s, RK4 dt 0.01 s, control period 0.02 s, seed 7 and ideal full-state measurements were used for both. Synthetic Q/R and PD gains were selected for a working baseline, not through a controlled tuning or held-out protocol. Full fixture, version data, and trace hashes are in [batch_a_comparison.json](batch_a_comparison.json). The script regenerates q, body rate, measured/reference states, requested/actual torque traces and metrics as NPZ/JSON.

| Controller | Final attitude error rad | Final rate rad/s | Settling time s | Torque effort N²m²s |
|---|---:|---:|---:|---:|
| Geometric PD | 0.00069808 | 0.00260987 | 4.54 | 0.57361 |
| Local LQR | 0.00127605 | 0.00088653 | 3.94 | 0.15739 |

Settling means both angle and rate remain below 0.02 until the end of this finite horizon. Effort is the sum of held actual torque squared times the control period, a proxy not electrical energy. These are observations for one case, not a general performance ranking or proof of global stability.

## Remaining work and limitations

No first-order actuator, saturation, control allocation, disturbances, measurement noise, time-varying tracking, sweeps or true EDF are included. Torque feasibility and hover are not assessed. No S1 E1–E5 study, formal proof package, independent closed-loop solver convergence study, or S1 gate decision has been performed. Batch B should add the separately specified coupled actuator dynamics and feasibility study without hiding actuator state in S0's stateless wrench interface.
