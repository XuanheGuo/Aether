# 04 · Experimental Matrix, Metrics and Gate S1

**Task mapping:** RMP-S1-03, 04, 06 · **Taxonomy:** C.02.01/04, C.07.05, K.03.04, K.06.04, L.07.02/03

## 1. E0–E5 experiments (from Roadmap)
| ID | Experiment | Minimum variation | Expected evidence |
|---|---|---|---|
| E0 | Nominal | same fixed inertia, ideal feedback, zero disturbance | regulation trajectory, command/actual torque, comparison |
| E1 | Initial-condition scan | multiple rotations incl. near pi, multiple body rates | success/failure map with seed and criterion |
| E2 | Model uncertainty | true inertia vs controller-internal inertia; synthetic effectiveness mismatch | distribution of errors, robust failure boundary |
| E3 | Actuator dynamics | positive first-order lag time constants; sample clock | response time, overshoot, effect of lag |
| E4 | Constraints | torque box or ball, optional rate limit, declared feasible set | saturation occupancy, max torque, recovery/failure |
| E5 | Disturbance and noise | time-varying bounded torques; independently seeded measurement noise | error distributions, delay sensitivity, pathological cases |

## 2. Unified test protocol
- Freeze scenario fixtures before tuning comparisons. Separate tuning cases from held-out evaluation cases.
- Same initial states, disturbance sequences and controller sampling rates for both baselines; record actual torque separately from requested torque.
- Use at least one deliberately favorable and one challenging case; no cherry-picked trajectory-only claims.
- For random sweeps, predeclare seed list, distribution, number of runs, and failure-handling procedure. Report successes and failures, confidence intervals where statistical estimation is meaningful; do not treat deterministic parameter-grid counts as sampling confidence intervals.
- Run a smaller high-accuracy solver / step-refinement subset to distinguish controller failure from numerical instability.

## 3. Outcome definitions
- `theta_error_rad`: geodesic attitude distance in `[0,pi]`.
- `omega_error_rad_s`: body-frame rate error to the appropriately transformed reference.
- `settling_time_s`: first time after which both errors remain below **predeclared** thresholds until the end of a sufficiently long horizon. If not settled, record `not_reached`, not zero.
- `overshoot`: only define for a specified signed scalar maneuver, not a general SO(3) rotation without a chosen axis.
- `torque_effort_Nm2_s`: time integral `∫||tau_actual||² dt`; energy proxy, **not electrical energy**.
- `saturation_fraction`: share of control ticks with requested torque outside the feasible set.
- `failure_category`: `unsettled`, `constraint_violation`, `numerical_failure`, `model_domain_exceeded`, `invalid_setup`, etc. Distinguish these explicitly.

Avoid declaring universal pass thresholds before the fixture and controller normalization is frozen. Before executing Gate S1, publish a `gate_thresholds.yaml` with numeric tolerances, time horizons, reference trajectories, seeds and expected test semantics. Threshold revisions require reasons and a new review entry.

## 4. Feasibility analysis
For an abstract full torque box `|tau_i|<=tau_max_i`, the feasible set is stated explicitly. Under zero torque and `omega=0`, *any constant attitude* is a rigid-body rotational equilibrium. A hovering *translational* equilibrium additionally requires `R F_B + m g_W = 0`. S1 can demonstrate feasibility only for synthetic `F_B`, never for a real EDF vector/nozzle/vane without calibrated effectivity. Record uncontrollable/failure regions due to saturation but do not confuse small reachable torque with reachability of all SO(3) attitudes.

## 5. Gate S1 requirements
**G1** B0 geometric PD and B1 local LQR run using shared interfaces and identical specified scenarios; optional PID is not required.
**G2** At least one properly stated controller derivation/proof memo with assumptions, critical equilibria and limitations; LQR local stability derivation recorded.
**G3** E0–E5 executable, reproducible from frozen config/seed/SHA, with all failed runs preserved.
**G4** Constraints and lag enter inside the simulation plant; not post-hoc filtered result curves.
**G5** Closed-loop numerical validation has convergence / step-size or independent reference evidence for selected cases.
**G6** Results include metrics, comparability checks, uncertainty/validity ranges, and a report distinguishing proved vs observed behavior.
**G7** CI/lint/tests/clean install and versioned artifacts succeed, with existing Gate S0 regressions passing.

Gate decision `PASS / FAIL / NOT RUN / WAIVED` for each criterion. Gate S1 PASS does **not** certify a real EDF actuator, estimator, hovering capability, original algorithm or flight safety.
