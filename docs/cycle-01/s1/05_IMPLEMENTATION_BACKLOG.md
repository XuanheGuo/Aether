# 05 · Implementation Backlog and Codex Handoff

**Status:** implementation planned, not started. Baseline: accepted S0 tested SHA `6be3e48d764efbdc4fcfe5376145bc052041215a`; report SHA `4d1f412214859d4f1bfb7a502f54107141035ce1`.

| Work package | Roadmap ID | Deliverable | Dependencies | Acceptance |
|---|---|---|---|---|
| S1-T01 | RMP-S1-01 | control `Reference/Measurement/TorqueCommand`, scheduler, geometric error functions | S0 | unit tests: frames, q/-q, sample timing |
| S1-T02 | RMP-S1-01 | geometric PD regulation baseline | T01 | math formula test, nominal closed-loop run |
| S1-T03 | RMP-S1-01 | local attitude LQR baseline and documented linearization | T01 | CARE/eigenvalue checks and same-scene baseline |
| S1-T04 | RMP-S1-02 | ideal torque limits, augmented first-order torque plant | T01/T02 | actuator integrated by RK4 stages, no hidden mutable substep state |
| S1-T05 | RMP-S1-03 | torque feasible-set and synthetic hover balance analyses | T04 | analytic cross-check; no real EDF claims |
| S1-T06 | RMP-S1-04 | E0/E1 scenario runner, log schemas, closed-loop regression | T02/T03 | reproducible metrics, failures retained |
| S1-T07 | RMP-S1-04 | E2–E5 sweeps, seeded measurement/disturbance models | T04/T06 | common random inputs, stress/constraint scans |
| S1-T08 | RMP-S1-05 | geometric PD Lyapunov memo and local LQR proof | T02/T03 | reviewed assumptions, critical-set caveats |
| S1-T09 | RMP-S1-06 | statistical/evaluation report, model validity and failure maps | T05–T08 | reconstruct tables/plots from versioned outputs |
| S1-T10 | all S1 | formal Gate S1 and artifact freeze | T01–T09 | G1–G7 status, CI, evidence, explicit decision |

## Delivery sequencing
**Batch A (first Codex task):** S1-T01/T02/T03, plus the minimum closed-loop harness needed to run two baseline controllers. No actuator lag, parameter sweeps, real EDF or new algorithms. Stop and report for review.
**Batch B:** S1-T04/T05, including augmented actuator dynamics and feasibility.
**Batch C:** S1-T06/T07 experiment matrix.
**Batch D:** S1-T08/T09 theory and evidence.
**Batch E:** S1-T10 gate.

## Required branch/commit discipline
Prefer feature branch `cycle-01/s1-baselines` and PR review. Do not overwrite/falsify historical S0 gate evidence. Keep S0 tests passing and explicitly record any interface migrations. Commit implementation and reports with full SHA and actual test output; do not mark tasks complete without execution.

## Copy-ready instruction for Codex — Batch A
You are implementing Aether Cycle-01 S1, starting from S0 accepted baseline. Read the complete S1 specification package, the S0 interface contract, mathematical model, S0 gate report and current repo before editing. Implement **only S1-T01 to T03** and the minimal closed-loop runner to compare geometric PD regulation and local LQR on identical synthetic rigid-body scenarios. Use ENU/FLU, Hamilton wxyz q_WB, SI units and the S0 numerical baseline unchanged. The controller returns requested body torque; no real EDF mapping. Controller updates only at declared control ticks, not RK4 substages. Include unit tests for SO(3) error, q versus -q, local LQR linearization/CARE and fixed-rate scheduling. Preserve clean versions and S0 regressions; run available tests and lint. Write an implementation report listing code SHA, files, actual commands/results, assumptions, numerical metrics, known gaps and next batch. **Stop after Batch A for independent review.**
