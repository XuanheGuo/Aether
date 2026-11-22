# Aether · Cycle-01 / S1 Research Specification

**Document ID:** AETHER-C01-S1-001 · **Version:** 1.0.0 · **Status:** Approved for implementation planning; not a completed S1 gate · **Date:** 2026-11-22

## Normative sources and boundaries
- `AETHER-RMP-001` Research Roadmap v1.0, especially RMP-S1-01…06, E0…E5 and Gate S1.
- `AETHER-RTP-001` Research Taxonomy v1.0; existing Topic identifiers must not be renumbered.
- S0 accepted synthetic 6-DOF baseline, tested code commit `6be3e48d764efbdc4fcfe5376145bc052041215a`; acceptance evidence/report commit `4d1f412214859d4f1bfb7a502f54107141035ce1`.

S1 goal: **a reproducible closed-loop rigid-body attitude-control research baseline**, not a flight-certified EDF controller, original publishable algorithm, or autonomous vehicle. Preserve S0 reference code, tests, and interface conventions; add controller / actuator / experiment layers without silently modifying S0 numerical baseline.

## Package reading order
1. `01_CONTROL_PROBLEM.md`: controlled system, tracking/settling, mathematical assumptions and edge cases.
2. `02_INTERFACE_AND_MODELS.md`: controller / actuator / estimator interfaces and simulation-time semantics.
3. `03_ALGORITHMS_AND_PROOFS.md`: PD, PID, local LQR, geometric controller and proof obligations.
4. `04_EXPERIMENT_AND_GATE.md`: E0–E5, metrics, statistics, acceptance and traceability.
5. `05_IMPLEMENTATION_BACKLOG.md`: phased work packages and Codex handoff.

## Fundamental research rules
- Mark all unsourced EDF-specific numerical coefficients **synthetic / placeholder**; do not present them as calibrated EDF parameters.
- Separate *attitude regulation* from *attitude trajectory tracking* and *hover equilibrium feasibility*. A torque-controlled rigid body does not establish that an EDF can hover.
- Prove only what assumptions support; numerical success does not imply global stability.
- Every comparison uses identical initial states, trajectories, sampled feedback information, constraints, disturbances and declared tuning protocol.
- Gate S1 may PASS without an original control algorithm or a paper, provided the Roadmap's credible baseline criteria are met.
