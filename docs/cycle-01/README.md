# Aether · Cycle-01 / S0 Technical Baseline v1.0

**Status:** proposed implementation specification (not implementation or verified results)  
**Parent:** AETHER-RMP-001 v1.0 (2026-11-22), AETHER-RTP-001 v1.0 (2026-11-21)  
**Scope:** RMP-S0-01 through RMP-S0-04. S1 controller design is outside the gate.

## Contents
- `01_INTERFACE_CONTRACT.md` — coordinate frames, conventions, APIs, time, serialization.
- `02_DYNAMICS_MODEL.md` — 6-DOF rigid-body equations, physical assumptions, integration.
- `03_VERIFICATION_PLAN.md` — analytic invariants, convergence, property tests, reproducibility, release gate.
- `04_IMPLEMENTATION_BACKLOG.md` — executable work packages, dependency graph, acceptance criteria, Codex handoff.
- `scenario.example.yaml` — illustrative reproducible scenario (unit mass/inertia are mathematical placeholders, **not vehicle parameters**).

## What S0 does and does not prove
S0 validates numerical and coordinate conventions for a generic rigid body. It does **not** validate a real EDF flight model, prove closed-loop stability, calibrate any actuator, or certify hardware flight safety.

## Fixed decisions
World frame ENU; body frame FLU; active body-to-world quaternion `q_WB` in Hamilton `wxyz`; body rates in body axes; SI units; gravity in world `-z`; mass and inertia constant and positive; body wrench `[F_B, tau_B]`; deterministic fixed-step RK4 as initial reference, optional independent adaptive solver used as cross-check.

## Planned minimum command
`python -m aether_sil run scenarios/free_rotation.yaml --seed 7 --output runs/free_rotation/`

This is a **target command**, not a claim that executable code already exists.

## Exit gate
All required tests in `03_VERIFICATION_PLAN.md` pass on clean environment; a command reproduces an immutable report with config digest, code revision, solver settings and seed. Any exception requires a documented waiver.
