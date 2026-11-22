# S0 · Implementation Tasks and Codex Handoff

**Authority:** AETHER-RMP-001 v1.0 §4.2, §7.1, §10. This document decomposes RMP-S0-01–04 without adding S1 scope.

## 1. Technology baseline (proposal)

- Python 3.12+, NumPy, SciPy, pytest, PyYAML; `ruff` optional. Pin exact versions in lockfile at implementation time.
- First numerical reference uses float64, classical fixed-step RK4; SciPy DOP853 independently cross-checks.
- No dependency on PX4, ROS, Gazebo, game engines or GPU in S0. These can be introduced later behind interfaces.
- Use Git; config files and scripts are source-controlled; run output goes to ignored `runs/` and preserves manifests.

## 2. Work packages (ordered by hard dependencies)

| Ticket | Roadmap task | Deliverable | Done when | Depends |
|---|---|---|---|---|
| S0-T01 | RMP-S0-01 | `docs/interface_contract.md`, `frames.py` | T01–T04 pass; sign/axis examples documented | none |
| S0-T02 | RMP-S0-02 | `types.py`, `dynamics.py` with physical validation | analytic derivative and translational tests T05,T06,T09 pass | T01 |
| S0-T03 | RMP-S0-02 | `wrenches.py` zero/constant wrenches; quaternion kinematics | T07 and initial invariant tests pass | T02 |
| S0-T04 | RMP-S0-03 | `integrators/rk4.py`, normalization diagnostics | T08 and refinement scaffold work | T03 |
| S0-T05 | RMP-S0-03 | `integrators/reference.py` SciPy comparison | T10,T11 pass with recorded tolerances | T04 |
| S0-T06 | RMP-S0-04 | `scenarios.py`, YAML schema, `cli.py` | `run` CLI can generate deterministic trajectories | T04 |
| S0-T07 | RMP-S0-04 | `manifest.py`, `metrics.py`, file writer | T12–T14 pass and provenance is complete | T05,T06 |
| S0-T08 | All S0 | CI, clean install check, `S0_GATE_REPORT.md` | all tests pass on clean environment; verdict and residual risks | T01–T07 |

## 3. Repository target tree

```text
Aether/
  docs/
    RESEARCH_TAXONOMY.md
    RESEARCH_ROADMAP.md
    cycle-01/
      README.md
      interface_contract.md
      dynamics_model.md
      verification_plan.md
      S0_GATE_REPORT.md         # generated after running tests; absent initially
  src/aether_sil/
    __init__.py
    types.py
    frames.py
    dynamics.py
    wrenches.py
    integrators/{rk4.py,reference.py}
    scenarios.py
    metrics.py
    manifest.py
    cli.py
  scenarios/
    free_rotation.yaml
    ballistic.yaml
    smooth_wrench.yaml
  tests/
    test_frames.py
    test_validation.py
    test_analytic_cases.py
    test_invariants.py
    test_numerical_convergence.py
    test_cli_reproducibility.py
  pyproject.toml
  uv.lock                  # or equivalent pinned lockfile
  runs/                   # gitignored
```

## 4. Non-goals / scope guard

Do **not** implement feedback controllers (PD/PID/LQR), ESC, real-time firmware, IMU estimator, EDF flow map, CAD/CFD, free flight, HIL or physical tests in S0. Reserve abstractions and transition notes only. `WrenchModel` is applied net force/torque; it is not an actual actuator command.

## 5. Codex starter instruction (copyable)

> Implement Aether Cycle-01 S0 **only** from the five files in this package, preserving ENU/FLU, `q_WB` Hamilton `wxyz`, body-frame `omega`, force and torque about COM, gravity added once, and SI units. Start with T01–T04 and tests. Use Python, NumPy, SciPy, pytest; no real EDF or controller yet. Build each ticket S0-T01 to S0-T08 in order; run full tests after each. Write measured results to `S0_GATE_REPORT.md` only after test execution. Never report a pass without actually running the tests. Track assumptions and unknown vehicle parameters explicitly.

## 6. Deliverable and review checklist

- All physics and coordinate conventions in one source of truth; no contradictory duplicated definitions.
- Every publicly callable function documents frames, units, shapes and allowed values.
- Every test uses an analytic, independent, or invariant reference; not only self-comparison.
- Any change to mass/inertia, coordinate conventions, solver, or recorded schema triggers regression.
- On passing S0: snapshot a Git release tag `s0-baseline-v1.0`, freeze test fixtures, and begin RMP-S1-01/02. **Tag only after passing**.
