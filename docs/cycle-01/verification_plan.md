# S0-03/04 · Verification and Reproducibility Plan

**Tasks:** RMP-S0-01 through RMP-S0-04. **Status:** proposed tests; no tests have been executed.

## 1. Validation layers

- **Unit:** shape/type/units, quaternion multiplication, rotation matrix, cross-product, SPD inertia rejection.
- **Analytical:** known translational/rotational trajectories and invariants.
- **Numerical:** step refinement, cross-integrator comparison, quaternion consistency.
- **System:** CLI, determinism, manifest coverage, corrupted input handling.

## 2. Required test matrix

| ID | Scenario / property | Expected behavior | Acceptance / initial tolerance |
|---|---|---|---|
| T01 | Quaternion identity and 90° yaw about world +z | `R_WB [1,0,0] -> [0,1,0]` | vector L2 error < 1e-12 |
| T02 | `q` versus `-q` | Same rotation and attitude metric | rotation matrix max error < 1e-12 |
| T03 | Round-trip random vector `Rᵀ(Rv)` | Recover vector | relative error < 1e-12, seeded samples |
| T04 | Invalid mass, non-SPD inertia, zero/NaN quaternion | Fail with clear validation error | 100% rejected |
| T05 | No wrench, `g=0`, zero omega | Uniform translation; constant orientation | position max error < 1e-10 at T=5 s, dt=0.01 |
| T06 | No wrench, `g=9.80665`, zero omega | Exact ballistic path | position error < 1e-9 at T=5 s, dt=0.01 |
| T07 | Spherical inertia, no torque, constant nonzero omega | Analytic quaternion exponential | attitude geodesic error < 1e-7 rad at T=2 s, dt=0.01 |
| T08 | Asymmetric SPD inertia, no torque | Rotational energy and world angular momentum nearly constant | normalized drift < 1e-6 at T=10 s, dt=0.001 (provisional) |
| T09 | Identity attitude, `F_B=(0,0,mg)` | Zero vertical acceleration at start | abs accel < 1e-12 m/s² |
| T10 | Smooth constant body wrench and nonzero body rate | `h` vs `h/2` vs `h/4` consistency | 4th-order *trend* in selected smooth fixture; no fixed order claim at roundoff floor |
| T11 | RK4 vs independent DOP853 | Consistent states at common times | position < 1e-5 m; attitude < 1e-5 rad in documented fixture |
| T12 | Fixed scenario, seed and code revision | Repeated run identical numeric outputs | trajectories/checksums identical on same runtime and platform |
| T13 | Missing/unexpected config, negative dt, NaN state | Explicit error; never a silent default | all fail clearly |
| T14 | Scenario CLI and manifest | End-to-end run produces trace + resolved config + metrics + environment metadata | required fields complete; clean-run instructions work |

**Tolerance policy:** These are *starting engineering tolerances* and are not universal numerical guarantees. Test fixtures, durations, CPU/platform and reference solutions must be frozen in test code. If a tolerance is revised, record evidence and review it; no silent relaxation to pass. `T08` normalized drift: `abs(E(t)-E(0))/max(abs(E(0)),E_floor)` and `||H_W(t)-H_W(0)||/max(||H_W(0)||,H_floor)`, with positive floors documented in code.

## 3. T10 precise convergence protocol

Choose smooth ODE fixture with nontrivial rotation (non-commuting orientation), run RK4 at `h, h/2, h/4, h/8`, compare to a high-accuracy independent DOP853 solution. Compute `e(h)` from position/velocity/geodesic attitude/rate error, and calculate observed order `p=log2(e(h)/e(h/2))`. Request approximately fourth-order behavior in an intermediate asymptotic range (e.g., p ≥ 3.5 for at least two consecutive refinements) **only** when error is clearly above roundoff, reference error is negligible, and no discontinuities are present. Otherwise label as inconclusive; investigate, do not manufacture a passing result.

## 4. Property-based and metamorphic tests

- Quaternion scale is rejected or normalized by documented input policy; always reject degenerate q.
- Quaternion sign flip leaves forces in world and physical trajectory unchanged.
- Rotation matrices remain orthogonal with determinant +1.
- In a zero-gravity zero-wrench test, shifting initial position by a constant shifts entire position trajectory by same constant without changing velocities/attitudes.
- For zero wrench, changing initial position does not change angular dynamics.

## 5. Metrics and artifacts

Record at every sample: `t,p_W,v_W,q_WB,omega_B,F_B,tau_B`, quaternion norm before correction (where applicable), integrator type and integration step. Produce `metrics.json` with maxima: energy drift, angular momentum drift, quaternion norm drift, analytic error, cross-solver differences. On failure, preserve traces and precise exception.

## 6. Gate S0 formal decision

**PASS** when all T01–T14 pass on clean setup and results are reproducible on a pinned environment; no untracked input changes. **FAIL** if any required test fails or manifest is incomplete. **WAIVED** allowed only with explicit rationale, issue ID, risk owner, review date and unchanged failed test; a waived gate is not equivalent to passing.
