# S0 gate closure evidence — 2026-11-22

**Gate S0: FAIL.** The tested implementation is commit
`a7f142a7e55cc8f77aab5456392acd67b66de37b`; this report is a subsequent
report-only commit. The runner records the exact ODE stage time for evaluated-stage exceptions,
but reports null for failures outside an ODE evaluation. The requested failure time is
therefore not guaranteed for all runtime exceptions; T14 remains FAIL. No waiver has been granted. Do not tag or start S1.

## Environment and commands

A new clean Git checkout at the tested SHA had empty `git status --porcelain`.
Linux x86_64, CPython 3.12.14, uv 0.13.9. The repository `uv.lock` pins dependency
versions and distribution hashes. `UV_CACHE_DIR=/tmp/aether-final-cache uv sync
--locked --extra test --python 3.12.14` installed NumPy 2.3.5, SciPy 1.17.0,
PyYAML 6.0.3, pytest 8.3.5, Ruff 0.11.13, iniconfig 2.3.1, packaging 26.3,
and pluggy 1.7.0. Build isolation pins setuptools 80.9.0 by version, but its distribution hash is not
locked in uv.lock; this remains a clean-build reproducibility risk.

`.venv/bin/ruff check .` passed. `.venv/bin/pytest -q` passed **19 tests in 27.89 s**.
[GitHub Actions run 44987850117](https://github.com/XuanheGuo/Aether/actions/runs/44987850117)
succeeded for the same code SHA, including Ruff and pytest. CI currently uses pip,
not the uv locked environment.

## Matrix

| ID | Status | Executed evidence |
|---|---|---|
| T01 | PASS | `test_t01_t03`: identity and +90° yaw |
| T02 | PASS | `test_t01_t03`: antipodal rotation and attitude |
| T03 | PASS | `test_t01_t03`: 100 seeded vector round trips |
| T04 | PASS | `test_t04_invalid`: mass, SPD, degenerate/NaN quaternion |
| T05 | PASS | `test_free_and_ballistic`: 5 s, 0.01 s, 1e-10 m tolerance |
| T06 | PASS | same test, Earth gravity, 1e-9 m tolerance |
| T07 | PASS | `test_angular_invariants_and_rotation`: spherical analytic quaternion |
| T08 | PASS | same test, 10 s, 0.001 s, energy and momentum at every step |
| T09 | PASS | `test_t09_and_body_force`: instantaneous force balance |
| T10 | PASS | frozen four-step fixture and tighter DOP853 reference below |
| T11 | PASS | `test_trajectory_reference_and_sign`: all 101 common samples |
| T12 | PASS | two locked-environment runs at identical clean Git SHA; trajectory bytes identical |
| T13 | PASS | invalid shape, missing/extra keys, dt and nonfinite tests |
| T14 | FAIL | end-to-end injected ODE-stage failure records time; other runtime failures can lack exact failure time |

No tests are WAIVED. A scenario's `run_status` is not an S0 gate verdict.

## T10

`.venv/bin/python scripts/t10_evidence.py` produced the machine-readable
[t10.json](evidence/t10.json). The fixture has constant wrench, nonspherical inertia,
noncommuting attitude and nonzero body rate; all parameters are in the JSON. Composite
error is L2 of position, velocity, geodesic attitude and angular rate. Reference is
DOP853 (rtol=1e-12, atol=1e-13), checked against a tighter independent adaptive solve
(rtol=2.3e-14, atol=2e-15, max step 0.025 s). Both solve the same ODE, so this is a
reference convergence cross-check rather than an independent physical model.

| dt (s) | error | observed order |
|---:|---:|---:|
| 0.2 | 6.467020505682845e-5 | — |
| 0.1 | 4.099013893494038e-6 | 3.979752399834084 |
| 0.05 | 2.576950669289903e-7 | 3.991540054972813 |
| 0.025 | 1.6147810101006957e-8 | 3.996254488887297 |

Reference disagreement is 4.891502238579476e-13, or 3.0292e-5 of the finest
RK4 error, below the preselected 0.01 threshold.

## T12 and T14

Two commands `.venv/bin/python -m aether_sil run scenarios/free_rotation.yaml --seed 7
--output <distinct-directory>` produced identical `trajectory.npz` SHA-256:
`712162c95e6b51b3aa16d172a783541091af255e2a9107e1534ceb936f6a34e2`.
Both manifests record the tested SHA, `git_dirty=false`, seed, platform and versions.
Manifest byte hashes differ because output path and generation time differ; manifest
byte identity is not claimed. Machine-readable manifests, hashes and checkpoint samples
are in [reproducibility.json](evidence/reproducibility.json).

Injected `ValueError("injected solver failure")` exited 2 and produced FAIL,
last successful step 3 at 0.03 s, and four checkpoint samples at 0, 0.01, 0.02 and
0.03 s. The recorded `failure_step_start_s=0.03` and `failure_time_s=0.035`
identify the step and evaluated substage. Exceptions outside ODE evaluation can
still produce a null failure time. Partial-checkpoint SHA-256:
`edcd221f4963c8910b7880e8c4f3418ee9e140e7e14d1e14f2e923944a934f27`.

The current physical model is synthetic; no EDF, control or hardware safety claim is
made. Analytic error is explicitly `not_applicable` when a scenario has no analytic
reference. Invariant changes under applied torque are labeled as changes, not drift.
