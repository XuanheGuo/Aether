# S0 gate report — local evidence, 2026-11-22

**Gate S0: FAIL.** The formal criteria are not met: no locked transitive environment,
no clean Git checkout validation in this runtime, and incomplete failure artifacts.
No waiver has been granted. A scenario run is not a gate decision.

Command: `PYTHONPATH=src python -m unittest discover -s tests -v` — 18 tests passed
in 12.884 s on Python 3.12.14, NumPy 2.3.5, SciPy 1.17.0, PyYAML 6.0.3.
This is a workspace without Git metadata. The statuses below distinguish tested
properties from the unexecuted full formal acceptance protocol.

| ID | Status | Evidence / limitation |
|---|---|---|
| T01 | PASS | `test_t01_t03`: identity and +90° yaw |
| T02 | PASS | `test_t01_t03`: antipodal rotation and distance |
| T03 | PASS | `test_t01_t03`: 100 seeded rotation round trips |
| T04 | PASS | `test_t04_invalid`: invalid mass, inertia and quaternions |
| T05 | PASS | `test_free_and_ballistic`: RK4, 5 s, 0.01 s, 1e-10 m tolerance |
| T06 | PASS | `test_free_and_ballistic`: RK4, 5 s, 0.01 s, 1e-9 m tolerance |
| T07 | PASS | `test_angular_invariants_and_rotation`: spherical analytic quaternion, 2 s, 0.01 s |
| T08 | PASS | same test: 10 s, 0.001 s; both normalized drifts < 1e-6 |
| T09 | PASS | `test_t09_and_body_force`: instantaneous force balance |
| T10 | NOT RUN | Four-step convergence test passed with DOP853, but independent tighter-reference-error check and recorded numeric order evidence for this revision are not yet frozen |
| T11 | PASS | `test_trajectory_reference_and_sign`: all 101 common samples within 1e-5 m and rad |
| T12 | NOT RUN | Repeated NPZ bytes match locally, but the workspace is unversioned and the fixed-revision clean-environment protocol was not run |
| T13 | PASS | `test_bad_configs` and `test_invalid_inputs`: missing/extra keys, negative dt and NaN |
| T14 | FAIL | CLI outputs verified, but clean pinned installation was not run and failure paths do not preserve partial trajectory traces |

A direct free-rotation run used `PYTHONPATH=src python -m aether_sil run
scenarios/free_rotation.yaml --seed 7 --output /tmp/aether-s0-validation-01`.
The maximum RK4/DOP853 attitude difference over the full 201 samples was
3.8103e-12 rad; maximum pre-correction quaternion norm error was 5.5511e-16.
World angular momentum and energy changes were 3.4915e-12 and 3.7116e-13
relative respectively. This run has `git_sha=unversioned` and is not release evidence.

Pytest, Ruff and GitHub Actions outcomes were not available at the time of this local
report. Exact top-level dependencies are pinned, but a transitive lockfile and clean
install have not been established. Analytic error is currently reported as null when
there is no scenario-specific analytic reference. No release tag was created.
