# S0 gate report — 2026-11-22

**Gate S0: PASS for tested code commit `6be3e48d764efbdc4fcfe5376145bc052041215a`**, subject to independent review. This report-only commit follows the tested code commit; it does not change executable code. No waiver is used and no S1 work or tag is started. The result validates the generic synthetic model and the scope of the specified tests, not a real vehicle, EDF model or flight safety.

## Environment and execution

A new Git checkout at the tested SHA had empty `git status --porcelain`. Platform: Linux x86_64, glibc 2.41; CPython 3.12.14; uv 0.13.9. Commands:

```sh
UV_CACHE_DIR=/tmp/aether-gate-check-uv uv sync --locked --extra test --python 3.12.14
.venv/bin/ruff check .
.venv/bin/pytest -q
.venv/bin/pytest -v tests/test_failure.py
.venv/bin/python scripts/t10_evidence.py
.venv/bin/python -m aether_sil run scenarios/free_rotation.yaml --seed 7 --output <new-directory>
```

Ruff passed; full pytest **24 passed in 32.64 s**; the separate fault-injection run **6 passed in 9.67 s**. The CLI command was run twice into separate directories. [GitHub Actions 44988476804](https://github.com/XuanheGuo/Aether/actions/runs/44988476804) succeeded on the tested SHA: Python 3.12.14, `uv sync --locked --extra test`, Ruff and pytest all passed.

`uv.lock` pins resolved project dependencies with distribution hashes. Build isolation specifies `setuptools==80.9.0` but its distribution hash is not locked in uv.lock; CI's `pip install uv==0.13.9` also does not enforce an artifact hash. Therefore this is a version-pinned environment with hashed project distributions, **not a fully hash-reproducible build toolchain**. The run covers this platform only.

## Verification matrix

| ID | Status | Evidence |
|---|---|---|
| T01 | PASS | `test_t01_t03`: identity and +90° yaw |
| T02 | PASS | `test_t01_t03`: antipodal quaternion rotation |
| T03 | PASS | `test_t01_t03`: 100 seeded vector round trips |
| T04 | PASS | `test_t04_invalid`: mass, SPD, degenerate/NaN quaternion |
| T05 | PASS | `test_free_and_ballistic`: 5 s, dt 0.01, position tolerance 1e-10 m |
| T06 | PASS | same test with gravity, position tolerance 1e-9 m |
| T07 | PASS | `test_angular_invariants_and_rotation`: spherical analytic quaternion |
| T08 | PASS | same test, 10 s, dt 0.001, every-step energy and momentum drift |
| T09 | PASS | `test_t09_and_body_force`: force balance |
| T10 | PASS | frozen four-step convergence and reference cross-check below |
| T11 | PASS | `test_trajectory_reference_and_sign`: 101 shared sample times |
| T12 | PASS | two clean fixed-SHA seeded CLI runs produced identical trajectory hashes |
| T13 | PASS | invalid configuration and nonfinite/time validation tests |
| T14 | PASS | clean CLI output and six injected failure cases, details below |

No test is FAIL, NOT RUN, or WAIVED for this code and environment. The scenario's run validation is not itself a gate verdict.

## T10 convergence

Machine-readable fixture, solver settings and results: [t10.json](evidence/t10.json). Error combines position, velocity, attitude geodesic and body-rate differences.

| dt (s) | error | observed order |
|---:|---:|---:|
| 0.2 | 6.467020505682845e-5 | — |
| 0.1 | 4.099013893494038e-6 | 3.979752399834084 |
| 0.05 | 2.576950669289903e-7 | 3.991540054972813 |
| 0.025 | 1.6147810101006957e-8 | 3.996254488887297 |

A stricter DOP853 run (rtol 2.3e-14, atol 2e-15, max step 0.025 s) differed from the standard reference by 4.8915e-13, 0.00303% of the finest RK4 error, below the 1% check threshold. Both references solve the same ODE; this does not independently validate its physics.

## T12 reproducibility

Two runs of the fixed free-rotation scenario and seed 7 generated the same trajectory SHA-256 `712162c95e6b51b3aa16d172a783541091af255e2a9107e1534ceb936f6a34e2`. Both manifests record the tested SHA and `git_dirty=false`. Manifests have differing bytes due to generation time and output path; no manifest byte reproducibility is claimed. [Machine-readable manifests and hashes](evidence/reproducibility.json).

## T14 fault injection

[Recorded pytest results](evidence/failure-tests.txt) and `tests/test_failure.py` cover integration substage, reference solver, postprocessing, trajectory write, metrics write and failed checkpoint write. They assert nonzero exit, `FAIL`, phase, exception details, last successful step and time, time semantics, and checkpoint length/state shape. The integration failure occurs at stage time about 0.035 s after successful step 3 at 0.03 s; other tested exceptions have `failure_time_s=null` with an explicit reason. The checkpoint-write case verifies its error is separately reported and the original integration error remains the raised error. Failure record writing is also best effort: if the output location becomes unwritable, the original exception is retained and the record-write error is printed, but no on-disk record can be guaranteed.
