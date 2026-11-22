# Aether SIL — Cycle-01 / S0 (T01–T05)

Generic rigid-body dynamics in SI units. The original conventions are in
[the interface contract](docs/cycle-01/interface_contract.md); the mathematical reference and
verification plan are alongside it. These synthetic models are not vehicle or EDF models.

Python 3.12+: `python -m pip install -e '.[test]'`, then `python -m pytest -q` and
`ruff check .`. Alternatively run `PYTHONPATH=src python -m unittest discover -s tests -v`.

`RigidBodyState` normalizes finite quaternions with norm >= 1e-12 at construction.
RK4 passes a `StageState` to every `WrenchModel.evaluate`: its quaternion is the
unmodified, finite, nondegenerate ODE stage value, which need not have unit norm.
An attitude-dependent model must use `state.physical_rotation()` to obtain a physical
rotation from a normalized copy, without changing the stage quaternion. This resolves
the original contract's state type ambiguity for intermediate force evaluation; model
authors must accept `StageState` and must not retain mutable hidden dynamics. The ODE
uses the same physical rotation from a normalized copy for force transformation. The
quaternion derivative uses the original stage quaternion; no stage state is modified.

`RK4.step_with_diagnostics` returns the accepted state and that step's pre-correction
quaternion norm and signed error; callers collecting a trajectory should retain each
step's diagnostic. `step` is the contract-compatible state-only convenience method.
The SciPy DOP853 reference uses the same ODE and wrench evaluation with an independent
adaptive integration algorithm, default rtol=1e-12 and atol=1e-13, and normalizes
returned sample states. Analytic and invariant tests independently check the equations.

No runnable scenario, manifest, S0 gate verdict, real EDF or control model is included.

Run the included synthetic scenario with:

```sh
python -m pip install -e '.[test]'
python -m aether_sil run scenarios/free_rotation.yaml --seed 7 --output runs/free_rotation
```

The output directory must not already exist. It contains a resolved configuration,
manifest, NPZ trajectory, metrics and run validation. The initial sample has no
preceding step, so its `q_norm_before` is NaN. The configuration digest hashes
canonical sorted compact JSON of the resolved YAML. See the gate report for current
verification status and limitations.
