# S0-01 · Interface Contract

**Task:** RMP-S0-01 · **Taxonomy:** A.03.02 / A.03.03  
**Version:** 1.0.0 · **Contract stability:** breaking edits require a major version.

## 1. Frames and orientation

- **World W:** right-handed ENU: `x=East`, `y=North`, `z=Up`; gravity `g_W=[0,0,-g]` with `g>0` in m/s².
- **Body B:** right-handed FLU: `x=Forward`, `y=Left`, `z=Up`; the initial body/world alignment is arbitrary and is **not** a vehicle geometry claim.
- `R_WB` maps body-frame vectors to world-frame vectors: `v_W = R_WB v_B`. Thus `v_B = R_WBᵀ v_W`.
- Quaternion `q_WB = [w,x,y,z]` (Hamilton, scalar first, active rotation); `R(q_WB)=R_WB`. `q` and `-q` encode the same attitude.
- Quaternion product `⊗` is Hamilton. Angular velocity `omega_B` is relative angular velocity of B w.r.t. W **expressed in B**. With `q̇_WB = 0.5*q_WB ⊗ [0,omega_B]`, `Ṙ_WB = R_WB [omega_B]_×`.
- Right-hand-rule positive rotations. `[a]_× b = a × b`.
- Quaternion error/distance: `theta(q1,q2)=2 acos(clip(|dot(q1,q2)|,0,1))` radians; use antipodal-invariant comparisons.

## 2. Physical states, inputs and outputs

| Field | Type / shape | Frame | Units | Requirement |
|---|---|---|---|---|
| `t` | float64 scalar | simulation clock | s | finite, nonnegative, monotone |
| `p_W` | float64[3] | W | m | finite |
| `v_W` | float64[3] | W | m/s | finite |
| `q_WB` | float64[4] | W←B | dimensionless | Hamilton wxyz, normalized, finite |
| `omega_B` | float64[3] | B | rad/s | finite |
| `F_B` | float64[3] | B | N | net applied **non-gravity** force at COM |
| `tau_B` | float64[3] | B | N·m | net applied torque about COM |
| `mass` | float64 scalar | invariant | kg | >0 |
| `J_B` | float64[3,3] | B | kg·m² | symmetric positive definite |
| `g` | float64 scalar | W down | m/s² | ≥0 for synthetic zero-gravity tests; 9.80665 Earth reference default |

Do not include gravity in `F_B`. An external effect model may return `F_B`,`tau_B`; translation already adds gravity once. Moment reference point is always COM, not nozzle pivot.

### Contracts (language-neutral pseudo-signatures)

```python
@dataclass(frozen=True)
class RigidBodyState:
    p_W: Vec3
    v_W: Vec3
    q_WB: QuaternionWXYZ
    omega_B: Vec3

@dataclass(frozen=True)
class RigidBodyParams:
    mass_kg: float
    J_B_kg_m2: Mat33
    gravity_m_s2: float = 9.80665

@dataclass(frozen=True)
class BodyWrench:
    force_B_N: Vec3
    torque_B_Nm: Vec3

class WrenchModel(Protocol):
    def evaluate(self, t_s: float, state: RigidBodyState) -> BodyWrench: ...

class Integrator(Protocol):
    def step(self, t_s: float, state: RigidBodyState,
             params: RigidBodyParams, wrench: WrenchModel,
             dt_s: float) -> RigidBodyState: ...

class Scenario(Protocol):
    def build(self, config: Mapping) -> tuple[RigidBodyState, RigidBodyParams, WrenchModel]: ...
```

A `WrenchModel` is evaluated at the integrator's internal substages to preserve RK4 order. Later actuator models can implement a **separate stateful coupled dynamics system**, rather than silently using time-varying state hidden in this stateless interface.

### S1-facing extension (reserved; not S0 acceptance)

```python
class Controller(Protocol):
    def reset(self, seed: int): ...
    def update(self, t_s: float, estimate: RigidBodyState, reference: Mapping) -> BodyWrench: ...
class Estimator(Protocol):
    def reset(self, seed: int): ...
    def update(self, t_s: float, observation: Mapping) -> RigidBodyState: ...
class Actuator(Protocol):
    def update(self, t_s: float, command: Mapping, dt_s: float) -> BodyWrench: ...
```

These are preliminary boundaries; S1 must specify actuator physical controls, command scheduling and estimator outputs before adoption. **A generic wrench is not evidence of an implementable EDF actuator.**

## 3. Time and discrete execution

- Simulation starts at `t=0`; seconds as float64; all logged samples include timestamp.
- Initial state is sample `k=0`. Fixed-step integration: `t_k = k*dt` (computed from integer tick count to avoid accumulated timestamp drift).
- No wall-clock dependence; simulation can run faster or slower than real time.
- For later sampled-data control, control commands are held piecewise constant on `[t_k,t_{k+1})` unless a scenario explicitly specifies a different schedule. Integration substeps are not control update events.
- Randomness is via explicitly seeded `numpy.random.Generator`, or alternative explicitly recorded RNG; never implicitly seed from time.

## 4. Input validation and boundary rules

Reject non-finite states or parameters; reject `mass<=0`, non-SPD `J_B`, `dt<=0`, invalid shape, and quaternion norm below a defined minimum threshold. Renormalize quaternions at accepted integration steps and document the normalization; do **not** renormalize zero/NaN silently. Configuration parsing must reject unknown fields or report them explicitly.

## 5. Records and versioning

Every run stores: `contract_version`, `scenario_id`, resolved config and SHA-256 digest, implementation git SHA (or `unversioned` flagged), Python/dependency versions, platform, `dt`, integration method/tolerances, seed, UTC generation time (metadata only), and results path. Save state trajectories with names and units; do not infer units from column order.

Suggested outputs: `manifest.json`, `resolved_config.yaml`, `trajectory.csv` or `.npz`, `metrics.json`, `validation.json`. Distinguish raw traces from plots.

## 6. Open decisions (not blocking S0)

Vehicle body axes relative to a specific physical nozzle, true inertia, thrust direction, spin-induced moments, aero coefficients, estimator sensor frame, and hardware bus/clock are intentionally unresolved.

## Implementation clarification: solver stage state

`WrenchModel.evaluate` receives a finite `StageState` during ODE evaluations. Its
quaternion is unmodified and may be non-unit. Physical rotations are computed from
a normalized copy, consistently with the force transformation in `dxdt`. The
accepted external `RigidBodyState` remains normalized. This resolves the stage-state
ambiguity without modifying intermediate values.
