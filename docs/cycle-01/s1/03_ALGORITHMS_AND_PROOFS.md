# 03 · Controller Baselines and Proof Obligations

**Task mapping:** RMP-S1-01, 05 · **Taxonomy:** C.03.01/02/03, C.07.01/03/05

## 1. Mandatory baseline controllers
**B0: ideal geometric PD, attitude regulation.** With `e_R=(R_d^T R-R^T R_d)^vee/2`, `e_omega=omega` for constant desired attitude, propose

```math
\tau_{cmd} = -k_R e_R-k_\omega e_\omega+\omega\times J\omega,
\qquad k_R>0,\ k_\omega>0.
```

This is a simple analytically tractable comparison, assuming exact inertia, ideal torque, no disturbance, no saturation. If compensation is disabled, treat as another explicitly labeled controller.

**B1: locally linear LQR.** Let small attitude-error vector `delta theta` and rate `delta omega` near a specified equilibrium satisfy `dot(delta theta)≈delta omega`, `J dot(delta omega)≈delta tau`. Construct `A=[[0,I],[0,0]]`, `B=[[0],[J^{-1}]]`; solve continuous-time CARE with stated symmetric `Q>=0`, `R>0` and assumptions. Ensure the quaternion/SO(3) error-to-local-coordinate mapping is checked. LQR is local; the error chart and angular region must be stated. Compare with B0 under identical torque limits.

**B2: classical PID** is an *optional third baseline*, not a proof substitute. If implemented, define integral state on a local chart, anti-windup, integral reset/limits, and exclude ambiguous global Euler-angle PID. For S1 Gate, two legitimate distinct baselines B0/B1 are sufficient.

**B3: geometric tracking** is an *optional advanced candidate*, not necessary to pass S1. For full time-varying desired attitude, use the usual geometric tracking error and an explicitly derived feedforward term. INDI is research backlog until a well-defined acceleration measurement, discrete differentiation/filters and actuator effectivity assumptions exist.

## 2. Minimum proof exercise
For B0 in the disturbance-free ideal-torque regulation case, consider

```math
V = \tfrac12 e_\omega^T J e_\omega+k_R\Psi(R,R_d).
```

Using `dot Psi = e_R · e_omega` and the specified compensation, establish `dot V = -k_omega ||e_omega||²`. This yields non-increase of V, but **does not alone establish global convergence to the desired attitude**: analyze the critical set including 180° errors and state an appropriate (local or almost-global, with qualifications) conclusion. For a rigorous local asymptotic/exponential statement, provide the missing arguments (e.g., an appropriate cross term, invariance/local linearization) and explicit domain assumptions.

For LQR: document linearization and CARE assumptions, verify closed-loop eigenvalues, and state why linear local stability does not imply global nonlinear stability.

## 3. Robustness proof boundaries
With bounded disturbance d_B, the ideal B0 identity becomes `dot V=-k_omega||omega||²+omega·d_B`; this alone does not prove strict ISS of full attitude error or a global ultimate bound. If claiming ISS/practical stability, provide a new strict Lyapunov argument and a clearly defined domain. With actuator lag/saturation or `J_model≠J_true`, do not transplant the ideal proof without re-derivation; record the gap.

## 4. Research memo artifacts
`proofs/B0_geometric_PD.md` or `.tex`: assumptions, proposition, derivation, caveats, counterexamples; `proofs/B1_local_LQR.md`: state ordering, local coordinates, CARE result and closed-loop linear proof. Include a numerical counterexample/critical set at pi attitude error and a demonstration of saturation breaking the ideal proof assumptions.

## 5. Publication claims
This S1 work is baseline research and reproduction. A new paper is warranted only after literature review and a separate novel result; never call standard PD/LQR or numerical plots an original algorithm.
