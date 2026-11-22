# 01 · Mathematical Control Problem

**Task mapping:** RMP-S1-01, 03, 05 · **Taxonomy:** C.01.04, C.02.01, C.02.04, C.03.01, C.03.02, C.07.01, C.07.03

## 1. Coordinate and sign convention (inherited from S0)
World W: ENU, body B: FLU, active Hamilton quaternion `q_WB` in `[w,x,y,z]`; `R_WB` maps B to W; `omega_B` is body-frame angular rate. `g_W=(0,0,-g)`; SI units. Never introduce Euler angles as the dynamic state. Euler angles may appear in labeled human-readable plots only.

## 2. Plant
Let `R in SO(3)`, body inertia constant symmetric positive definite `J`, mass `m>0`; external non-gravity force `F_B` and torque `tau_B` about COM:

```math
\dot p=v,\qquad m\dot v=RF_B+mg_W,\qquad \dot R=R[\omega]_\times,\qquad J\dot\omega+\omega\times J\omega=\tau_B+d_B.
```

In S1, the controller's abstract output is a *requested body torque* `tau_cmd,B` (N·m). The actuator maps that to applied torque. The translation force is supplied independently by a synthetic non-gravity force model; no inverse EDF allocation is implied. Never count `d_B` twice if the disturbance is already included in the applied wrench.

## 3. Distinct problems
- **P1 Attitude regulation:** constant reference `R_d`, `omega_d=0`; reference `q_d` and `-q_d` are identical. Goal: `R→R_d`, `omega→0` in a specified admissible set.
- **P2 Attitude tracking:** differentiable `R_d(t)`, `omega_d(t)` with `dot R_d=R_d[omega_d]_×`; if using feedforward, require and log `dot omega_d`. Fix whether desired rates are expressed in desired body frame.
- **P3 Robustness:** inertia mismatch, bounded torque disturbances, delayed/noisy measured state, finite actuator response and saturation. Claims must state whether trajectories are bounded, converge to a neighborhood, or converge asymptotically.
- **P4 Feasibility:** determine whether requested torque lies in the actuator's admissible set; independently determine if translation/hover force can be balanced. Hover requires `R F_B + m g_W = 0`; a torque-only closed loop does not prove it.

## 4. Attitude error definitions
For `R_e = R_d^T R`, define `Psi = (1/2) tr(I - R_e) in [0,2]` and
`e_R = (1/2)(R_e - R_e^T)^vee`; rate error `e_omega = omega - R^T R_d omega_d`. With constant desired attitude `e_omega=omega`. `e_R=0` also at rotation error pi: do not mistake this for the desired equilibrium. Report geodesic attitude error `theta = arccos(clip((tr(R_e)-1)/2,-1,1))` in `[0,pi]`, with numerically stable implementation near zero and pi.

## 5. Topology and scope of stability claims
No smooth time-invariant continuous feedback on SO(3) can make one attitude globally asymptotically stable without qualification. For the simplest smooth geometric PD, the target may be almost globally attractive while pi-rotation equilibria exist. Explicitly separate local exponential, almost-global asymptotic, practical/ISS-like, and numerical evidence. Do not claim global results by Monte Carlo.

## 6. Synthetic reference fixture policy
Use positive definite *synthetic* inertia matrices, bounded torque and response time constants chosen explicitly for numerical experiments, not estimated from a real rocket/EDF. Record ranges, uncertainty distributions, and dimensionless ratios where possible. A physically reasonable EDF vehicle model is a later S2 deliverable.
