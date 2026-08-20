# Code Baseline Audit

## Scope and evidence

This is a static baseline audit. No simulation was executed and no MATLAB, CasADi,
LaTeX, result, or Git file was changed. The review compares the current code with
`01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`, in particular
Eqs. (19)--(24), and with the reviewer mapping workbook.

## 1. NMPC Implementation

**File:** `02_code_revision/ET_NMPC_revision_code/solve_nmpc_casadi.m`  
**Function:** `solve_nmpc_casadi(x_curr, Pi_ref_local, obs_pred, buildings, params)`

### Current implementation

The function builds a CasADi `Opti` nonlinear program with state
`X=[p_x,p_y,p_z,v_x,v_y,v_z]`, control `U=[u_x,u_y,u_z]`, horizon `H=15`, and the
discrete double-integrator dynamics used in the manuscript. It constrains initial
state, each velocity component, each control component, and altitude. IPOPT is used;
failure is caught and replaced by a local fallback controller/prediction.

The implemented objective is

```text
J_code = alpha*J_path + beta*J_smooth + gamma*J_energy + delta*J_risk
         + J_terminal_progress + J_terminal_pos + J_terminal_vel.
```

Its terms are as follows.

| Term | Current code behavior |
| --- | --- |
| `J_path` | Sum of squared position errors to `Pi_ref_local(:,k)`: `sumsqr(p_{k+1}-ref_k)`. This is a reference-tracking cost. |
| `J_smooth` | Sum of squared consecutive-control differences: `sumsqr(U(:,k+1)-U(:,k))`. This penalizes input variation (jerk-like behavior), not velocity variation directly. |
| `J_energy` | `c1*sumsqr(dp) + c2*sumsqr(U(:,k)) + c3*smooth_max(0,dz)` at each step. The climb term is enabled by default. |
| `J_risk` | Differentiable soft penalties for dynamic predicted obstacles and the three static-building shapes. Dynamic penalties use a cubic violation inside the inflated radius plus a quadratic near-boundary term. Static penalties use fourth-power violations, near-boundary penalties, and an additional climb incentive below roofs. |
| Terminal-progress cost | Present. It attracts every predicted state toward the local terminal reference with weight `terminal_path_weight*(k/H)^2`. |
| Terminal position cost | Present. `terminal_pos_weight*||p_{H+1}-p_terminal_ref||^2`. |
| Terminal velocity cost | Present. `terminal_vel_weight*||v_{H+1}||^2`. |

There is therefore both reference tracking and explicit goal/terminal attraction. The
terminal reference is `Pi_ref_local(:,end)` normally, but becomes the physical goal
`params.p_goal` inside `terminal_goal_attract_radius`. In addition, risk costs are
scaled down near that terminal reference, but never below 45% of their nominal value.

### Consistency with manuscript Eqs. (19)--(24)

**Partially consistent at the high level; not equation-level consistent.** The
four-term structure in Eq. (19), the double-integrator model, positive-climb penalty,
and predicted-obstacle inflation concept are represented. The details differ materially:

1. **Eq. (19): extra unreported terms.** The code adds terminal-progress, terminal
   position, and terminal-velocity costs outside the published four-term objective.
   It also introduces terminal-dependent risk decay. None is defined in Eqs. (19)--(24).
2. **Eq. (20): path-length versus tracking mismatch.** The paper defines
   `sum ||p_{k+tau+1}-p_{k+tau}||`; the code minimizes squared tracking error to a
   reference point. The code is consistent with the paper's narrative claim of tracking,
   but not with the displayed path-length equation.
3. **Eq. (21): smoothness mismatch.** The paper penalizes consecutive velocity
   differences. The code penalizes consecutive control-input differences. Under the
   stated dynamics, velocity difference is proportional to `U`, whereas the implemented
   term is proportional to a difference of `U`; they are not the same quantity.
4. **Eq. (22): distance-term mismatch.** The paper uses `c1*sum ||dp||`; the code
   uses `c1*sum ||dp||^2`. The `c2` and asymmetric positive-climb components are
   conceptually aligned, although `max(0,dz)` is replaced by a smooth approximation.
5. **Eq. (24): risk-form mismatch.** The paper shows a reciprocal dynamic-obstacle
   proximity cost. The code uses bounded smooth polynomial barrier penalties, includes
   static buildings, hard-coded gains (`3000`, `12000`, `2500`, etc.), and near-goal
   risk decay. This is a different risk function.
6. **Published safety constraints are not implemented as NLP constraints.** The paper
   presents norm-bounded velocity/control constraints and hard dynamic/static obstacle
   constraints. The code applies component-wise bounds (which allow vector norm up to
   `sqrt(3)` times the stated scalar bound) and uses only soft obstacle costs; no
   `Opti.subject_to` obstacle-separation or static-workspace constraint is present.

## 2. Energy Model

**Primary files:**

- `solve_nmpc_casadi.m` -- online surrogate cost.
- `main_comparison.m` -- Table III estimated energy.
- `main_monte_carlo_final_v2.m` -- Table V estimated energy.
- `init_params.m` -- optimization, risk, and trigger settings.

### Parameter configuration from `init_params.m`

| Group | Current values |
| --- | --- |
| NMPC setup | `dt=0.1 s`, `H=15`, `v_max=5.0 m/s`, `u_max=3.0 m/s^2`, `z=[0.5,20.0] m`, `r_u=0.3 m` |
| NMPC weights | `alpha=1.0`, `beta=0.5`, `gamma=0.8`, `delta=5.0` |
| Online energy weights | `use_asymmetric_energy=true`, `c1=1.0`, `c2=0.1`, `c3=3.0` |
| Terminal additions | position `120.0`, velocity `20.0`, path `8.0`, risk-decay radius `12 m`, minimum factor `0.45`, goal-attraction radius `25 m` |
| Prediction/risk | `sigma_q=0.1`, `sigma_r=0.05`, `chi2_alpha=7.81`, `d_m=0.2 m`, default obstacle radius `1.5 m` |
| Trigger/risk fusion | `R_on=0.6`, `R_off=0.2`, `T_ref=5` steps, `w1=0.4`, `w2=0.4`, `w3=0.2`, `T_c=3.0 s`, `D_c=2.0 m`, `e_max=2.0 m` |

### Current implementation and consistency

The manuscript's Eq. (23) is a post-run physical estimate:
`E_est=P_base*T + m*g*Delta_z_plus/eta_c + k_u*integral(||u||^2)dt`.
`main_comparison.m` implements this form with defaults of mass `1.5 kg`, gravity
`9.81 m/s^2`, climb efficiency `0.70`, hover/base power `180 W`, and control coefficient
`2.0`. This is conceptually consistent with Eq. (23).

However, these physical parameters are not centrally defined by `init_params.m`.
`main_monte_carlo_final_v2.m` separately defaults to base power `45 W`, mass `1.2 kg`,
gravity `9.81 m/s^2`, climb efficiency `0.70`, and control coefficient `0.08`.
Consequently, Table III and Table V can report numerically incomparable energy values
even for similar trajectories. The online NMPC cost is explicitly a surrogate, as the
paper says, but its squared-distance implementation differs from Eq. (22).

The paper's Table II lists `alpha=0.14`, `beta=0.07`, `gamma=0.11`, and `delta=0.68`.
The executable baseline uses `1.0`, `0.5`, `0.8`, and `5.0`, respectively. The two sets
are proportional only approximately (the ratios differ), so this is not a harmless
normalization difference and must be reconciled before claiming Table II is the executed
configuration.

## 3. Experimental Pipeline

| Entry point | Intended paper artifact | Current output / behavior |
| --- | --- | --- |
| `main_simulation.m` | Figs. 2--4: encounter snapshot, complete proposed trajectory, and risk/trigger logic | Runs one proposed ET-NMPC scenario with heterogeneous noisy obstacles. Exports `Avoidance_Snapshot_MultiObstacle.png`, `Avoidance_Final_MultiObstacle.png`, `Risk_Triggering_MultiObstacle.png`, and a single-run CSV. |
| `main_comparison.m` | Table III, quantitative baseline comparison over 30 randomized trials | Evaluates Proposed ET-NMPC, C-NMPC, NP-ET-NMPC, SE-ET-NMPC, 3D-APF, and RRT*. Writes summary/detail CSVs under `results_reviewer_response/comparison_results/`; it deliberately does not create figures. |
| `main_comparison_trajectory.m` | Fig. 5 (the manuscript's baseline trajectory-comparison figure), supplementary to Table III | Runs the six algorithms in one fixed scenario with precomputed common obstacle truth trajectories. Exports PNG/PDF/FIG overlay and metrics/path/workspace files under `comparison_trajectory/`. |
| `main_proposed_et_nmpc_profiling.m` | Table IV, computational profiling | Runs Proposed ET-NMPC 30 times; writes per-step, per-run, and summary profiling CSVs. It maintains a duplicated `solve_nmpc_casadi_profile` implementation and sets IPOPT acceptable tolerance to `1e-3`, while the production solver baseline is `1e-4`. |
| `main_monte_carlo_final_v2.m` | Table V, scalability Monte Carlo | Runs obstacle counts `[1,3,5]`, 100 repetitions per count, for the four NMPC variants. Saves scalability summary and detail CSVs; figure generation is disabled. |

The event logic in `main_simulation.m` follows the manuscript's hysteresis shape:
it turns off at `R_off`, turns on at `R_on` subject to `T_ref`, otherwise retains the
previous mode. It adds one behavior not stated in the switching equation: direct-goal
guidance forces `sigma_k=1` before the ordinary `R_on` test.

## 4. Potential Issues for Revision

### Highest-priority manuscript/code alignment

1. **R1-3 -- NMPC goal/cost formulation:** reconcile Eqs. (19)--(24) with the actual
   solver. Either revise the manuscript to specify tracking, control-increment,
   terminal/progress, and polynomial-barrier terms, or change the code to implement the
   published equations. The current text does not support the exact implementation.
2. **R1-4 -- energy penalty/model:** make Eq. (22) agree with `sumsqr(dp)`, or make the
   code use the documented norm. Define one physical Eq. (23) parameter set in
   `init_params.m` and have Table III/V scripts consume it. State clearly that online
   `J_energy` is a surrogate and reported Wh is post-run estimation.
3. **Table II reproducibility:** synchronize the published NMPC weights with the executed
   weights and report terminal weights/risk-decay parameters if those remain part of the
   method.
4. **Safety claim precision:** the paper describes hard dynamic/static separation
   constraints, but the solver uses soft penalties. Either introduce actual constraints
   and norm bounds or qualify the paper as a soft-penalty formulation and report collision
   checking separately.

### Reviewer-related experimental risks

5. **R1-2 -- real-time claim:** Table IV profiles a copied solver with a different
   acceptable tolerance. Verify and document equivalence to the production solver before
   using its IPOPT/timing results as direct evidence for the reported implementation.
6. **R1-5 -- baselines:** `main_comparison.m` provides the requested expanded baselines
   and same-repeat scenario seed, while `main_comparison_trajectory.m` uses a common
   precomputed obstacle truth trajectory. Preserve the exact seeds, code revision, and
   success-only metric policy with each CSV so Table III remains auditable.
7. **Table III vs. Table V energy comparability:** the two scripts use materially
   different default physical-energy coefficients. Do not compare absolute Wh values
   across these tables until unified.
8. **R1-4 ablation traceability:** the SE-ET-NMPC switch is `use_asymmetric_energy=false`;
   confirm in the response letter that all other settings, including terminal and safety
   terms, remain identical for the ablation.

### Secondary technical items

9. The code's component-wise `u_max`/`v_max` constraints are not the manuscript's norm
   constraints. This affects the feasible set and should be corrected or described.
10. Hard-coded risk gains and static-building shaping terms are outside `init_params.m`;
    centralize or document them for reproducibility and sensitivity discussion.
11. Terminal goal attraction and forced-NMPC direct-goal guidance can affect trigger rate,
    path shape, and clearance. They should be included in the stated method and applied
    consistently across all baselines.

