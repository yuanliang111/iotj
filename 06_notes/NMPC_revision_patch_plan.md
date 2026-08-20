# NMPC LaTeX Revision Patch Plan

## Scope and decision gate

This is a LaTeX patch **plan**, not a source edit. It is based on `nmpc_latex_location.md` and the code-baseline audit. The recommended route is to make Section III and Table II faithfully document the current tested MATLAB/CasADi solver. If the authors instead choose to preserve the current paper equations, the solver must be changed and all affected figures/tables rerun; that route is outside this document's recommended path.

## 1. Current Eq. (19)--(24) issue analysis

| Current equation | Current paper meaning | Current solver behavior | Patch issue |
| --- | --- | --- | --- |
| Eq. (19), `eq:total_cost` | Four-term cost: path, smoothness, energy, risk | Uses the four weighted terms **plus** terminal-progress, terminal-position, and terminal-velocity costs | Add the three terminal terms to the published objective and define their references/weights. |
| Eq. (20), `eq:jpath` | Sum of Euclidean path increments | Sum of squared position errors to `Pi_ref_local(:,k)` | Replace path-length formula/name with reference-tracking cost. |
| Eq. (21), `eq:jsmooth` | Squared velocity increments | Squared consecutive-control increments | Replace velocity-difference formula/name with control-increment regularization. |
| Eq. (22), `eq:jenergy` | Norm of displacement + control effort + `max(0,dz)` | Squared displacement + squared control + smooth positive-part climb penalty | Replace the first term with squared displacement and define the differentiable positive-part approximation. |
| Eq. (23), `eq:estimated_energy` | Post-run physical estimated-energy metric | Conceptually present, but physical coefficients are not centrally fixed and Table III/V scripts currently differ | Retain it as a **reporting metric**, explicitly separate from online objective, and freeze/disclose one coefficient set before final results. |
| Eq. (24), `eq:jrisk` | Reciprocal dynamic-obstacle proximity cost | Differentiable polynomial soft penalties for dynamic obstacles and static buildings, with near-boundary terms and terminal risk decay | Replace reciprocal expression with the actual soft-risk construction, or define a compact generic form and move all gains to Table II/supplement. |

Related constraint text after Eq. (24) also needs alignment: the paper currently states norm-bounded control/velocity and hard dynamic/static constraints, while the solver uses component-wise bounds and soft obstacle penalties. The revised prose must state the selected implementation truthfully.

## 2. Proposed new objective-function structure

### Recommended notation

Use `\vect{r}_{k+\tau}` for the stage reference and `\vect{p}^{\mathrm{term}}_k` for the terminal reference. Define `\vect{p}^{\mathrm{term}}_k` as the local-reference endpoint by default and the physical goal inside the goal-attraction radius. Let `\phi_\epsilon(s)=\frac{1}{2}(s+\sqrt{s^2+\epsilon})` denote the differentiable positive-part approximation used by the solver.

### Proposed replacement for Eq. (19)

```latex
\begin{equation}
\begin{aligned}
J ={}& \alpha J_{\mathrm{trk}} + \beta J_{\Delta u}
      + \gamma J_{\mathrm{eng}} + \delta J_{\mathrm{risk}} \\
    & + J_{\mathrm{term\text{-}prog}}
      + J_{\mathrm{term\text{-}pos}}
      + J_{\mathrm{term\text{-}vel}},
\end{aligned}
\label{eq:total_cost}
\end{equation}
```

This preserves the solver's actual weighted core objective and exposes the currently undocumented terminal components. The text must state that `J_risk` is a soft penalty, not a hard safety constraint.

### Proposed component definitions

```latex
\begin{equation}
J_{\mathrm{trk}} = \sum_{\tau=0}^{H-1}
\left\|\vect{p}_{k+\tau+1}-\vect{r}_{k+\tau}\right\|^2 .
\label{eq:jpath}
\end{equation}

\begin{equation}
J_{\Delta u} = \sum_{\tau=0}^{H-2}
\left\|\vect{u}_{k+\tau+1}-\vect{u}_{k+\tau}\right\|^2 .
\label{eq:jsmooth}
\end{equation}

\begin{equation}
\begin{aligned}
J_{\mathrm{eng}} ={}& c_1\sum_{\tau=0}^{H-1}
\left\|\vect{p}_{k+\tau+1}-\vect{p}_{k+\tau}\right\|^2 \\
&+c_2\sum_{\tau=0}^{H-1}\left\|\vect{u}_{k+\tau}\right\|^2
+c_3\sum_{\tau=0}^{H-1}\phi_\epsilon(z_{k+\tau+1}-z_{k+\tau}).
\end{aligned}
\label{eq:jenergy}
\end{equation}
```

The revised text should call the first component “squared displacement regularization,” not path length or physical propulsion energy. It should retain the statement that this online cost is a lightweight surrogate.

### Terminal-cost definitions to add after the energy/risk definitions

```latex
\begin{equation}
J_{\mathrm{term\text{-}prog}} = \lambda_{\mathrm{prog}}
\sum_{\tau=1}^{H}\left(\frac{\tau}{H}\right)^2
\left\|\vect{p}_{k+\tau}-\vect{p}^{\mathrm{term}}_k\right\|^2,
\end{equation}

\begin{equation}
J_{\mathrm{term\text{-}pos}} = \lambda_p
\left\|\vect{p}_{k+H}-\vect{p}^{\mathrm{term}}_k\right\|^2,\qquad
J_{\mathrm{term\text{-}vel}} = \lambda_v\left\|\vect{v}_{k+H}\right\|^2.
\end{equation}
```

Map `\lambda_{\mathrm{prog}}`, `\lambda_p`, and `\lambda_v` to `terminal_path_weight`, `terminal_pos_weight`, and `terminal_vel_weight`, respectively. Add a short definition of terminal risk decay and its lower bound if it remains enabled in the final solver.

### Proposed compact risk-cost structure

Do not reproduce every building-type branch in the main manuscript. Define `d^{\mathrm{dyn}}_{i,\tau}` as predicted UAV-obstacle distance, `\rho_{i,\tau}` as the inflated dynamic radius, and `d^{\mathrm{stat}}_{j,\tau}` as the shape-aware horizontal clearance to static building `j`. Then document the implemented soft structure:

```latex
\begin{equation}
\begin{aligned}
J_{\mathrm{risk}} = \sum_{\tau=1}^{H}\eta_\tau\Bigg[
&\sum_{i=1}^{N_o}\Big(kappa_{\mathrm{dyn}}\phi_\epsilon(\rho_{i,\tau}-d^{\mathrm{dyn}}_{i,\tau})^3 \\
&\quad+\kappa_{\mathrm{dyn,near}}\phi_\epsilon(\rho_{i,\tau}+d_{\mathrm{near}}-d^{\mathrm{dyn}}_{i,\tau})^2\Big) \\
&+\sum_{j=1}^{N_s} J^{\mathrm{stat}}_{j,\tau}\Bigg],
\end{aligned}
\label{eq:jrisk}
\end{equation}
```

Define `J^{\mathrm{stat}}_{j,\tau}` in prose or an appendix as the shape-aware fourth-power violation, near-boundary penalty, and below-roof weighting used for cuboid, cylinder, and cone obstacles. Define `\eta_\tau` as the terminal-distance-dependent risk factor and state that it is bounded below by `terminal_risk_min_factor`. List every gain in Table II or a supplementary reproducibility table.

## 3. Formula-by-formula replacement map

| Existing LaTeX location | Existing equation | Planned replacement | Required prose update |
| --- | --- | --- | --- |
| Lines 275--280, `eq:total_cost` | Eq. (19) | Replace four-term objective with the eight-term structure above. | Explain terminal reference selection, soft safety role, and non-normalized weights. Remove “may satisfy `alpha+beta+gamma+delta=1`.” |
| Lines 282--286, `eq:jpath` | Eq. (20) | Replace path increment formula with `J_trk`. Keep label `eq:jpath` only if references are preserved; otherwise rename it consistently. | Rename “path-length cost” to “reference-tracking cost.” |
| Lines 287--292, `eq:jsmooth` | Eq. (21) | Replace velocity-increment formula with `J_Delta u`. | Rename it “control-increment regularization”; do not claim direct velocity smoothness. |
| Lines 294--304, `eq:jenergy` | Eq. (22) | Replace norm-displacement and non-smooth `max` with squared displacement and `phi_epsilon`. | State surrogate role; define `epsilon`; state climb asymmetry. |
| Lines 306--311, `eq:estimated_energy` | Eq. (23) | Keep physical reporting equation, pending one unified parameter set. | State it is post-run only, list coefficient values/units, and do not present it as rotor-level validation. |
| Lines 313--319, `eq:jrisk` | Eq. (24) | Replace reciprocal dynamic-only expression with compact soft dynamic/static risk expression. | Define obstacle shapes, gains, near-boundary distance, and terminal decay; state that this is a penalty rather than hard constraint. |
| Lines 323--349, `eq:opt_problem` | NLP constraints | Replace norm/hard-obstacle statements with component-wise bounds and soft-risk formulation, **or** change the code to enforce the published constraints. | Select one route before final experiments; do not describe unimplemented constraints. |

## 4. Table II modification plan

### Values to synchronize with the current tested solver

Replace the currently displayed core weights with the executable baseline values:

| Parameter group | Planned Table II entry |
| --- | --- |
| Core objective weights | `alpha=1.0`, `beta=0.5`, `gamma=0.8`, `delta=5.0` |
| Online energy weights | `c1=1.0`, `c2=0.1`, `c3=3.0`; asymmetric energy enabled |
| Terminal weights | `lambda_p=120.0`, `lambda_v=20.0`, `lambda_prog=8.0` |
| Terminal guidance/risk | goal-attraction radius `25 m`; risk-decay radius `12 m`; minimum risk factor `0.45` |
| Dynamic safety parameters | `r_u=0.3 m`, `d_m=0.2 m`, default obstacle radius `1.5 m`, `chi2_alpha=7.81` |
| Risk/trigger parameters | retain `w1=0.4`, `w2=0.4`, `w3=0.2`, `R_on=0.6`, `R_off=0.2`, `T_ref=5` |

### Additional required disclosure

1. Add a compact “soft-risk penalty parameters” row or a supplementary reproducibility table for `dyn_risk_gain=3000`, `static_risk_gain=12000`, `static_near_gain=2500`, dynamic near gain `2`, static extra margin `0.55 m`, and the building-shape-dependent near margins.
2. State that `u_max` and `v_max` are per-component bounds if the code is retained unchanged.
3. Add Eq. (23) physical energy coefficients only after P6 in `R1_revision_plan.md` is frozen; do not copy the incompatible Table III and V defaults into a final paper.
4. Keep Table II readable by moving detailed static-building branch parameters to a short appendix/supplement if necessary.

## 5. Reviewer 1 Comment 3 response logic

### Response structure

1. **Acknowledge the concern.** State that the original presentation did not fully expose the implemented NMPC objective, terminal terms, soft risk construction, and parameterization.
2. **Describe the corrective action.** State that Section III-C now gives the complete online objective, matches the reference-tracking/control-increment/energy/risk terms to the solver, and distinguishes online surrogate energy from post-run estimated energy.
3. **Clarify constraints and safety.** State precisely whether final implementation uses soft obstacle penalties and component-wise bounds; do not claim hard constraint enforcement unless it is implemented and tested.
4. **Provide reproducibility evidence.** State that Table II now contains the executed weights and material terminal/risk settings, and cite the frozen configuration and result CSVs.
5. **State experimental implication honestly.** If the final change is manuscript-only alignment to an unchanged solver, say that reported experiments are generated by that same solver. If any solver, weight, constraint, or energy parameter changes, rerun and replace all affected figures/tables before making this statement.

### Draft response logic (to be finalized after implementation)

> We thank the reviewer for highlighting the need to make the NMPC formulation fully consistent with the implementation. We revised Section III-C to present the complete objective used by the solver, including reference tracking, control-increment regularization, the asymmetric surrogate energy term, differentiable soft risk penalties, and terminal progress/position/velocity terms. We also clarified the safety-penalty formulation and expanded the parameter disclosure in Table II. The post-run energy estimate is now explicitly separated from the online surrogate cost. These revisions improve reproducibility and ensure that the mathematical description, algorithmic implementation, and reported experiments refer to the same frozen configuration.

Do not use the final sentence until Table II, solver configuration, and all affected experimental artifacts have been verified against the frozen baseline.

## Implementation checklist before touching LaTeX

- [ ] Confirm P1 source-of-truth decision in `R1_revision_plan.md`.
- [ ] Freeze physical estimated-energy coefficients and resolve Table III/V inconsistency.
- [ ] Confirm whether risk gains remain hard-coded or are moved into `init_params.m`.
- [ ] Confirm whether final paper documents component-wise soft constraints or code changes to norm/hard constraints.
- [ ] Update all cross-references if labels/names are changed.
- [ ] Compile the manuscript and verify Eq. numbering, Table II layout, Algorithm 1 references, and no undefined labels.
- [ ] Regenerate affected results before revising any numerical values or claims.
