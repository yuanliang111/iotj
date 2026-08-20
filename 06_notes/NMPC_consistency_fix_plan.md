# NMPC LaTeX Consistency Fix Plan

## Scope and Guardrails

- Target manuscript: `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`.
- This is a plan only. Do not modify the manuscript, MATLAB/CasADi code, experimental results, Git history, or remote repository while executing this planning task.
- In the future LaTeX edit, every newly inserted or rewritten visible item must remain inside `\textcolor{blue}{...}`.
- The numerical values below are taken from the active defaults in `init_params.m`, not inferred from the current Table II values.

## 1. Required Symbol Refactor: Objective Weights

### Current inconsistency

Eq. (19) uses $w_1,\ldots,w_5$ as NMPC objective weights. However, Section III-B and Table II already use $w_1,w_2,w_3$ as the TCPA, predictive-occupancy, and reference-deviation **risk-fusion** weights. The same symbols therefore have two different meanings.

### Planned change

Replace the Eq. (19) objective by

\[
J = \rho_1 J_{\mathrm{track}} + \rho_2 J_{\mathrm{smooth}}
  + \rho_3 J_{\mathrm{energy}} + \rho_4 J_{\mathrm{risk}}
  + \rho_5 J_{\mathrm{terminal}}.
\]

Replace the accompanying sentence so that $\rho_1,\ldots,\rho_5$ are explicitly the NMPC objective weights. Retain $w_1,w_2,w_3$ exclusively for the pre-existing composite-risk fusion in Section III-B and Table II.

### Mandatory collision avoidance

The manuscript already defines $\rho_i(k+\tau)$ as the dynamic-obstacle safety/inflation radius in Eq. (18), and uses it in Eq. (17) and the current Eq. (24). Therefore, using $\rho_i$ for both roles would create a second ambiguity.

Before introducing objective weights $\rho_1,\ldots,\rho_5$, rename the obstacle radius notation consistently in Eqs. (17), (18), and the risk-cost definition. Recommended notation:

\[
d^{\mathrm{safe}}_i(k+\tau)=r_u+r_i+\Delta_i(k+\tau)+d_m.
\]

Then use $d^{\mathrm{safe}}_i(k+\tau)$ as the dynamic-obstacle safety radius in the predictive-risk and NMPC-risk equations. This is a manuscript-only notation change; do not rename MATLAB variables.

## 2. Table II Synchronization

### NMPC cost-weight block

Replace the four obsolete $\alpha$--$\delta$ rows with the following NMPC objective-weight rows, using the actual `init_params.m` defaults:

| Table II symbol | Description | Source field | Value |
|---|---|---|---:|
| $\rho_1$ | Reference-tracking cost weight | `params.alpha` | 1.0 |
| $\rho_2$ | Control-increment cost weight | `params.beta` | 0.5 |
| $\rho_3$ | Energy-aware cost weight | `params.gamma` | 0.8 |
| $\rho_4$ | Differentiable soft risk-cost weight | `params.delta` | 5.0 |
| $\rho_5$ | Outer terminal-cost weight | direct addition in `J_total` | 1.0 |

$\rho_5=1.0$ is not an additional MATLAB parameter: it documents that the assembled terminal contribution is added directly to `J_total` without an outer multiplier. It must not be implemented as a new code parameter unless a later, separately authorized code revision introduces one.

### Terminal-weight rows

Replace the current blue `TODO` values with the verified code values:

| Table II symbol | Description that matches code | Source field | Value |
|---|---|---|---:|
| $\lambda_p$ | Terminal position weight | `terminal_pos_weight` | 120.0 |
| $\lambda_v$ | Terminal velocity weight | `terminal_vel_weight` | 20.0 |
| $\lambda_{\mathrm{prog}}$ | Stage-wise terminal-attraction/path weight | `terminal_path_weight` | 8.0 |

Keep the risk-assessment rows $w_1,w_2,w_3$ unchanged; they are not NMPC objective weights. Colour every changed Table II cell blue.

## 3. Terminal-Cost Alignment

### Current inconsistency

The manuscript currently defines a negative path-progress reward, $-\lambda_{\mathrm{prog}}J_{\mathrm{progress}}$. The solver instead adds a positive, later-horizon-weighted terminal-attraction penalty.

### Planned replacement

Replace the terminal formulation with an expression that matches the solver:

\[
J_{\mathrm{terminal}}
=\lambda_p\left\|p_{k+H}-p_{\mathrm{terminal\_ref}}\right\|_2^2
+\lambda_v\left\|v_{k+H}\right\|_2^2
+\lambda_{\mathrm{prog}}\sum_{\tau=0}^{H-1}
\left(\frac{\tau+1}{H}\right)^2
\left\|p_{k+\tau+1}-p_{\mathrm{terminal\_ref}}\right\|_2^2.
\]

Define $p_{\mathrm{terminal\_ref}}$ exactly as in the solver: it equals $p_{\mathrm{goal}}$ when the current UAV position is within `terminal_goal_attract_radius = 25.0 m`; otherwise it is the final local-reference point. Remove the standalone negative-reward definition of $J_{\mathrm{progress}}$ rather than retaining an unused Eq. (26).

This preserves the reviewer-requested path-progress behaviour while accurately describing the current implementation as progressively stronger terminal attraction.

## 4. Risk-Cost Definition Completion

### Current incompleteness

The current Eq. (24) invokes $\ell_{\mathrm{dyn}}$, $\ell_{\mathrm{stat}}$, $N_s$, and $\mathcal{O}_j$ without definitions. It is therefore not reproducible from the paper.

### Planned additions

Immediately after the revised risk-cost equation:

1. Define $N_o$ and $N_s$ as the numbers of dynamic and static obstacles; define $\mathcal{O}_j$ as the geometry of the $j$th static obstacle.
2. Define the smooth positive-part operator used by the CasADi implementation, for example

   \[
   [s]_+^{\epsilon}=\tfrac{1}{2}\left(s+\sqrt{s^2+\epsilon^2}\right),\qquad \epsilon>0.
   \]

3. Define the dynamic penalty in the same form as the solver's smooth polynomial penalties:

   \[
   \ell_{\mathrm{dyn}}(d,d^{\mathrm{safe}})
   =g_{\mathrm{dyn}}[d^{\mathrm{safe}}-d]_+^{\epsilon\,3}
   +g_{\mathrm{near}}[d^{\mathrm{safe}}+b_{\mathrm{dyn}}-d]_+^{\epsilon\,2}.
   \]

4. Define $\ell_{\mathrm{stat}}(p,\mathcal{O}_j)$ as the corresponding geometry-dependent smooth polynomial penalty used for static obstacles, including the static safety margin, near-obstacle buffer, and applicable altitude/roof factor. The exact final notation must preserve the solver's geometry cases; do not substitute an inverse-distance formula.
5. State that the gains and numerical smoothing constant are configured by the MATLAB parameter structure. Only include their numerical values in Table II if the table is intentionally expanded to cover risk-shaping parameters.

All added definitions and replacements must be blue in the manuscript.

## 5. Label Correction

### Required edit

At the reference-tracking equation, change

```latex
\label{eq:jpath}
```

to

```latex
\label{eq:jtrack}
```

Before making that change, search the full project for `eq:jpath` and update every `\ref`, `\eqref`, or other cross-reference to `eq:jtrack`. This prevents unresolved references.

## 6. Execution Order and Acceptance Checks

1. Search all uses of `w_1`, `w_2`, `w_3`, `\rho_i`, `eq:jpath`, and the affected equation references.
2. Apply the safety-radius notation refactor first.
3. Replace the objective weights with $\rho_1,\ldots,\rho_5$ and revise the explanatory sentence.
4. Replace the terminal expression with the stage-wise terminal-attraction form and remove the obsolete standalone progress-reward equation.
5. Add complete dynamic/static soft-risk definitions.
6. Synchronize Table II with the values listed above while retaining the independent risk-fusion $w_i$ rows.
7. Verify every visible changed manuscript element is wrapped in `\textcolor{blue}{...}`.
8. Compile LaTeX to a temporary output directory; confirm no undefined references, duplicate labels, or equation-number gaps.
9. Confirm Eq. (19)--(25) remain consecutive after removing the standalone progress equation.

## No-Change Statement

This plan does not alter `my paper3.1.tex`, MATLAB/CasADi code, experimental outputs, commits, or remotes.
