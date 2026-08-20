# NMPC LaTeX Revision Record

## 1. Modified File Location

- Manuscript source: `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`
- Section: Section III-C, *Local Predictive Optimization Formulation*.
- Modified source locations: the NMPC objective and component-cost text/formulas around lines 277--335; the NMPC cost-weight block of Table II around lines 520--525.
- All newly added or rewritten manuscript text, equations, and Table II entries are enclosed in `\textcolor{blue}{...}`.

## 2. Formula Correspondence Before and After

| Original item | Original formulation | Revised formulation |
|---|---|---|
| Total objective, Eq. (19) | $J=\alpha J_{path}+\beta J_{smooth}+\gamma J_{energy}+\delta J_{risk}$ | $J=w_1J_{track}+w_2J_{smooth}+w_3J_{energy}+w_4J_{risk}+w_5J_{terminal}$ |
| Path term, Eq. (20) | Path-length penalty $\sum \|p_{k+\tau+1}-p_{k+\tau}\|$ | Reference-tracking cost $\sum \|p_{k+\tau+1}-r_{k+\tau+1}\|_2^2$ |
| Smoothness term, Eq. (21) | Velocity-difference penalty $\sum \|v_{k+\tau+1}-v_{k+\tau}\|^2$ | Control-increment penalty $\sum \|u_{k+\tau+1}-u_{k+\tau}\|_2^2$ |
| Energy term, Eq. (22) | Motion distance, control effort, and climb penalty | Formula retained; its description now explicitly identifies horizontal motion, control effort, asymmetric climb penalty, and its role as an energy-aware objective. |
| Risk term, Eq. (24) | Inverse-distance penalty | Differentiable soft collision-avoidance penalty for dynamic and static obstacles, stated as consistent with the CasADi implementation. |
| Terminal term | Not present | New $J_{terminal}$ with terminal position error, terminal velocity penalty, and a path-progress incentive $J_{progress}$. |

## 3. Reviewer Comment Correspondence

Reviewer comment: “The NMPC optimization objective lacks explicit reference tracking, path progress, and terminal goal cost.”

- **Explicit reference tracking:** Eq. (20) now defines $J_{track}$ using the predicted position and local reference position at every horizon step.
- **Path progress:** the new $J_{progress}$ compares the predicted trajectory with the local reference trajectory in terms of goal-distance reduction; it enters $J_{terminal}$ as an incentive.
- **Terminal goal cost:** the new $J_{terminal}$ includes a terminal position-error penalty to $p_{goal}$ and a terminal-velocity penalty.
- **Implementation terminology:** the revised smoothness and risk descriptions use control increments and differentiable soft collision avoidance, respectively, to align the manuscript description with the MATLAB/CasADi implementation terminology.

## 4. Table II Update

The existing Table II rows and numeric values were retained. The following blue entries were appended under the NMPC cost-weight block:

- $\lambda_p$: terminal position weight.
- $\lambda_v$: terminal velocity weight.
- $\lambda_{prog}$: terminal progress weight.

Their values are marked `TODO`; no numerical value was inferred or changed, so the manuscript revision does not alter the reported experimental configuration.

## 5. Unmodified Portions

- No MATLAB/CasADi source file was changed.
- No experimental setting, execution, result, figure, table value, or reported conclusion was changed.
- The estimated-energy expression (Eq. (23)), NMPC constraints, dynamics, event-triggering mechanism, and all manuscript sections outside the requested Section III-C and the three added Table II parameter rows were left unchanged.
- No Git commit or push was performed.
