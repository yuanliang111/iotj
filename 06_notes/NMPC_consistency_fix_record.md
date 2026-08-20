# NMPC Consistency Fix Record

## 1. Modified Location

- Manuscript: `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`
- Section modified: Section III-C, *Local Predictive Optimization Formulation*.
- Table modified: Table II, the *NMPC cost weights* block only.
- No MATLAB/CasADi source, experiment, result, figure, Table III--V, Git commit, or remote repository was modified.

All newly inserted or rewritten visible manuscript text, formulas, and Table II cells are marked using `\textcolor{blue}{...}`.

## 2. Before-and-After Changes

| Item | Before | After |
|---|---|---|
| Objective weights | $w_1,\ldots,w_5$, colliding with the existing risk-fusion $w_1,w_2,w_3$ | $\rho_1,\ldots,\rho_5$ for the NMPC objective; risk-fusion weights remain $w_1,w_2,w_3$. |
| Reference tracking label | `eq:jpath` | `eq:jtrack`; no other live manuscript cross-reference to `eq:jpath` was found. |
| Terminal cost | Terminal position and velocity penalties plus a negative $J_{\mathrm{progress}}$ reward | Positive terminal position, velocity, and stage-wise terminal-attraction penalties matching the solver structure. |
| Terminal reference | Fixed use of $p_{\mathrm{goal}}$ in the displayed terminal-position term | $p_{\mathrm{terminal\_ref}}$: $p_{\mathrm{goal}}$ within 25 m of the goal, otherwise the last local reference point. |
| Dynamic risk radius in III-C | $\rho_i(k+\tau)$ | $d_i^{\mathrm{safe}}(k+\tau)$, defined explicitly in the local NMPC formulation. |
| Risk cost | Undefined $\ell_{\mathrm{dyn}}$, $\ell_{\mathrm{stat}}$, $N_s$, and $\mathcal{O}_j$ | Defined dynamic/static obstacle sets, smooth positive operator, dynamic polynomial soft penalty, and geometry-dependent static polynomial soft penalty. No inverse-distance penalty remains in the NMPC risk cost. |
| Table II NMPC weights | $\alpha,\beta,\gamma,\delta$ with obsolete values | $\rho_1=1.0$, $\rho_2=0.5$, $\rho_3=0.8$, $\rho_4=5.0$, and $\rho_5=1.0$. |
| Table II terminal values | Blue `TODO` entries | $\lambda_p=120$, $\lambda_v=20$, and $\lambda_{\mathrm{prog}}=8$, taken from `init_params.m`. |

## 3. Code-Consistency Mapping

- $\rho_1$--$\rho_4$ document the existing MATLAB coefficients `alpha=1.0`, `beta=0.5`, `gamma=0.8`, and `delta=5.0`.
- $\rho_5=1.0$ documents the direct addition of the assembled terminal term in `J_total`; it is not a new MATLAB parameter.
- $\lambda_p$, $\lambda_v$, and $\lambda_{\mathrm{prog}}$ map to `terminal_pos_weight=120.0`, `terminal_vel_weight=20.0`, and `terminal_path_weight=8.0`, respectively.
- The terminal attraction uses $((\tau+1)/H)^2$, as in the stage-wise solver term. It is a positive penalty under minimization rather than a negative progress reward.

## 4. Reviewer Comment Coverage

Reviewer comment: “The NMPC optimization objective lacks explicit reference tracking, path progress, and terminal goal cost.”

- **Reference tracking:** retained explicitly as $J_{\mathrm{track}}$ and relabelled `eq:jtrack`.
- **Path progress:** represented by the stage-wise terminal-attraction term, whose weight grows toward the end of the prediction horizon.
- **Terminal goal cost:** represented by terminal position and velocity penalties using the solver's conditional terminal reference.
- **Implementation consistency:** the objective weight names, terminal-weight values, and soft collision penalties now use the current MATLAB/CasADi terminology and parameter values.

## 5. Compilation Check

The revised document was compiled in an isolated temporary copy of the manuscript directory:

- `pdflatex` exit code: `0`.
- Eq. (19)--(25) resolve consecutively as total cost, tracking, smoothness, energy, estimated energy, risk, and terminal cost.
- `eq:jtrack` resolves as Eq. (20); no live target-manuscript occurrence of `eq:jpath` remains.
- The final isolated compilation reported no LaTeX error, undefined reference, or overfull-box warning.

## 6. Scope Note

The user-authorized editing scope was Section III-C and Table II only. Accordingly, the pre-existing Section III-B predictive-risk equations retain their original $\rho_i(k+\tau)$ notation; Section III-C defines and uses $d_i^{\mathrm{safe}}(k+\tau)$ for the NMPC risk formulation to avoid collision with the new objective weights. A full-paper notation unification would require a separately authorized Section III-B edit.
