# NMPC Revision Review

## Review Scope

- Reviewed file: `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`.
- Review mode: read-only for the manuscript, MATLAB/CasADi code, Git history, and remote repository.
- Verification: the current manuscript compiled successfully with `pdflatex` in an external temporary output directory (exit code `0`).

## 1. Git Diff Scope Check

**Result: PASS, with one non-semantic exception.**

The substantive diff is restricted to:

- Section III-C, *Local Predictive Optimization Formulation*, source lines 277--335: objective, tracking, smoothness, energy explanation, risk, and terminal/progress cost text and equations.
- Table II, source lines 526--528: three terminal-weight entries.

No prose, equations, results, figures, tables of results, or other manuscript sections were substantively changed. The diff also removes two blank lines after `\end{document}`; this is an end-of-file whitespace-only change, not a content change.

## 2. Blue Revision Marking

**Result: PASS for visible revised content.**

All added or rewritten prose, mathematical expressions, and Table II cells are enclosed in `\textcolor{blue}{...}`. The only added lines without this wrapper are LaTeX structural commands (`\begin`, `\end`, and `\label`), which do not produce visible manuscript content.

## 3. Equation Numbering

Temporary compilation resolves the equation labels as follows:

| Equation | Label | Content | Status |
|---|---|---|---|
| Eq. (19) | `eq:total_cost` | Five-term NMPC objective | Continuous |
| Eq. (20) | `eq:jpath` | $J_{track}$ reference-tracking cost | Continuous |
| Eq. (21) | `eq:jsmooth` | $J_{smooth}$ control-increment penalty | Continuous |
| Eq. (22) | `eq:jenergy` | Energy-aware surrogate cost | Continuous |
| Eq. (23) | `eq:estimated_energy` | Estimated physical energy metric | Continuous |
| Eq. (24) | `eq:jrisk` | Differentiable soft risk cost | Continuous |
| Eq. (25) | `eq:jterminal` | Terminal position, velocity, and progress cost | Continuous |
| Eq. (26) | `eq:jprogress` | Explicit path-progress incentive | New and continuous |

Eq. (19)--(25) are consecutive. The newly introduced definition of $J_{progress}$ is Eq. (26), so any later reviewer response or cross-reference that describes the full revised objective as only Eqs. (19)--(25) should also cite Eq. (26).

## 4. Mathematical Consistency Check

| Term | Review result | Finding |
|---|---|---|
| $J_{track}$ | Conditionally complete | It explicitly penalizes squared predicted-position deviation from the local reference over the horizon, satisfying the reference-tracking requirement. The retained label `eq:jpath` is now semantically outdated, but does not affect compiled output. |
| $J_{smooth}$ | Complete | It penalizes consecutive control increments, which is logically consistent with the stated control-smoothness purpose. |
| $J_{energy}$ | Complete as a surrogate | Its unchanged formula retains displacement, control effort, and asymmetric climb terms. The revised explanation correctly states the intended energy-aware role and includes horizontal motion. Note that the displayed displacement is the full 3-D norm, not a horizontal-only norm. |
| $J_{risk}$ | Formally incomplete | It correctly replaces inverse distance with differentiable soft dynamic/static obstacle penalties. However, $\ell_{\mathrm{dyn}}$, $\ell_{\mathrm{stat}}$, $N_s$, and $\mathcal{O}_j$ are not defined in this formulation, so the expression is not independently reproducible from the manuscript. |
| $J_{terminal}$ | Internally coherent | Positive terminal position and velocity terms are penalties. Since $J_{progress}$ is positive when the predicted trajectory is closer to the goal than the reference, the negative $\lambda_{prog}J_{progress}$ term correctly acts as an incentive under minimization. |

## 5. Reviewer Comment 3 Coverage

Reviewer comment: "The NMPC optimization objective lacks explicit reference tracking, path progress, and terminal goal cost."

**Result: conceptually covered.**

- Reference tracking is explicitly supplied by Eq. (20), $J_{track}$.
- Path progress is explicitly supplied by Eq. (26), $J_{progress}$, and rewarded in Eq. (25).
- Terminal goal cost is explicitly supplied by the terminal position-error term in Eq. (25); a terminal-velocity penalty is also included.

The response is suitable as a first manuscript-level answer, subject to the consistency issues below.

## 6. Issues to Resolve Before Submission

1. **High -- objective-weight symbol collision.** Section III-B and Table II already use $w_1$, $w_2$, and $w_3$ as composite-risk fusion weights. Eq. (19) now reuses $w_1,\ldots,w_5$ as NMPC objective weights. These have different meanings in the same section of the paper and are ambiguous.
2. **High -- Table II/objective mapping is incomplete.** Table II retains $\alpha$, $\beta$, $\gamma$, and $\delta$ for the old four costs, whereas Eq. (19) uses $w_1,\ldots,w_5$. No mapping or numerical values for the five objective weights are provided.
3. **High -- terminal parameters remain unresolved.** The newly added $\lambda_p$, $\lambda_v$, and $\lambda_{prog}$ rows are `TODO`. They must be assigned documented values, or Table II must explicitly explain their relationship to the actual implementation, before submission.
4. **Medium -- soft-risk definition is underspecified.** Define the forms of $\ell_{\mathrm{dyn}}$ and $\ell_{\mathrm{stat}}$, and introduce $N_s$ and $\mathcal{O}_j$, or cite an earlier precise definition.
5. **Low -- legacy label.** Rename `eq:jpath` to a tracking-oriented label only after checking and updating any cross-references.

## 7. Actions Not Performed

- No persistent change was made to the manuscript, MATLAB/CasADi code, experiment, result, Git commit, or remote repository as part of this review.
