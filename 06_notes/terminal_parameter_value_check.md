# Terminal Parameter Value Check

## Scope

Static source inspection only. No MATLAB run, code change, manuscript change, Git commit, or push was performed.

Reviewed files:

- `02_code_revision/ET_NMPC_revision_code/init_params.m`
- `02_code_revision/ET_NMPC_revision_code/solve_nmpc_casadi.m`

## 1. Configured Values

| Parameter | Value in `init_params.m` | Source location | Solver fallback / protection |
|---|---:|---|---|
| `terminal_pos_weight` | `120.0` | lines 350--354 | Missing field defaults to `120.0`; any value below `120.0` is raised to `120.0` (solver lines 116--138). |
| `terminal_vel_weight` | `20.0` | lines 356--358 | Missing field defaults to `20.0`; any value below `20.0` is raised to `20.0` (solver lines 120--139). |
| `terminal_path_weight` | `8.0` | lines 360--363 | Missing field defaults to `8.0` (solver lines 141--144). No positive lower clamp is imposed; `init_params.m` only rejects negative values. |

Therefore, for a normal call using `init_params.m`, the active values are:

\[
\texttt{terminal\_pos\_weight}=120.0,\qquad
\texttt{terminal\_vel\_weight}=20.0,\qquad
\texttt{terminal\_path\_weight}=8.0.
\]

## 2. Role in the NMPC Objective

### Terminal position weight: `terminal_pos_weight = 120.0`

At solver line 564, the terminal position term is

\[
J_{\mathrm{terminal,pos}}=
\texttt{terminal\_pos\_weight}\,
\left\|X_{1:3,H+1}-p_{\mathrm{terminal\_ref}}\right\|_2^2.
\]

It penalizes the final predicted position error. The terminal reference is `params.p_goal` only when the current UAV position is within `terminal_goal_attract_radius = 25.0 m`; otherwise, it is the last local-reference point `Pi_ref_local(:, end)` (solver lines 229--239).

### Terminal velocity weight: `terminal_vel_weight = 20.0`

At solver line 565, the terminal velocity term is

\[
J_{\mathrm{terminal,vel}}=
\texttt{terminal\_vel\_weight}\,
\left\|X_{4:6,H+1}\right\|_2^2.
\]

It penalizes nonzero velocity at the end of the prediction horizon, discouraging overshoot and circling near the terminal reference.

### Terminal path weight: `terminal_path_weight = 8.0`

At solver lines 287--291, the stage-wise terminal-attraction term is

\[
J_{\mathrm{terminal,progress}}=
\sum_{k=1}^{H}
\texttt{terminal\_path\_weight}
\left(\frac{k}{H}\right)^2
\left\|X_{1:3,k+1}-p_{\mathrm{terminal\_ref}}\right\|_2^2.
\]

The quadratic factor $\left(k/H\right)^2$ makes the attraction stronger later in the prediction horizon. This is a positive penalty on distance to the terminal reference at every stage; it does **not** implement a negative progress-reward term.

## 3. Assembly in the Objective

At solver lines 567--573, the total objective is assembled as

\[
J_{\mathrm{total}}
=\alpha J_{\mathrm{path}}
+\beta J_{\mathrm{smooth}}
+\gamma J_{\mathrm{energy}}
+\delta J_{\mathrm{risk}}
+J_{\mathrm{terminal,progress}}
+J_{\mathrm{terminal,pos}}
+J_{\mathrm{terminal,vel}}.
\]

The three terminal contributions are added directly; they are not multiplied by $\alpha$, $\beta$, $\gamma$, or $\delta$.

## 4. Manuscript Mapping Note

The code uses the parameter names `terminal_pos_weight`, `terminal_vel_weight`, and `terminal_path_weight`; it does not use $\lambda_p$, $\lambda_v$, or $\lambda_{\mathrm{prog}}$ as identifiers. A faithful manuscript/Table II mapping would be:

| Manuscript concept | Code parameter | Current code value |
|---|---|---:|
| Terminal position weight ($\lambda_p$) | `terminal_pos_weight` | `120.0` |
| Terminal velocity weight ($\lambda_v$) | `terminal_vel_weight` | `20.0` |
| Terminal path/progress weight ($\lambda_{\mathrm{prog}}$) | `terminal_path_weight` | `8.0` |

The final row should be described as a **stage-wise terminal-attraction/path weight** if the manuscript is intended to match the present solver exactly. The current code does not contain the manuscript's explicit negative $J_{\mathrm{progress}}$ reward formulation.
