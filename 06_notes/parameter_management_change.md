# NMPC Parameter Management Change

## Scope

This change centralizes the requested NMPC risk and smoothing parameters without changing the objective structure, constraints, solver options, experiment entry points, or paper source. No experiment was run and no result file was regenerated.

## Modified files

- `02_code_revision/ET_NMPC_revision_code/init_params.m`
- `02_code_revision/ET_NMPC_revision_code/solve_nmpc_casadi.m`

## Before: hard-coded locations in `solve_nmpc_casadi.m`

| Parameter | Previous local definition | Previous source location |
| --- | --- | --- |
| `smooth_eps` | `smooth_eps = 1e-4;` | Original line 181 |
| `dyn_risk_gain` | `dyn_risk_gain = 3000;` | Original line 184 |
| `static_risk_gain` | `static_risk_gain = 12000;` | Original line 188 |
| `static_extra_margin` | `static_extra_margin = 0.55;` | Original line 192 |
| `static_near_gain` | `static_near_gain = 2500;` | Original line 195 |
| `terminal_risk_decay_radius` | Already read as `params.terminal_risk_decay_radius`, with default `12.0` | Original lines 149--151 |
| `terminal_risk_min_factor` | Already read as `params.terminal_risk_min_factor`, with default `0.45` and clamp `[0.45, 1.0]` | Original lines 155--165 |

## After: centralized parameter fields

| Parameter field | Value preserved in `init_params.m` | Usage in `solve_nmpc_casadi.m` |
| --- | --- | --- |
| `params.smooth_eps` | `1e-4` | Smooth climb term, smooth positive-part penalties, and smoothed cuboid distance calculation |
| `params.dyn_risk_gain` | `3000` | Dynamic-obstacle cubic violation penalty |
| `params.static_risk_gain` | `12000` | Static-building fourth-power violation penalty |
| `params.static_near_gain` | `2500` | Static-building near-boundary penalty |
| `params.static_extra_margin` | `0.55 m` | Static-building planning safety radius |
| `params.terminal_risk_decay_radius` | `12.0 m` | Terminal-distance-dependent risk decay denominator |
| `params.terminal_risk_min_factor` | `0.45` | Terminal risk-decay lower bound; existing solver clamp retained |

## Default-value protection

`solve_nmpc_casadi.m` now assigns the original numerical defaults when a caller supplies an older `params` structure missing any newly centralized field:

```matlab
if ~isfield(params, 'smooth_eps')
    params.smooth_eps = 1e-4;
end
```

The same protection is present for `dyn_risk_gain`, `static_risk_gain`, `static_near_gain`, and `static_extra_margin`. Existing protections for `terminal_risk_decay_radius` and `terminal_risk_min_factor`, including the latter's `[0.45, 1.0]` clamp, were retained unchanged.

## Behavioral preservation

- All parameter values are identical to the prior hard-coded values.
- Cost terms, their exponents, and their multiplication order are unchanged.
- State/control/altitude constraints are unchanged.
- IPOPT/CasADi solver settings and fallback behavior are unchanged.
- Experiment scripts and existing result files are unchanged.

## Validation performed

1. Static search confirmed that all operational uses of the five migrated constants now reference `params.<field>`; only compatibility-default checks and a descriptive comment retain bare field names.
2. `git diff --check` completed with no whitespace errors.
3. MATLAB R2022b loaded `init_params()` and asserted the seven requested field values:
   `smooth_eps=1e-4`, `dyn_risk_gain=3000`, `static_risk_gain=12000`, `static_near_gain=2500`, `static_extra_margin=0.55`, `terminal_risk_decay_radius=12.0`, and `terminal_risk_min_factor=0.45`.
4. MATLAB `checkcode` inspected `solve_nmpc_casadi.m`; it reported one pre-existing warning-format recommendation at the fallback warning call, not a syntax error. No simulation was executed.
