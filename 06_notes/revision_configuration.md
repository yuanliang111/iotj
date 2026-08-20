# IEEE IoTJ R1 Revision Configuration

> Configuration snapshot for the current revision workspace. Any value marked `TODO` must be verified from the execution environment before final submission or before regenerating numerical results.

## Git Information

Branch: `revision-development`

Commit: `671af440d58cd650e9b161aba6f9e014207be221`

## Hardware

CPU: AMD Ryzen 5 5600, 3.50 GHz

RAM: 16 GB

OS: Windows 11

## Software

MATLAB: R2022b

CasADi: Optimization suite used; exact version `TODO`

IPOPT: IPOPT solver; exact version `TODO` (profiling configuration uses MUMPS linear solver)

## UAV Simulation Parameters

dt: 0.1 s

Prediction horizon: 15 steps

Velocity limit: 5.0 m/s per velocity component

Acceleration limit: 3.0 m/s^2 per control component

Altitude range: 0.5--20.0 m

## Experiment Settings

Dynamic obstacle numbers: Monte Carlo scalability cases `[1, 3, 5]`; default main scenario uses 5 dynamic obstacles

Monte Carlo trials: 100 independent trials per obstacle-count case

Random seed policy: Fixed base seed `2026` in `init_params.m`; Monte Carlo scenarios derive `2026 + 10000*obs_case_idx + 1000*algo_idx + mc`. Profiling uses 30 runs with base seed `20260615`; baseline-comparison trial count is 30 according to the manuscript.

## Result Files

Table III: `02_code_revision/ET_NMPC_revision_code/results_reviewer_response/comparison_results/comparison_summary_results.csv` (trial detail: `comparison_detail_results.csv`)

Table IV: `02_code_revision/ET_NMPC_revision_code/results_proposed_et_nmpc_profiling/profiling_summary_results.csv` (supporting files: `profiling_run_results.csv`, `profiling_step_detail_results.csv`)

Table V: `02_code_revision/ET_NMPC_revision_code/results_reviewer_response/scalability_multi_obstacle_results.csv` (trial detail: `scalability_multi_obstacle_detail_results.csv`)
