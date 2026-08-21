# Reviewer #1 Comment 4: Asymmetric-Energy Ablation

## Purpose

This experiment supports the response to Reviewer #1 Comment 4: the benefit of the asymmetric climb penalty must be evaluated through a controlled ablation. It compares the proposed asymmetric-energy ET-NMPC with a symmetric-energy ET-NMPC baseline while preserving the same sensing, prediction, risk, event-triggering, NMPC, map, and dynamic-obstacle settings.

## Compared controllers

- **Asymmetric ET-NMPC:** `use_event_trigger=true`, `use_prediction=true`, `use_asymmetric_energy=true`, and `c3=3.0`.
- **Symmetric-energy ET-NMPC:** `use_event_trigger=true`, `use_prediction=true`, `use_asymmetric_energy=false`, and `c3=0.0`.

All other controller and scenario parameters are inherited unchanged from the current revision code. In particular, the observed result records show `c1=1.0` and `c2=0.1` for both modes.

## Paired Monte Carlo setting

The experiment contains 30 paired trials. For trial index `i`, the scenario seed is

```text
scenario_seed = 2026 + i
```

The asymmetric controller is run first, then the symmetric controller is run after resetting MATLAB's random stream with `rng(scenario_seed, 'twister')`. Thus, each pair uses the same static map, UAV initial state, goal, dynamic-obstacle initialization, stochastic obstacle motion, and measurement-noise realization. The recorded seeds range from 2027 through 2056.

## Output files

- `results/asymmetric_energy_ablation_detail.csv` — one row for each controller execution in each trial. It records the seed, controller mode, energy-cost parameters, success/collision outcomes, and scalar metrics.
- `results/asymmetric_energy_ablation_trajectory.csv` — complete UAV trajectories, including the initial state, with time, position, trial, seed, and controller-mode identifiers.
- `results/asymmetric_energy_ablation_summary.csv` — mode-wise mean, standard deviation, median, and 95% confidence interval, plus the paired difference defined as **Asymmetric - Symmetric**.

## Recorded metrics

The detail output records success, collision, total positive climb, estimated energy, path length, flight time, minimum dynamic-obstacle clearance, trigger rate, control effort, and executed steps. Total positive climb is defined as `sum(max(0, diff(z)))`; path length is the accumulated three-dimensional distance along the complete saved trajectory; flight time is `Steps * dt`.

## Energy-interpretation boundary

`EstimatedEnergyWh` is a model-based mission-energy proxy computed from base power over flight time, positive-climb work, and control effort. It is **not** a physical battery measurement, a hardware power measurement, or an embedded-flight validation result. It should therefore be reported as an estimated energy proxy only.
