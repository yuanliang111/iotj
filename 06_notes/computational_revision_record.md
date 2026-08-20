# Computational Revision Record

## Reviewer Comment 2

> "The real-time feasibility claims need to be reconsidered."

## Modified File and Locations

| Location | Revision |
|---|---|
| Abstract | Replaced the unqualified real-time efficiency wording with computational efficiency and resource-aware computation reduction. The reported 21.1--24.2\% CPU-time reduction is explicitly identified as a desktop MATLAB/CasADi profiling result. |
| Introduction | Added a limitation statement that event-triggering reduces optimization activations and workload, while the present work provides desktop MATLAB/CasADi characterization rather than strict onboard real-time validation. |
| Section IV-D title | Renamed to **Computational Performance Profiling and Deadline-Oriented Analysis**. |
| Section IV-D timing scope | Stated that the timer measures MATLAB/CasADi controller computation, including triggered NMPC solving, rather than complete physical closed-loop latency with communication and hardware execution. |
| Section IV-D profiling table | Expanded the existing profiling table without creating a new table number. The added deadline-oriented part reports pooled all-step and triggered-step mean, median, P95, P99, maximum time, the 100 ms deadline, and deadline-violation ratios. |
| Section IV-D disclosure and limitation | Clarified that IPOPT, MUMPS, warm start, and memory observations refer to the desktop MATLAB/CasADi profiling configuration; process-memory values are neither solver-specific nor embedded-memory measurements. Explicitly states that triggered desktop optimization does not meet the 100 ms deadline. |
| Conclusion | Reframed computational results as desktop computational characterization and relative CPU savings, not hard real-time or embedded feasibility. Embedded platforms, HIL, and compiled implementations are stated only as future validation work. |

## Deadline-Oriented Values Added

Values are taken from the step-level profiling analysis documented in `computational_revision_analysis.md`.

| Metric | All controller steps | Triggered NMPC steps |
|---|---:|---:|
| Mean (ms) | 2077.9 | 2658.3 |
| Median (ms) | 2551.9 | 2596.2 |
| P95 (ms) | 2964.7 | 2987.2 |
| P99 (ms) | 3150.4 | 3187.8 |
| Maximum (ms) | 3982.8 | 3982.8 |
| 100 ms deadline violation ratio | 3183/4072 (78.168\%) | 3183/3183 (100.0\%) |

## Revision Rationale

The existing profiling data support relative desktop computational savings and solver-behavior disclosure. They do not support a 100 ms hard real-time claim or embedded-platform feasibility. The revision therefore retains the reported CPU reduction while making the timing scope, deadline misses, desktop limitation, solver disclosure, and future validation boundary explicit.

## TODO Status

No timing-statistic TODO remains: the required mean, median, P95, P99, maximum, and deadline-violation values were available from the existing step-level profiling analysis. CasADi/IPOPT version identifiers remain outside the scope of this text-only revision and are not added as inferred values.
