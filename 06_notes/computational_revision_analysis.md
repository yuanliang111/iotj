# Computational Revision Analysis - Reviewer Comment 2

## Scope and Evidence Sources

This is a read-only analysis of:

- `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`
- `02_code_revision/ET_NMPC_revision_code/results_proposed_et_nmpc_profiling/profiling_run_results.csv`
- `02_code_revision/ET_NMPC_revision_code/results_proposed_et_nmpc_profiling/profiling_step_detail_results.csv`
- `02_code_revision/ET_NMPC_revision_code/results_proposed_et_nmpc_profiling/profiling_summary_results.csv`
- `02_code_revision/ET_NMPC_revision_code/main_proposed_et_nmpc_profiling.m`
- `02_code_revision/ET_NMPC_revision_code/solve_nmpc_casadi.m`, `init_params.m`, and relevant files in `06_notes`.

No manuscript, MATLAB code, experiment, Git commit, or push was modified.

## 1. Current Manuscript Statements Related to Computation and Real-Time Feasibility

| Location in `my paper3.1.tex` | Current statement / role | Assessment |
|---|---|---|
| Abstract, line 71 | Claims "real-time computational efficiency" and reports 21.1--24.2% average CPU-time reduction. | The relative desktop CPU-time reduction is supported for tested scenarios. "Real-time" is not supported by the measured 0.1 s control period and 2.07 s mean desktop controller time. |
| Introduction, lines 80, 85, 89, 93, and 95 | Motivates limited onboard resources, real-time collision risk, and reduced solver calls. | Appropriate motivation; it must not be read as evidence of onboard or hard real-time validation. |
| System model, lines 156 and 182 | Calls the dynamics/predictor lightweight or suitable for online computational efficiency. | Qualitative and defensible if restricted to relative complexity; it is not a timing guarantee. |
| Framework, lines 186, 192, and 273--473 | States that triggering reduces unnecessary optimization calls and supports future onboard companion computers; gives a complexity argument. | Supports reduced solver activation and average computational burden, not a deadline guarantee. |
| Simulation setup, line 513 | Declares MATLAB R2022b, CasADi, Windows 11, AMD Ryzen 5 5600, and 16 GB RAM. | Useful desktop disclosure; CasADi/IPOPT versions are still absent. |
| Baseline/sensitivity/scalability results, lines 604, 678, and 785 | Reports CPU-time comparisons, parameter sensitivity, and 21.1--24.2% savings. | Supports relative CPU savings, not absolute real-time feasibility. |
| Section IV profiling subsection, lines 684--738 | Provides timing, IPOPT, memory, solver settings, and a desktop-prototype/embedded-future discussion. | This substantially addresses the reviewer's disclosure request, but needs qualification and data-quality/reproducibility fixes below. |
| Conclusion, lines 791--795 | States that profiling supports a "real-time feasibility discussion" and lists embedded validation as future work. | Should be narrowed to desktop computational characterization; no embedded result supports real-time feasibility. |

## 2. What the Existing Profiling Data Supports

### Profiling data inventory

- 30 profiling runs.
- 4,072 rows in `profiling_step_detail_results.csv`; 3,183 rows have `Sigma=1` (triggered NMPC).
- `profiling_run_results.csv` contains run-level CPU, IPOPT, success/failure, fallback, and memory summaries.
- `profiling_summary_results.csv` records the aggregate values used by the manuscript profiling table.

### Conclusions that are directly supported

1. **Event triggering reduces NMPC invocation frequency in this desktop experiment.** The reported mean trigger rate is 77.810% with a 3.474% run-level standard deviation, so the event-triggered controller does not solve NMPC at every simulated step.
2. **Triggered NMPC calls dominate the measured controller-decision time.** The summary reports 2,658.3 ms mean triggered time, while non-triggered steps have a 0.015 ms mean time.
3. **The profiling helper converged reliably on its recorded triggered solves.** The summary reports mean/max IPOPT iterations of 21.959/87, below the configured 100-iteration cap, and zero run-level fallback or solver-failure counts.
4. **MATLAB process memory did not exhibit mean growth over these 30 runs.** Mean memory changed from 6,361.1 MB before a run to 6,356.9 MB after a run; peak observed MATLAB memory was 6,367.6 MB.
5. **The proposed event-triggered scheduling shows relative desktop computational savings against continuous NMPC in the reported comparison/scalability experiments.** This is the defensible meaning of the reported 21.1--24.2% reduction.

### Conclusions that are not supported

- Hard real-time execution at the simulated 0.1 s sampling period.
- Real-time feasibility on an embedded flight computer or companion computer.
- Memory suitability for an embedded target.
- Equivalence between the profiling helper and the current production `solve_nmpc_casadi.m` without an explicit reconciliation.

## 3. Timing Distributions Available from the Step-Level CSV

The CSV does **not** contain saved percentile columns, but it contains all per-step timing values needed to compute them. The following were calculated from valid recorded rows using linearly interpolated empirical percentiles.

| Metric | N | Mean (ms) | Median (ms) | P90 (ms) | P95 (ms) | P99 (ms) | Max (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| All recorded controller steps | 4,072 | 2,077.9 | 2,551.9 | 2,889.2 | 2,964.7 | 3,150.4 | 3,982.8 |
| Triggered NMPC steps | 3,183 | 2,658.3 | 2,596.2 | 2,913.1 | 2,987.2 | 3,187.8 | 3,982.8 |
| Non-triggered steps | 889 | 0.015 | 0.007 | 0.009 | 0.011 | 0.071 | 1.884 |
| Triggered IPOPT iterations | 3,183 | 22.0 | 19 | 42 | 53 | 64 | 87 |

The manuscript's 2,074.1 ms value is the **mean of per-run mean CPU times** in the summary CSV, whereas the 2,077.9 ms row above is the pooled step-level mean. Both are valid but answer different aggregation questions and should be labelled accordingly.

## 4. Existing IPOPT and Solver Disclosure

The profiling script records the following profiling-specific configuration:

- IPOPT maximum iterations: 100.
- IPOPT tolerance: `1e-4`.
- IPOPT acceptable tolerance: `1e-3`.
- IPOPT print level: 0.
- MUMPS sparse linear solver.
- CasADi graph expansion: enabled.
- Warm start: shifted-reference initialization.

This information is largely present in the manuscript profiling table. However, two reproducibility issues must be disclosed or resolved before relying on it as evidence for the production solver:

1. `main_proposed_et_nmpc_profiling.m` contains a separate `solve_nmpc_casadi_profile()` implementation rather than calling the current `solve_nmpc_casadi.m` directly.
2. The active production `init_params.m` and `solve_nmpc_casadi.m` use `ipopt_acceptable_tol = 1e-4`, whereas the profiling helper/CSV/manuscript table use `1e-3`; the production solver also does not explicitly set `linear_solver` in its shown IPOPT settings block. Therefore, the profiling results are evidence for the profiling configuration, not automatically for the current production solver.

## 5. Memory-Usage Disclosure

### What is present

The profiling script calls MATLAB `memory()` and records `MemUsedMATLAB` before/after each run and at each simulation step. This supports a desktop MATLAB process-memory summary:

- Mean before-run memory: 6,361.1 MB.
- Mean after-run memory: 6,356.9 MB.
- Mean delta: -4.2 MB.
- Maximum observed MATLAB memory: 6,367.6 MB.

### What remains missing for the reviewer

- Peak memory attributable to one NMPC solve, IPOPT, CasADi, or the generated NLP - not merely the total MATLAB process.
- Embedded memory footprint, allocation behavior, peak resident set size, and deterministic memory-management evidence.
- Timing/memory separation for initialization, graph construction, prediction/risk assessment, solving, fallback, logging, and visualization.

The current wording "no continuous memory growth" is acceptable only as a limited observation of the MATLAB process over these profiling runs, not as proof of embedded memory feasibility.

## 6. Deadline-Violation Analysis

The configured sampling time is 0.1 s, i.e., a 100 ms nominal control deadline. The manuscript does not report a deadline-violation ratio, but it can be derived from the step CSV:

| Deadline basis | Violations | Denominator | Ratio |
|---|---:|---:|---:|
| All recorded controller steps with CPU time >100 ms | 3,183 | 4,072 | 78.168% |
| Triggered NMPC steps with CPU time >100 ms | 3,183 | 3,183 | 100.0% |

Thus, every recorded triggered NMPC computation exceeds the 100 ms simulated control period on the stated desktop MATLAB/CasADi setup. This directly contradicts any unqualified hard real-time or onboard-feasibility claim at that sampling rate.

Data-quality note: one step CSV row is a placeholder (`RunID=0`, `Step=0`, zero CPU time, missing status/NaN memory). It explains the difference between 889 non-triggered rows and 888 `not_triggered` statuses. The summary's run-level solver failure count remains zero, but the placeholder should be removed or explicitly filtered before publishing step-level aggregate statistics.

## 7. Reviewer Requirement Coverage Matrix

| Reviewer request | Current status | Evidence / gap |
|---|---|---|
| IPOPT iterations | Partially addressed | Mean, standard deviation, and maximum are in CSV/Table IV; percentiles are computable but unpublished. |
| Solver settings | Partially addressed | The profiling configuration is disclosed, but its divergence from the production solver is not. CasADi/IPOPT versions are `TODO`. |
| Memory usage | Partially addressed | MATLAB process-memory summary is available; solver-specific and embedded memory measurements are absent. |
| Timing distributions | Partially addressed | Mean, standard deviation, maximum, triggered/non-triggered mean are disclosed. Median/P90/P95/P99 and a distribution graphic are absent. |
| Deadline-violation ratio | Missing from manuscript | It is derivable and is 78.168% overall / 100.0% for triggered steps at 100 ms. |
| Embedded-platform evidence | Missing | No Jetson/flight-controller/compiled-code benchmark, HIL test, or onboard experiment exists. |
| Embedded-platform discussion | Present but insufficient for a feasibility claim | Lines 738 and 795 appropriately frame embedded validation as future work, but Abstract/Conclusion still overstate real-time feasibility. |

## 8. Recommended Manuscript Revisions

### Abstract

- Replace "real-time computational efficiency" with "reduced average computational burden in desktop MATLAB/CasADi simulations."
- Keep the 21.1--24.2% result as a relative desktop comparison.
- Do not claim real-time or onboard applicability; no embedded experiment supports it.

### Introduction

- Retain onboard-resource limitations as motivation, then explicitly state that this revision evaluates desktop profiling only.
- State that event-triggering reduces the number of local NLP solves; it does not by itself establish embedded real-time operation.
- Position compiled C++/code generation, HIL, and Jetson/flight-controller testing as future validation.

### Section IV

- Rename the profiling subsection to emphasize **desktop computational profiling**, rather than "Real-Time Feasibility Analysis."
- Add a timing-scope sentence: the timer begins immediately before controller computation and does not constitute full closed-loop latency unless prediction, risk assessment, logging, visualization, and communication are included and measured.
- Expand the profiling table with median, P90, P95, P99, maximum, and the 100 ms deadline-violation counts/ratios. Clearly distinguish pooled-step statistics from mean-of-run statistics.
- Add a reproducibility note listing MATLAB, CasADi, IPOPT, and MUMPS versions; CPU, RAM, OS; solver options; warm start; and whether graph construction is included in timing.
- Reconcile the profiling helper with `solve_nmpc_casadi.m`, or label all results as helper-specific and explain the acceptable-tolerance difference (`1e-3` versus `1e-4`).
- Qualify the memory result as MATLAB process memory and state that it is not an embedded memory footprint.
- Insert a concise desktop-versus-embedded limitation paragraph immediately after the profiling results: at $\Delta t=0.1$ s, triggered desktop MATLAB/CasADi solves miss the 100 ms deadline in all recorded triggered steps.

### Conclusion

- Replace "supported the real-time feasibility discussion" with "characterized desktop computational behavior and relative CPU savings."
- State explicitly that embedded feasibility remains unvalidated and is future work.
- Retain future code generation/HIL/Jetson testing, but do not present it as current evidence.

## 9. Recommended Additions

### Table

Add or expand the computational-profiling table with:

- Timing scope and hardware/software version manifest.
- All-step and triggered-step median, P90, P95, P99, and maximum latency.
- IPOPT iteration median, P95/P99, maximum, `Solve_Succeeded` count, fallback count, and data-cleaning rule for placeholder rows.
- Deadline (100 ms), violation count, and violation ratio.
- MATLAB process-memory label, before/after/peak values, and a clear statement that this is not solver-specific or embedded memory.

### Figure

Add one timing-distribution figure derived from `profiling_step_detail_results.csv`:

- Preferred: triggered-step empirical CDF or complementary CDF of controller time, with a vertical 100 ms deadline marker.
- Alternative: log-scale histogram/boxplot separating triggered and non-triggered steps, with median/P95 annotations.

This can be generated from existing CSV data; it does not require a new simulation campaign.

### Discussion

Add a short limitations paragraph covering:

- Desktop MATLAB/CasADi profiling versus compiled embedded runtime.
- The observed 100 ms deadline violation result.
- Expected deployment work: C/C++ code generation, solver pre-compilation, deterministic memory management, HIL, and target-hardware benchmarking.
- The distinction between reduced average solver workload and demonstrated real-time onboard feasibility.

## Bottom Line for the Response Letter

The revision can credibly state that it now discloses desktop profiling data, solver behavior, and MATLAB process memory, and that event triggering reduces average desktop computational workload. It should **not** state that the current data demonstrate real-time operation at 0.1 s or onboard applicability. The most defensible reviewer response is to add distribution/deadline transparency, reconcile the profiling and production solvers, qualify all claims, and present embedded validation as explicitly future work unless new target-platform experiments are run.
