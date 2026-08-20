# Manuscript Baseline Audit

## Scope

Static review of `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`. No TeX, figure, result, Git state, commit, or remote was changed. Reviewer mapping AE-1 and R1-2--R1-5 was used for the final matrix.

## Abstract

### Current main contributions

The abstract identifies three contributions: risk-triggered dual-mode control; Kalman-based prediction of heterogeneous dynamic obstacles; and an asymmetric climb penalty. It claims 100% obstacle avoidance in the reported noise/density tests and 21.1%--24.2% CPU-time reduction relative to C-NMPC.

### Real-time description

“Real-time computational efficiency” is too strong without qualification. Table IV reports 2074.1 ms average CPU time per step while the simulation sampling time is 0.1 s. This desktop MATLAB/CasADi prototype does not demonstrate hard real-time execution at its simulated rate. Use qualified wording such as “reduced average computational burden on a desktop prototype.”

### IoT description

IoT is absent from the abstract, keywords, system model, architecture, and experiments as a concrete system component. The paper presently reads as a UAV control/planning paper that cites IoT literature, rather than an IoT-system paper.

### Issues

- Limit the 100% claim to the tested map, 1/3/5 obstacle counts, selected motion/noise models, and Monte Carlo settings.
- Name the tested density range in the abstract.
- Distinguish the online surrogate energy cost from the post-run estimated Wh metric.

## Introduction

### Current IoT positioning

IoT appears only in the energy-efficiency paragraph through UAV communication and IoT-network citations. The actual formulation contains UAV state, obstacle observations, prediction, and local optimization, but no IoT sensing, connectivity, edge/cloud coordination, communication constraint, data source, or network metric.

### Insufficiencies

- The motivation is dynamic obstacle avoidance/onboard computation, not an IoT-specific technical problem.
- Observation acquisition, communication delay/loss, and reliability are not defined.
- Communication/resource-allocation citations are not tied to a model, constraint, or metric.
- IoT is absent from both contributions and keywords, weakening IoTJ fit.

### Recommended revision direction

Use one consistent and defensible position. Either define the UAV as an IoT-enabled edge node, explaining how event-triggered scheduling protects local compute/energy resources, or add a true networked sensing/control model with latency/loss/bandwidth/offload experiments. The first option is feasible without new experiments if all network-performance claims are avoided. Add a concise system-level paragraph and figure annotation for sensors, onboard predictor/risk estimator/NMPC, and control loop; explicitly state that cloud offloading and communication latency are out of scope if unmodeled.

## Section III

### NMPC cost formulation

The subsection is logically ordered but not aligned with the executable solver at equation level. Eqs. (19)--(24) show path length, velocity-difference smoothness, norm-distance energy, reciprocal dynamic risk, and hard safety constraints. The code uses squared reference tracking, consecutive-control variation, squared displacement energy, polynomial soft penalties for dynamic/static obstacles, and undocumented terminal position/velocity/progress terms with near-goal risk decay. Table II weights (`0.14, 0.07, 0.11, 0.68`) also differ from the code baseline (`1.0, 0.5, 0.8, 5.0`). Document the executable formulation and full parameter set, or revise the implementation to match the equations. Do not claim hard constraints if the final solver uses soft penalties.

### Event-trigger mechanism

Risk decomposition, hysteresis thresholds, and refractory condition are clear. The inter-event theorem needs correction: the paper defines `sigma_k=1` as activation and resets `k_last` whenever `sigma_k=1`; implementation also solves NMPC and resets `k_last` on every high-risk step. A sustained high-risk period may have adjacent `sigma_k=1` steps, so it is false that every two consecutive solver-active steps are separated by `T_ref`. The valid claim is an interval between **off-to-on transitions/re-activations**, provided `k_last` updates only on that transition. Otherwise, describe `T_ref` as re-entry protection after deactivation. Eq. (18), the theorem, Algorithm 1, and implementation need one shared definition.

### Energy model

The manuscript correctly distinguishes online surrogate cost from post-run Eq. (23) estimation, but Eq. (22)'s distance norm conflicts with the squared-distance code term. Eq. (23) physical parameters are missing from the parameter table. Further, current Table III and Table V scripts use different physical-energy default sets, so absolute Wh values are not comparable until unified. The theoretical text only discusses recursive feasibility/practical stability under assumptions; present it as conditional discussion, not a proven closed-loop guarantee for the actual soft-constraint moving-obstacle solver.

## Section IV

### Experimental setting

Strengths: MATLAB/CasADi, desktop hardware, 0.1-s sampling, 15-step horizon, heterogeneous obstacle motions, measurement noise, and trial counts are disclosed. Figures 2--4 demonstrate proposed trajectory and trigger signal.

Required reproducibility details:

- Specify random-seed policy, obstacle initial-state ranges, building map, motion/noise parameters, and stopping/success/collision criteria.
- State whether metrics average all trials or successful trials only.
- Report physical estimated-energy parameters and the exact code/configuration revision.
- Make Table II match executed NMPC weights and include terminal/risk parameters.
- Restrict “robust” to the stated simulated uncertainty envelope.

### Baselines

The C-NMPC, NP-ET-NMPC, SE-ET-NMPC, 3D-APF, and RRT* expansion is appropriate. The claim of identical maps, initialization, and seeds should be retained only when all algorithms consume identical precomputed obstacle truth trajectories; otherwise clarify the fairness protocol. Table III should disclose update frequency, prediction availability, energy setting, tuning/fairness policy, and why RRT* has no continuous metrics after failure.

### Timing analysis

The profiling disclosure is useful and candidly labels a desktop prototype. Its title and conclusion should not characterize a 2.07-s mean step time under a 0.1-s sample time as “real-time feasibility.” It supports relative saving and solver-behavior characterization, not real-time onboard feasibility. The profiling script duplicates the NMPC solver and uses IPOPT acceptable tolerance `10^-3`, while the primary solver uses `10^-4`; verify equivalence or state it as profiling-specific. Also disclose whether prediction, visualization, initialization, and fallback are included in CPU timing.

## Conclusion

### Does the current conclusion exceed the evidence?

Partly. It is supported for the reported **desktop simulations**: event triggering reduces average solver use/CPU relative to C-NMPC in tested scenarios, with reported success and clearance outcomes. It exceeds evidence where it implies real-time/embedded deployment, general collision-free behavior, rigorous stability/recursive feasibility, an IoT contribution beyond citations, or universally lower energy rather than a specified surrogate/estimate in tested cases. The future-work statements on code generation, HIL, and Jetson validation are appropriately framed as unvalidated future work.

## Reviewer Comment Mapping

| Reviewer comment | Section location | Required revision |
| --- | --- | --- |
| AE-1: weak IoT connection | Abstract; Introduction; architecture; Conclusion | State a concrete IoT role, align terminology/figure/citations, and add network variables only when supported by experiments. |
| R1-2: real-time claim | Abstract; Section IV timing/Table IV; Conclusion | Replace unqualified real-time wording; reconcile 2074.1 ms/step with `dt=0.1 s`; retain embedded validation as future work. |
| R1-3: NMPC goal/cost | Section III, Eqs. (19)--(24), Table II, Algorithm 1 | Make equations, constraint type, terminal terms, and weights equal to final solver; correct the trigger theorem. |
| R1-4: energy penalty | Section III energy; Table II; Section IV Tables III/V; Conclusion | Align Eq. (22) with code, disclose/unify Eq. (23) parameters, and describe SE-ET-NMPC as controlled ablation. |
| R1-5: baselines | Section IV baseline subsection, Table III, Fig. 5 | Document common-scenario fairness, baseline parameterization, failure-metric policy, and limits of 3D-APF/RRT* comparisons. |
