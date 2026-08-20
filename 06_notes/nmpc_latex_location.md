# NMPC LaTeX Location Map

**Source file:** `01_manuscript_revision/Event_Triggered_NMPC_IoTJ_R1/my paper3.1.tex`  
**Scope:** location-only audit; the source file was not modified.

## 1. Section numbering

| Item | Manuscript location | Source lines |
| --- | --- | --- |
| NMPC formulation block | Section III, *Energy-Aware Event-Triggered NMPC Framework* | Section starts at 184 |
| Objective and cost definitions | Section III-C, *Local Predictive Optimization Formulation* | 273--321 |
| Online solver and Algorithm 1 | Section III-E, *NMPC-Based Online Solver* | 421--472 |
| Parameter table | Section IV, *Simulation Results*, immediately before Section IV-A | 479--522 |

## 2. Current NMPC objective function

- **Paper equation number:** Eq. (19)
- **LaTeX label:** `eq:total_cost`
- **Source location:** lines 275--280, Section III-C

```latex
When $\sigma_k=1$, the system constructs a finite-horizon local predictive optimization problem to generate a safe and dynamically feasible avoidance trajectory. The decision variables for this problem consist of the future position sequence $\Pi_k = \{\vect{p}_{k+1}, \vect{p}_{k+2}, \ldots, \vect{p}_{k+H}\}$ and the control-input sequence $U_k = \{\vect{u}_k, \vect{u}_{k+1}, \ldots, \vect{u}_{k+H-1}\}$, where $H$ denotes the prediction horizon.

To balance path efficiency, trajectory smoothness, energy consumption, and collision avoidance, the overall objective function $J$ is defined as
\begin{equation}
	J = \alpha J_{path} + \beta J_{smooth} + \gamma J_{energy} + \delta J_{risk},
	\label{eq:total_cost}
\end{equation}
where $J_{path}, J_{smooth}, J_{energy}$, and $J_{risk}$ represent the specific cost components. The weighting coefficients $\alpha, \beta, \gamma, \delta \ge 0$ balance these objectives and may satisfy $\alpha+\beta+\gamma+\delta=1$.

The path-length cost $J_{path}$ is defined as
\begin{equation}
	J_{path} = \sum_{\tau=0}^{H-1} \|\vect{p}_{k+\tau+1} - \vect{p}_{k+\tau}\|,
	\label{eq:jpath}
\end{equation}
```

## 3. Current cost function definitions

| Cost term | Paper equation | LaTeX label | Source lines |
| --- | --- | --- | --- |
| Path-length cost | Eq. (20) | `eq:jpath` | 282--286 |
| Smoothness cost | Eq. (21) | `eq:jsmooth` | 287--292 |
| Energy cost | Eq. (22) | `eq:jenergy` | 294--304 |
| Estimated energy metric | Eq. (23) | `eq:estimated_energy` | 306--311 |
| Obstacle risk cost | Eq. (24) | `eq:jrisk` | 313--319 |

```latex
which penalizes unnecessary detours to minimize the flight distance. The smoothness cost $J_{smooth}$ is defined as
\begin{equation}
	J_{smooth} = \sum_{\tau=0}^{H-2} \|\vect{v}_{k+\tau+1} - \vect{v}_{k+\tau}\|^2,
	\label{eq:jsmooth}
\end{equation}
which suppresses abrupt changes in velocity to improve trajectory executability.

To explicitly reflect the asymmetric energy characteristics of 3D flight, the local energy cost $J_{energy}$ is defined as
\begin{equation}
	\small
	\begin{aligned}
		J_{energy} 
		&= c_1 \sum_{\tau=0}^{H-1} \| \vect{p}_{k+\tau+1} - \vect{p}_{k+\tau} \| + c_2 \sum_{\tau=0}^{H-1} \| \vect{u}_{k+\tau} \|^2 \\
		&\quad + c_3 \sum_{\tau=0}^{H-1} \max(0, z_{k+\tau+1} - z_{k+\tau}),
	\end{aligned}
	\label{eq:jenergy}
\end{equation}
```

## 4. Current energy model formula locations

- **Online NMPC energy cost:** Eq. (22), `eq:jenergy`, lines 294--304, Section III-C.
- **Post-run estimated-energy model:** Eq. (23), `eq:estimated_energy`, lines 306--311, Section III-C.

```latex
where $c_1$, $c_2$, and $c_3$ are non-negative weighting coefficients. The first term approximates distance-related propulsion energy, the second accounts for energy from control actions, and the third implements an asymmetric climb penalty penalizing unnecessary positive altitude gains. $J_{energy}$ serves as a normalized cost within the NMPC optimization. It should be noted that $J_{energy}$ is used as a lightweight surrogate energy cost for online NMPC rather than a full rotor-level power model.

To provide a quantitative energy assessment beyond geometric metrics, the estimated energy consumption is computed using a simplified physical model
\begin{equation}
	E_{est}=P_{base}T+\frac{m g \Delta z_+}{\eta_c}+k_u\int_0^T \|u(t)\|^2 dt,
	\label{eq:estimated_energy}
\end{equation}
where $P_{base}$ is the baseline propulsion power, $T$ is the flight duration, $m$ is the UAV mass, $g$ is the gravitational acceleration, $\Delta z_+$ denotes the accumulated positive altitude change, $\eta_c$ is the climb efficiency, and $k_u$ is the maneuvering-energy coefficient.
```

## 5. Current obstacle-avoidance risk cost formula location

- **Paper equation number:** Eq. (24)
- **LaTeX label:** `eq:jrisk`
- **Source location:** lines 313--321, Section III-C

```latex
The risk cost $J_{risk}$ is defined as
\begin{equation}
	\small
	J_{risk} = \sum_{\tau=1}^{H} \sum_{i=1}^{N_o} \frac{1}{\|\vect{p}_{k+\tau} - \hat{\vect{\xi}}_i(k+\tau|k)\| - \rho_i(k+\tau) + \varepsilon},
	\label{eq:jrisk}
\end{equation}
which penalizes trajectories that approach the inflated occupancy boundaries $\rho_i(k+\tau)$.

This formulation enables the UAV to track a reference trajectory $\vect{r}_k$ while autonomously avoiding dynamic threats. Unlike conventional continuous schemes, we employ an on-demand strategy where the predictive optimizer is activated only when the instantaneous collision risk exceeds a prescribed threshold.

The resulting local predictive optimization problem is formulated as
\begin{subequations}
	\label{eq:opt_problem}
```

## 6. Table II parameter table location

- **Table number:** Table II, *Key Parameters Used in the Simulations*
- **LaTeX label:** `tab:key_parameters`
- **Source location:** lines 479--522, Section IV before Section IV-A.

```latex
\begin{table}[t]
	\centering
	\caption{Key Parameters Used in the Simulations}
	\label{tab:key_parameters}
	\scriptsize
	\renewcommand{\arraystretch}{1.12}
	\setlength{\tabcolsep}{2.5pt}
	\resizebox{\columnwidth}{!}{%
		\begin{tabular}{l l c c}
			\toprule
			\textbf{Symbol} & \textbf{Description} & \textbf{Unit} & \textbf{Value} \\
			\midrule
			\multicolumn{4}{l}{\textit{UAV kinematic parameters}} \\
			\midrule
			$v_{\max}$ & Maximum UAV speed & m/s & 5.0 \\
			$u_{\max}$ & Maximum UAV acceleration & m/s$^2$ & 3.0 \\
			$z_{\min}$ & Minimum flight altitude & m & 0.5 \\
			$z_{\max}$ & Maximum flight altitude & m & 20.0 \\
			$r_u$ & UAV safety radius & m & 0.3 \\
			$\Delta t$ & Sampling time & s & 0.1 \\
			$H$ & Prediction horizon & steps & 15 \\
			\midrule
			\multicolumn{4}{l}{\textit{NMPC cost weights}} \\
			\midrule
			$\alpha$ & Path-length cost weight & -- & 0.14 \\
			$\beta$ & Trajectory smoothness cost weight & -- & 0.07 \\
			$\gamma$ & Energy cost weight & -- & 0.11 \\
```

## 7. Algorithm 1 location

- **Algorithm number:** Algorithm 1
- **Caption:** *Event-Triggered Dual-Mode NMPC for Dynamic Obstacle Avoidance*
- **LaTeX label:** `alg:etpp`
- **Source location:** lines 437--472, Section III-E.

```latex
\begin{algorithm}[!h]
	\caption{Event-Triggered Dual-Mode NMPC for Dynamic Obstacle Avoidance}
	\label{alg:etpp}
	\begin{algorithmic}[1]
		\REQUIRE $\vect{x}_0, \vect{p}_g, \mathcal{R}, \mathcal{O}_s, H, R_{on}, R_{off}, T_{ref}, \epsilon_{tol}, k_{\max}$
		\ENSURE Control sequence $\{\vect{u}_k\}$, state trajectory $\{\vect{x}_k\}$
		
		\STATE Initialize $k \leftarrow 0$, $k_{last} \leftarrow -T_{ref}$, $\sigma_{-1} \leftarrow 0$
		
		\WHILE{$\|\vect{p}_k - \vect{p}_g\| > \epsilon_{tol} \wedge k < k_{\max}$}
		
		\STATE Acquire state $\vect{x}_k$, reference $\vect{r}_k \in \mathcal{R}$, and observations $\vect{y}_i(k)$
		\STATE Predict obstacle motion $\hat{\vect{\xi}}_i$ and construct occupancy regions $\mathcal{O}_i(k+\tau)$
		\STATE Compute risk components and composite risk $R_k = w_1\tilde{R}_k + w_2\hat{R}_k + w_3\bar{R}_k$
		
		\IF{$(R_k \ge R_{on}) \wedge (k - k_{last} \ge T_{ref})$}
		\STATE $\sigma_k \leftarrow 1$
		\ELSIF{$R_k \le R_{off}$}
		\STATE $\sigma_k \leftarrow 0$
		\ELSE
		\STATE $\sigma_k \leftarrow \sigma_{k-1}$
		\ENDIF
```

```latex
		\IF{$\sigma_k == 1$}
		\STATE Formulate and solve the NMPC problem over horizon $H$
		\STATE Apply optimal input $\vect{u}_k \leftarrow \vect{u}_{k|k}^\ast$
		\STATE $k_{last} \leftarrow k$
		\ELSE
		\STATE Apply nominal tracking control $\vect{u}_k \leftarrow \mathcal{K}(\vect{x}_k, \vect{r}_k)$
		\ENDIF
		
		\STATE Propagate the UAV dynamics to obtain $\vect{x}_{k+1}$
		\STATE $k \leftarrow k+1$
		\ENDWHILE
	\end{algorithmic}
\end{algorithm}
```
