# Reviewer #1 Major Comment 1: IoT-Relevance Revision Analysis

## Reviewer Comment

> The connection to the Internet of Things should be strengthened.

## Review Basis and Revision Principle

IEEE *Internet of Things Journal* covers IoT architectures, enabling technologies, resource-constrained systems, in-situ processing, embedded software, and IoT applications. The relevant fit for this manuscript is therefore an **IoT-enabled cyber-physical perception--decision--actuation system**, not a claim that the proposed NMPC is a new communication, networking, or offloading method. See the official [IEEE IoT-J scope](https://ieee-iotj.org/).

The revision should position the UAV as a mobile sensing-and-actuation node that turns locally available obstacle observations into safety-critical control decisions. The event-triggered contribution should be stated precisely as a reduction in unnecessary NMPC activations and the associated controller-computation workload. It must **not** be described as a measured reduction in radio transmissions, bandwidth usage, communication energy, end-to-end network delay, or cloud/edge offloading, because none of these quantities is modeled or experimentally evaluated.

No change to the ET-NMPC objective, risk trigger, solver, obstacle model, or experiment is required. All visible manuscript edits should use `\textcolor{blue}{...}` in accordance with the current revision convention.

## Current Gap Assessment

| Manuscript part | Current state | Gap relative to Comment 1 |
|---|---|---|
| Abstract | Lines 68--70 describe a UAV control framework and desktop profiling; IoT does not appear. | The application-level IoT relevance and the UAV's functional role are absent. |
| Introduction | Lines 78--87 discuss UAV control and use IoT only to contrast communication-centric energy studies. | The manuscript does not define the proposed system as an IoT/CPS perception--control loop, nor distinguish computation savings from communication savings. |
| System Description | Lines 141--182 define workspace, kinematics, and the observation variable `\vect{y}_i(k)`. | The source of observations and the system boundary are unstated; the reader may incorrectly infer a networking or embedded-deployment contribution. |
| Conclusion | Lines 801--805 summarize control, energy, and desktop computational results. | It does not state the IoT-system contribution or clearly delimit the unmodeled networking layer. |

## Recommended LaTeX Revisions

### 1. Abstract

**Location:** `my paper3.1.tex`, line 69, after the first sentence and before “To balance ...”.

**Purpose:** Establish an IoT/CPS role without claiming an implemented IoT testbed or communication optimization.

**Suggested insertion:**

```latex
\textcolor{blue}{The considered scenario is motivated by an IoT-enabled cyber--physical system, in which the UAV acts as a mobile sensing-and-actuation node that uses locally available obstacle observations to make safety-critical motion decisions.}
```

**Keyword update (line 73):** append `Internet of Things` or `IoT-enabled cyber--physical systems` to the existing keyword list. This is a positioning change only; it does not introduce a new technical claim.

### 2. Introduction

**Location A:** after line 78, before the paragraph beginning “Accurate environmental representation ...”.

**Purpose:** Explain why a risk-aware local control loop is relevant to IoT-J.

**Suggested insertion:**

```latex
\textcolor{blue}{In IoT-enabled cyber--physical applications, a UAV can serve as a mobile sensing-and-actuation node: it receives or acquires environmental observations, performs local situation assessment, and executes safety-critical motion commands. For such resource-constrained mobile nodes, avoiding unnecessary optimization activations is relevant because it reduces the local decision workload while preserving the responsiveness required for dynamic environmental interaction.}
```

**Location B:** revise the transition after the current IoT-energy paragraph at lines 85--87, immediately before “In complex 3D navigation ...”.

**Purpose:** Make the manuscript's IoT distinction explicit. The existing paragraph cites communication-oriented IoT work, but it currently leaves the present paper's own IoT contribution implicit.

**Suggested insertion:**

```latex
\textcolor{blue}{Different from communication-centric UAV-enabled IoT studies, this work addresses the in-situ perception--control function of a mobile IoT node in a dynamic physical environment. The contribution is a risk-aware scheduling mechanism for local predictive control; communication scheduling, radio-resource allocation, and communication-energy optimization are outside the scope of the present model and experiments.}
```

**Location C:** after line 89, before “The main contributions ...”.

**Purpose:** Prevent the computational claim from being misread as a network-resource result and align it with the revised deadline-oriented disclosure.

**Suggested insertion:**

```latex
\textcolor{blue}{From an IoT-system perspective, the reported benefit is resource-aware local decision scheduling: the trigger reduces unnecessary NMPC activations and the associated desktop controller-computation workload. It is not a measured claim about wireless traffic, communication latency, bandwidth consumption, or communication energy.}
```

### 3. System Description

**Location A:** Section II, immediately after the opening paragraph at line 141 and before `\subsection{Workspace and UAV Kinematics}`.

**Purpose:** Supply the missing system-level context while retaining the existing kinematic and observation models unchanged.

**Suggested insertion:**

```latex
\textcolor{blue}{We consider a functional IoT-enabled cyber--physical setting in which the UAV is a mobile sensing-and-actuation node. At each sampling instant, obstacle observations are made available to the local perception--control loop, which estimates obstacle motion, assesses collision risk, and selects a nominal-tracking or predictive-replanning action. This paper models that local decision loop and its motion consequences; it does not model a specific wireless protocol, packet delivery process, communication delay, or computation offloading architecture.}
```

**Location B:** Section II-B, after the definition of `\vect{y}_i(k)` at lines 175--180.

**Purpose:** Clarify the abstraction behind the observation input without fabricating a sensing-network implementation.

**Suggested insertion:**

```latex
\textcolor{blue}{The measurement `\vect{y}_i(k)` abstracts the obstacle observation made available by the local perception interface, which may be obtained from onboard sensing or an IoT sensing interface. Its communication mechanism is not specified here; the present study assumes that the observation is available to the controller at each sampling instant.}
```

**Optional figure-caption clarification:** Amend the Fig. 1 caption at line 146 only if the figure can truthfully support the wording. Add the following final sentence; do not draw network links, gateways, or cloud nodes that are not modeled:

```latex
\textcolor{blue}{The illustration represents a local perception--decision--actuation loop; it does not model a communication-network topology.}
```

### 4. Conclusion

**Location A:** after the first paragraph at line 801, before “Simulation results verified ...”.

**Purpose:** State the resulting IoT relevance at the appropriate, evidence-supported level.

**Suggested insertion:**

```latex
\textcolor{blue}{From the IoT-system perspective, the framework provides a resource-aware local perception--control component for a mobile sensing-and-actuation node operating in a dynamic cyber--physical environment. Its demonstrated resource benefit is the reduction of unnecessary NMPC activations and relative desktop controller-computation time.}
```

**Location B:** append to the computational-boundary statement in the blue sentence at line 803, after “embedded feasibility.”

**Purpose:** Avoid an unsupported implication that IoT relevance establishes communication performance.

**Suggested insertion:**

```latex
\textcolor{blue}{The study does not quantify communication latency, packet loss, radio-resource usage, or communication energy; these networking effects are outside the present simulation scope.}
```

**Location C:** line 805, before the existing future-work sentence or as its final sentence.

**Suggested insertion:**

```latex
\textcolor{blue}{Future work may evaluate the same local perception--control framework under explicitly modeled sensing-network delays and resource constraints, together with embedded and hardware-in-the-loop validation.}
```

## Recommended Response Logic for Reviewer #1 Comment 1

The eventual response should acknowledge that the original manuscript used IoT mainly as background and did not explicitly explain the UAV's role in an IoT/CPS architecture. It should then state that the revision clarifies the UAV as a mobile sensing-and-actuation node, the ET mechanism as local computational-resource scheduling, and the boundary excluding communication optimization. It should not claim new networking experiments, wireless savings, edge deployment, or end-to-end latency validation.

## Claims to Keep and Claims to Avoid

| Keep (supported by current manuscript) | Avoid (not supported by current manuscript) |
|---|---|
| Mobile sensing-and-actuation role in an IoT-enabled CPS | Implemented IoT network/testbed or deployment |
| Local perception--decision--actuation abstraction | Measured wireless bandwidth or transmission reduction |
| Fewer NMPC activations and desktop CPU-time savings | Reduced communication energy or radio-resource allocation |
| Scope boundary: no communication protocol/delay model | End-to-end IoT latency, packet-loss resilience, or cloud/edge offloading claims |

## Minimal Revision Set

The minimum credible revision is: one Abstract sentence plus keyword; two Introduction paragraphs that establish the IoT/CPS role and scope boundary; one Section II system-context paragraph plus one observation-interface clarification; and two Conclusion sentences. This gives the paper an explicit IoT systems narrative without changing the control formulation or requiring additional experiments.
