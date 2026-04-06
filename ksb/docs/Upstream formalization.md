# Upstream Feedforward Control — Formalization

## 1. Motivation

Gap compression after a skip occurs because the post-skip object must decelerate aggressively inside the buffer while the object behind it is still traveling at $v_u$. The feedforward controller addresses this by accelerating the upstream belt before each skip, building a velocity surplus that is spent in a controlled deceleration immediately after the skip. The net effect is that the post-skip object arrives at the buffer entry earlier — requiring a less aggressive buffer correction — while every object's entry state remains nominal.

The controller is **event-triggered and open-loop**: it does not react to a measured error but computes the correction analytically from the known skip event. Because skip timing is detected sequentially — input $i$ is known to have skipped before input $i+1$ spawns — the correction for $i+1$ is fully determined before $i+1$'s upstream transit begins.

---

## 2. The Jerk Timeline

### 2.1 Kinematic Constraints

The upstream belt operates under the following kinematic bounds, which constrain how 
segments are appended to $\mathcal{T}$:

- A single jerk magnitude $j_{\max}$ is used for all ramps in both $\Pi_k^{\text{acc}}$ and $\Pi_k^{\text{dec}}$.
- The acceleration is bounded by $a_{\max}^{\text{acc}}$ during $\Pi_k^{\text{acc}}$ and by $-a_{\max}$ during $\Pi_k^{\text{dec}}$, where $a_{\max}$ is inherited from the system formalization and applies symmetrically.
- The belt velocity must not exceed the upstream speed ceiling at any time:

$$v^{\text{up}}(t) \leq v_{\max}^{\text{up}}$$

$v_{\max}^{\text{up}}$ is a tunable control parameter, bounded above by the physical belt speed limit. It determines $A_{\max}$ via:

$$A_{\max} = v_{\max}^{\text{up}} - v_u$$

Note that $v_{\max}$ in the buffer analysis is a distinct quantity — the maximum speed permissible for objects transiting the buffer, set by physical constraints and not tunable.

### 2.2 Definition

The upstream controller maintains a single global signal $j^{\text{up}}(t)$, the **jerk timeline** $\mathcal{T}$. From $j^{\text{up}}(t)$ all kinematic quantities are derived analytically:

$$a^{\text{up}}(t) = a_0 + \int_0^t j^{\text{up}}(\tau) \, d\tau$$

$$v^{\text{up}}(t) = v_u + \int_0^t a^{\text{up}}(\tau) \, d\tau$$

$$p^{\text{up}}(t) = \int_0^t v^{\text{up}}(\tau) \, d\tau$$

$\mathcal{T}$ is represented as an ordered, **append-only** list of constant-jerk segments:

$$\mathcal{T} = \bigl[(t_0, j^{(0)}, T^{(0)}), \; (t_1, j^{(1)}, T^{(1)}), \; \ldots \bigr]$$

where $t_n$ is the absolute start time of segment $n$, $j^{(n)}$ is the constant jerk value, and $T^{(n)}$ is its duration. The kinematic state at the start of each segment follows directly from the end state of the previous one. Segments are never modified or removed — only appended. This makes the full history available for analysis.

![[Pasted image 20260404161625.png]]

### 2.3 Initial Condition

At $t = 0$ the belt is at velocity $v_u$ with zero acceleration and zero jerk. The acceleration profile for $\Pi_1^{\text{acc}}$ is appended to $\mathcal{T}$ immediately — the belt begins building velocity surplus before the first skip is detected. $\Pi_1$ follows the same structure as every subsequent phase: acceleration segments are appended up to $s_1$, at which point $\Pi_1^{\text{dec}}$ is computed and appended. The first input can never trigger a skip (no slot has been filled before it), so $\Pi_1^{\text{acc}}$ always spans at least one full input transit.

### 2.4 Skip Times

A skip event $s_k$ is defined by the input index $i$ at which $\delta_i = 1$ is detected. The **skip time** $s_k$ is the buffer entry time of that input:

$$s_k = t_i^{\text{in}} \quad \text{where} \quad \delta_i = 1 \text{ for the } k\text{-th time}$$

### 2.5 Inter-Skip Phases

The **inter-skip phases** are the intervals between consecutive recovery and skip times. They are collected in an ordered list:

$$\Pi = \bigl[\Pi_1, \Pi_2, \Pi_3, \ldots \bigr]$$
$\Pi_k$​ is a contiguous subsequence of segments in $\mathcal{T}$, identified by a start and end index into the segment list: 

$$\Pi_k = \mathcal{T}[n_k^{\text{start}} \,\ldots\, n_k^{\text{end}}]$$

It contains all segments belonging to the $k$-th acceleration cycle: the acceleration ramp, the plateau, and the post-skip deceleration. $\Pi_1$​ is the degenerate case — a single constant-velocity segment with zero jerk.

> *Note: statistical properties of $\Pi_k$ — mean duration, expected number of inputs $N_k$, area integral — are discussed in Section [TBD].*

### 2.6 Individual Input Trajectories

The upstream trajectory of input $i$ is captured by its **upstream window** $\mathcal{W}_i$ — 
the projection of $\mathcal{T}$ onto the time interval $[t_i^{\text{spawn}}, t_i^{\text{in}}]$:

$$\mathcal{W}_i = \mathcal{T}\big|_{[t_i^{\text{spawn}},\, t_i^{\text{in}}]}$$

$t_i^{\text{in}}$ is the first time input $i$ has covered distance $L_{\text{up}}$ under $\mathcal{T}$:

$$t_i^{\text{in}} = \min \left\lbrace t > t_i^{\text{spawn}} \mid 
\int_{t_i^{\text{spawn}}}^{t} v^{\text{up}}(\tau) \, d\tau = L_{\text{up}} \right\rbrace$$

$\mathcal{W}_i$ is orthogonal to the phase decomposition $\Pi$: its boundaries are defined. In general, $t_i^{\text{spawn}}$ does not coincide with the start time of any segment in $\mathcal{T}$ — it falls in the interior of some segment $n$, i.e. $t_n < t_i^{\text{spawn}} < t_n + T^{(n)}$. Consequently $\mathcal{W}_i$ begins mid-segment and its boundaries do not align with those of any $\Pi_k$.

![[Pasted image 20260404161839.png]]
### 2.7 Timeline Extension

$\mathcal{T}$ is only populated up to $t_i^{\text{in}}$ after processing input $i$. Before input $i+1$ can be queried, $\mathcal{T}$ must be extended to cover at least $t_{i+1}^{\text{in}}$. The extension rule depends on whether input $i$ triggered a skip:

- **No skip** ($\delta_i = 0$): the acceleration phase continues. Segments are appended to continue ramping toward $a_{\max}$, or to hold at the plateau if $a_{\max}$ has already been reached, until the query for $i+1$ is satisfied.
- **Skip** ($\delta_i = 1$): the deceleration profile for skip $s_k$ is appended immediately at $s_k$, followed at $t_k^{\text{rec}}$ by the start of the acceleration profile for $\Pi_{k+1}$.

---

## 3. Inter-Skip Phases

### 3.1 Structure

Each phase $\Pi_k$ is a contiguous subsequence of segments in $\mathcal{T}$ forming a complete jerk cycle. $\Pi_1$ is the degenerate case — a single zero-jerk segment corresponding to the initial constant-velocity regime. For $k \geq 2$, the segments of $\Pi_k$ are:

$$[+j_{\max}, ; 0, ; -j_{\max}, ; 0^*, ; -j_{\max}, ; 0, ; +j_{\max}]$$

where the first four segments are the acceleration $\Pi_k^{\text{acc}}$ (ramp up, hold at $a_{\max}^{\text{acc}}$, ramp down, zero-jerk plateau) and the last three are the mirrored deceleration $\Pi_k^{\text{dec}}$ (ramp down, hold at $-a_{\max}$, ramp up). The hold segments collapse when the accumulated velocity surplus is small enough that $a_{\max}^{\text{acc}}$ or $a_{\max}$ is never saturated, reducing the sequence accordingly.

The segments of $\Pi_k^{\text{acc}}$ are appended incrementally as inputs are processed. The duration of the plateau $0^*$ is not known in advance — it ends when $\delta_i = 1$ is detected. The segments of $\Pi_k^{\text{dec}}$ are appended in full at that moment.

### 3.2 Quantities at Skip Time

When a skip is detected at $s_k$, two quantities characterise the state accumulated during $\Pi_k^{\text{acc}}$.

**Acceleration at skip time.** The net change in acceleration over the acceleration phase:

$$a_k = \sum_{n \in \Pi_k^{\text{acc}}} j^{(n)} T^{(n)}$$

This has units m/s$^2$ and equals the instantaneous acceleration of the belt at $t = s_k$. If the ramp saturated and the belt has been holding at $a_{\max}^{\text{acc}}$, then $a_k = a_{\max}^{\text{acc}}$. If the skip was detected during the ramp, $a_k < a_{\max}^{\text{acc}}$.

**Velocity surplus.** The integral of acceleration over the acceleration phase — equivalently, the area under the piecewise-linear acceleration curve:

$$\Delta v_k = \sum_{n \in \Pi_k^{\text{acc}}} \left[ a_n^{\text{start}}, T^{(n)} + \tfrac{1}{2}, j^{(n)} \bigl(T^{(n)}\bigr)^2 \right]$$

This has units m/s and equals $v^{\text{up}}(s_k) - v_u$, the velocity surplus that the deceleration must remove. Each term in the sum is the area of one segment: a rectangle $a_n^{\text{start}} \cdot T^{(n)}$ plus a triangle $\tfrac{1}{2}, j^{(n)} (T^{(n)})^2$, where $a_n^{\text{start}}$ is the acceleration at the start of segment $n$ (the end state of the previous segment, or zero for the first segment of $\Pi_k^{\text{acc}}$).

### 3.3 Phase Invariants

The segments of $\Pi_k$ form a complete acceleration cycle: the belt starts and ends each phase at zero acceleration and at velocity $v_u$. This imposes two invariants on the stored tuples.

**Acceleration invariant.** The net jerk sum over all segments of $\Pi_k$ is zero:

$$\sum_{n \in \Pi_k} j^{(n)} T^{(n)} = 0 \qquad \forall, k \geq 1$$

This guarantees that $a^{\text{up}}$ returns to zero at the end of the phase.

**Velocity invariant.** The net velocity change over all segments of $\Pi_k$ is zero:

$$\sum_{n \in \Pi_k} \left[ a_n^{\text{start}}, T^{(n)} + \tfrac{1}{2}, j^{(n)} \bigl(T^{(n)}\bigr)^2 \right] = 0 \qquad \forall, k \geq 1$$

This guarantees that $v^{\text{up}}$ returns to $v_u$ at the end of the phase. The acceleration invariant constrains the jerk ramp durations of $\Pi_k^{\text{dec}}$; the velocity invariant constrains its hold duration at $-a_{\max}$. Together they fully determine the deceleration profile given $a_k$ and $\Delta v_k$.

### 3.4 Velocity Surplus Bound

The velocity surplus must not exceed the maximum permissible value:

$$\Delta v_k \leq \Delta v_{\max} = v_{\max}^{\text{up}} - v_u$$

where $v_{\max}^{\text{up}}$ is a tunable upstream speed ceiling, bounded above by the physical belt speed limit. Increasing $\Delta v_{\max}$ allows more effective skip compensation at the cost of a longer deceleration window and a stricter feasibility condition on the inter-arrival gap (Section 5.2). Note that $v_{\max}$ in the buffer analysis is a distinct quantity — the maximum speed permissible for objects transiting the buffer, set by physical constraints and not tunable.

---

## 4. Skip Response

### 4.1 Trigger

When $\delta_i = 1$ is detected at $s_k = t_i^{\text{in}}$, the plateau $0^*$ is closed. The quantities $a_k$ and $\Delta v_k$ are computed from the segments of $\Pi_k^{\text{acc}}$. The segments of $\Pi_k^{\text{dec}}$ are immediately appended to $\mathcal{T}$, completing $\Pi_k$.

### 4.2 Deceleration Profile

The deceleration segments mirror the acceleration: they ramp acceleration negatively using $-j_{\max}$, hold at $-a_{\max}$ if needed, then ramp back to zero. The deceleration must satisfy both phase invariants simultaneously:

$$\sum_{n \in \Pi_k^{\text{dec}}} j^{(n)} T^{(n)} = -a_k$$

$$\sum_{n \in \Pi_k^{\text{dec}}} \left[ a_n^{\text{start}}, T^{(n)} + \tfrac{1}{2}, j^{(n)} \bigl(T^{(n)}\bigr)^2 \right] = -\Delta v_k$$

The first equation sizes the jerk ramps to cancel $a_k$. The second equation sizes the hold at $-a_{\max}$ to cancel $\Delta v_k$. When $\Delta v_k$ is small enough that $a_{\max}$ is never saturated during deceleration, the hold segment collapses and the profile consists of ramps only.

At $t_k^{\text{rec}}$ — the end time of the last segment of $\Pi_k^{\text{dec}}$ — the belt has returned to $(v_u, 0)$ and the segments of $\Pi_{k+1}^{\text{acc}}$ begin being appended to $\mathcal{T}$.

### 4.3 Per-Skip Budget

$\Delta v_k$ depends on how many inputs transited during $\Pi_k^{\text{acc}}$ before the skip was triggered, and on whether the acceleration ramp had saturated at $a_{\max}^{\text{acc}}$. Because the skip may be detected before the ramp completes, $\Delta v_k \leq \Delta v_{\max}$. The deceleration is sized exactly to match $a_k$ and $\Delta v_k$, not their maxima.

---

## 5. Nominal Entry Condition

### 5.1 Statement

The buffer solver requires $v_i^{\text{in}} = v_u$ and $a_i^{\text{in}} = 0$ at buffer entry. This holds if and only if the deceleration following $s_k$ completes before the next input $j = i+1$ reaches buffer entry:

$$t_k^{\text{rec}} \leq t_j^{\text{in}}$$

### 5.2 Minimum Inter-Arrival Gap

The recovery duration is determined by $a_k$, $\Delta v_k$, $j_{\max}$, and $a_{\max}$:

$$\Delta t_k^{\text{rec}} = f\bigl(a_k, \Delta v_k, j_{\max}, a_{\max}\bigr)$$

The feasibility condition requires a minimum inter-arrival gap of $\Delta t_k^{\text{rec}}$ between $s_k$ and $t_j^{\text{in}}$. When violated, the feedforward scheme is **infeasible** for this skip event — the deceleration cannot complete in time and the entry state will be non-nominal.

### 5.3 Infeasibility as a Function of $\sigma_u$

The inter-arrival gap is a random variable whose distribution depends on $\sigma_u$. Higher upstream variability increases the probability of short gaps following a skip, making infeasibility more likely. The scheme degrades gracefully: a non-nominal entry state is flagged and the buffer solver falls back to handling it directly. Under the AR(1) model, gaps following a large-gap skip are expected to be above average, making infeasibility less likely immediately after severe skips — a structural property absent from the i.i.d. model.

---

## 6. Notation Summary

| Symbol                  | Meaning                                                                                                                    |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| $\mathcal{T}$           | Jerk timeline — append-only list of constant-jerk segments                                                                 |
| $\mathcal{T}\|_{[a,b]}$ | Projection of $\mathcal{T}$ onto time interval $[a, b]$                                                                    |
| $n$                     | Segment index into $\mathcal{T}$                                                                                           |
| $t_n$                   | Absolute start time of segment $n$                                                                                         |
| $j^{(n)}$               | Constant jerk value of segment $n$                                                                                         |
| $T^{(n)}$               | Duration of segment $n$                                                                                                    |
| $a_n^{\text{start}}$    | Acceleration at the start of segment $n$; end state of segment $n-1$, or $0$ for the first segment of $\Pi_k^{\text{acc}}$ |
| $\Pi$                   | Ordered list of inter-skip phases                                                                                          |
| $\Pi_k$                 | $k$-th inter-skip phase; contiguous subsequence $\mathcal{T}[n_k^{\text{start}} \ldots n_k^{\text{end}}]$                  |
| $\Pi_k^{\text{acc}}$    | Acceleration segments of $\Pi_k$ — appended incrementally up to $s_k$                                                      |
| $\Pi_k^{\text{dec}}$    | Deceleration segments of $\Pi_k$ — appended in full at $s_k$                                                               |
| $s_k$                   | Time of the $k$-th skip event; $s_k = t_i^{\text{in}}$ where $\delta_i = 1$                                                |
| $t_k^{\text{rec}}$      | End time of $\Pi_k^{\text{dec}}$; belt restored to $(v_u,, 0)$                                                             |
| $a_k$                   | Acceleration at skip time; $\sum_{n \in \Pi_k^{\text{acc}}} j^{(n)} T^{(n)}$ (m/s²)                                        |
| $\Delta v_k$            | Velocity surplus at skip time; area under acceleration curve over $\Pi_k^{\text{acc}}$ (m/s)                               |
| $\Delta v_{\max}$       | Maximum permissible velocity surplus; $v_{\max}^{\text{up}} - v_u$ (m/s)                                                   |
| $j^{\text{up}}(t)$      | Upstream jerk signal                                                                                                       |
| $a^{\text{up}}(t)$      | Upstream acceleration; derived from $\mathcal{T}$ by integrating $j^{\text{up}}$                                           |
| $v^{\text{up}}(t)$      | Upstream velocity; $v_u + \int a^{\text{up}} , d\tau$                                                                      |
| $v_{\max}^{\text{up}}$  | Tunable upstream speed ceiling                                                                                             |
| $v_{\max}$              | Maximum object speed in the buffer (physical constraint, not tunable)                                                      |
| $t_i^{\text{spawn}}$    | Absolute spawn time of input $i$                                                                                           |
| $t_i^{\text{in}}$       | Absolute buffer entry time of input $i$                                                                                    |
| $\delta_i$              | Skip indicator for input $i$                                                                                               |
| $j_{\max}$              | Maximum jerk                                                                                                               |
| $a_{\max}$              | Maximum deceleration magnitude (used in $\Pi_k^{\text{dec}}$)                                                              |
| $a_{\max}^{\text{acc}}$ | Maximum acceleration magnitude (used in $\Pi_k^{\text{acc}}$)                                                              |
| $v_u$                   | Nominal upstream belt velocity                                                                                             |
| $L_{\text{up}}$         | Upstream control distance                                                                                                  |