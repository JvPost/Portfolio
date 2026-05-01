# Per-Segment Synchronization — Budget, Cost, Slack, Design Objective

This document develops the formal apparatus for measuring synchronization feasibility on the buffer. It assumes the system definitions, distributional models, and skip mechanism established in *Buffer formalization*, and provides the per-segment slack matrix and design objective $\mathcal{L}$ referenced by the *KSB system — Design Narrative*. The conceptual reading of the buffer as a control system — open-loop per segment, semi-closed-loop across segments — is developed in *Buffer formalization* §5; this document is the formal substrate that view sits on.

The feasibility question at segment $k$ between inputs $i$ and $i+1$ reduces to: *is the free-window long enough for the segment to execute the required state transition under its own kinematic bounds?* This splits cleanly into **how much time we have**, **how much time we need**, their difference, and the scalar objective that drives design optimization.

## 1. Boundaries, segments, events

The buffer is partitioned by $N^B + 1$ **boundaries** at positions

$$
P^B_0 = 0, \quad P^B_1, \quad \ldots, \quad P^B_{N^B} = L^B,
\qquad P^B_k \;=\; \sum_{j=1}^{k} L^B_j.
$$

**Segments** are the intervals between adjacent boundaries: segment $k$ occupies $[P^B_{k-1}, P^B_k]$ and has length $L^B_k$, for $k \in \{1, \ldots, N^B\}$. The baseline is uniform segmentation $L^B_k = L^B / N^B$; §5 generalizes.

Let $p_i(t)$ be the leading-edge position of input $i$ in buffer-local coordinates and $l_i$ its length. For each pair $(i, i+1)$, the relevant boundary-crossing times are:

$$
\begin{aligned}
t_i^{k,\text{out}} &: p_i(t) \;=\; P_k^B + l_i & \text{(trailing edge of } i \text{ clears boundary } k\text{)}\\[2pt]
t_{i+1}^{k,\text{in}} &: p_{i+1}(t) \;=\; P_{k-1}^B & \text{(leading edge of } i+1 \text{ arrives at boundary } k{-}1\text{)}
\end{aligned}
$$

The **free window** of segment $k$ for the pair is

$$\mathcal{W}_{i,k} \;=\; \bigl[\, t_i^{k,\text{out}},\; t_{i+1}^{k,\text{in}} \,\bigr]$$

— the interval during which segment $k$ carries no object. Each pair has $N^B$ such free windows, bracketed by events at the $N^B + 1$ boundaries.

## 2. Endpoint kinematic states

During occupancy, the straddling constraint forces segment $k$ to run at the occupying input's kinematics. At the endpoints of $\mathcal{W}_{i,k}$ the segment must therefore match:

$$
\begin{aligned}
\bigl(v^-_{i,k},\, a^-_{i,k}\bigr) &= \bigl(\dot p_i,\, \ddot p_i\bigr)\big|_{t_i^{k,\text{out}}} & \text{(leader hands off at boundary } k\text{)}\\[2pt]
\bigl(v^+_{i,k},\, a^+_{i,k}\bigr) &= \bigl(\dot p_{i+1},\, \ddot p_{i+1}\bigr)\big|_{t_{i+1}^{k,\text{in}}} & \text{(follower demands at boundary } k{-}1\text{)}
\end{aligned}
$$

Sign convention: superscript $-$ for the state *handed off from* input $i$; $+$ for the state *required by* input $i+1$. Note the spatial asymmetry: the start-of-window BC is measured at the *far* boundary $k$ (leader trail), the end-of-window BC at the *near* boundary $k{-}1$ (follower lead). Segment $k$'s retiming primitive must travel from $(v^-, a^-)$ to $(v^+, a^+)$ within the free window.

## 3. Budget, cost, slack

Let $b$ be the batch size. Diagnostic matrices $\mathbf{W}, \mathbf{C}^{\pi}, \mathbf{S}^{\pi}, \boldsymbol{\Delta j}^\pi$ all have shape $(b-1) \times N^B$.

### 3.1 Budget $\mathbf{W}$

The wall-clock time segment $k$ has available to retime itself:

$$
W_{i,k} \;=\; t_{i+1}^{k,\text{in}} - t_i^{k,\text{out}} \;=\; |\mathcal{W}_{i,k}|.
$$

Units: seconds. Determined entirely by the committed input trajectories — a property of the *schedule*, independent of how the segment executes. $W_{i,k}$ can be negative: two inputs share segment $k$, i.e. the abstraction of one primitive per $(i,k)$ is violated. This is a design defect, not a numerical failure, and is handled naturally by the penalty in §4.

Monotonicity worth keeping in mind: holding the schedule fixed, shrinking $L^B_k$ monotonically increases $W_{i,k}$. (Shorter segment ⟹ leader clears its exit sooner; follower entry time unchanged.) The strict guarantee $W_{i,k} > 0$ requires pitch-minus-length $> L^B_k$, where pitch is the center-to-center input spacing — so the gap between consecutive inputs must exceed the segment length. Shrinking $L^B_k$ pushes toward this guarantee from the wrong direction; only when $L^B_k$ drops below the minimum inter-input gap is positivity structural.

We denote the per-segment **policy** — the algorithm for computing kinematic transitions under motor constraints — as $\pi$. Specific policy instances include $\pi_3$ (cubic-in-velocity) and $\pi_{\text{BB}}$ (bang-bang jerk).

### 3.2 Cost function $C^{\pi}$

Given the endpoint kinematic states, the **minimum transition time** under policy $\pi$ and the free-window motor envelope $(v^F_{\max}, a^F_{\max}, j^F_{\max})$ is an analytical function of the four boundary conditions:

$$
C^{\pi}_{\min}\bigl(v^-, a^-, v^+, a^+\bigr) \;=\; \tau^\star_{\pi}\!\bigl(v^-, a^-, v^+, a^+\bigr).
$$

Evaluated pointwise per $(i, k)$:

$$
C^{\pi}_{i,k} \;=\; C^{\pi}_{\min}\bigl(v^-_{i,k},\, a^-_{i,k},\, v^+_{i,k},\, a^+_{i,k}\bigr).
$$

The matrix is storage for debugging and visualization; the underlying object is the function. Its derivation is the subject of [[Buffer minimum-time cost]].

Units: seconds. A property of the *policy* and the *free-window motor envelope*, independent of input-trajectory timing.

Two policy instances of interest:

| Policy            | Symbol            | Notes                                                                    |
| ----------------- | ----------------- | ------------------------------------------------------------------------ |
| Cubic-in-velocity | $\pi_3$           | Uniquely determined by 4 BCs; linear jerk; preferred (gentler on motors) |
| Bang-bang jerk    | $\pi_{\text{BB}}$ | Time-optimal lower bound; piecewise-closed form                          |

By optimality of bang-bang: $C^{\pi_{\text{BB}}}_{i,k} \le C^{\pi_3}_{i,k}$ for all $(i,k)$.

For $\pi = \pi_{\text{BB}}$ the policy is taken in its **adaptive jerk** variant ([[Buffer minimum-time cost]] §6): the closed form is evaluated at $\tilde{j} = \max(j^B_\max, j_\text{req})$ rather than at $j^B_\max$ directly. Consequence: $C^{\pi_\text{BB}}$ is a total function, finite on all $(v^-, a^-, v^+, a^+)$, with no primitive-existence sentinel. The deviation $\tilde{j} - j^B_\max$ is recorded separately as a post-hoc diagnostic (§3.5).

### 3.3 Slack $\mathbf{S}$ — diagnostic

Element-wise difference:

$$
S^{\pi}_{i,k} \;=\; W_{i,k} - C^{\pi}_{i,k}.
$$

| Value               | Meaning                                                                                                    |
| ------------------- | ---------------------------------------------------------------------------------------------------------- |
| $S^{\pi}_{i,k} > 0$ | Segment $k$ completes the transition with margin                                                           |
| $S^{\pi}_{i,k} = 0$ | Transition saturates the free window                                                                       |
| $S^{\pi}_{i,k} < 0$ | The policy fits in principle but exceeds $W_{i,k}$ — no trajectory under $\pi$ fits in $\mathcal{W}_{i,k}$ |

The ordering $C^{\pi_{\text{BB}}} \le C^{\pi_3}$ induces $S^{\pi_{\text{BB}}} \ge S^{\pi_3}$ element-wise, so bang-bang slack is a feasibility *ceiling* and cubic slack a *floor*. Cells with $S^{\pi_3}_{i,k} < 0 \le S^{\pi_{\text{BB}}}_{i,k}$ are precisely where the design must fall back from cubic to bang-bang.

**Sentinel regime.** Under $\pi_\text{BB}$ in its adaptive-jerk form (§3.2), $C^{\pi_\text{BB}}$ is finite by construction and there is no primitive-existence sentinel. Sentinels under $\pi_\text{BB}$ remain only for the *envelope* regimes — the closed form's $|a_p| \le a^B_\max$ and $|v| \le v^B_\max$ assumptions ([[Buffer minimum-time cost]] §5.4–§5.5). When violated, the closed form is no longer self-consistent and a richer policy (3-phase a-saturated, 7-phase v-saturated) is required; the implementation flags such cells and they are reported but not silently penalized. Within the intended KSB operating range these envelope sentinels should be empty; their occurrence is diagnostic, not nominal.

For $\pi_3$, sentinel handling depends on its own envelope conditions and is treated separately.

$\mathbf{S}$ is a diagnostic object. The scalar used to drive design optimization is defined in §4.

### 3.4 Boundary asymmetry

Not all boundaries are equal. At a generic *interior* boundary $k \in \{1, \ldots, N^B - 1\}$, both the leader-hand-off state and the neighbouring follower-demand state are shaped by the buffer's own planning; upstream noise has been partially absorbed, registrar demands haven't yet arrived. The buffer has authority over both endpoints.

At the *edge* boundaries, one endpoint is pinned by an external system:

- **Boundary $0$** (buffer ↔ upstream). The follower's leading-edge arrival state $(v^+_{i, 1}, a^+_{i, 1})$ is the state handed over by upstream, carrying whatever variance the upstream controller could not shape out of the infeed process.
- **Boundary $N^B$** (buffer ↔ registrar). The leader's trailing-edge departure state $(v^-_{i, N^B}, a^-_{i, N^B})$ is rigidly determined by the registrar handoff — the buffer must exit at the registrar's demanded velocity, with its demanded acceleration.

Consequently the first and last segments ($k = 1$ and $k = N^B$) each have one external BC they cannot negotiate. Interior segments have two internal BCs. This predicts — and simulation should confirm — that $C_{i,k}$ runs higher and $S_{i,k}$ runs lower at $k = 1$ and $k = N^B$. Edge-concentrated violations are a structural feature of the architecture, not a bug.

The segment-length parametrization in §5 is the mechanism through which the optimizer responds to this asymmetry.

### 3.5 Adaptive jerk diagnostic $\boldsymbol{\Delta j}^\pi$

For $\pi = \pi_\text{BB}$, the per-cell jerk overspec

$$
\Delta j^\pi_{i,k} \;:=\; \max\bigl\{0,\; j_\text{req}(v^-_{i,k}, a^-_{i,k}, v^+_{i,k}, a^+_{i,k}) - j^B_\max\bigr\}
$$

records by how much the jerk bound would need to be raised to render cell $(i,k)$'s primitive feasible at the nominal $j^B_\max$ ([[Buffer minimum-time cost]] §6.3). Units: m/s³.

$\boldsymbol{\Delta j}^\pi$ is **not an optimization signal**: $j^B_\max$ is a fixed motor-spec parameter, not a design variable, so penalizing $\Delta j$ separately would add no gradient direction the slack pressure on $\boldsymbol{\theta}_c$ does not already supply. It is purely a *post-hoc readout*: cells with $\Delta j > 0$ at the optimum identify transitions that the actuator could not have executed at nominal jerk; sums and maxima across the cell grid quantify the jerk overspec a robustness-minded motor redesign would need.

For $\pi = \pi_3$ the construction is not directly applicable; cubic-in-velocity has no analogous single-knob feasibility extension.

## 4. Design objective

Let $\boldsymbol{\theta}$ collect the design variables: the continuous set $\{L^B, \beta, \gamma, \boldsymbol{\theta}_{\text{upstream}}\}$ plus the integer $N^B$. ($\boldsymbol{\theta}_{\text{upstream}}$ is deferred to separate formalization; for the purposes of this document it is an exogenous producer of the endpoint states in §2.) A chosen policy instance $\pi \in \{\pi_3, \pi_{\text{BB}}\}$ is fixed for the optimization run.

The scalar driving optimization is a sum of four terms — one likelihood-like (physical feasibility) and three prior-like (preferences on tightness and structural complexity). Under a MAP reading: $\Phi$ plays the role of negative log-likelihood; the remaining three are log-priors on the design.

### 4.1 Feasibility barrier $\Phi$

Per-cell quadratic hinge on infeasibility:

$$
\Phi^{\pi}_{i,k} \;=\; \bigl(\max\{0,\, -S^{\pi}_{i,k}\}\bigr)^{\!2} \;=\; \bigl(\max\{0,\, C^{\pi}_{i,k} - W_{i,k}\}\bigr)^{\!2}.
$$

Units: seconds². Properties:
- $C^1$ smooth everywhere, including the kink at $S = 0$.
- Well-defined for all $W \in \mathbb{R}$, including $W = 0$ and $W < 0$ (overlap regime). Overlap simply produces a larger $C^{\pi} - W$ and a larger penalty — no special case, no NaN, no $\infty$.
- Under $\pi_\text{BB}$ in its adaptive-jerk form, $C^\pi$ is finite by §3.2, so $\Phi^\pi$ is finite by construction. The previous sentinel branch on primitive non-existence is structurally empty.
- Gradient $\partial \Phi^{\pi} / \partial W = -2\,(C^{\pi} - W)$ when infeasible, zero when feasible. Pushes $W$ up where it matters, silent where it doesn't.

### 4.2 Utilization prior $U$

Per-cell linear hinge on positive slack:

$$
U^{\pi}_{i,k} \;=\; \max\{0,\, S^{\pi}_{i,k}\}.
$$

Units: seconds. Properties:
- Linear, not quadratic — waste is roughly linear in time; every spare second is equally wasteful. Curvature needs physical justification we don't have.
- Zero on the infeasible side ($S < 0$), so it does not fight the barrier.
- Primary gradient role: driving $(\beta, \gamma)$ to redistribute segment length toward the cells where it is needed. Secondary pressure on $L^B$ (realization-aware).

**Compact-robust principle.** $U$ alone would drive segments toward saturation — zero margin, fragile to arrival-noise realizations outside the training distribution. $R_L$ alone (next section) would drive $L^B$ to zero regardless of feasibility — infeasibility wins. The combination of $\Phi + \lambda_U U + \lambda_L L^B$ produces a design that is *compact* (nothing oversized) and *robust* (margin concentrated where the slack term doesn't consume it). An oversized design with large slack absorbs noise statistically by virtue of headroom; a compact design with small slack absorbs noise structurally by virtue of the machinery $(\beta, \gamma, \ldots)$ having been tuned to the noise's actual shape. The second is what we want.

### 4.3 Structural priors

Linear priors on the two structural parameters:

$$
R_L(L^B) \;=\; \lambda_L\, L^B, \qquad R_N(N^B) \;=\; \lambda_N\, N^B.
$$

- **$R_L$** complements the slack pressure on $L^B$ with a realization-blind flat gradient; prevents the optimizer from sizing the buffer against the tail of a single stochastic realization.
- **$R_N$** is constant within a fixed-$N^B$ inner problem (no gradient role), but discriminates across outer-sweep points (§6).
- Linear penalties align with physical hardware cost — "cost per meter," "cost per segment" — rather than an unjustified quadratic curvature.

### 4.4 Aggregate objective

$$
\mathcal{L}(\boldsymbol{\theta}) \;=\; \underbrace{\sum_{i,k} \Phi^{\pi}_{i,k}}_{\text{feasibility (NLL)}} \;+\; \lambda_U \underbrace{\sum_{i,k} U^{\pi}_{i,k}}_{\text{utilization prior}} \;+\; \underbrace{\lambda_L\, L^B}_{\text{size prior}} \;+\; \underbrace{\lambda_N\, N^B}_{\text{complexity prior}}.
$$

The three weights are exchange rates between terms with unequal units, so each has its own dimension:

| Weight      | Units | Reads as                                               |
| ----------- | ----- | ------------------------------------------------------ |
| $\lambda_U$ | s     | "how many s² of infeasibility is 1 s of slack worth?"  |
| $\lambda_L$ | s²/m  | "how many s² of infeasibility is 1 m of buffer worth?" |
| $\lambda_N$ | s²    | "how many s² of infeasibility is 1 segment worth?"     |

They should be calibrated rather than guessed, either by physical reasoning or by Pareto-style sweeping.

Which components of $\nabla_{\boldsymbol{\theta}} \mathcal{L}$ are nonzero depends on what is placed in the continuous part of $\boldsymbol{\theta}$; the objective does not itself decide whether the response to infeasibility is "grow $L^B$" or "redistribute via $\beta, \gamma$" — the chain rule does.

**Sentinel handling.** Under $\pi_\text{BB}$ in its adaptive-jerk form, $\Phi^\pi$ is finite-valued by construction (§4.1) and $\mathcal{L}$ has no $+\infty$ branch from primitive non-existence. The remaining sentinel sources are the envelope regimes (§3.3) — peak-acceleration and peak-velocity violations of the closed-form's self-consistency assumptions. Cells in envelope-sentinel regions should be flagged in the optimizer trace; if they persist at the optimum, the policy's closed form is wrong for the operating regime and a richer extension (a-saturated, v-saturated) is needed.

## 5. Segment-length parametrization

The segment-length vector $\{L^B_k\}_{k=1}^{N^B}$ is parametrized by two scalars $(\beta, \gamma)$ via a log-quadratic softmax over centered, normalized indices:

$$
\tilde{k}_j \;=\; \frac{2j - (N^B + 1)}{\max(N^B - 1,\, 1)}, \qquad j = 1, \ldots, N^B.
$$

Normalization makes $\tilde{k}_j \in [-1, 1]$ for $N^B \ge 2$, so $(\beta, \gamma)$ have $N^B$-independent meaning — values are directly comparable across outer-sweep points (§6). For $N^B = 1$ the parametrization is degenerate: $L^B_1 = L^B$ trivially.

$$
w_j \;=\; \exp\bigl(\beta\, \tilde{k}_j + \gamma\, \tilde{k}_j^2\bigr), \qquad p_j \;=\; \frac{w_j}{\sum_{m=1}^{N^B} w_m}.
$$

$$
L^B_k \;=\; L^B_\min + \bigl(L^B - N^B L^B_\min\bigr)\, p_k.
$$

Properties and signs:

- $(\beta, \gamma) = (0, 0)$ recovers uniform ($L^B_k = L^B / N^B$).
- $\beta$ — log-linear skew. $\beta > 0$ favours late segments (longer near $k = N^B$), $\beta < 0$ favours early.
- $\gamma$ — log-quadratic curvature. $\gamma > 0$ is **edge-heavy**: edges long, middle compressed toward $L^B_\min$. $\gamma < 0$ is **middle-heavy**: middle long, edges short.
- Structural floor: every $L^B_k \ge L^B_\min$ by construction; no barrier term needed.
- Box constraint on $L^B$: $L^B \ge N^B L^B_\min$.
- Smooth in $(\beta, \gamma)$, so slack-term gradients propagate cleanly into segment-length redistribution.

**Predicted direction of optimization.** §3.4 argued that edge segments run hot. The response that relieves them is to make edge segments *shorter* (smaller $L^B_1, L^B_{N^B}$ ⟹ larger $W_{i,1}, W_{i,N^B}$ by §3.1 monotonicity ⟹ more room against $C^{\pi}$). That is $\gamma < 0$, middle-heavy. The pitch-related intuition goes the same way: inputs accelerate through the middle of the buffer, opening inter-input spacing there; longer middle segments are therefore affordable whereas longer edge segments are not. So the optimizer is expected to settle at $\gamma^\star < 0$ for realistic problems. A consistent finding of $\gamma^\star > 0$ would indicate a mis-modeled envelope or an upstream controller failing to deliver the assumed infeed statistics.

## 6. Optimization structure

Because $N^B$ is integer, it is handled by outer enumeration rather than gradient.

**Outer loop.** $N^B \in \{1, \ldots, 20\}$.

**Inner loop (per $N^B$).** Continuous optimization over

$$
\boldsymbol{\theta}_c \;=\; \{L^B,\; \beta,\; \gamma,\; \boldsymbol{\theta}_{\text{upstream}}\}
$$

minimizing $\mathcal{L}(\boldsymbol{\theta})$ subject to the box constraint $L^B \ge N^B L^B_\min$. Record $\mathcal{L}^\star(N^B)$ and the optimal $\boldsymbol{\theta}_c^\star(N^B)$.

**Selection.** Global optimum across the sweep: $N^{B\star} = \arg\min_{N^B} \mathcal{L}^\star(N^B)$.

The $\lambda_N N^B$ term has no role inside the inner loop — it is a constant offset there — but it is the mechanism by which the outer sweep balances complexity against the slack and feasibility pressures resolved at each $N^B$.

## 7. Relation to existing signals

- **Gap violation** $\mathbf{1}[\min_t g_i(t) < g_{\min}]$ — proxy observable. Binary, whole-buffer, post-hoc. Superseded by $\mathbf{S}$ (diagnostic) and $\Phi$ (optimization signal).
- **Whole-buffer pair window** — recovered as the $N^B = 1$ collapse: $W_{i,1}$ with $k=1$ spanning the entire buffer equals the whole-buffer pair-window width used in earlier diagnostics.
- **Hard feasibility** of a plan under policy $\pi$: $\Phi^{\pi}_{i,k} = 0$ for all $(i,k)$, equivalently $S^{\pi}_{i,k} \ge 0$ for all $(i,k)$.