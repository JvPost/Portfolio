# KSB whole-line design optimization

## Scope

Static, fixed-policy, zero-slip whole-line design optimization. The optimizer chooses both the synchronization-line geometry (buffer length, segmentation, kinematic headroom) and the line-level rate/slot configuration. This is the **greenfield framing**: we are sizing a synchronization line from scratch given an input stream of products of length $l$, not retrofitting a buffer between a fixed upstream and fixed downstream.

Out of scope: MPC-style replanning, registrar geometry ($L^R$, $N_R$ are fixed), arrival-statistics parameters ($\sigma_u$ is exogenous to the design), and mechanical limits ($V_{\max}, A_{\max}, j_{\max}$ — set externally as hardware constants, not optimized).

## Reparameterization: headroom factors

The earlier parameterization had $v^{BR}$, $v_u^{\max}$, $a_u^{\max}$, $\eta_r$, $\eta_s$ as independent decision variables. This is overdetermined: $v^{BR} \le V_{\max}$, $v^{BR} \ge v_d$, $v_d \le V_{\max}$, and the trajectory-feasibility coupling between $a_u^{\max}$ and $v^{BR}$ form a tangled feasibility manifold. The optimizer kept falling off it, producing sentinel exceptions ("Required velocity exceeds $V_{\max}$", "Duration T must be positive", etc.).

The fix: change coordinates so the feasibility manifold becomes axis-aligned. Three scalars, all interpretable as **headroom factors** that convert stochastic upstream behavior into deterministic downstream delivery:

- $\eta_r > 1$ — **temporal headroom**. Slot rate exceeds arrival rate; surplus slots absorb timing variance. Skip rate $\approx (\eta_r - 1)/\eta_r$ in the deterministic limit.
- $\eta_s > 1$ — **spatial headroom**. Slot pitch exceeds object length; surplus pitch absorbs intra-slot positional error.
- $\eta_v > 1$ — **kinematic headroom**. Buffer release velocity exceeds downstream velocity; surplus velocity is what enables phase correction.

Upstream FF velocity and acceleration ceilings are pinned to $V_{\max}$ and $A_{\max}$ (hardware bounds — the optimizer should not be discovering hardware specs). Buffer release velocity is derived: $v^{BR} = \eta_v \cdot v_d$.

## Derived quantities

Downstream velocity:
$$
v_d \;=\; \underbrace{\eta_r \cdot r_u}_{\text{slot rate}} \cdot \underbrace{\eta_s \cdot l}_{\text{slot pitch}} \;=\; \eta_r \eta_s \cdot l \cdot r_u
$$
where $r_u$ is the upstream arrival rate (pieces/s).

Buffer release velocity:
$$
v^{BR} \;=\; \eta_v \cdot v_d
$$

Hardware ceiling: $\eta_v \cdot v_d \le V_{\max}$. Violations trigger a sentinel rather than being clipped silently — the optimizer should feel the wall.

The $(\eta_r, \eta_s)$ pair is bilinear in $v_d$ but **not** degenerate in the objective: $\eta_r$ acts on temporal precision (skip rate, queue dynamics) while $\eta_s$ acts on spatial precision (per-slot correction budget). The objective sees both effects independently.

## Decision variables

**Continuous, $\boldsymbol{\theta}_c$ (6 dims):**

| Symbol | Meaning | Range |
|---|---|---|
| $L^B$ | Buffer length | $[N^B L^B_\min,\, 8.0]$ m |
| $\beta$ | Segment-length log-linear skew (softmax) | $[-2,\, 2]$ |
| $\gamma$ | Segment-length log-quadratic curvature | $[-2,\, 2]$ |
| $\eta_r$ | Temporal headroom (slot rate / arrival rate) | $[1,\, 2]$ |
| $\eta_s$ | Spatial headroom (slot pitch / object length) | $[1,\, 2]$ |
| $\eta_v$ | Kinematic headroom ($v^{BR} / v_d$) | $[1,\, 2]$ |

Sentinel triggered if $\eta_v \cdot v_d > V_{\max}$.

**Outer integer:**

| Symbol | Meaning | Range |
|---|---|---|
| $N^B$ | Buffer segment count | $\{3, 4, \ldots, 20\}$ |

**Pinned (not optimized):**

| Symbol | Value | Source |
|---|---|---|
| $V_{\max}$ | hardware constant | servo spec |
| $A_{\max}$ | hardware constant | servo spec |
| $j_{\max}$ | hardware constant | servo spec |
| $v_u^{\max}$ | $V_{\max}$ | upstream FF saturates at hardware ceiling |
| $a_u^{\max}$ | $A_{\max}$ | upstream FF saturates at hardware ceiling |

Upstream control is fully determined by skip events given these ceilings — no design freedom there.

## Objective

$$
\mathcal{L}(\boldsymbol{\theta}_c;\, N^B) \;=\; \underbrace{\sum_{i,k} \Phi^{\mathcal{P}}_{i,k}}_{\text{NLL: feasibility}} \;+\; \lambda_U \underbrace{\sum_{i,k} U^{\mathcal{P}}_{i,k}}_{\text{prior: utilization}} \;+\; \lambda_L L^B \;+\; \lambda_T\, \eta_r
$$

Outer-loop selection augments with $\lambda_N N^B$ (constant inside an inner solve, handled at selection time).

The four inner terms:

| Term | Role | Units of weight |
|---|---|---|
| $\sum \Phi$ | Negative log-likelihood: feasibility violations | dimensionless |
| $\lambda_U \sum U$ | Prior: discourages oversized slack (waste) | s |
| $\lambda_L L^B$ | Prior: line footprint cost | s²/m |
| $\lambda_T \eta_r$ | Prior: throughput cost — penalizes over-provisioning slots vs. inputs | s² |

Throughput term is necessary because $\eta_r$ is in $\boldsymbol{\theta}_c$. Without it the optimizer pushes $\eta_r \to 2$ (skip every other slot, trivially feasible). $\lambda_T$ should be calibrated so the trade-off "more slots ↔ more slack" lands at a sensible operating point. Initial guess: $\lambda_T \sim \sum \Phi$ at the all-defaults configuration.

$\eta_s$ and $\eta_v$ are not explicitly priced — $\eta_s$'s cost is carried implicitly by $\lambda_L L^B$ (larger slots demand a longer buffer for the same input rate); $\eta_v$'s cost is bounded above by feasibility ($V_{\max}$). If wear/energy considerations on $v^{BR}$ become relevant, add a $\lambda_V \eta_v$ term.

## Method

Forward sim: ~0.83 ms for $b = 100$. Inner problem is small ($d = 6$) and cheap. CMA-ES is the right tool.

**Step 1 — Coarse grid sweep first (~few seconds).** Before any CMA-ES, do a $4^d \approx 4{,}000$ Latin-hypercube or random sample of $\mathcal{L}$ across the box. Purpose: spot sentinel regions ($C = +\infty$ cells), detect multimodality, validate qualitative predictions ($\gamma^\star < 0$, edge-segment sensitivity). At 0.83 ms × 4 k samples ≈ 3 seconds. Cheap insurance against trusting an optimum on a pathological surface.

**Step 2 — CMA-ES per $N^B$.** Population 20, max 100 generations, 4 random restarts. Fixed $N^B$ per inner solve. Per-$N^B$ wall time should be lower than before (fewer dims, fewer feasibility-induced sentinels). Full sweep over $\{3, \ldots, 20\}$ ≈ 1–2 min.

**Step 3 — Outer selection.** Plot $\mathcal{L}^\star(N^B) + \lambda_N N^B$. Pick the knee, not blindly the argmin — $\lambda_N$ is a guess and the curve shape is more informative than the literal minimum.

**Step 4 — Validate the optimum.** At $\boldsymbol{\theta}^\star$, run a fresh simulation with multiple seeds, plot the slack matrix $S_{i,k}$ as a heatmap, confirm the structural-asymmetry prediction (edges run hot, $\gamma^\star < 0$). Failure to confirm is interesting, not bad — but flag it.

## What the optimizer is allowed to discover

The reparameterization bakes in three structural facts as hard constraints rather than letting the optimizer rediscover them via sentinels:

- $v^{BR} \ge v_d$ — enforced by $\eta_v \ge 1$.
- $r_d \ge r_u$ — enforced by $\eta_r \ge 1$.
- $d_s \ge l$ — enforced by $\eta_s \ge 1$.

What remains free:

- **The exact shape of headroom trade-offs.** Does the optimizer prefer to absorb upstream variance via slot rate ($\eta_r$), slot pitch ($\eta_s$), or velocity surplus ($\eta_v$)? The objective sees all three independently and the answer should be data-driven.
- **Buffer geometry.** $L^B$, segmentation skew/curvature, segment count.
- **Hardware-ceiling activity.** If the optimum sits at $\eta_v v_d \approx V_{\max}$, that's a finding — the design is hardware-limited and the next conversation is whether to spec a faster servo.

## Diagnostic value of remaining sentinels

After reparameterization, the surviving exception types are diagnostically meaningful rather than parameterization artifacts:

- `Required velocity exceeds V_max` — only if $\eta_v v_d > V_{\max}$ at the sampled point, i.e. the optimizer is probing the hardware ceiling. Expected near the boundary.
- `Duration T must be positive` — trajectory generator getting degenerate inputs. Likely a generator robustness issue rather than a coupling problem; investigate independently.
- `SlotAssignmentError` / `Leader cannot reach segment exit` — genuine kinematic infeasibility for the disturbance realization. Real signal about feasibility margin under the chosen $(\eta_r, \eta_s, \eta_v)$.

If sentinels of the second category persist after reparameterization, the next pass is on the trajectory generator, not the search.

## Portfolio framing

The story is: **whole-line synchronization design as a small chance-constrained optimization problem**. 6-dim continuous + 1 integer, ~0.83 ms forward sim, CMA-ES + grid sweep — the methodology is intentionally boring because the formalization is what's load-bearing. The deliverable isn't "I optimized something with CMA-ES"; it's "I formalized a packaging-line synchronization problem cleanly enough that an off-the-shelf optimizer found a coherent design point, and the predicted structural features ($\gamma^\star < 0$, edge-loaded slack) showed up in the result."

The reparameterization itself is part of the story: the headroom-factor framing makes explicit that synchronization design is fundamentally about budgeting variance absorption across three orthogonal axes (time, space, velocity).

## Glossary

- **CMA-ES** — Covariance Matrix Adaptation Evolution Strategy. Gradient-free black-box optimizer; maintains a Gaussian search distribution and adapts its covariance to the local loss landscape.
- **FF** — feedforward (upstream controller).
- **NLL** — negative log-likelihood. Used here as a Bayesian-flavored framing for $\sum \Phi$: under a MAP reading, $\Phi$ is the data-fit term and the other components of $\mathcal{L}$ are priors over the design.
- **Headroom factor** — dimensionless ratio $\ge 1$ representing surplus capacity in one of three dimensions (temporal, spatial, kinematic) for absorbing upstream variance.