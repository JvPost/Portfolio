# KSB whole-line design optimization

## Scope

Static, fixed-policy, zero-slip whole-line design optimization. The optimizer chooses both the synchronization-line geometry (buffer length, segmentation, exit velocity, upstream FF parameters) and the line-level rate/slot configuration. This is the **greenfield framing**: we are sizing a synchronization line from scratch given an input stream of products of length $l$, not retrofitting a buffer between a fixed upstream and fixed downstream.

Out of scope for now: MPC-style replanning, registrar geometry ($L^R$, $N_R$ are fixed), arrival-statistics parameters ($\sigma_u$ is exogenous to the design).

## Decision variables

**Continuous, $\boldsymbol{\theta}_c$ (8 dims):**

| Symbol | Meaning | Range |
|---|---|---|
| $L^B$ | Buffer length | $[N^B L^B_\min,\, 8.0]$ m |
| $\beta$ | Segment-length log-linear skew (softmax) | $[-2,\, 2]$ |
| $\gamma$ | Segment-length log-quadratic curvature | $[-2,\, 2]$ |
| $v^{BR}$ | Buffer exit velocity | $[0.5,\, V_{\max}]$ m/s — note $v^{BR} \ge v_d$ is **not enforced**; the optimizer should discover it |
| $a_u^{\max}$ | Upstream FF accel ceiling | $[0.2,\, A_{\max}]$ m/s² |
| $v_u^{\max}$ | Upstream FF velocity ceiling | $[\bar{v}_u,\, V_{\max}]$ m/s |
| $\eta_s$ | Slot-length factor: $d_s = \eta_s \cdot l$ | $[1,\, 2]$ |
| $\eta_r$ | Rate ratio: $r_d = \eta_r \cdot r_u$ (so $\rho = 1/\eta_r$) | $[1,\, 2]$ |

Upstream FF jerk $j_u^{\max} = j_{\max}$ is shared with the buffer (mechanical bound), not a separate decision variable. Bounds $a_u^{\max} \le A_{\max}$ and $v_u^{\max} \le V_{\max}$ are enforced via the sampling box, not the loss.

**Outer integer:**

| Symbol | Meaning | Range |
|---|---|---|
| $N^B$ | Buffer segment count | $\{3, 4, \ldots, 20\}$ |

## Objective

$$
\mathcal{L}(\boldsymbol{\theta}_c;\, N^B) \;=\; \underbrace{\sum_{i,k} \Phi^{\mathcal{P}}_{i,k}}_{\text{NLL: feasibility}} \;+\; \lambda_U \underbrace{\sum_{i,k} U^{\mathcal{P}}_{i,k}}_{\text{prior: utilization}} \;+\; \lambda_L L^B \;+\; \lambda_T\, \eta_r
$$

Outer-loop selection augments with $\lambda_N N^B$ (constant inside an inner solve, so handled at selection time).

The four inner terms:

| Term | Role | Units of weight |
|---|---|---|
| $\sum \Phi$ | Negative log-likelihood: feasibility violations | dimensionless |
| $\lambda_U \sum U$ | Prior: discourages oversized slack (waste) | s |
| $\lambda_L L^B$ | Prior: line footprint cost | s²/m |
| $\lambda_T \eta_r$ | Prior: throughput cost — penalizes over-provisioning of slots relative to inputs | s² |

The throughput term is necessary because $\eta_r$ is in $\boldsymbol{\theta}_c$. Without it the optimizer pushes $\eta_r \to 2$ (skip every other slot, trivially feasible). $\lambda_T$ should be calibrated so the trade-off "more slots ↔ more slack" lands at a sensible operating point. Initial guess: $\lambda_T \sim \sum \Phi$ at the all-defaults configuration.

The slot-length factor $\eta_s$ is **not** explicitly priced — its cost is carried implicitly by $\lambda_L L^B$ (larger slots demand a longer buffer to maintain feasibility for the same input rate).

## Method

Forward sim: ~0.83 ms for $b = 100$. Inner problem is small ($d = 8$) and cheap. CMA-ES is the right tool.

**Step 1 — Coarse grid sweep first (~30 s).** Before any CMA-ES, do a $4^d$ Latin-hypercube or random sample of $\mathcal{L}$ across the box. Purpose: spot sentinel regions ($C = +\infty$ cells), detect multimodality, validate qualitative predictions ($\gamma^\star < 0$, edge-segment sensitivity). At 0.83 ms × 65 k samples ≈ 1 minute. Cheap insurance against trusting an optimum that's a local minimum in a pathological surface.

**Step 2 — CMA-ES per $N^B$.** Population 20, max 100 generations, 4 random restarts. Fixed $N^B$ per inner solve. Per-$N^B$ wall time ≈ 5–10 s. Full sweep over $\{3, \ldots, 20\}$ ≈ 2–3 min.

**Step 3 — Outer selection.** Plot $\mathcal{L}^\star(N^B) + \lambda_N N^B$. Pick the knee, not blindly the argmin — $\lambda_N$ is a guess and the curve shape is more informative than the literal minimum.

**Step 4 — Validate the optimum.** At $\boldsymbol{\theta}^\star$, run a fresh simulation with multiple seeds, plot the slack matrix $S_{i,k}$ as a heatmap, confirm the structural-asymmetry prediction (edges run hot, $\gamma^\star < 0$). Failure to confirm is interesting, not bad — but flag it.

## What the optimizer is allowed to discover

The greenfield framing means we don't pre-impose:

- **$v^{BR} \ge v_d$.** The registrar formalism assumes deceleration; if the optimizer prefers $v^{BR} < v_d$ (acceleration on the registrar), that's a finding, not an error. Add the constraint later if the data motivates it.
- **$\eta_r \ge 1$.** Hard-bounded at 1 — equality means no skips. The optimizer should be free to push toward $\eta_r = 1$ if the throughput penalty $\lambda_T$ is high enough.
- **$\eta_s$.** No prior on whether tight or loose slots are better; the trade-off is between slot capacity and per-input correction time horizon.

The only hard constraint at sampling time is the box, which encodes mechanical limits ($A_{\max}, V_{\max}$) and physical sanity ($\eta_r \ge 1$, $\eta_s \ge 1$).

## Portfolio framing

The story is: **whole-line synchronization design as a small chance-constrained optimization problem**. 8-dim continuous + 1 integer, ~0.83 ms forward sim, CMA-ES + grid sweep — the methodology is intentionally boring because the formalization is what's load-bearing. The deliverable isn't "I optimized something with CMA-ES"; it's "I formalized a packaging-line synchronization problem cleanly enough that an off-the-shelf optimizer found a coherent design point, and the predicted structural features ($\gamma^\star < 0$, edge-loaded slack) showed up in the result."

## Glossary

- **CMA-ES** — Covariance Matrix Adaptation Evolution Strategy. Gradient-free black-box optimizer; maintains a Gaussian search distribution and adapts its covariance to the local loss landscape.
- **FF** — feedforward (upstream controller).
- **NLL** — negative log-likelihood. Used here as a Bayesian-flavored framing for $\sum \Phi$: under a MAP reading, $\Phi$ is the data-fit term and the other components of $\mathcal{L}$ are priors over the design.