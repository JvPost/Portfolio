# Per-Segment Synchronization — Budget, Cost, Slack

The feasibility question at segment $k$ between inputs $i$ and $i+1$ reduces to: *is the free-window long enough for the segment to execute the required state transition under its own kinematic bounds?* This splits cleanly into **how much time we have**, **how much time we need**, and their difference.

## 1. Endpoint events

Let $p_i(t)$ be the leading-edge position of input $i$ in buffer-local coordinates. Segment $k \in \{1, \ldots, N^B\}$ occupies $[P_{k-1}^B,\, P_k^B]$ with $p_k^B = k L^B / N^B$. Where $P^B = 0$ for the start of the buffer.

$$
\begin{aligned}
t_i^{k,\text{out}} &: p_i(t) = P_k^B + l_i & \text{(trailing edge of } i \text{ leaves segment } k\text{)}\\[2pt]
t_{i+1}^{k,\text{in}} &: p_{i+1}(t) = P_{k-1}^B & \text{(leading edge of } i+1 \text{ enters segment } k\text{)}
\end{aligned}
$$

The **free window** of segment $k$ for the pair $(i, i+1)$ is

$$\mathcal{W}_{i,k} = \bigl[\, t_i^{k,\text{out}},\; t_{i+1}^{k,\text{in}} \,\bigr]$$

— the interval during which segment $k$ carries no object.

## 2. Endpoint kinematic states

By the straddling constraint, during occupancy the segment runs at the occupying input's kinematics. At the endpoints of $\mathcal{W}_{i,k}$ the segment must match:

$$
\begin{aligned}
\bigl(v^-_{i,k},\, a^-_{i,k}\bigr) &= \bigl(\dot p_i,\, \ddot p_i\bigr)\big|_{t_i^{k,\text{out}}} & \text{(state at window start)}\\[2pt]
\bigl(v^+_{i,k},\, a^+_{i,k}\bigr) &= \bigl(\dot p_{i+1},\, \ddot p_{i+1}\bigr)\big|_{t_{i+1}^{k,\text{in}}} & \text{(state at window end)}
\end{aligned}
$$

Sign convention: superscript $-$ for the state *handed off from* input $i$; $+$ for the state *required by* input $i+1$.

## 3. The three matrices

Let $b$ be the batch size and $N^B$ the buffer segment count. All three matrices have shape $(b-1) \times N^B$.

### 3.1 Budget matrix $\mathbf{W}$

The wall-clock time segment $k$ has available to retime itself:

$$
W_{i,k} \;=\; t_{i+1}^{k,\text{in}} - t_i^{k,\text{out}} \;=\; |\mathcal{W}_{i,k}|
$$

Units: seconds. Determined entirely by the committed input trajectories — a property of the *schedule*, independent of how the segment executes.

### 3.2 Cost matrix $\mathbf{C}$

The minimum time required to transition $(v^-_{i,k}, a^-_{i,k}) \to (v^+_{i,k}, a^+_{i,k})$ under the free-window kinematic bounds $(v^F_{\max},\, a^F_{\max},\, j^F_{\max})$ using a chosen primitive $\mathcal{P}$:

$$
C_{i,k}^{\mathcal{P}} \;=\; \tau^\star_{\mathcal{P}}\!\bigl(v^-_{i,k},\, a^-_{i,k},\; v^+_{i,k},\, a^+_{i,k}\bigr)
$$

Units: seconds. A property of the *primitive* and the *free-window motor envelope*, independent of the input trajectories' timing.

Two primitives of interest:

| Primitive | Symbol | Notes |
|---|---|---|
| Cubic-in-velocity | $\mathcal{P}_3$ | Uniquely determined by 4 BCs; linear jerk; preferred (gentler on motors) |
| Bang-bang jerk | $\mathcal{P}_{\text{BB}}$ | Time-optimal lower bound; closed form when $a^- = a^+ = 0$ |

For each primitive, $\mathbf{C}^{\mathcal{P}_3}$ and $\mathbf{C}^{\mathcal{P}_{\text{BB}}}$ are separate matrices. By optimality of bang-bang:

$$
C_{i,k}^{\mathcal{P}_{\text{BB}}} \;\le\; C_{i,k}^{\mathcal{P}_3} \qquad \forall\, (i,k)
$$

### 3.3 Slack matrix $\mathbf{S}$

Element-wise difference:

$$
S_{i,k}^{\mathcal{P}} \;=\; B_{i,k} - C_{i,k}^{\mathcal{P}}
$$

| Value | Meaning |
|---|---|
| $S_{i,k}^{\mathcal{P}} > 0$ | Segment $k$ completes the transition with margin |
| $S_{i,k}^{\mathcal{P}} = 0$ | Transition saturates the free window |
| $S_{i,k}^{\mathcal{P}} < 0$ | No trajectory under $\mathcal{P}$ connects the endpoints within $\mathcal{W}_{i,k}$ |

The ordering $C^{\text{BB}} \le C^{\mathcal{P}_3}$ induces

$$
S^{\mathcal{P}_{\text{BB}}} \ge S^{\mathcal{P}_3} \qquad \text{(element-wise)}
$$

so the bang-bang slack is a feasibility *ceiling* and the cubic slack a feasibility *floor*. Cells with $S_{i,k}^{\mathcal{P}_3} < 0 \le S_{i,k}^{\mathcal{P}_{\text{BB}}}$ are exactly the cells where the design must fall back from cubic to bang-bang.

## 4. Relation to existing signals

- **Gap violation** $\mathbf{1}[\min_t g_i(t) < g_{\min}]$ — proxy observable. Binary, whole-buffer, post-hoc. Superseded by $\mathbf{S}$.
- **Whole-buffer PairRecord window** — recovered as the $N^B = 1$ collapse: $W_{i,1}$ with $k=1$ spanning the entire buffer equals the current `PairRecord` window width.
- **Hard feasibility** of a plan under primitive $\mathcal{P}$: $S_{i,k}^{\mathcal{P}} \ge 0$ for all $(i,k)$.