# Registrar Formalization

## 1. Role

The registrar is the third stage of the pipeline, positioned between the KSB and the downstream carrier. It receives each input $i$ at boundary $\mathcal{B}^{BR}$ traveling at velocity $v^{BR}$ and must deliver it to boundary $\mathcal{B}^{RD}$ traveling at exactly $v_d$.

The registrar's sole task is **velocity equalization**: bringing $v^{BR}$ to $v_d$ across $N_R$ segments in a controlled, jerk-limited manner. The sign of $v^{BR} - v_d$ is not fixed — the buffer generally produces $v^{BR} > v_d$, but after a skip event it may produce $v^{BR} < v_d$. The registrar handles both cases with the same mechanism.

Position error accumulated on the KSB is not corrected here. The registrar receives the object when it arrives, equalizes velocity, and hands it off. Each input is a new problem.

The registrar is decoupled from the KSB except through their shared interface: $v^{BR}$ and $v_d$. The KSB does not need to know how the registrar equalizes velocity; the registrar does not need to know how the KSB produced $v^{BR}$. The buffer solver uses the average registrar transit velocity as a timing input, which is sufficient.

---

## 2. Segment Geometry

The registrar consists of $N_R$ independently-controllable conveyor segments of equal length $L^R / N_R$. Segment $n$ spans from sub-boundary $\mathcal{B}^{R,n-1}$ to $\mathcal{B}^{R,n}$, with $\mathcal{B}^{R,0} \equiv \mathcal{B}^{BR}$ and $\mathcal{B}^{R,N_R} \equiv \mathcal{B}^{RD}$.

Equal segment lengths are a physical design constraint. The kinematic parameters $N_R$, $L^R$, and $j_{\max}^R$ are design parameters fixed at construction time and optimized offline.

The hard geometric constraint is $L^R / N_R > l_i$ for all inputs $i$. If any segment is shorter than the input, the trailing edge never fully clears the entry boundary and control authority is never gained.

---

## 3. The Straddling Constraint and Gap Safety

The gap $g_i(t) = p_i(t) - p_{i+1}(t)$ is defined consistently with *Buffer formalization.md* §2.4, where $p_i(t)$ is the position of input $i$ and $p_{i+1}(t)$ is the position of its follower. The buffer guarantees $g_i(t) \geq g_{\min}$ at $\mathcal{B}^{BR}$ for all pairs. The actual entry gap varies per pair due to upstream variance, but $g_{\min}$ is a hard lower bound inherited from the buffer.

When input $i$'s leading edge crosses sub-boundary $\mathcal{B}^{R,n}$, segment $n$ locks to the current crossing velocity $v^{R,n}$ for the straddling duration:

$$\tau_n = \frac{l_i}{v^{R,n}}$$

Control authority on segment $n+1$ begins only once input $i$'s trailing edge clears $\mathcal{B}^{R,n}$. Since $v^{R,n}$ decreases with $n$ — each segment decelerates the input further — the coast duration $\tau_n$ increases with $n$. Later segments coast for longer.

The safety condition on segment $n$ is: when input $i+1$ gains control authority on segment $n$ — meaning its trailing edge clears $\mathcal{B}^{R,n-1}$ — input $i$ must already be coasting on segment $n$. If input $i$ is still in its active deceleration phase when $i+1$ begins its deceleration, the two inputs are decelerating simultaneously at different velocities, and gap compression may occur.

Since all inputs execute the same position curve on each segment (because $v^{BR}$ is fixed), consecutive inputs are time-shifted copies of each other. The time offset at segment $n$ entry is $\tau_n^i = g_i^{R,n} / v^{R,n-1}$, where $g_i^{R,n}$ is the gap of pair $(i, i+1)$ entering segment $n$. The timing condition is therefore a discrete constraint at each segment boundary, not a continuous gap condition.

The signal variable for post-hoc verification is $\Delta_a^{(i,i+1)}$, the acceleration difference between consecutive inputs computed by the item-pair class. If $\Delta_a^{(i,i+1)} \neq 0$ at any point on the registrar, the timing condition was violated — input $i+1$ began its active deceleration while input $i$ was still in its active phase. The last input in the batch, $i = b$, has no follower and is excluded from this check.

---

## 3.1 Deceleration Weight Design

The safety condition becomes progressively harder to satisfy as $n$ increases, because $\tau_n$ grows and the relative deceleration per segment must also grow — later segments have longer dwell times and therefore a larger correction budget. This coupling makes an analytical solution intractable. The weights are instead determined by offline optimization.

The free variables are the deceleration weights $w_1, w_2, \ldots, w_{N_R}$, subject to:

$$\sum_{n=1}^{N_R} w_n = 1, \qquad w_1 < w_2 < \cdots < w_{N_R}$$

The monotonicity constraint encodes the structural requirement that later segments absorb a larger share of the total deceleration. The exit velocity at segment $n$ is then:

$$v^{R,n} = v^{BR} - \left(\sum_{k=1}^{n} w_k\right) \cdot \Delta v, \qquad \Delta v = v^{BR} - v_d$$

The optimizer finds weights such that $\Delta_a^{(i,i+1)} = 0$ for all pairs and all segments under worst-case conditions, where $g_i^{R,n} = g_{\min}$ for all $n$. This optimization is performed once at construction time and produces design parameters that are valid for all inputs.

If no feasible weight vector exists for the given $N_R$ and $L^R$, an exception is raised. The mechanical design must then be revised by increasing $N_R$, increasing $L^R$, or both.

Note: $v^{BR}$ is treated as a fixed design parameter for this optimization. In principle it is also a free variable, but changing it affects the gap statistics $g_i$ produced by the buffer, requiring a full system simulation to evaluate. It is therefore deferred to the system-level optimization, where buffer and registrar parameters are tuned jointly. That optimization is hierarchical — the registrar weight problem and the buffer parameter problem interact non-linearly and will likely require separate optimizers.

---

## 4. Per-Segment Solver

Each segments runs a solver that outputs a displacement curve achieving boundary velocities, followed by a coasting period. Instances of solvers could be a 3-phase jerk limited S-curve, or a full 5th order polynomial.

| Input           | Value                                                  |
| --------------- | ------------------------------------------------------ |
| Displacement    | $L^R / N_R$                                            |
| Entry velocity  | $v_n$ (exit velocity of segment $n-1$; $v_0 = v^{BR}$) |
| Target velocity | $v_d$                                                  |
| Bounds          | $(j_{\max}^R,\; a_{\max},\; v_{\max})$                 |

The solver applies the maximum feasible correction toward $v_d$ within the segment. Once $v_n = v_d$, subsequent segments coast at $v_d$ — the 0-jerk case is handled natively by the solver and requires no special casing.

The composite registrar solver chains $N_R$ segment solvers sequentially, passing the exit velocity of each as the entry velocity of the next.

---

## 5. Feasibility

Feasibility is a mechanical design guarantee, not a per-input computation. $N_R$ and $j_{\max}^R$ are sized such that any $v^{BR}$ the buffer produces can be equalized to $v_d$ within $L^R$.

The upstream filter enforcing this is the slot assignment logic in the buffer: a candidate slot is only accepted if the resulting $v^{BR}$ is equalizable by the registrar. If not, the slot is rejected. The skip condition and the registrar feasibility bound are therefore coupled design parameters.

---

## 6. Notation Summary

| Symbol         | Type      | Meaning                                                                   |
| -------------- | --------- | ------------------------------------------------------------------------- |
| $N_R$          | design    | Number of registrar segments                                              |
| $L^R$          | design    | Total registrar length                                                    |
| $j_{\max}^R$   | design    | Maximum jerk on registrar segments                                        |
| $v^{BR}$       | per-input | Entry velocity at $\mathcal{B}^{BR}$; produced by KSB                     |
| $v_d$          | system    | Downstream carrier velocity; exit target                                  |
| $v_n$          | per-input | Crossing velocity at $\mathcal{B}^{R,n}$; $v_0 = v^{BR}$, $v_{N_R} = v_d$ |
| $\tau_i^{R,n}$ | per-input | Straddling duration at $\mathcal{B}^{R,n}$                                |
| $l_i$          | per-input | Physical length of input $i$                                              |