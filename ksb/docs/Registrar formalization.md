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

When input $i$'s leading edge crosses $\mathcal{B}^{R,n}$, both segments $n$ and $n+1$ lock to the current crossing velocity for the straddling duration:

$$\tau_i^{R,n} = \frac{l_i}{v_i^{R,n}}$$

where $v_i^{R,n}$ is the velocity at boundary $\mathcal{B}^{R,n}$. Control authority on segment $n+1$ begins only once the trailing edge clears $\mathcal{B}^{R,n}$.

Gap compression is structurally eliminated on the registrar. When two consecutive inputs simultaneously straddle the same sub-boundary, both adjacent segments run at the same velocity — no relative motion, no gap change. The clearance constraint $g_i(t) \geq g_{\min}$ is satisfied by the mechanical design, not enforced by a solver.

---

## 4. Per-Segment Solver

Each segment runs a 4-phase jerk-limited S-curve solver. Segment $n$ receives:

| Input | Value |
|---|---|
| Displacement | $L^R / N_R$ |
| Entry velocity | $v_n$ (exit velocity of segment $n-1$; $v_0 = v^{BR}$) |
| Target velocity | $v_d$ |
| Bounds | $(j_{\max}^R,\; a_{\max},\; v_{\max})$ |

The solver applies the maximum feasible correction toward $v_d$ within the segment. Once $v_n = v_d$, subsequent segments coast at $v_d$ — the 0-jerk case is handled natively by the solver and requires no special casing.

The composite registrar solver chains $N_R$ segment solvers sequentially, passing the exit velocity of each as the entry velocity of the next.

---

## 5. Feasibility

Feasibility is a mechanical design guarantee, not a per-input computation. $N_R$ and $j_{\max}^R$ are sized such that any $v^{BR}$ the buffer produces can be equalized to $v_d$ within $L^R$.

The upstream filter enforcing this is the slot assignment logic in the buffer: a candidate slot is only accepted if the resulting $v^{BR}$ is equalizable by the registrar. If not, the slot is rejected. The skip condition and the registrar feasibility bound are therefore coupled design parameters.

---

## 6. Notation Summary

| Symbol | Type | Meaning |
|---|---|---|
| $N_R$ | design | Number of registrar segments |
| $L^R$ | design | Total registrar length |
| $j_{\max}^R$ | design | Maximum jerk on registrar segments |
| $v^{BR}$ | per-input | Entry velocity at $\mathcal{B}^{BR}$; produced by KSB |
| $v_d$ | system | Downstream carrier velocity; exit target |
| $v_n$ | per-input | Crossing velocity at $\mathcal{B}^{R,n}$; $v_0 = v^{BR}$, $v_{N_R} = v_d$ |
| $\tau_i^{R,n}$ | per-input | Straddling duration at $\mathcal{B}^{R,n}$ |
| $l_i$ | per-input | Physical length of input $i$ |