# KSB System — Formalization

## 1. Stages and Design Parameters

The pipeline consists of four stages in series:

| Stage        | Label | Length | Nominal exit velocity |
| ------------ | ----- | ------ | --------------------- |
| Upstream     | $U$   | $L^U$  | $v^{UB}$              |
| KSB (buffer) | $B$   | $L^B$  | $v^{BR}$              |
| Registration | $R$   | $L^R$  | $v^{RD}$              |
| Downstream   | $D$   | $L^D$  | $v^{RD}$              |

where $v^{UB} \equiv v_u$ is the upstream belt speed and $v^{RD} \equiv v_d$ is the downstream belt speed. These are retained as $v_u$ and $v_d$ where the context is unambiguous. $v^{BR}$ is a free design parameter — the KSB exit velocity — decoupled from both $v_u$ and $v_d$.

The registration stage is subdivided into $N_R$ segments with lengths $L^{R,1}, \ldots, L^{R,N_R}$ summing to $L^R$, with monotonically decreasing lengths to equalize dwell time (Section 5).

Input length is $l_i$ (uniform across batch, written $l$ where unambiguous).

---

## 2. Stage Boundaries

A **stage boundary** $\mathcal{B}^k$ is the interface between two adjacent stages. It has a fixed absolute position and a required crossing velocity:

$$\mathcal{B}^k = (P^k,\; v^k)$$

| Boundary | Position | Crossing velocity |
|---|---|---|
| $\mathcal{B}^{UB}$ | $P^{UB} = L^U$ | $v^{UB} = v_u$ |
| $\mathcal{B}^{BR}$ | $P^{BR} = L^U + L^B$ | $v^{BR}$ |
| $\mathcal{B}^{R,n}$ | $P^{R,n} = P^{BR} + \sum_{k=1}^{n} L^{R,k}$ | $v^{R,n}$ |
| $\mathcal{B}^{RD}$ | $P^{RD} = P^{BR} + L^R$ | $v^{RD} = v_d$ |

Every crossing velocity is a **design constant**. Setting it determines both the exit constraint of the upstream stage and the entry constraint of the downstream stage simultaneously.

For registration sub-boundaries under uniform velocity allocation:

$$v^{R,n} = v^{BR} + n \cdot \Delta V, \qquad \Delta V = \frac{v_d - v^{BR}}{N_R} \leq 0$$

---

## 3. The Straddling Constraint

When input $i$ straddles boundary $\mathcal{B}^k$ — leading edge past $P^k$, trailing edge before it — both adjacent belts must run at $v^k$. This is a hard mechanical constraint imposed by the input's rigidity.

- **Leading edge arrives** at $P^k$ at time $t_i^{k,\text{lead}}$. Both belts lock at $v^k$.
- **Straddling duration**: $\tau_i^k = l_i / v^k$
- **Trailing edge clears** at $t_i^{k,\text{trail}} = t_i^{k,\text{lead}} + \tau_i^k$. The downstream stage gains full control authority at this moment.

During the straddling window:

$$v_i(t) = v^k \qquad \forall\, t \in [t_i^{k,\text{lead}},\; t_i^{k,\text{trail}}]$$

---

## 4. Per-Input Control Windows

For each input $i$ on stage $S$:

| Symbol | Meaning |
|---|---|
| $t_i^{S,\text{in}}$ | Time trailing edge clears the entry boundary — **control starts** |
| $t_i^{S,\text{out}}$ | Time leading edge reaches the exit boundary — **control must be complete** |
| $T_i^S = t_i^{S,\text{out}} - t_i^{S,\text{in}}$ | Control window duration |
| $\Lambda^S = L^S - l_i$ | Effective control length |

The solver for stage $S$ on input $i$ receives:

$$\left(\Lambda^S,\; v^{k_\text{in}},\; v^{k_\text{out}},\; T_i^S,\; \text{bounds}\right)$$

where $v^{k_\text{in}}$ and $v^{k_\text{out}}$ are the crossing velocities of the entry and exit boundaries of stage $S$. These are design constants — the solver never has to discover them.

For registration segment $n$:

$$\Lambda^{R,n} = L^{R,n} - l_i, \qquad v^{k_\text{in}} = v^{R,n-1}, \qquad v^{k_\text{out}} = v^{R,n}$$

with the hard requirement $L^{R,n} > l_i$ for all $n$.

---

## 5. Entry State at Each Stage

The **entry state** $\mathbf{x}_i^S$ is the kinematic state of input $i$ at $t_i^{S,\text{in}}$:

$$\mathbf{x}_i^S = \begin{bmatrix} 0 \\ v^{k_\text{in}} \\ 0 \end{bmatrix}$$

- Position is zero (delta semantics — each stage uses its own local frame).
- Velocity is exactly $v^{k_\text{in}}$ by the straddling constraint.
- Acceleration is zero because the straddling window is at constant velocity.

This means **every solver receives $a = 0$ at entry and must deliver $a = 0$ at exit** — structurally guaranteed by the straddling constraint, not a convenience assumption.

---

## 6. Position Error and Registration

The position error $\Delta p_i$ is defined at $\mathcal{B}^{BR}$:

$$\Delta p_i = p_i^{\text{actual}}\!\left(t_i^{B,\text{out}}\right) - p_i^{\text{target}}$$

where $p_i^{\text{target}}$ is the position of the assigned slot at $t_i^{B,\text{out}}$.

The registration stage distributes the correction across $N_R$ segments:

$$\sum_{n=1}^{N_R} \delta_i^n = -\Delta p_i$$

where $\delta_i^n$ is the displacement correction applied by segment $n$. The solver for segment $n$ covers displacement $\Lambda^{R,n} + \delta_i^n$ with net velocity change $\Delta V$ within window $T_i^{R,n}$.

The per-segment correction capacity $\delta_{\max}^n$ is a closed-form function of $(\Lambda^{R,n},\; v^{R,n-1},\; \Delta V,\; j_{\max},\; a_{\max})$. Feasibility requires:

$$|\Delta p_i| \leq \sum_{n=1}^{N_R} \delta_{\max}^n$$

---

## 7. Notation Summary

| Symbol | Type | Meaning |
|---|---|---|
| $L^S$ | design | Length of stage $S$ |
| $N_R$ | design | Number of registration segments |
| $v^k$ | design | Crossing velocity at boundary $\mathcal{B}^k$ |
| $v_u \equiv v^{UB}$ | design | Upstream belt speed |
| $v^{BR}$ | design | KSB exit velocity — free tuning parameter |
| $v_d \equiv v^{RD}$ | design | Downstream belt speed |
| $\Delta V$ | design | Net velocity change per registration segment |
| $l_i$ | input | Physical length of input $i$ |
| $\Lambda^S$ | derived | Effective control length of stage $S$: $L^S - l_i$ |
| $\tau_i^k$ | derived | Straddling duration at boundary $k$: $l_i / v^k$ |
| $t_i^{S,\text{in}}$ | per-input | Control-start time for input $i$ on stage $S$ |
| $t_i^{S,\text{out}}$ | per-input | Control-end time (leading edge at exit boundary) |
| $T_i^S$ | per-input | Control window duration |
| $\mathbf{x}_i^S$ | per-input | Entry state $[0,\; v^{k_\text{in}},\; 0]^\top$ |
| $\Delta p_i$ | per-input | Position error at $\mathcal{B}^{BR}$ |
| $\delta_i^n$ | per-input | Displacement correction by registration segment $n$ |