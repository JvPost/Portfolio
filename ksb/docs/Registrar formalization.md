# Registrar Formalization

## 1. Role

The Registration Stage (registrar) is the third stage of the pipeline, positioned between the KSB and the downstream carrier. It receives each input $i$ at boundary $\mathcal{B}^{BR}$ traveling at $v^{BR} \geq v_d$ and must deliver it to boundary $\mathcal{B}^{RD}$ traveling at exactly $v_d$, synchronized to its assigned downstream slot.

The registrar's primary job is **staged deceleration**: shedding the velocity surplus $v^{BR} - v_d$ across $N_R$ segments in a controlled, jerk-limited manner that does not compress gaps between consecutive inputs.

Position error correction is a secondary consequence of the same mechanism. Each segment decelerates by a nominal amount $\Delta V = (v_d - v^{BR}) / N_R$. By adjusting each segment's deceleration slightly above or below $\Delta V$, the registrar advances or retards the input relative to its assigned slot, absorbing the position error $\Delta p_i$ accumulated on the KSB. Deceleration and correction are not separate tasks sharing a budget — they are the same task, with the per-segment deceleration amount as the single control variable.

When $v^{BR} = v_d$, the deceleration task vanishes and the registrar degenerates to a pure position corrector, applying small symmetric S-curves with zero net velocity change.

---

## 2. Segment Geometry

The registrar consists of $N_R$ independently-controllable conveyor segments with lengths $L^{R,1}, \ldots, L^{R,N_R}$ summing to $L^R$. Each segment $n$ spans from sub-boundary $\mathcal{B}^{R,n-1}$ to $\mathcal{B}^{R,n}$, with $\mathcal{B}^{R,0} \equiv \mathcal{B}^{BR}$ and $\mathcal{B}^{R,N_R} \equiv \mathcal{B}^{RD}$.

The nominal crossing velocity at each sub-boundary under uniform deceleration allocation is:

$$v^{R,n} = v^{BR} + n \cdot \Delta V, \qquad \Delta V = \frac{v_d - v^{BR}}{N_R} \leq 0$$

Segment $n$ receives inputs at $v^{R,n-1}$ and nominally delivers them at $v^{R,n}$.

### 2.1 Segment Length Distribution

Because each segment exits at a lower velocity than it entered, inputs traverse upstream segments faster than downstream ones. Equal-length segments would give downstream segments more dwell time and therefore more correction capacity, distributing the correction budget unevenly. To equalize dwell time per segment, lengths decrease monotonically:

$$L^{R,1} > L^{R,2} > \cdots > L^{R,N_R}$$

The average transit velocity on segment $n$ is:

$$\bar{v}^{R,n} \approx v^{BR} + \left(n - \tfrac{1}{2}\right)\Delta V$$

Equal-dwell-time segment lengths follow proportionally:

$$L^{R,n} = L^R \cdot \frac{\bar{v}^{R,n}}{\sum_{k=1}^{N_R} \bar{v}^{R,k}}$$

The hard geometric constraint is $L^{R,n} > l_i$ for all $n$. If any segment is shorter than the input, the trailing edge never fully clears the entry boundary and control authority is never gained.

---

## 3. The Straddling Constraint and Gap Safety

When input $i$'s leading edge crosses $\mathcal{B}^{R,n}$, both segments $n$ and $n+1$ lock to velocity $v^{R,n}$ for the straddling duration:

$$\tau_i^{R,n} = \frac{l_i}{v^{R,n}}$$

Control authority on segment $n+1$ begins only once the trailing edge clears $\mathcal{B}^{R,n}$ at $t_i^{R,n,\text{trail}} = t_i^{R,n,\text{lead}} + \tau_i^{R,n}$.

Gap compression is structurally eliminated on the registrar. When two consecutive inputs simultaneously straddle the same sub-boundary, both adjacent segments are running at the same velocity $v^{R,n}$ — no relative motion, no gap change. The clearance constraint $g_i(t) \geq g_{\min}$ is satisfied by the mechanical design, not enforced by a solver.

---

## 4. Per-Input Timing

The total registrar time window for input $i$ is fixed by the slot:

$$T_i^R = t_i^{\text{slot}} - t_i^{R,\text{in}}$$

where $t_i^{\text{slot}}$ is the time the assigned slot arrives at $\mathcal{B}^{RD}$ and $t_i^{R,\text{in}}$ is the time the trailing edge of input $i$ clears $\mathcal{B}^{BR}$. This decomposes into control windows and straddling windows:

$$T_i^R = \sum_{n=1}^{N_R} T_i^{R,n} + \sum_{n=1}^{N_R - 1} \tau_i^{R,n}$$

Since straddling durations are fixed design constants, the available control time is:

$$\sum_{n=1}^{N_R} T_i^{R,n} = T_i^R - \sum_{n=1}^{N_R - 1} \frac{l_i}{v^{R,n}}$$

---

## 5. Per-Segment Solver

Each registrar segment runs the same 7-phase jerk-limited S-curve solver as the KSB (*Buffer formalization.md*, §2.4). For segment $n$ the solver receives:

| Input | Value |
|---|---|
| Displacement | $\Lambda^{R,n}$ |
| Entry velocity | $v^{R,n-1}$ |
| Exit velocity | $v^{R,n} + \epsilon_i^n$ |
| Time horizon | $T_i^{R,n}$ |
| Bounds | $(j_{\max},\; a_{\max},\; v_{\max}^R)$ |

where $\epsilon_i^n$ is the per-segment correction offset (Section 6) and $\Lambda^{R,n} = L^{R,n} - l_i$ is the effective control length.

The solver finds a peak velocity $v_{\text{peak}}^{R,n} \in [\min(v^{R,n-1}, v^{R,n} + \epsilon_i^n),\; v_{\max}^R]$ via bisection — identical to the KSB solver. The profile is asymmetric when $v^{R,n-1} \neq v^{R,n} + \epsilon_i^n$.

When $\epsilon_i^n = 0$ and $\Delta V = 0$ the profile collapses to a pure coast. When $\epsilon_i^n = 0$ and $\Delta V \neq 0$ it is a pure deceleration ramp.

---

## 6. Position Correction via Deceleration Modulation

Position error $\Delta p_i$ is defined at $\mathcal{B}^{BR}$ (see *KSB System — Formalization.md*, §6). The registrar corrects it by adjusting each segment's exit velocity slightly above or below the nominal $v^{R,n}$:

$$\epsilon_i^n = \text{correction offset for segment } n$$

A positive $\epsilon_i^n$ means segment $n$ decelerates less than nominal — the input exits faster, gains time relative to the slot, and advances in position. A negative $\epsilon_i^n$ means more deceleration than nominal — the input is retarded. The total position correction must equal $-\Delta p_i$:

$$\sum_{n=1}^{N_R} \epsilon_i^n \cdot T_i^{R,n} \approx -\Delta p_i$$

This is an approximation because the relationship between exit velocity offset and position gain depends on the profile shape. The exact relationship is computable from the S-curve geometry but is treated as a linear approximation for the purposes of the orchestrator.

The simplest allocation is uniform: $\epsilon_i^n = \epsilon_i$ for all $n$, with $\epsilon_i$ chosen to satisfy the total correction requirement. More sophisticated allocations are deferred.

The key insight is that deceleration and correction are not separate tasks. The nominal exit velocity $v^{R,n}$ is the deceleration target; $\epsilon_i^n$ is the signed deviation from it. One control variable per segment handles both.

---

## 7. Feasibility

Each segment has a maximum achievable exit velocity deviation $\epsilon_{\max}^n$, set by the kinematic bounds and the segment geometry. The total correction budget is:

$$\Delta p_{\max}^{\text{total}} = f\!\left(\epsilon_{\max}^1, \ldots, \epsilon_{\max}^{N_R},\; T_i^{R,1}, \ldots, T_i^{R,N_R}\right)$$

Feasibility requires $|\Delta p_i| \leq \Delta p_{\max}^{\text{total}}$ for all $i$. When exceeded, the orchestrator applies the maximum correction the budget allows and logs the residual.

---

## 8. Notation Summary

| Symbol | Type | Meaning |
|---|---|---|
| $N_R$ | design | Number of registrar segments |
| $L^{R,n}$ | design | Length of segment $n$; decreasing in $n$ |
| $v^{R,n}$ | design | Nominal crossing velocity at $\mathcal{B}^{R,n}$; $v^{R,0} = v^{BR}$, $v^{R,N_R} = v_d$ |
| $\Delta V$ | design | Nominal velocity change per segment: $(v_d - v^{BR})/N_R \leq 0$ |
| $\Lambda^{R,n} = L^{R,n} - l_i$ | derived | Effective control length of segment $n$ |
| $\tau_i^{R,n} = l_i / v^{R,n}$ | derived | Straddling duration at $\mathcal{B}^{R,n}$ |
| $T_i^R$ | per-input | Total registrar time window; fixed by slot timing |
| $T_i^{R,n}$ | per-input | Control window on segment $n$ |
| $\Delta p_i$ | per-input | Position error at $\mathcal{B}^{BR}$; produced by KSB |
| $\epsilon_i^n$ | per-input | Exit velocity offset from nominal on segment $n$ |
| $\Delta p_{\max}^{\text{total}}$ | derived | Total correction budget |
| $v_{\text{peak}}^{R,n}$ | per-input | Peak velocity on segment $n$; found by bisection |