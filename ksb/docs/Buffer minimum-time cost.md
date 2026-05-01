# Buffer minimum-time cost

This document is specific to the bang-bang policy: $\pi = \pi_{\text{BB}}$ throughout. Where unambiguous the policy index is suppressed.

Derivation of $C^\pi_\min(v^-, a^-, v^+, a^+)$ — the minimum transition time of the time-optimal bang-bang jerk primitive, under the assumption that the buffer envelope's $a^B_\max, v^B_\max$ are not reached. Only the jerk bound $j^B_\max$ is active.

## 1. Setup

### 1.1 Problem statement

Given endpoint kinematic states $(v^-, a^-)$ and $(v^+, a^+)$ at the endpoints of a free window, find the minimum-time jerk-bounded trajectory connecting them:

$$
\min_{\tau,\, j(t)} \;\tau \quad \text{s.t.}\quad |j(t)| \le j^B_\max,\; \dot a = j,\; \dot v = a,\; \text{BCs met.}
$$

Position is not a constraint — the buffer's primitive serves only to retime, not reposition.

### 1.2 Assumption: jerk-only bound

The buffer envelope nominally has $(j^B_\max, a^B_\max, v^B_\max)$, but for the KSB operating range the motor limits $a^B_\max, v^B_\max$ are set higher than any realizable transition demands. Within the range of $(v^-, a^-, v^+, a^+)$ values produced by the upstream controller and the schedule, the inequality $|a(t)| < a^B_\max$ and $|v(t)| < v^B_\max$ holds strictly along any time-optimal profile. Only $j^B_\max$ is active.

Consequence: the time-optimal trajectory is the unsaturated two-phase bang-bang profile — jerk at $\pm j^B_\max$ in two phases, with a single phase boundary.

### 1.3 Profile structure

Let $\Delta v = v^+ - v^-$. The profile has two phases with jerk magnitudes $j^B_\max$ and opposite signs; write:

$$
j(t) \;=\; \begin{cases} +j^B_\max \cdot \sigma & t \in [0, \tau_1) \\ -j^B_\max \cdot \sigma & t \in [\tau_1, \tau_1 + \tau_2] \end{cases}, \qquad \sigma \in \{+1, -1\}
$$

The sign $\sigma = +1$ is the **up case**: acceleration rises from $a^-$ to a peak $a_p$ and descends to $a^+$, with $a_p \ge \max(a^-, a^+)$.

The sign $\sigma = -1$ is the **down case**: acceleration dips to a trough $a_p$ and rises to $a^+$, with $a_p \le \min(a^-, a^+)$.

Time-optimality selects whichever case is feasible and smaller; both can be feasible simultaneously, in which case the formulae agree by construction.

## 2. Up case derivation

Impose the peak-acceleration constraint at the phase boundary:

$$
a_p \;=\; a^- + j^B_\max\, \tau_1 \;=\; a^+ + j^B_\max\, \tau_2
$$

so

$$
\tau_1 \;=\; \frac{a_p - a^-}{j^B_\max}, \qquad \tau_2 \;=\; \frac{a_p - a^+}{j^B_\max}.
$$

Non-negativity of $\tau_1, \tau_2$ requires $a_p \ge \max(a^-, a^+)$, consistent with the up-case definition.

Integrate acceleration over both phases. Acceleration is piecewise-linear; each phase contributes a trapezoidal area to $\Delta v$:

$$
\Delta v \;=\; \tfrac{1}{2}(a^- + a_p)\tau_1 \;+\; \tfrac{1}{2}(a_p + a^+)\tau_2.
$$

Substituting $\tau_1, \tau_2$ and multiplying by $2 j^B_\max$:

$$
2 j^B_\max\, \Delta v \;=\; (a^- + a_p)(a_p - a^-) + (a_p + a^+)(a_p - a^+) \;=\; 2 a_p^2 - a^{-2} - a^{+2}.
$$

Solve for $a_p$:

$$
a_p \;=\; +\sqrt{\;j^B_\max\, \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2})\;}.
$$

(Positive root; negative root is the down case, treated below.)

Total time:

$$
C^\pi_\uparrow \;=\; \tau_1 + \tau_2 \;=\; \frac{2 a_p - a^- - a^+}{j^B_\max}.
$$

Feasibility: real $a_p$ requires $j^B_\max\, \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2}) \ge 0$; non-negativity of $\tau_1, \tau_2$ requires $a_p \ge \max(a^-, a^+)$. Both conditions hold whenever $\Delta v$ is large enough positive, or when $a^{-2} + a^{+2}$ is large enough to dominate.

## 3. Down case derivation

By the symmetry $j \to -j,\, a \to -a,\, v \to -v$, and writing the trough as $a_p \le \min(a^-, a^+)$:

$$
a_p \;=\; -\sqrt{\;-j^B_\max\, \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2})\;}.
$$

$$
C^\pi_\downarrow \;=\; \frac{-2 a_p + a^- + a^+}{j^B_\max}.
$$

Feasibility: real $a_p$ requires $-j^B_\max\, \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2}) \ge 0$, i.e., $\Delta v$ sufficiently negative or $a^{-2} + a^{+2}$ large.

## 4. Combined formula

$$
\boxed{\;C^\pi_\min(v^-, a^-, v^+, a^+) \;=\; \min\bigl\{\, C^\pi_\uparrow,\; C^\pi_\downarrow \,\bigr\}\;}
$$

where infeasible cases are excluded. In the generic KSB operating regime one case is strictly feasible and one strictly infeasible — the boundary $\Delta v = 0$ with $a^- = a^+$ is where both collapse to $C = 0$.

### 4.1 Reduction: $a^\pm = 0$

Up case: $a_p = \sqrt{j^B_\max\, \Delta v}$, feasible when $\Delta v \ge 0$, gives $C = 2\sqrt{\Delta v / j^B_\max}$.

Down case: $a_p = -\sqrt{-j^B_\max\, \Delta v}$, feasible when $\Delta v \le 0$, gives $C = 2\sqrt{-\Delta v / j^B_\max} = 2\sqrt{|\Delta v| / j^B_\max}$.

Merged:

$$
C^\pi_\min(v^-, 0, v^+, 0) \;=\; 2\sqrt{\frac{|v^+ - v^-|}{j^B_\max}}.
$$

## 5. Properties

### 5.1 Scaling

$C$ grows as $\sqrt{|\Delta v|}$ for $|a^\pm|$ small — quadrupling the velocity jump only doubles the required time. Large jumps are cheaper per unit $\Delta v$ than small ones.

For large $|a^\pm|$ (relative to $\sqrt{j^B_\max |\Delta v|}$), the $a^{-2} + a^{+2}$ term dominates under the square root and $C$ scales linearly with $|a^\pm|/j^B_\max$. Endpoint acceleration mismatch, not velocity mismatch, dominates cost in this regime.

### 5.2 Invariants

$C$ depends only on $\Delta v$ and the pair $(a^-, a^+)$, not on $v^-, v^+$ individually. A handoff $2.0 \to 2.5$ m/s at given $(a^-, a^+)$ costs the same as $0.5 \to 1.0$ m/s at the same $(a^-, a^+)$.

$C$ is symmetric in $(a^-, a^+)$: swapping endpoint accelerations does not change cost (the profile is time-reversible under the jerk-only bound).

### 5.3 Smoothness and gradient behaviour

$C$ is smooth in $(v^-, a^-, v^+, a^+)$ wherever the chosen case is strictly feasible. The square root gives:

$$
\frac{\partial C^\pi_\uparrow}{\partial \Delta v} \;=\; \frac{1}{a_p} \cdot 1 \;=\; \frac{1}{\sqrt{j^B_\max \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2})}}, \qquad \frac{\partial C^\pi_\uparrow}{\partial a^-} \;=\; \frac{a^-/a_p - 1}{j^B_\max}, \; \text{etc.}
$$

- $\partial C / \partial \Delta v > 0$: increasing velocity mismatch always increases cost. Matches intuition.
- $\partial C / \partial \Delta v \to \infty$ at $a_p \to 0$ — the cusp at $(\Delta v, a^\pm) = 0$. Propagates to $\mathcal{L}$ only in the infeasible region ($W < C$); in the feasible region $\Phi = 0$ and the cusp is gradient-invisible.
- Case switching at $\Delta v = 0,\, a^- = a^+$ is $C^0$ smooth — both cases give $C = 0$ at the switch point. $C^1$ continuity at the switch is inherited from the symmetric construction.

### 5.4 Peak acceleration along the profile

$|a_p|$ from either case is the peak acceleration reached. The jerk-only-bound assumption (§1.2) requires

$$
|a_p| \;\le\; a^B_\max
$$

for the derivation to be self-consistent. In practice this should be checked post-hoc as a sentinel — a violation indicates that the acceleration bound is being approached and the 3-phase (a-saturated) extension is needed.

### 5.5 Peak velocity along the profile

When $a^-, a^+$ are small, velocity is monotone between $v^-$ and $v^+$ during the profile, so peak $|v|$ equals $\max(|v^-|, |v^+|)$. When $a^-, a^+$ have opposite signs, velocity can overshoot beyond the endpoint interval — the overshoot magnitude is bounded by integration of the profile but is small under the jerk-only regime. Post-hoc check against $v^B_\max$ is a sentinel analogous to §5.4.

## 6. Adaptive jerk and primitive existence

The combined formula in §4 is a partial function: $C^\pi_\min = +\infty$ on cells where neither up-case nor down-case is feasible at the given $j^B_\max$. This forces ad-hoc penalty handling downstream (cf. *Per-Segment Synchronization Slack* §3.3, §4.4). This section extends $C^\pi$ to a total function by computing the cost at the *minimum jerk that would render the primitive feasible*, when $j^B_\max$ does not suffice.

### 6.1 Required jerk $j_\text{req}$

Up-case feasibility requires $a_p \ge \max(a^-, a^+) =: M$, equivalently

$$
j\, \Delta v + \tfrac{1}{2}(a^{-2} + a^{+2}) \;\ge\; M^2.
$$

(The discriminant condition $a_p^2 \ge 0$ is subsumed when $M \ge 0$.) When $M > 0$ and $\Delta v > 0$, the minimum $j$ that satisfies this is

$$
j^\uparrow_\text{req} \;=\; \frac{M^2 - \tfrac{1}{2}(a^{-2} + a^{+2})}{\Delta v} \;=\; \frac{M^2 - m^2}{2\, \Delta v}, \qquad m := \min(a^-, a^+),
$$

using $M^2 - \tfrac{1}{2}(M^2 + m^2) = \tfrac{1}{2}(M^2 - m^2)$. When $M \le 0$ the peak constraint is trivially satisfied (since $a_p = +\sqrt{\cdot} \ge 0 \ge M$), giving $j^\uparrow_\text{req} = 0$. Down-case is symmetric — trough constraint $a_p \le m$ binds only when $m < 0$:

$$
j^\downarrow_\text{req} \;=\; \frac{m^2 - M^2}{-2\, \Delta v} \quad (\Delta v < 0).
$$

The two cases collapse to a single signed expression. For up case ($\Delta v > 0$): $(M^2 - m^2)/(2\Delta v)$ is the formula directly. For down case ($\Delta v < 0$): $(m^2 - M^2)/(-2\Delta v) = (M^2 - m^2)/(2\Delta v)$ — same expression, with $\Delta v$ signed throughout. The unified form is therefore:

$$
\boxed{\; j_\text{req}(v^-, a^-, v^+, a^+) \;=\; \max\!\left\{\,0,\; \frac{M^2 - m^2}{2\, \Delta v}\,\right\} \;}
$$

where $\Delta v = v^+ - v^-$ is signed. The $\max\{0, \cdot\}$ handles all four trivial branches automatically: if up case applies ($\Delta v > 0$) but $M \le 0$, the peak constraint is structurally satisfied — the numerator is non-positive (since $M^2 \le m^2$ when both are non-positive doesn't quite hold, but the case-feasibility takes over before then; see below). If down case applies ($\Delta v < 0$) but $m \ge 0$, similarly trivial. The sign combinations work out so that the numerator-over-denominator construction is positive only in the genuinely binding regimes.

**Reading.** $j_\text{req}$ is "endpoint-asymmetry over signed velocity-change". Symmetric endpoints ($a^- = a^+$) yield $j_\text{req} = 0$ — the symmetric profile satisfies the peak constraint at any positive jerk. Vanishing $|\Delta v|$ with asymmetric endpoints yields $j_\text{req} \to \infty$ — a tall peak with no velocity gain demands arbitrarily fast jerk.

**Verification by counter-example.** Two cells from the implementation discussion:

- $(a^- = 3, a^+ = 0, \Delta v = -2)$: $M = 3$, $m = 0$. Numerator $9 > 0$, denominator $-4 < 0$, ratio $= -2.25$. $\max\{0, \cdot\} = 0$. Correct: down case applies, $m = 0$, trough constraint trivially satisfied.
- $(a^- = 1, a^+ = -3, \Delta v = -2)$: $M = 1$, $m = -3$. Numerator $1 - 9 = -8 < 0$, denominator $-4 < 0$, ratio $= 2$. $\max\{0, \cdot\} = 2$. Correct: down case applies, $m < 0$, trough constraint binding.
### 6.2 Adaptive policy $\pi^\text{adj}$

Define the policy with adaptive jerk:

$$
C^{\pi^\text{adj}}_\min(v^-, a^-, v^+, a^+) \;:=\; C^\pi_\min\bigl(v^-, a^-, v^+, a^+;\; \tilde{j}\bigr), \qquad \tilde{j} \;:=\; \max\bigl(j^B_\max,\; j_\text{req}\bigr).
$$

Properties:

- $j_\text{req} \le j^B_\max$: $\tilde{j} = j^B_\max$ and $C^{\pi^\text{adj}} = C^\pi$ — no change in the standard regime.
- $j_\text{req} > j^B_\max$: $\tilde{j} = j_\text{req}$ and $C^{\pi^\text{adj}}$ is the bang-bang minimum time computed *as if* the jerk bound were tight. Finite and well-defined.
- $C^{\pi^\text{adj}}$ is total: defined on all $(v^-, a^-, v^+, a^+)$. The discriminant-sentinel regime in *Per-Segment Synchronization Slack* §3.3 is structurally empty under $\pi^\text{adj}$.
- Continuous at the regime boundary $j_\text{req} = j^B_\max$ — both branches evaluate the same closed form at the same $j$. $C^0$, with a kink in $\partial C / \partial \theta$ across the boundary; smoothness within each regime is inherited from §5.3. Acceptable for derivative-free optimization (CMA-ES); would need attention for gradient-based methods.

### 6.3 Diagnostic: jerk overspec

The quantity

$$
\Delta j \;:=\; \max\{0,\; j_\text{req} - j^B_\max\}
$$

reads as "by how much the jerk bound would need to be raised to render this transition feasible at $j^B_\max$". It is a post-hoc diagnostic, not an optimization signal: $j^B_\max$ is a motor-spec parameter held fixed across the design optimization, and the slack pressure on the segment-length / pitch-shaping parameters already drives the optimizer toward configurations where $j_\text{req} \le j^B_\max$ (cf. *Per-Segment Synchronization Slack* §3.5). Cells with $\Delta j > 0$ at the optimum identify transitions for which the actuator is undersized; sums or maxima across the cell grid quantify the jerk overspec a robustness-minded redesign of the motor would need.

The peak/velocity-bound sentinels from §5.4–§5.5 remain — those are distinct regimes (a-saturated, v-saturated extensions) and are unaffected by the adaptive jerk extension.

## 7. Implementation notes

Numerical evaluation is stable. The discriminant under the square root is strictly positive throughout the operating regime (and made so by construction in $\pi^\text{adj}$ when needed) and is bounded away from zero except at the exact case-switching boundary. Near the boundary ($\Delta v \approx 0,\, a^- \approx a^+ \approx 0$), both $C_\uparrow$ and $C_\downarrow$ tend to zero smoothly; an implementation can compute both and take the min without branching.

The function evaluates in $O(1)$. For an $(b{-}1) \times N^B$ cell grid, vectorized evaluation over BC arrays is trivial in NumPy/JAX. The adaptive extension adds one $\max$ over $j$ per cell.

---

*Extensions beyond the jerk-only regime (3-phase a-saturated, 7-phase v-saturated) are deferred until a sentinel from §5.4 or §5.5 indicates they are needed in the operating envelope.*