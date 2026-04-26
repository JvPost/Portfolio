# KSB System — Design Narrative

## 1. The Synchronization Problem

A packaging line delivers discrete inputs from an upstream conveyor at a roughly constant spacing with real variability — inputs arrive slightly early or late relative to the mean. A downstream carrier runs at fixed velocity $v_d$ with evenly spaced slots at spacing $d_s$. Between them sits a servo-driven buffer conveyor whose job is to take each irregularly timed input and deliver it into the correct downstream slot at exactly the right moment.

The upstream runs at velocity $v_u$ and delivers inputs at mean spacing $\mu_u$ with standard deviation $\sigma_u$, giving an arrival rate $r_u = v_u / \mu_u$. The downstream slot rate is $r_d = v_d / d_s$. The system operates with $\rho = r_u / r_d < 1$ — the upstream is slightly slower, so some slots will always go unfilled. When the buffer determines that an input cannot feasibly reach the next available slot, it skips that slot and targets the one after it.

The performance condition is **synchronization feasibility**: across the batch, every internal segment of the buffer must be able to execute the required kinematic state transition between consecutive inputs within the time window the schedule provides. The full system definition, the per-segment slack apparatus that formalizes this condition, and the upstream skip mechanism that drives variance in segment boundary conditions are developed in **Buffer formalization.md** and **Per-Segment Synchronization Slack.md**. The minimum-clearance requirement $g_i(t) \geq g_{\min}$ that motivated earlier framings is absorbed structurally into the segment-length parametrization rather than tracked as a runtime curve constraint.

---

## 2. Why a Single Buffer Stage Isn't Enough

The buffer does per-input slot assignment via jerk-limited motion profiles. Each input gets a different trajectory — it targets a different slot, requires a different correction magnitude, and covers a different velocity envelope. This per-input asymmetry is the buffer's job; it cannot be avoided. But it concentrates feasibility pressure at two specific places — the buffer's two edge boundaries — and each calls for a different intervention.

The two edges have a structural feature in common: each one is an interface where the buffer cannot negotiate one of the boundary conditions. At the upstream edge, the leading-edge state of an arriving input is set by whatever the upstream stage hands over — the buffer inherits it. At the registrar edge, the trailing-edge state of a departing input must match what the next stage demands — the buffer is required to deliver it. Interior segments have two BCs they fully control through their own planning; edge segments each have one external BC pinned by the world. That asymmetry is the structural reason edge segments are where things go wrong.

### 2.1 Boundary-Condition Variance at the Upstream Edge

When input $i+1$ enters the buffer, its leading-edge state — velocity and acceleration — is determined by what the upstream stage handed over. The first segment of the buffer (segment 1) inherits this state as a boundary condition and has to execute its synchronization with whatever it gets. If the upstream is running steadily and the input arrives on the nominal mean spacing, the BC is nominal too; if the upstream has just been disturbed by a skip event or by stochastic spacing variance, the BC is off-nominal. Skips are the dominant source of disturbance: the upstream feedforward control modulates belt velocity around each skip to shape the post-skip arrival state, but the residual variance lands directly on segment 1's receiving boundary condition.

This is not a regime-specific issue. The mean rate of skip events is set by the load ratio $1 - \rho$ — a structural invariant — independent of upstream variability $\sigma_u$. What $\sigma_u$ controls is dispersion: at low $\sigma_u$, skips are nearly periodic and segment 1 sees the same BC profile every $Q$ inputs. At high $\sigma_u$, skip timing becomes stochastic and the BC variance scatters irregularly across batch position. In both regimes, slack pressure concentrates on segment 1 — only the temporal pattern differs.

### 2.2 Boundary-Condition Mismatch at the Registrar Edge

Every input — not just post-skip ones — must hand off to the downstream carrier at velocity $v_d$. If the buffer is solely responsible for that velocity equalization, the last segment (segment $N^B$) has to deliver every input at $(v^+, a^+) = (v_d, 0)$ at the buffer's exit boundary, while having received it at whatever peak velocity its trajectory required to hit the assigned slot. The terminal BC is pinned externally — the buffer cannot negotiate it — and segment $N^B$'s feasibility budget is consumed by a deceleration that varies in magnitude across inputs because each trajectory carries a different peak velocity.

The root cause is trajectory asymmetry. Consecutive inputs have different velocity profiles because they target different slots, so the demanded BC transition at segment $N^B$ varies across the batch in both endpoint state and time horizon. The slack pressure is structural, not event-driven: every input pair sees it. Spreading the deceleration over a longer window — using a smaller deceleration rate — does not solve it; the time integral of the velocity mismatch is set by the velocity difference $v_{\text{peak}} - v_d$, not by the deceleration rate. You cannot fix an asymmetric terminal BC by making the deceleration gentler; you can only fix it by handing the symmetric tail of the deceleration off to a separate stage.

---

## 3. The Three-Subsystem Architecture

The two failure modes have different causes and require different interventions. Upstream-edge BC variance is driven by skip transients in the upstream arrival stream. Registrar-edge BC mismatch is driven by the buffer's need to decelerate each input back to $v_d$. The architecture addresses each with a dedicated subsystem: upstream feedforward control to shape the BC distribution at the upstream edge, the buffer for per-input slot assignment across the interior segments, and a registrar stage for symmetric velocity equalization that absorbs the terminal-edge BC mismatch.

### 3.1 Upstream Feedforward Control

The upstream belt normally runs at constant $v_u$, but a feedforward controller modulates it around skip events. Between skips, the controller accelerates the belt, building a velocity surplus $\Delta v_k$ that causes inputs to arrive at the buffer entry with larger-than-nominal gaps. This shapes the BC distribution at the buffer's upstream edge favorably, reducing the baseline slack pressure on segment 1. When a skip is detected at time $s_k$, the controller immediately decelerates, spending the accumulated surplus in a controlled recovery. This slows the input following the skip, giving the post-skip input more time to pull away before its follower enters the buffer.

The controller operates in inter-skip phases $\Pi_k$, each consisting of an acceleration ramp (appended incrementally as inputs are processed) followed by a deceleration profile (appended in full at the skip). Two phase invariants guarantee that the belt returns to $(v_u, 0)$ after each cycle: the acceleration invariant ($\sum j^{(n)} T^{(n)} = 0$, ensuring $a \to 0$) and the velocity invariant ($\sum \text{area}_n = 0$, ensuring $v \to v_u$). The deceleration is sized exactly to match the accumulated state $(a_k, \Delta v_k)$ at each skip — it adapts to how much surplus was built, not to a fixed maximum.

The full formalization is in **Upstream formalization.md**.

![[upstream_control_phases.png]] _The upstream acceleration signal $a^{\text{up}}(t)$ across four inter-skip phases $\Pi_1, \ldots, \Pi_4$. Top: the raw signal, showing acceleration between skips ($s_1, \ldots, s_4$) and deceleration after each. Middle: the upstream window $\mathcal{W}_i$ for a single input, showing its projection of the jerk timeline during its transit from spawn to buffer entry. Bottom: the upstream window $\mathcal{W}_{i+1}$ for the following input — note that the windows overlap in time but each input experiences a different slice of the acceleration signal._

### 3.2 Buffer

The buffer is a servo-driven conveyor of length $L^B$ divided into $N^B$ segments. Its job is slot assignment and velocity shaping: given each input's arrival time and the downstream slot schedule, determine which slot to target and execute a chained jerk-limited trajectory across the segments to get there. The buffer handles the per-input, asymmetric part of the problem — every input gets a different trajectory because it targets a different slot.

The key design choice is the exit velocity $v^{BR}$. Instead of requiring the buffer to decelerate all the way to $v_d$, we set $v^{BR} > v_d$ and let the buffer exit at a higher velocity. This shortens (or eliminates) the buffer's terminal deceleration phase, directly relieving slack pressure on the last segment $N^B$. The cost is that the input arrives at the registrar above $v_d$ — the remaining deceleration is delegated to the next stage.

### 3.3 Registrar

The registrar receives each input at $\mathcal{B}^{BR}$ traveling at $v^{BR}$ and decelerates it to $v_d$ for handoff to the downstream carrier. Because $v^{BR}$ is a fixed design constant, every input enters the registrar at the same velocity. Consecutive inputs follow the identical position curve, time-shifted by $\Delta t = g_{\min}/v^{BR}$. The deceleration is symmetric — same profile, same gap compression, fully predictable — and this symmetry is what makes it safe.

The registrar is stateless with respect to position error: it does not know or care where input $i$ is relative to its assigned slot. It solves a pure velocity equalization problem. The interface between buffer and registrar collapses to a single scalar: $v^{BR}$.

The full formalization is in **Registrar formalization.md**.

---

## 4. The Logical Split

The buffer and registrar operate in two different control regimes. Above $v^{BR}$, each input follows its own trajectory — per-input, asymmetric, BC variance distributed across the batch. Below $v^{BR}$, all inputs follow the same trajectory — shared, symmetric, gap safety by construction. The buffer handles the first regime, the registrar handles the second. The boundary between them is not a physical necessity but a control regime boundary, defined by a single scalar: $v^{BR}$.

The buffer performs a complete jerk-limited profile on each input: acceleration from $v_u$ to a peak velocity $v_{\text{peak}}$, then deceleration back to $v^{BR}$. All 7 phases of this profile occur strictly on the buffer — the profile completes _before_ the leading edge reaches $\mathcal{B}^{BR}$. The input then crosses $\mathcal{B}^{BR}$ at constant $v^{BR}$ — a straddle window of duration $\tau_i^{BR} = l_i/v^{BR}$ during which both adjacent belts are locked at $v^{BR}$ (the hard straddling constraint from **KSB System — Formalization.md** §3). Only once the trailing edge clears $\mathcal{B}^{BR}$ does the registrar's separate 3-phase deceleration from $v^{BR}$ to $v_d$ begin. The straddle is not an artefact of the implementation — it is a mechanical consequence of the input's rigidity at a stage boundary, and the buffer's solver must therefore complete its work on the effective control length $\Lambda^B = L^B - l_i$, leaving the final $l_i$ of the buffer belt as the straddle zone.


![[buffer_registrar_accel.png]]
_The acceleration signal $a_i(t)$ of a single input across buffer, straddle, and registrar. On the buffer: a full 7-phase profile — acceleration from $v_u$ to $v_{\text{peak}}$ (phases 1–3), then deceleration back to $v^{BR}$ (phases 5–7), with a coast at $v_{\text{peak}}$ in between (phase 4). The 7-phase profile completes before $\mathcal{B}^{BR}$. Across the stage boundary, the input straddles $\mathcal{B}^{BR}$ at constant $v^{BR}$ for duration $\tau_i^{BR} = l_i/v^{BR}$ — both adjacent belts locked at $v^{BR}$, no acceleration activity. On the registrar, after the trailing edge has cleared $\mathcal{B}^{BR}$: a separate 3-phase deceleration from $v^{BR}$ to $v_d$. The buffer acceleration integral decomposes as $\int_B a_i(t),dt = (v_{\text{peak}} - v_u) + (v^{BR} - v_{\text{peak}})$. The first term — phases 1–3 — is what builds the gap between input $i$ and its follower. The second term — phases 5–7 — compresses it back. The net buffer deposit is $v^{BR} - v_u$, and the registrar withdrawal is $v^{BR} - v_d$. What matters for gap safety is the peak deposit $v_{\text{peak}} - v_u$, which is always larger than the net._


Because the buffer's per-input planning maximizes $v_{\text{peak}}$ for the available time horizon, normal (non-skip) inputs reach high peak velocities and build large gap deposits during the acceleration phase. The subsequent deceleration to $v^{BR}$ on the buffer compresses the gap, but the peak deposit was large enough to absorb it. On the buffer, the minimum-clearance condition is enforced *structurally* — by the segment-length parametrization in the per-segment formalism, where the lower bound $L^B_k \geq L^B_\min$ together with the schedule's free-window construction guarantees that whenever a segment hands off to the next, the spatial gap between consecutive inputs at the boundary instant exceeds $g_{\min}$ by construction. The buffer's planner does not enforce $g_i(t) \geq g_{\min}$ as a runtime curve constraint on the gap signal; segment sizing absorbs it geometrically, and the per-segment slack feasibility test ($S^{\mathcal{P}}_{i,k} \geq 0$) does the rest. On the registrar the spatial gap $g_i(t)$ continues to compress as input $i$ decelerates while input $i+1$ is still at higher velocity, and it may drop well below $g_{\min}$. This is harmless. Registrar sizing (§5) guarantees that whenever two consecutive inputs are simultaneously on the same segment, both adjacent belts are equalized at the same velocity with zero acceleration — either a straddle at $v^{BR}$ or a coast at $v_d$. The spatial gap is instantaneously frozen at those moments, and physical collision requires a velocity mismatch that cannot arise. On the buffer, $g_{\min}$ is enforced structurally by segment geometry; on the registrar, the same spatial condition is a coordinate artefact rendered harmless by construction.

The architecture works because the dangerous part of the problem — stochastic arrivals, per-input slot assignment, skip-induced transients — is separated from the safe part — deterministic, shared deceleration — by $v^{BR}$. The buffer's job is to solve the hard problem (slot alignment under uncertainty) and hand off a clean, uniform stream. The registrar's job is to solve the easy problem (symmetric velocity equalization) without disturbing the gap structure the buffer worked to establish.

The registrar is not solving a problem "created by" the buffer. It is solving the half of the original synchronization problem that can be made symmetric. The buffer solves the half that cannot.

---

## 5. Registrar Sizing

The registrar's safety condition reduces to a per-segment constraint: the active deceleration phase on each segment must complete before the following input begins its active phase on the same segment.

Because consecutive inputs are time-shifted copies separated by $\Delta t = g_{\min}/v^{BR}$, and because this temporal offset is invariant across segments (the spatial gap compresses as velocity decreases, but the time offset does not), every segment has the same temporal safety budget $\Delta t$. The binding constraint on each segment is the smaller of the control window $T_n = \Lambda^{R,n}/\bar{v}_n$ (how long the input is on the segment) and $\Delta t$ (how long before the follower starts).

At realistic parameters ($g_{\min} = 1.0$ m, $v^{BR} = 1.5$ m/s, $\Delta v = 0.18$ m/s, $j_{\max} = 100$ m/s³), the active phase duration for a single-segment registrar is $T^{\text{act}} = 2\sqrt{\Delta v / j_{\max}} = 0.085$ s, fitting comfortably within both the control window ($T_1 = 0.187$ s for a 0.6 m segment) and the safety interval ($\Delta t = 0.667$ s). A single registrar segment of length 0.6 m can handle a velocity drop of 0.88 m/s — nearly 5× the required 0.18 m/s.

The registrar is the easy part of the system. The buffer — with its per-input trajectory planning, skip-dependent corrections, and stochastic gap dynamics — is the hard part. The registrar sizing confirms this quantitatively: the design effort concentrates on the buffer and upstream controller, where the problem is genuinely difficult, while the registrar is a closed-form sizing exercise whose feasibility is guaranteed by wide margins.

Multi-segment registrar designs ($N_R > 1$) are not required at current parameters but provide headroom: the ability to handle larger $v^{BR}$ or tighter $g_{\min}$ without mechanical redesign. The general $N_R$-segment formalization — including the per-segment weight optimization — is retained in **Registrar weight optimization.md** for the case where parameters tighten and the problem becomes non-trivial.

---

## 6. Role of Each Subsystem

| Subsystem | Addresses | Mechanism |
|---|---|---|
| **Upstream control** | BC variance at the upstream edge (skip transients) | Feedforward velocity modulation: builds gap surplus between skips, spends it in controlled deceleration after each skip |
| **Buffer** | Per-input slot assignment across the interior segments | Chained jerk-limited primitives targeting individual slots; exits at $v^{BR} > v_d$ to relieve slack pressure on the last segment |
| **Registrar** | BC mismatch at the registrar edge (the symmetric residual) | Shared deceleration profile from $v^{BR}$ to $v_d$; gap safety by construction via time-shift invariance |

---

## 7. Design Parameters

| Parameter                     | Meaning                                  | Effect                                                                                   |
| ----------------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------- |
| $v^{BR}$                      | KSB exit velocity                        | Higher → shorter buffer terminal decel, less slack pressure on segment $N^B$, more registrar load |
| $L^B$, $N^B$                  | Buffer length and segment count          | Sets buffer footprint and the number of internal state-observation instants; more segments → bounded propagation of segment-level state error, more BC re-anchoring opportunities |
| $L^R$, $N_R$                  | Registrar total length and segment count | More segments → headroom for larger $v^{BR}$; $N_R = 1$ sufficient at current parameters |
| $\Delta v_{\max}^{\text{up}}$ | Upstream controller velocity ceiling     | Higher → more aggressive BC pre-conditioning at the upstream edge, longer recovery windows |
| $j_{\max}$, $a_{\max}$        | Kinematic bounds                         | Shared across subsystems; set by mechanical limits                                       |
| $g_{\min}$                    | Minimum clearance                        | Hard safety condition; absorbed structurally via $L^B_\min$ on the buffer and via $\Delta t$ on the registrar |

---
