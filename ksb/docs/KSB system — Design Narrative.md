# KSB System — Design Narrative

## 1. The Synchronization Problem

A packaging line delivers discrete inputs from an upstream conveyor at a roughly constant spacing with real variability — inputs arrive slightly early or late relative to the mean. A downstream carrier runs at fixed velocity $v_d$ with evenly spaced slots at spacing $d_s$. Between them sits a servo-driven buffer conveyor whose job is to take each irregularly timed input and deliver it into the correct downstream slot at exactly the right moment.

The upstream runs at velocity $v_u$ and delivers inputs at mean spacing $\mu_u$ with standard deviation $\sigma_u$, giving an arrival rate $r_u = v_u / \mu_u$. The downstream slot rate is $r_d = v_d / d_s$. The system operates with $\rho = r_u / r_d < 1$ — the upstream is slightly slower, so some slots will always go unfilled. When the buffer determines that an input cannot feasibly reach the next available slot, it skips that slot and targets the one after it.

The performance metric is the violation probability $\varepsilon$ — the fraction of consecutive input pairs for which the gap $g_i(t)$ between them drops below the minimum clearance $g_{\min}$ at any point during their shared time on the buffer. The goal is $\varepsilon < \varepsilon_{\max}$. The full system definition, gap metrics, and skip mechanism are formalized in **Buffer formalization.md**.

---

## 2. Why a Single Buffer Stage Isn't Enough

The buffer does per-input slot assignment via jerk-limited motion profiles. Each input gets a different trajectory — it targets a different slot, requires a different correction magnitude, and covers a different velocity envelope. This per-input asymmetry is the buffer's job; it cannot be avoided. But it creates two distinct failure modes, visible in the per-input gap curves $g_i(t)$.

### 2.1 Start-of-Window Violations

When input $i+1$ enters the buffer, its gap to input $i$ — already accelerating toward its slot — is set by the upstream spacing $g_i^u$. If $g_i^u$ is small, $i+1$ enters close behind $i$, and while the buffer's acceleration widens the gap as $i$ pulls away, the initial compression may already violate $g_{\min}$. Skips make this worse: after a skip, the post-skip input decelerates aggressively to hit an earlier slot, while the input behind it is still at $v_u$. The compression at the start of that pair's co-occupancy window is deeper and faster than the baseline case.

Start-of-window violations are event-driven. They concentrate around skip events and their severity scales with the phase error at the skip boundary. At low upstream variability ($\sigma_u \approx 0$), skips are nearly periodic and the violations are predictable. At high $\sigma_u$, skip timing becomes stochastic and violations scatter irregularly — but the long-run violation rate is set by $\rho$, not by $\sigma_u$.

### 2.2 End-of-Window Violations

Every input — not just post-skip ones — must decelerate from its peak velocity back to $v_d$ near the end of its buffer transit. During this deceleration, the following input is still at a higher velocity, so the gap compresses. The longer and steeper the deceleration, the deeper the compression. Unlike start-of-window violations, this occurs on every input pair. It is structural, not event-driven.

The root cause is trajectory asymmetry: consecutive inputs have different velocity profiles, so during the deceleration phase of input $i$, the gap $g_i(t)$ depends on the difference between two unrelated trajectories. Making the deceleration gentler spreads the compression over a longer window but does not reduce the total gap loss — the integral is set by the velocity difference, not the acceleration rate. You cannot fix asymmetric deceleration by making it slower; you can only fix it by making it symmetric.

---

## 3. The Three-Subsystem Architecture

The two failure modes have different causes and require different interventions. Start-of-window violations are driven by skip transients in the upstream arrival stream. End-of-window violations are driven by the buffer's need to decelerate each input back to $v_d$. The architecture addresses each with a dedicated subsystem: upstream feedforward control for skip compensation, the buffer for per-input slot assignment, and a registrar stage for symmetric velocity equalization.

### 3.1 Upstream Feedforward Control

The upstream belt normally runs at constant $v_u$, but a feedforward controller modulates it around skip events. Between skips, the controller accelerates the belt, building a velocity surplus $\Delta v_k$ that causes inputs to arrive at the buffer entry with larger-than-nominal gaps. This shifts the gap distribution favorably, reducing baseline start-of-window violations. When a skip is detected at time $s_k$, the controller immediately decelerates, spending the accumulated surplus in a controlled recovery. This slows the input following the skip, giving the post-skip input more time to pull away before its follower enters the buffer.

The controller operates in inter-skip phases $\Pi_k$, each consisting of an acceleration ramp (appended incrementally as inputs are processed) followed by a deceleration profile (appended in full at the skip). Two phase invariants guarantee that the belt returns to $(v_u, 0)$ after each cycle: the acceleration invariant ($\sum j^{(n)} T^{(n)} = 0$, ensuring $a \to 0$) and the velocity invariant ($\sum \text{area}_n = 0$, ensuring $v \to v_u$). The deceleration is sized exactly to match the accumulated state $(a_k, \Delta v_k)$ at each skip — it adapts to how much surplus was built, not to a fixed maximum.

The full formalization is in **Upstream formalization.md**.

![[upstream_control_phases.png]] _The upstream acceleration signal $a^{\text{up}}(t)$ across four inter-skip phases $\Pi_1, \ldots, \Pi_4$. Top: the raw signal, showing acceleration between skips ($s_1, \ldots, s_4$) and deceleration after each. Middle: the upstream window $\mathcal{W}_i$ for a single input, showing its projection of the jerk timeline during its transit from spawn to buffer entry. Bottom: the upstream window $\mathcal{W}_{i+1}$ for the following input — note that the windows overlap in time but each input experiences a different slice of the acceleration signal._

### 3.2 Buffer

The buffer is a servo-driven conveyor of length $L^B$ divided into $N_B$ segments. Its job is slot assignment and velocity shaping: given each input's arrival time and the downstream slot schedule, determine which slot to target and execute a jerk-limited profile to get there. The buffer handles the per-input, asymmetric part of the problem — every input gets a different trajectory because it targets a different slot.

The key design choice is the exit velocity $v^{BR}$. Instead of requiring the buffer to decelerate all the way to $v_d$, we set $v^{BR} > v_d$ and let the buffer exit at a higher velocity. This shortens (or eliminates) the buffer's deceleration phase, directly reducing end-of-window gap compression. The cost is that the input arrives at the registrar above $v_d$ — the remaining deceleration is delegated to the next stage.

### 3.3 Registrar

The registrar receives each input at $\mathcal{B}^{BR}$ traveling at $v^{BR}$ and decelerates it to $v_d$ for handoff to the downstream carrier. Because $v^{BR}$ is a fixed design constant, every input enters the registrar at the same velocity. Consecutive inputs follow the identical position curve, time-shifted by $\Delta t = g_{\min}/v^{BR}$. The deceleration is symmetric — same profile, same gap compression, fully predictable — and this symmetry is what makes it safe.

The registrar is stateless with respect to position error: it does not know or care where input $i$ is relative to its assigned slot. It solves a pure velocity equalization problem. The interface between buffer and registrar collapses to a single scalar: $v^{BR}$.

The full formalization is in **Registrar formalization.md**.

---

## 4. The Logical Split

The buffer and registrar operate in two different control regimes. Above $v^{BR}$, each input follows its own trajectory — per-input, asymmetric, gap constraint active. Below $v^{BR}$, all inputs follow the same trajectory — shared, symmetric, gap safety by construction. The buffer handles the first regime, the registrar handles the second. The boundary between them is not a physical necessity but a control regime boundary, defined by a single scalar: $v^{BR}$.

The buffer performs a complete jerk-limited profile on each input: acceleration from $v_u$ to a peak velocity $v_{\text{peak}}$, then deceleration back to $v^{BR}$. All 7 phases of this profile occur strictly on the buffer — the profile completes _before_ the leading edge reaches $\mathcal{B}^{BR}$. The input then crosses $\mathcal{B}^{BR}$ at constant $v^{BR}$ — a straddle window of duration $\tau_i^{BR} = l_i/v^{BR}$ during which both adjacent belts are locked at $v^{BR}$ (the hard straddling constraint from **KSB System — Formalization.md** §3). Only once the trailing edge clears $\mathcal{B}^{BR}$ does the registrar's separate 3-phase deceleration from $v^{BR}$ to $v_d$ begin. The straddle is not an artefact of the implementation — it is a mechanical consequence of the input's rigidity at a stage boundary, and the buffer's solver must therefore complete its work on the effective control length $\Lambda^B = L^B - l_i$, leaving the final $l_i$ of the buffer belt as the straddle zone.

![[buffer_registrar_acceleration.png]] _The acceleration signal $a_i(t)$ of a single input across buffer, straddle, and registrar. On the buffer: a full 7-phase profile — acceleration from $v_u$ to $v_{\text{peak}}$ (phases 1–3), then deceleration back to $v^{BR}$ (phases 5–7), with a coast at $v_{\text{peak}}$ in between (phase 4). The 7-phase profile completes before $\mathcal{B}^{BR}$. Across the stage boundary, the input straddles $\mathcal{B}^{BR}$ at constant $v^{BR}$ for duration $\tau_i^{BR} = l_i/v^{BR}$ — both adjacent belts locked at $v^{BR}$, no acceleration activity. On the registrar, after the trailing edge has cleared $\mathcal{B}^{BR}$: a separate 3-phase deceleration from $v^{BR}$ to $v_d$. The buffer acceleration integral decomposes as $\int_B a_i(t),dt = (v_{\text{peak}} - v_u) + (v^{BR} - v_{\text{peak}})$. The first term — phases 1–3 — is what builds the gap between input $i$ and its follower. The second term — phases 5–7 — compresses it back. The net buffer deposit is $v^{BR} - v_u$, and the registrar withdrawal is $v^{BR} - v_d$. What matters for gap safety is the peak deposit $v_{\text{peak}} - v_u$, which is always larger than the net._

Because the buffer's time-optimal solver maximizes $v_{\text{peak}}$ for the available time horizon, normal (non-skip) inputs reach high peak velocities and build large gap deposits during phases 1–3. The subsequent deceleration to $v^{BR}$ on the buffer compresses the gap, but the peak deposit was large enough to absorb it — on the buffer, $g_{\min}$ is an active safety constraint and the solver enforces it directly. On the registrar the constraint is no longer active in the same sense: the spatial gap $g_i(t)$ continues to compress as input $i$ decelerates while input $i+1$ is still at higher velocity, and it may drop well below $g_{\min}$. This is harmless. Registrar sizing (§5) guarantees that whenever two consecutive inputs are simultaneously on the same segment, both adjacent belts are equalized at the same velocity with zero acceleration — either a straddle at $v^{BR}$ or a coast at $v_d$. The spatial gap is instantaneously frozen at those moments, and physical collision requires a velocity mismatch that cannot arise. On the buffer, $g_{\min}$ is a hard safety constraint; on the registrar, the same spatial condition is a coordinate artefact rendered harmless by construction.

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

|Subsystem|Addresses|Mechanism|
|---|---|---|
|**Upstream control**|Start-of-window violations (skip transients)|Feedforward velocity modulation: builds gap surplus between skips, spends it in controlled deceleration after each skip|
|**Buffer**|Per-input slot assignment (the asymmetric problem)|Jerk-limited profiles targeting individual slots; exits at $v^{BR} > v_d$ to minimize deceleration-phase compression|
|**Registrar**|End-of-window violations (the symmetric residual)|Shared deceleration profile from $v^{BR}$ to $v_d$; gap safety by construction via time-shift invariance|

---

## 7. Design Parameters

| Parameter                     | Meaning                                  | Effect                                                                                   |
| ----------------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------- |
| $v^{BR}$                      | KSB exit velocity                        | Higher → shorter buffer decel, fewer end-of-window violations, more registrar load       |
| $L^B$, $N_B$                  | Buffer length and segment count          | Sets effective control length; more segments reduce co-occupancy                         |
| $L^R$, $N_R$                  | Registrar total length and segment count | More segments → headroom for larger $v^{BR}$; $N_R = 1$ sufficient at current parameters |
| $\Delta v_{\max}^{\text{up}}$ | Upstream controller velocity ceiling     | Higher → more aggressive gap pre-conditioning, longer recovery windows                   |
| $j_{\max}$, $a_{\max}$        | Kinematic bounds                         | Shared across subsystems; set by mechanical limits                                       |
| $g_{\min}$                    | Minimum clearance                        | Hard safety constraint; determines $\Delta t$ and therefore registrar feasibility        |