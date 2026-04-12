# KSB System  — Design Narrative

## 1. The Physical System

A packaging line consists of three mechanically decoupled conveyors in series. An upstream conveyor delivers discrete product inputs at a roughly constant spacing, but with real variability — inputs arrive slightly early or late relative to the mean. A downstream carrier runs at a fixed velocity $v_d$ and carries evenly spaced slots at spacing $d_s$, into which inputs must be placed precisely. Between them sits a servo-driven buffer conveyor, the KSB, whose job is to take each irregularly timed input and deliver it into the correct downstream slot at exactly the right moment.

The upstream runs at velocity $v_u$ and delivers inputs at mean spacing $\mu_u$ with standard deviation $\sigma_u$, giving an arrival rate $r_u = v_u / \mu_u$. The downstream runs at rate $r_d = v_d / d_s$. The system is designed with $\rho = r_u / r_d < 1$ — the upstream is slightly slower than the downstream, so on average fewer inputs arrive than there are slots. This means some slots will always go unfilled, and the buffer must decide which slot each input targets and execute a jerk-limited motion profile to get it there in time. The full system definition and gap metrics are formalized in **Buffer formalization.md**.

The performance metric is the violation probability $\varepsilon$ — the fraction of consecutive input pairs for which the gap $g_i(t)$ between them drops below the minimum clearance $g_{\min}$ at any point during their shared time on the buffer. The goal is to keep $\varepsilon$ below a threshold $\varepsilon_{\max}$ across the full range of upstream variability $\sigma_u$.

---

## 2. The KSB and Its Failure Modes

The KSB is a servo-driven conveyor of length $L^B$ divided into $N_B$ independently controllable segments. Each input enters at approximately $v_u$ and the KSB assigns it to the earliest feasible downstream slot, then executes a jerk-limited S-curve to deliver it there. The motion profile accelerates the input to a peak velocity, then decelerates back to $v_d$ for handoff, all within the kinematic bounds $(j_{\max}, a_{\max}, v_{\max})$ and subject to the clearance constraint $g_i(t) \geq g_{\min}$ between consecutive inputs.

Running this system in simulation reveals two distinct gap violation patterns, visible in the per-input gap curves $g_i(t)$: violations concentrated at the start of the co-occupancy window, and violations concentrated at the end.

### 2.1 Start-of-Window Violations

Start-of-window violations are a fundamental consequence of the upstream gap distribution. When input $i+1$ enters the KSB, its gap to input $i$ — which is already accelerating toward its slot — is determined by the upstream spacing $g_i^u$. If $g_i^u$ is small, input $i+1$ enters close behind input $i$, and the KSB's initial acceleration phase widens the gap as $i$ pulls away. But if $g_i^u$ is small enough, the gap starts below $g_{\min}$ or drops below it immediately, and no amount of acceleration can recover it fast enough. This is not primarily a skip problem — it is a structural consequence of the upstream arrival distribution. Skips exacerbate it: after a skip, the post-skip input must decelerate aggressively to hit an earlier slot, and the input immediately behind it is still traveling at $v_u$ unaware of the skip. The compression at the start of that pair's co-occupancy window is deeper and faster than the baseline case.

### 2.2 End-of-Window Violations

End-of-window violations are caused by the KSB's deceleration phase. Every input — not just post-skip ones — must decelerate from its peak velocity back to $v_d$ near the end of its buffer transit. During this deceleration, the following input is still arriving at $v_u \approx v_d$, so the gap compresses as the leading input slows. The longer and steeper the deceleration, the deeper the compression. With the KSB required to land inputs at exactly $v_d$, this deceleration cannot be avoided — it is built into every profile. Unlike start-of-window violations which are event-driven, end-of-window violations occur continuously on every input pair.

---

## 3. The Upstream Controller

To address start-of-window violations, we add an upstream feedforward controller. The upstream belt normally runs at constant $v_u$, but the controller modulates it with two objectives.

**Objective 1 — reduce baseline gap violations.** By running the upstream belt slightly faster between skips, inputs arrive at the KSB entry with higher velocity and larger gaps than the nominal distribution would produce. The KSB's acceleration phase starts from a better position, reducing the frequency of start-of-window violations. The upstream controller cannot eliminate them entirely — they are a tail event of the upstream spacing distribution — but it shifts the distribution favorably.

**Objective 2 — reduce post-skip compression.** Immediately after a skip is detected, the upstream controller briefly decelerates. This slows the input immediately following the skip relative to where it would have been, giving the post-skip input more time to pull away before the follower enters the KSB. The deceleration is sized to match the skip's phase error, and the belt returns to $v_u$ once recovery is complete. The upstream formalization is in **Upstream formalization.md**.

The upstream controller substantially reduces start-of-window violations and skip compression. It does not address end-of-window violations.

---

## 4. The Registrar

End-of-window violations require a different intervention. The root cause is that the KSB must decelerate all the way to $v_d$ before handoff. The solution is to stop requiring this: allow the KSB to exit at a higher velocity $v^{BR} > v_d$, and add a downstream stage — the Registrar — that performs the remaining deceleration gracefully.

### 4.1 Graceful Staged Deceleration

The registrar is a sequence of $N_R$ independently-controllable conveyor segments, each decelerating the input by an equal share of the total velocity drop:

$$\Delta V = \frac{v_d - v^{BR}}{N_R} \leq 0$$

Each segment applies a jerk-limited deceleration profile — a trapezoidal acceleration curve as shown in the system diagram — and hands the input to the next segment at the new, lower velocity. The input exits the final segment at exactly $v_d$, as required by the downstream carrier. Because the total deceleration $v^{BR} - v_d$ is distributed across $N_R$ segments, each individual deceleration event is small and gentle, producing minimal gap disturbance.

Segment lengths decrease monotonically from $L^{R,1}$ to $L^{R,N_R}$ to equalize dwell time per segment — upstream segments are traversed faster, so they must be longer to give each correction an equal time budget. The full geometry and timing are formalized in **Registrar formalization.md**.

### 4.2 Position Error Correction

By exiting the KSB at $v^{BR} > v_d$, the KSB's deceleration phase is shortened. A shorter deceleration means the KSB's motion profile is less symmetric — more of the kinematic budget is available for the acceleration phase, which is what drives inputs to their slots and maintains gap clearance. The trade-off is that the KSB is no longer required to land inputs exactly on their slots. It commits to a slot and hands off at $v^{BR}$, but a small position error $\Delta p_i$ accumulates — caused by belt-to-input slip under more aggressive accelerations and by co-occupancy effects under relaxed gap enforcement. This error is defined formally in **KSB System — Unified Kinematic Formalization.md**, Section 6.

The registrar absorbs this error by modulating the per-segment deceleration. If an input is running slightly ahead of its slot, the segment decelerates slightly more than the nominal $|\Delta V|$, retarding the input relative to the slot. If it is behind, the segment decelerates slightly less, advancing it. The correction is the residual degree of freedom in the deceleration profile: the baseline deceleration is $\Delta V$, and the correction $\delta_i^n$ adjusts the area of each segment's deceleration trapezoid above or below nominal.

This is a cleaner framing than treating position correction and deceleration as separate tasks sharing a budget. They are the same task: the registrar decelerates each input from $v^{BR}$ to $v_d$ across $N_R$ stages, and the precise shape of each stage's deceleration profile is tuned to simultaneously correct the accumulated position error.

### 4.3 Gap Safety on the Registrar

Gap compression cannot occur on the registrar by construction. When input $i+1$'s leading edge enters segment $n+1$ while input $i$'s trailing edge is still leaving segment $n$, both segments are velocity-matched at the shared boundary crossing velocity $v^{R,n}$ — required by the mechanical straddling constraint. No relative velocity means no gap change. The clearance constraint is satisfied by the design, not enforced by a solver.

---

## 5. Role of Each Subsystem

| Subsystem | Primary job | Secondary job |
|---|---|---|
| **Upstream** | Constant-velocity infeed at $v_u$ | Feedforward modulation to reduce start-of-window violations and post-skip compression |
| **KSB** | Slot assignment and velocity shaping; exits at $v^{BR}$ | Aggressive acceleration phase enabled by relaxed landing requirement |
| **Registrar** | Graceful staged deceleration from $v^{BR}$ to $v_d$ across $N_R$ segments | Position error correction $\Delta p_i$ via per-segment deceleration modulation |
| **Downstream** | Constant-velocity carrier at $v_d$ | — |

---

## 6. Design Parameters

The system has a small set of free geometric and velocity parameters that jointly determine $\varepsilon$:

| Parameter                     | Meaning                                  | Effect                                                                                   |          |     |
| ----------------------------- | ---------------------------------------- | ---------------------------------------------------------------------------------------- | -------- | --- |
| $v^{BR}$                      | KSB exit velocity                        | Higher → shorter KSB decel, fewer end-of-window violations, more registrar load          |          |     |
| $L^B$, $N_B$                  | KSB length and segment count             | Sets effective control length $\Lambda^B = L^B - l_i$; more segments reduce co-occupancy |          |     |
| $L^R$, $N_R$                  | Registrar total length and segment count | More segments → finer-grained correction, smaller per-segment $                          | \Delta V | $   |
| $L^{R,n}$                     | Per-segment registrar lengths            | Decreasing toward downstream to equalize dwell time                                      |          |     |
| $\Delta v_{\max}^{\text{up}}$ | Upstream controller velocity ceiling     | Higher → more aggressive gap pre-conditioning, longer recovery windows                   |          |     |

The optimal geometry minimizes $\varepsilon$ subject to the registrar's feasibility constraint $|\Delta p_i| \leq \sum_n \delta_{\max}^n$ across the full batch. This is the primary goal of the simulation.