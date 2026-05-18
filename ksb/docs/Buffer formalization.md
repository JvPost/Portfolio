# Kinematic Synchronization Buffer — System Analysis

## 1. Introduction

Many automated production and logistics systems share a common structural challenge: a
stream of discrete objects arrives at irregular intervals from an upstream source and must
be handed off to a downstream process that expects them at precise, fixed intervals. The
upstream and downstream are mechanically decoupled — they run at independently set speeds
— and a buffer element in between is responsible for absorbing the timing mismatch and
delivering each object into the correct position at the correct moment.

This document analyses one realization of that problem: the **Kinematic Synchronization
Buffer (KSB)**. The KSB is a servo-driven conveyor segment that receives objects one at a
time from an infeed and synchronizes each one to a target slot on a continuously moving
downstream carrier. Synchronization is achieved by executing a jerk-limited motion profile
over the object's transit through the buffer, adjusting its position so that it arrives at
the handoff point at the right time to enter its assigned slot.

The system operates under two sources of constraint. The first is kinematic: the motion
profile must respect bounds on velocity, acceleration, and jerk, and the object must reach
its target within the time window defined by the downstream carrier's slot timing. The
second is a safety constraint: consecutive objects sharing the buffer must never come
closer than a minimum clearance $g_{\min}$, where the instantaneous gap between object $i$
and its follower is defined as

$$
g_{i, i+1}(t) = p_{i+1}(t) - p_i(t)
$$

with $p_i(t)$ the position of object $i$ along the buffer axis. Physically, the system
must satisfy $g_{i, i+1}(t) \geq g_{\min}$ for all $i$ and all $t$. The analysis below
shows that this physical safety condition is best enforced not as a runtime constraint on
the gap curve $g_{i,i+1}(t)$, but structurally — through the buffer's segment-level
geometry — and that the natural primary signal is not gap-curve depth but a per-segment
synchronization-error measure introduced in §3.

When the upstream arrival rate is close to the downstream slot rate, the system operates
in a near-balanced regime and the kinematic demands on the buffer are modest. The
interesting regime — and the focus of this analysis — is when the rates differ slightly,
forcing the buffer to occasionally skip a downstream slot and assign an object to the next
available one. These **skip events** are the primary driver of synchronization-error
spikes and the primary source of feasibility pressure on the buffer's edge segments.

The core questions this document addresses are:

- What are the structural invariants of the system — quantities that depend only on the
  rate ratio and not on arrival variability?
- How does upstream arrival variability change the character of skip events and their
  consequences for the buffer's synchronization demands?
- What scalar signals best capture system health, and how do they behave as variability
  increases?
- Does the system require regime-specific solutions, or is there a single structural
  intervention that addresses the root cause across all variability levels?

The analysis proceeds from first principles, defining the system variables, motivating
the synchronization-error framing, and building toward a well-posed optimization problem.
The treatment is intentionally general: while the KSB is one physical instantiation, the
same mathematical structure appears in any system where a stochastic arrival stream must
be synchronized to a deterministic slot sequence under kinematic constraints — a pattern
that recurs in packaging, logistics sortation, assembly line feeding, and discrete-event
manufacturing more broadly.

## 2. The Buffer

### 2.1 Internal Structure

The KSB is a servo-driven buffer segment of length $L^B$ that sits between the upstream conveyor and the registrar. Its job is to transport each input from the upstream-side boundary to the registrar-side boundary while executing a jerk-limited motion profile that synchronizes the input to its assigned downstream slot.

The buffer is internally divided into $N^B$ segments. Segment $k$, for $k \in {1, \ldots, N^B}$, has length $L^B_k$, with $\sum_{k=1}^{N^B} L^B_k = L^B$. In buffer-local coordinates with the upstream boundary at zero, the upstream boundary of segment $k$ is at $p_k = \sum_{j=1}^{k-1} L^B_j$ (so $p_1 = 0$) and its downstream boundary is at $p_k + L^B_k$. The downstream boundary of segment $N^B$ is at $L^B$.

Each segment is independently servo-controlled and operates at its own velocity. The straddling constraint of the system formalization applies at every internal boundary: when an input straddles the boundary between segments $k$ and $k+1$ — leading edge past $p_{k+1} = p_k + L^B_k$, trailing edge before it — both segments must run at matched velocity. The same constraint applies at the buffer's two outer boundaries, where the adjacent stages (upstream conveyor, registrar) likewise lock to the buffer's edge segments during straddling.

There is a mechanical lower bound $L^B_\min$ on segment length, reflecting limits on how short an independently-controlled servo segment can physically be built. This bound is structural, not derived from synchronization requirements, and is taken as a given throughout this document.

### 2.2 Per-Segment Quantities

Each input is modeled as a point of mass located at its leading edge, with the trailing edge at $p_i(t) - l$ where $p_i(t)$ is the leading-edge position of input $i$ at time $t$ and $l$ is the (uniform) input length. The buffer transports inputs in one direction only; the point-mass model loses nothing relative to a centroid or extended-body formulation and simplifies all downstream definitions.

**Crossing events.** Two events define segment $k$'s control window for input $i$:

$$ t^\text{in}_{i,k} := \text{time } t \text{ such that } p_i(t) = p_k $$

— input $i$'s leading edge reaches segment $k$'s upstream boundary; segment $k$ begins to bear input $i$. And:

$$ t^\text{out}_{i,k} := \text{time } t \text{ such that } p_i(t) = p_k + L^B_k + l $$

— input $i$'s trailing edge clears segment $k$'s downstream boundary; segment $k$ is fully clear of input $i$.

**Segment states at crossing events.** During the intervals when input $i$ is being borne by segment $k$, segment $k$'s velocity is locked by the straddling constraint to match input $i$'s velocity. At each of the two crossing events, segment $k$'s state is recorded:

$$ \xi^\text{in}_{i,k} := (v^\text{in}_{i,k},; a^\text{in}_{i,k}) \qquad \xi^\text{out}_{i,k} := (v^\text{out}_{i,k},; a^\text{out}_{i,k}) $$

For internal segments, $\xi^\text{in}_{i,k}$ is inherited from segment $k-1$'s handoff and $\xi^\text{out}_{i,k}$ is delivered to segment $k+1$ at handoff. The two outer edges of the buffer are exceptions: $\xi^\text{in}_{i,1}$ is inherited from the upstream conveyor rather than from an internal segment, and $\xi^\text{out}_{i,N^B}$ is delivered to the registrar rather than to an internal segment. Asymmetric behavior at these edge boundaries is taken up in §5.3.

**Free windows.** Between consecutive inputs $i$ and $i+1$, segment $k$ has an interval during which no input is on it — the _free window_. It opens at $t^\text{out}_{i,k}$ (input $i$ leaves) and closes at $t^\text{in}_{i+1,k}$ (input $i+1$ arrives):

$$ W_{i,k} := t^\text{in}_{i+1,k} - t^\text{out}_{i,k}. $$

During this window, segment $k$ is uncoupled from any input and is free to swing its velocity from $v^\text{out}_{i,k}$ to $v^\text{in}_{i+1,k}$ under the synchronization policy $\pi^S$.

**Transitions.** The transition segment $k$ must execute over the free window between inputs $i$ and $i+1$ is the pair of states at the two endpoints:

$$ \zeta_{i,k} := (\xi^\text{out}_{i,k},; \xi^\text{in}_{i+1,k}). $$

**Minimum-time function.** Under $\pi^S$'s kinematic bounds (acceleration, jerk; see §2.4), the minimum duration required to execute the transition $\zeta_{i,k}$ is

$$ T_\min(\zeta_{i,k}) := \text{Ruckig duration for } \zeta_{i,k} \text{ under } \pi^S \text{ bounds, with no minimum-duration constraint}. $$

This is an operational definition: $T_\min$ is what Ruckig returns when called with boundary conditions $\zeta_{i,k}$ and $\pi^S$'s kinematic bounds. The synchronization is feasible on segment $k$ between inputs $i$ and $i+1$ exactly when

$$ W_{i,k} \geq T_\min(\zeta_{i,k}). $$

The two failure modes of the buffer follow directly. $W_{i,k} < 0$ is a _timing collision_ — input $i+1$ arrives at segment $k$'s upstream boundary before input $i$ has cleared its downstream boundary, so no $\pi^S$ trajectory exists at all. The case $0 \leq W_{i,k} < T_\min(\zeta_{i,k})$ is a _kinematic shortfall_ — a trajectory exists but cannot fit within the available window under $\pi^S$'s bounds. Both are characterized in the failure-modes document.

The structural skip mechanism that drives the distribution of $W_{i,k}$ across batch position is set by the load ratio $\rho = r_u / r_d$ and the resulting mean skip interval $Q = 1/(1-\rho)$, both defined in the system formalization. $\rho$ and $Q$ are recalled here as needed without re-introduction.

### 2.3 Physical Assumptions

Two physical assumptions underlie the formalization above and are made explicit here so the rest of the document can rest on them without restating them at each use.

**Zero-slip at input/segment contact.** While an input is being borne by a segment, the input moves at the segment's velocity with no slippage. This is the basis for the straddling constraint and for the recording of segment states at the crossing events $t^\text{in}_{i,k}$ and $t^\text{out}_{i,k}$. The zero-slip assumption also fixes the kinematic bounds available to the per-input control policy $\pi^I$: the acceleration and jerk a segment may impose on a borne input cannot exceed the contact-friction limit, beyond which the assumption fails and the input slides. See §2.4.

**Perfect synchronization under $W \geq T_\min$.** The synchronization policy $\pi^S$ achieves the demanded transition $\zeta_{i,k}$ exactly when the free window is sufficient, $W_{i,k} \geq T_\min(\zeta_{i,k})$. There is no partial credit and no degradation policy: the transition is either completed within the window, in which case $\pi^S$ delivers the demanded $\xi^\text{in}_{i+1,k}$ at $t^\text{in}_{i+1,k}$, or it cannot be completed, in which case the synchronization is infeasible and the kinematic-shortfall failure mode is recorded. The assumption is that the policy itself does not introduce error within its feasibility region — the only question is whether the region contains the demanded transition. This makes feasibility a binary property at the policy level and frames the analysis as a characterization of where the feasibility boundary sits.

### 2.4 Kinematic Bounds: $\pi^I$ vs $\pi^S$

The two policies $\pi^I$ (per-input control) and $\pi^S$ (segment synchronization) operate under different sets of kinematic bounds, because the physical regime they operate in is different.

**$\pi^I$ bounds: zero-slip at contact.** While a segment bears an input, the input must not slip. The maximum acceleration the segment can impose without slip is $a^I_\max = \mu g$, where $\mu$ is the static friction coefficient between input and segment surface and $g$ is gravitational acceleration. The jerk bound $j^I_\max$ is set by servo capability under the no-slip regime (the friction limit applies to acceleration, but the rate of change of acceleration is constrained by what the servo can deliver while remaining within the acceleration budget). These bounds apply to every motion of a segment during the intervals when it is bearing an input.

**$\pi^S$ bounds: servo capability.** During the free window $W_{i,k}$, no input is on segment $k$. Friction does not bind, because there is no input to slip. The kinematic bounds that apply to $\pi^S$ are therefore the servo's own bounds: $a^S_\max$ and $j^S_\max$, which are set by the motor and drive train rather than by contact friction. These are strictly greater than the $\pi^I$ bounds — typically by a substantial margin, because friction is the dominant constraint in the in-contact regime.

This bound separation is the reason the per-segment apparatus has explanatory power. If $\pi^I$ and $\pi^S$ shared the same bounds, the buffer would be a single coupled control problem; the segmentation would be cosmetic. The fact that $\pi^S$ enjoys a higher kinematic budget than $\pi^I$ is what makes the free window an exploitable resource and what makes the feasibility test $W_{i,k} \geq T_\min(\zeta_{i,k})$ a meaningful gate rather than a trivial one.

**Jerk boundary conditions are not enforced.** The boundary state $\xi$ at each crossing event records velocity and acceleration but not jerk. The model choice is that $\pi^I$ is piecewise linear in jerk — admitting infinite snap at segment boundaries — and $\pi^S$ inherits this convention for consistency. Jerk continuity across the boundary is not required, and the physical justification is that the snap discontinuity is absorbed by the elasticity of the drive train without observable consequence at the scale of the input motion. This is the same modeling choice made throughout the controls literature on piecewise-constant-jerk trajectories.

### 2.5 Operational Manifestation

In the simulator, the per-segment quantities of this section manifest as the `SegmentEvents` and `SegmentSyncResponse` types in `ksb.analysis`. `SegmentEvents` carries $t^\text{in}_{i,k}$, $t^\text{out}_{i,k}$, $\xi^\text{in}_{i,k}$, $\xi^\text{out}_{i,k}$, and $W_{i,k}$ as arrays indexed by pair and segment. `SegmentSyncResponse` derives $T_\min(\zeta_{i,k})$ via a per-cell Ruckig call under $\pi^S$ bounds, and exposes the kinematic margin $W_{i,k} - T_\min(\zeta_{i,k})$ and feasibility test $W_{i,k} \geq T_\min(\zeta_{i,k})$. The code uses 0-indexed pairs and segments where this document uses 1-indexed segments; otherwise the correspondence is direct.

## 3. Synchronization Error

A single batch generates many trajectory choices: each input enters the KSB, accelerates or decelerates to a chosen target slot, and exits at the registrar handoff. An earlier framing of this work tracked system health through the gap curve $g_i(t) = p_{i+1}(t) - p_i(t)$ between consecutive inputs — a continuous spatial signal, integrated and thresholded to produce scalar diagnostics (mean gap, violation integrals, violation durations). That framing is correct as physical description but operationally coupled: it conflates two mechanisms that turn out to require separate treatment, and it does not generalize cleanly when the buffer is internally divided into segments running at independent velocities.

The framing used throughout the rest of this work is the one already laid down in §2. The buffer is a sequence of segments, each of which is a control problem in its own right. Segment $k$ receives input $i$ at a known kinematic state $\xi^\text{out}_{i,k}$ at time $t^\text{out}_{i,k}$, must hand the next input $i+1$ off at a known target state $\xi^\text{in}_{i+1,k}$ at time $t^\text{in}_{i+1,k}$, and has the free window $W_{i,k}$ in which to execute the transition $\zeta_{i,k}$. **Synchronization** is the requirement that this happen everywhere — at every internal boundary, for every consecutive pair of inputs, for every segment of the buffer. When a segment has insufficient time, the requirement is violated.

There are two equivalent ways to describe such a violation.

**Time error.** The minimum time the segment needs to execute the demanded transition under its $\pi^S$ bounds, minus the time it has: $T_\min(\zeta_{i,k}) - W_{i,k}$. If this quantity is positive, the segment is temporally infeasible. This is a property of the transition $\zeta_{i,k}$ and the window $W_{i,k}$ alone; it makes no assumption about what the segment does when it cannot complete in time. The time error covers both of the failure modes introduced in §2: a timing collision ($W_{i,k} < 0$) gives a time error larger than $T_\min(\zeta_{i,k})$ itself, and a kinematic shortfall ($0 \leq W_{i,k} < T_\min(\zeta_{i,k})$) gives a positive but smaller time error.

**State error.** Given that the segment runs out of time, what kinematic state does it actually deliver at the receiving boundary, compared to the demanded $\xi^\text{in}_{i+1,k}$? This is a pair $(\Delta v, \Delta a)$ — a velocity and acceleration mismatch at the deadline. Unlike the time error, the state error depends on a degradation policy: it requires committing to _what the segment does_ when its transition cannot complete in the available window — for instance, saturate jerk and accept a residual velocity, or shorten the trajectory and accept a residual acceleration. The state error is well-defined only once such a policy is fixed.

The two readings carry equivalent information up to the choice of degradation policy. The time error is policy-free and is the natural object for failure-mode characterization, because it can be computed from $\zeta_{i,k}$ and $W_{i,k}$ alone — exactly the quantities §2 defines and the simulator records. The state error is the natural way to think about what the residual _means_ physically, and is the bridge to the control-theoretic reading of the buffer that §4 develops.

## 4. The Buffer as a Control Problem

§3 introduced synchronization error as the time-or-state gap between what a segment is asked to deliver and what it can deliver. This section steps back from the per-cell formalism and asks: what kind of control system is the buffer, viewed this way?

### 4.1 Per-Segment as Open-Loop Control

Take a single segment in isolation. At time $t^\text{out}_{i,k}$ — the moment input $i$'s trailing edge clears segment $k$'s downstream boundary — segment $k$ observes an initial kinematic state $\xi^\text{out}_{i,k}$. It is assigned a setpoint: the demanded state $\xi^\text{in}_{i+1,k}$, to be delivered when input $i+1$'s leading edge arrives at segment $k$'s upstream boundary at time $t^\text{in}_{i+1,k}$. It executes a fixed primitive — a closed-form motion profile, in the current implementation a Ruckig-generated jerk-limited trajectory under $\pi^S$ bounds — parametrized only by the transition $\zeta_{i,k}$ and the available window $W_{i,k}$. No measurement of input $i+1$ is made during execution. No correction is applied based on intermediate state. The segment runs the primitive open-loop from observed initial state to demanded terminal state.

The feasibility test is the comparison $W_{i,k} \geq T_\min(\zeta_{i,k})$ from §2.2. The kinematic margin $W_{i,k} - T_\min(\zeta_{i,k})$ is the signed feasibility slack of that one open-loop execution. Positive slack means the primitive completes early and there is excess window. Zero slack means the primitive saturates the window. Negative slack means the primitive cannot fit and the segment will deliver state error at the deadline — under whichever degradation policy is in force.

This is the simplest possible control structure: feedforward, fixed law, scalar feasibility test. It is well-suited to static implementation; the primitive has a closed form (via Ruckig) and the feasibility test is a single comparison.

### 4.2 Across-Segment as Semi-Closed-Loop

A buffer with $N^B = 1$ — a single segment spanning the whole length — is one open-loop execution per input. The trajectory is committed once at input entry and runs to completion at input exit, with no opportunity to re-anchor.

A buffer with $N^B > 1$ is structurally different. At each of the $N^B - 1$ internal boundaries, the segment that receives the input reads the _actual_ state at that boundary — including any state error left behind by the segment that just handed it off. The next segment's primitive is invoked from this true state, not from the planned one. The control law itself is fixed (no online optimization), but the trajectory is repeatedly re-anchored to the truth.

This is not a closed loop in the controller-design sense. No feedback gain is being tuned, no error signal is being fed back to a regulator. But it is a closed loop in the state-tracking sense: state errors injected by an earlier segment do not propagate unbounded down the buffer. They are absorbed into the next segment's feasibility budget, which either accommodates them (if its kinematic margin permits) or fails its own feasibility test (in which case the residual is again bounded by that segment, not by the entire buffer). Disturbance rejection is structural rather than active.

This observation gives $N^B$ a meaning beyond discretization. More segments means more state-observation instants along the buffer, which means earlier degradations are bounded faster. In the open-loop limit ($N^B = 1$), an upstream disturbance has the entire buffer length to express itself before any re-anchoring occurs. With $N^B$ large, the same disturbance is absorbed within a segment-length window. More segments mean more control authority but also more straddling overhead; this trade is a design choice rather than something this document optimizes.

### 4.3 Stochastic Upstream, Restated

The skip mechanism is set out in the system formalization: skips occur at structurally invariant rate $1 - \rho$, with individual skip intervals distributed around the mean $Q = 1/(1-\rho)$ with spread set by $\sigma_u$. In the per-segment view, this mechanism delivers itself to the buffer through one specific channel: the boundary conditions at the buffer's upstream-edge boundary — segment 1's upstream boundary.

At that boundary, $\xi^\text{in}_{i,1}$ is determined entirely by what the upstream stage hands over. Low $\sigma_u$ produces a narrow distribution of $\xi^\text{in}_{i,1}$ across the batch; segment 1 sees almost identical BCs every cycle, with predictable kinematic-margin pressure timed to the regular skip cadence. High $\sigma_u$ produces a broader distribution and scatters that pressure irregularly across batch position. The mean number of pressure events per batch is the same in either regime — set by the structural invariant $1 - \rho$ — but the dispersion across batch position differs.

A symmetric argument applies at segment $N^B$'s downstream boundary. There, $\xi^\text{out}_{i,N^B}$ is pinned by the registrar handoff: the input must be delivered to its assigned slot at the slot's velocity and acceleration. Both edge boundaries of the buffer are interfaces where the buffer cannot negotiate one of the BCs — the upstream edge inherits, the downstream edge is pinned — and the asymmetry between them and the internal boundaries is structural.

This asymmetry is the reason the segment-length parametrization $L^B_k$ matters at all. Internal segments have BCs negotiated by the segments on either side; their kinematic margins are coupled. The edge segments have one BC fixed by the surrounding system and the other negotiated only with their internal neighbor; their kinematic margins respond differently to upstream variance. The geometry of $L^B_k$ for $k \in {1, N^B}$ versus $k \in {2, \ldots, N^B - 1}$ is therefore a meaningful design degree of freedom even before any consideration of optimization.