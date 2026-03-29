
# Kinematic Synchronization Buffer — System Analysis

## 1. Introduction

Many automated production and logistics systems share a common structural challenge: a
stream of discrete objects arrives at irregular intervals from an upstream source and must
be handed off to a downstream process that expects them at precise, fixed intervals. The
upstream and downstream are mechanically decoupled — they run at independently set speeds
— and a buffer element in between is responsible for absorbing the timing mismatch and
delivering each object into the correct position at the correct moment.

This document analyzes one realization of that problem: the **Kinematic Synchronization
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

with $p_i(t)$ the position of object $i$ along the buffer axis. The hard constraint that
must hold at all times is $g_i(t) \geq g_{\min}$ for all $i$.

When the upstream arrival rate is close to the downstream slot rate, the system operates
in a near-balanced regime and the kinematic demands on the buffer are modest. The
interesting regime — and the focus of this analysis — is when the rates differ slightly,
forcing the buffer to occasionally skip a downstream slot and assign an object to the next
available one. These **skip events** are the primary driver of gap compression and the
primary source of safety constraint violations.

The core questions this document addresses are:

- What are the structural invariants of the system — quantities that depend only on the
  rate ratio and not on arrival variability?
- How does upstream arrival variability change the character of skip events and their
  downstream consequences?
- What scalar metrics best capture system health, and how do they behave as variability
  increases?
- Does the system require regime-specific solutions, or is there a single structural
  intervention that addresses the root cause across all variability levels?

The analysis proceeds from first principles, defining the system variables, deriving the
key metrics, and building toward a well-posed optimization problem. The treatment is
intentionally general: while the KSB is one physical instantiation, the same mathematical
structure appears in any system where a stochastic arrival stream must be synchronized to
a deterministic slot sequence under kinematic constraints — a pattern that recurs in
packaging, logistics sortation, assembly line feeding, and discrete-event manufacturing
more broadly.
## 2. System Definition

### 2.1 Upstream

The upstream is a conveyor running at constant velocity $v_u$. Objects arrive on it with
a spacing distribution that has mean $\mu_u$ and standard deviation $\sigma_u$. The
upstream delivery rate — the average number of objects arriving per unit time — is
therefore

$$
r_u = \frac{v_u}{\mu_u}
$$

The parameter $\sigma_u$ controls how regular the arrival spacing is. When $\sigma_u = 0$
objects arrive with perfectly uniform spacing $\mu_u$. As $\sigma_u$ increases, individual
spacings vary around the mean, and the timing of any particular object's arrival becomes
increasingly uncertain.

### 2.2 Downstream

The downstream is a continuously moving carrier with fixed slot spacing $d_s$, running at
constant velocity $v_d$. Slots pass the handoff point at a fixed rate

$$
r_d = \frac{v_d}{d_s}
$$

The downstream is entirely deterministic: slot timing is known exactly for all future
slots. The KSB's job is to deliver each object to the handoff point at the moment a slot
arrives there.

### 2.3 The KSB

The KSB is a servo-driven buffer segment of length $L$ that sits between upstream and
downstream. Each object enters the KSB at approximately $v_u$ and must exit at the
handoff point synchronized to an assigned downstream slot. The KSB achieves this by
executing a jerk-limited motion profile over the object's transit, subject to kinematic
bounds

$$
|v_i(t)| \leq v_{\max}, \quad |a_i(t)| \leq a_{\max}, \quad |\dot{a}_i(t)| \leq j_{\max}
$$

where $v_i(t)$, $a_i(t)$, and $\dot{a}_i(t)$ are the velocity, acceleration, and jerk of
object $i$ at time $t$. The position of object $i$ along the buffer axis is $p_i(t)$.

A batch of $B$ objects passes through the KSB in sequence. Objects are indexed
$i = 1, \ldots, B$, where $i = 1$ is the leading object and $i = B$ is the last. The
instantaneous gap between consecutive objects $i$ and $i+1$ while both are simultaneously
on the KSB is

$$
g_{i,i+1}(t) = p_{i+1}(t) - p_i(t)
$$

We write $g_i(t)$ for $g_{i,i+1}(t)$ throughout for readability. This gap must satisfy
the clearance constraint at all times:

$$
g_i(t) \geq g_{\min} \quad \forall\, i \in \{1, \ldots, B-1\}, \quad \forall\, t
$$
### 2.4 The Load Ratio and Skip Mechanism

The relationship between upstream and downstream is captured by a single dimensionless
number, the **load ratio**

$$
\rho = \frac{r_u}{r_d}
$$

For the system to be physically meaningful — upstream delivering into a downstream that
can absorb it — we require $\rho \leq 1$. The case $\rho = 1$ is perfect rate balance:
on average, one object arrives for every slot. The case $\rho < 1$ is the operating
regime of interest: the upstream is slightly slower than the downstream, so on average
fewer objects arrive than there are slots available.

This imbalance makes **skip events** structurally inevitable. Because objects arrive at
rate $r_u < r_d$, not every downstream slot can be filled. When the KSB determines that
an object cannot feasibly reach the next available slot in time — given its current
position and the kinematic bounds — it skips that slot and assigns the object to the
following one. The long-run fraction of slots that go unfilled is exactly $1 - \rho$, and
the average number of objects between consecutive skip events is

$$
Q = \frac{1}{1 - \rho}
$$

Both $\rho$ and $Q$ are structural invariants: they depend only on $v_u$, $\mu_u$,
$v_d$, and $d_s$, and are independent of $\sigma_u$. No matter how irregular the upstream
spacing becomes, the long-run skip rate is unchanged. What variability does affect is the
timing of individual skip events.

## 3. Gap Metrics and Violation Quantities

A single gap curve $g_i(t)$ tells you what happened between one specific object pair
during their shared time on the KSB. But to understand how the system performed across an
entire batch — or how changing $\sigma_u$ affects performance — we need scalar summaries
that can be computed per pair and compared across hundreds of objects or across different
simulation runs.

The quantities defined in this section serve exactly that purpose. Because $g_i(t)$ is a
continuous curve, integrating it over the co-occupancy interval collapses the full
time-varying behavior into a single number. The violation integral goes further: it
isolates only the region where the clearance constraint is breached and quantifies its
severity. Together these quantities are the primary diagnostic signals for batch-level
analysis — plot them against object index and you immediately see where compression
concentrates, how severe it is, and how long recovery takes.

### 3.1 Co-occupancy Interval

The gap $g_i(t)$ is only defined while objects $i$ and $i+1$ are simultaneously on the
KSB. We call this the **co-occupancy interval** $[t_a^i,\, t_b^i]$, where $t_a^i$ is the
moment object $i+1$ enters the KSB and $t_b^i$ is the moment object $i$ exits it. All
integrals below are taken over this interval.

### 3.2 Mean Gap

The total gap integral over the co-occupancy interval is

$$
G_i = \int_{t_a^i}^{t_b^i} g_i(t)\, dt \quad [\text{m} \cdot \text{s}]
$$

Normalized by the co-occupancy duration, this gives the **time-averaged gap**:

$$
\bar{g}_i = \frac{G_i}{t_b^i - t_a^i} = \frac{1}{t_b^i - t_a^i}
\int_{t_a^i}^{t_b^i} g_i(t)\, dt \quad [\text{m}]
$$

This is the most intuitive summary of the spacing between two objects during their shared
time on the KSB — simply the average clearance. It is directly comparable across
different belt lengths and conveyor speeds, because the normalization by co-occupancy
duration makes $\bar{g}_i$ invariant to those system parameters.

### 3.3 Violation Integral

When $g_i(t)$ drops below $g_{\min}$, the **violation integral** measures the total
severity of that shortfall:

$$
E_i = \int_{t_c^i}^{t_d^i} \bigl(g_{\min} - g_i(t)\bigr)\, dt \quad [\text{m} \cdot \text{s}]
$$

where $t_c^i$ and $t_d^i$ are the times at which $g_i(t)$ crosses $g_{\min}$ from above
and below respectively. This quantity measures how far below $g_{\min}$ the gap went,
integrated over time. If there is no violation, $E_i = 0$.

### 3.4 Violation Duration

The total time the gap spends below $g_{\min}$:

$$
D_i = t_d^i - t_c^i \quad [\text{s}]
$$

### 3.5 Reading the Three Quantities Together

$\bar{g}_i$ is a high-level health indicator for the pair — it tells you the average
clearance. $E_i$ tells you how severe a violation was when it occurred, combining depth
and duration into a single number. $D_i$ tells you how long the violation lasted.

A large $E_i$ with a small $D_i$ means a brief but deep compression. A small $E_i$ with
a large $D_i$ means a prolonged but shallow one. Both matter: deep compressions indicate
high collision risk, while prolonged ones indicate sustained mechanical stress. Used
together, the three quantities give a complete picture of clearance constraint behavior
for any object pair in the batch.

## 4. The Skip Mechanism

### 4.1 Phase Error

At any point during operation, each object $i$ arrives at the KSB entry with some timing
relative to the nearest available downstream slot. We define the **phase error** $\phi_i$
as the signed timing offset between object $i$'s arrival and its nearest slot, expressed
in units of slot periods $1/r_d$. A phase error of zero means the object arrives exactly
in phase with a slot. A phase error of $+0.5$ means it arrives halfway between two slots.

Between skip events, $\phi_i$ drifts upward at a rate determined by the mismatch between
upstream and downstream rates. Because on average one fewer object arrives per $Q$ slots,
the phase accumulates at rate $r_d - r_u$ per object. This produces the characteristic
sawtooth pattern: a gradual upward ramp in $\phi_i$ as objects arrive progressively later
relative to slots, followed by a downward jump of approximately one slot period when a
skip occurs and the phase resets.

A skip is triggered when $\phi_i$ exceeds the feasibility boundary — the maximum phase
error for which the KSB can still deliver object $i$ to the next slot given its current
position and kinematic bounds. At that point the KSB skips the next slot and assigns
object $i$ to the one after it, resetting $\phi_i$ by approximately one slot period.

### 4.2 Structural Invariants

Because $\rho$ is determined purely by the ratio of mean rates, several system-level
properties are invariant under changes in $\sigma_u$:

- The **load ratio** $\rho = r_u / r_d$ — by definition, independent of variability
- The **long-run skip fraction** $1 - \rho$ — the fraction of slots that go unfilled
- The **mean skip interval** $Q = 1 / (1 - \rho)$ — the expected number of objects
  between consecutive skip events
- The **mean upstream spacing** $\mu_u$ — a parameter of the arrival distribution,
  not a consequence of it
- The **slot timing sequence** — entirely deterministic, set by $v_d$ and $d_s$

These quantities define the skeleton of the system. They tell you how many skips to
expect per hundred objects and what the average correction load looks like. None of this
changes when $\sigma_u$ increases. What $\sigma_u$ does affect is the timing of individual
skip events — the subject of the next subsection.

### 4.3 Individual Skip Intervals

While the mean skip interval $Q$ is fixed, the **individual skip intervals** are not. In
the low-variability regime ($\sigma_u \approx 0$), skips occur almost periodically —
every $Q$ objects, with only minor fluctuation. The phase drift is smooth and nearly
linear between skips, and the sawtooth pattern is clean and regular.

When $\sigma_u$ increases, individual objects arrive early or late relative to the mean
spacing $\mu_u$. This perturbs the phase drift trajectory: an object arriving earlier
than expected reduces $\phi_i$ slightly, delaying the next skip; one arriving later
increases $\phi_i$, triggering a skip sooner. The result is that individual skip
intervals become random — distributed around the mean $Q$ with a spread that grows with
$\sigma_u$ — while the long-run average remains exactly $Q$.

The two regimes are therefore:

- **Low variability** ($\sigma_u \approx 0$): skips are nearly periodic, occurring at
  predictable positions in the object stream. The phase error sawtooth is clean and
  regular.
- **High variability** (large $\sigma_u$): skips are scattered irregularly across the
  object stream. The sawtooth is noisy and individual skip intervals vary considerably,
  but the histogram of skip intervals is centered on $Q$ in both cases.

This distinction has a direct consequence for gap compression, developed in the next
section: in the low-variability regime, compression events are predictable in both timing
and magnitude; in the high-variability regime, the same compression mechanism operates
but its timing is uncertain. The total compression load — averaged over a long batch —
is the same in both regimes.

## 5. Gap Compression in Both Regimes

### 5.1 The Compression Mechanism

After a skip event, the object immediately following — the **post-skip object** — is
assigned to a slot that is earlier than the one it would naturally reach. To hit this
earlier target, the KSB must decelerate it more aggressively than a typical correction.
The object behind it, however, is still traveling at approximately $v_u$, unaware of the
skip. The result is a transient reduction in $g_i(t)$ — the gap closes as the post-skip
object slows while its follower continues forward at infeed speed. This is **gap
compression**, and it is the primary mechanism by which the clearance constraint
$g_i(t) \geq g_{\min}$ is violated.

The severity of the compression is determined by the magnitude of the required
deceleration, which is in turn determined by how far the post-skip object needs to be
pulled back relative to its natural trajectory. This depends on the phase error $\phi_i$
at the moment of the skip: a larger phase error means a more aggressive correction and
deeper compression.

Gap compression is not instantaneous. It persists over a **recovery window** of $M$
objects following the skip, as the phase drift rebuilds from zero and successive objects
receive progressively lighter corrections. During this window $E_i$ and $D_i$ are
elevated above their baseline values, decaying back toward zero as the system recovers.

### 5.2 Low-Variability Regime

When $\sigma_u \approx 0$, the phase error at each skip is nearly identical — the
sawtooth reaches the feasibility boundary at the same height every $Q$ objects. The
post-skip correction is therefore the same every cycle: same deceleration magnitude, same
compression depth, same recovery shape. The violation integral $E_i$ and violation
duration $D_i$ produce clean, periodic spikes at every skip position, with the spike
shape repeating exactly every $Q$ objects.

This regime is fully predictable. A system designer can characterize the compression
pulse analytically from the kinematic bounds and the phase error at the skip boundary,
and the recovery window $M$ is fixed and known. The entire batch behavior is determined
by a single skip cycle.

*[Figure: $E_i$ vs object index for $\sigma_u \approx 0$ — clean periodic spikes at
skip positions, uniform height and spacing.]*

### 5.3 High-Variability Regime

When $\sigma_u$ is large, the same compression mechanism operates but its timing and
magnitude become random. Three things change relative to the low-variability regime:

- **Skip timing is uncertain.** The phase error sawtooth is noisy, so the feasibility
  boundary can be crossed earlier or later than the mean skip interval $Q$ would suggest.
  Compression pulses arrive at irregular positions in the object stream.
- **Compression magnitude varies.** The phase error at the moment of the skip is no
  longer the same every cycle — it depends on the accumulated spacing errors of the
  preceding objects. A skip triggered early (by a sequence of late-arriving objects)
  carries a larger phase error and produces deeper compression than one triggered late.
- **Recovery windows can overlap.** If two skips occur within $M$ objects of each other,
  the compression from the first has not fully decayed before the second begins. The gaps
  $g_i(t)$ in the overlap region are suppressed by both events simultaneously, increasing
  violation severity.

*[Figure: $E_i$ vs object index for moderate $\sigma_u$ — spikes of varying height at
irregular positions, occasionally overlapping.]*

*[Figure: $E_i$ vs object index for large $\sigma_u$ — more frequent overlap, higher
peak violations.]*

### 5.4 What the Two Regimes Share

Despite the difference in predictability, the underlying mechanism is identical. A skip
forces a post-skip deceleration. That deceleration compresses the gap. The compression is
the problem. This has a direct implication for solution design: any intervention that
reduces the required post-skip deceleration magnitude — regardless of when the skip
occurs — will reduce compression in both regimes. A solution that instead exploits the
periodicity of skips will work well when $\sigma_u \approx 0$ but degrade as skip timing
becomes uncertain.

The long-run compression load is also identical across regimes. Because $\rho$ and $Q$
are invariant under $\sigma_u$, the average number of skip events per batch is the same
regardless of variability. What changes is the distribution of that load across the
object stream — concentrated in predictable pulses at low $\sigma_u$, scattered
irregularly at high $\sigma_u$.

### 5.5 The Violation Probability

Aggregating across the batch, the primary performance metric is the **violation
probability per object pair**:

$$
\varepsilon = \frac{1}{B-1} \sum_{i=1}^{B-1} \mathbf{1}\!\left[\min_t\, g_i(t) < g_{\min}\right]
$$

This is the empirical fraction of object pairs for which the clearance constraint was
breached at least once during co-occupancy. It is the natural scalar KPI for the system:
a single number that summarizes constraint satisfaction across the entire batch, and the
quantity that must be kept below an application-specific threshold $\varepsilon_{\max}$.

As $\sigma_u$ increases from zero, $\varepsilon$ rises gradually from near zero and
eventually plateaus. The plateau is structurally explained by the skip mechanism: at high
$\sigma_u$, violations are almost entirely driven by post-skip compression events, which
occur at a rate set by $1 - \rho$ — a structural invariant independent of $\sigma_u$.
Once nearly every post-skip pair violates, adding more variability cannot increase
$\varepsilon$ further. The ceiling of $\varepsilon$ is therefore set by $\rho$, not by
$\sigma_u$.

*[Figure: $\varepsilon$ vs $\sigma_u$ — gradual rise from near zero, plateau around
$1 - \rho$.]*

The smooth shape of this curve — no sharp inflection point, no sudden collapse —
supports a single general solution targeting post-skip compression directly, rather than
two separate regime-specific designs.