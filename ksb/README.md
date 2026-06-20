# KSB — Kinematic Synchronization Buffer

A stochastic stream of items arrives at irregular spacing. A downstream carrier presents
fixed slots at a constant rate. Between them sits a servo-driven buffer that must place
every item into its slot — on time, in order, without collision — using only bounded-jerk
motion. This project formalizes that problem from first principles and characterizes,
quantitatively, where and why it fails.

The KSB is an open-source reconstruction of an industrial conveyor-synchronization problem.
It is built around a single diagnostic quantity, the **kinematic margin** $M_{i,k}$ — the
slack, in seconds, between the time a buffer segment has to reconfigure between two items
and the minimum time that reconfiguration physically takes under jerk-limited motion. Where
the margin is positive the buffer keeps up; where it goes negative the buffer fails, and the
two ways it can go negative — a *timing collision* (two items on one segment) and a
*kinematic shortfall* (not enough time to move) — are the two failure modes the whole
analysis turns on.

The interesting behavior lives in the gap between the arrival rate and the slot rate. The
analysis traces a chain of failure modes, each one the residual left by the fix for the last:

- **Unbounded drift.** With equal rates, the synchronization horizon performs a
  reflected random walk — a Lindley recursion at critical load — and drifts without bound.
  A downstream rate surplus gives the recursion negative drift and bounds it.
- **The exit edge.** The downstream handoff is pinned to a constant velocity, so its margin
  deficit is deterministic and structural. Widening the slot rate resolves it.
- **The entry edge.** The upstream boundary inherits the arrival *distribution*, so its
  deficit is stochastic — the negative tail of a margin distribution rather than a constant
  shortfall. A conditioning conveyor stretches the mean spacing to relieve it, but the same
  linear stretch inflates the variance it is fighting, and the two are locked together.
- **Consecutive placement.** Solving both edges leaves one residual that belongs to neither:
  the buffer relocates the upstream variance, spread evenly between items, into rare
  tightly-spaced pairs at the exit. This cannot be forbidden without reintroducing the drift,
  and is characterized as an open problem with candidate directions named, not solved.

The methodology throughout is to **characterize a failure mode completely before naming any
subsystem that responds to it**. The buffer, its segmentation, and the downstream rate
surplus are each derived as the necessary answer to a specific, quantified failure — not
assumed up front. The last three failure modes are presented as open frontier: posed
sharply, with the constraints any solution must satisfy stated explicitly, and left for
future work rather than overstated.

The full derivation is in [`docs/`](docs/): the system formalization, the per-segment buffer
formalization, and the architecture derivation that chains the failure modes above.

## Running it

<!-- TODO: install + run instructions go here once the packaging approach is settled -->