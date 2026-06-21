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

KSB is a pure-Python package. Clone it, install, and run the simulation:

```bash
git clone https://github.com/JvPost/Portfolio.git
cd Portfolio/ksb
pip install -e .
python run_sim.py
```

`run_sim.py` runs a full batch simulation and prints a diagnostic summary — slot
assignment and skips, the time-horizon distribution, phase error, and the fraction of
segment-boundary events that are infeasible (the kinematic-margin failures the analysis
characterizes). Configuration is read from `configs/system/default.yaml`; point at a
different config with `--config NAME` and set the arrival seed with `--seed INT`.

### Viewer (optional)

The interactive viewer renders the four-stage pipeline — source, conditioning, buffer,
downstream — with items animating through it in real time. It needs an extra dependency:

```bash
pip install -e ".[viewer]"
python run_viewer.py
```

The viewer uses pygame and requires a display. On native Linux, macOS, or Windows it
opens directly; under WSL you will need an X server configured for the window to appear.

### Requirements

Python 3.11+. Core dependencies (NumPy, SciPy, Ruckig, PyYAML) install automatically with
`pip install -e .`; the viewer's pygame dependency is the `[viewer]` extra above.