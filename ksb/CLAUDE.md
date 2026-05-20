# KSB — Kinematic Synchronization Buffer
## Project Context for Claude Code

---

## What This Project Is

KSB is a Python simulation library modeling a **kinematically constrained flow
synchronizer**: a buffer that receives inputs arriving stochastically and must
deliver each one to a fixed, deterministic departure schedule (a sequence of
equally spaced "slots") using bounded-jerk motion profiles.

The canonical physical instance is a high-speed packaging conveyor, but the
abstraction is general. The system is mathematically equivalent to a queueing
problem where service times are not random but kinematically constrained — the
"server" is a triple integrator (jerk → acc → vel → pos) with hard bounds on
each derivative.

**This is a portfolio project.** Correctness, clarity, and mathematical
elegance are first-class requirements. The code must be readable by someone
with a control engineering or robotics background who has never seen this repo.

---

## Repository Layout
```
ksb/                              ← repo root (run Claude Code from here)
  ksb/                            ← installable Python package
    motion/
      trajectories.py             ← trajectory primitives + CompositeTrajectory
      item_pair.py                ← PairRecord dataclass, compute_pairs()
    planning/
      contracts.py                ← bounds indices, Policy, exceptions, IProfileSolver
      planner.py                  ← MotionPlanner (thin solver wrapper)
      solvers/
        quintic.py                ← QuinticTrajectorySolver  (6×6 linear system)
        scurve.py                 ← SCurveSolver             (7-phase, bisection)
        linear.py                 ← LinearTrajectorySolver   (constant velocity)
    control/
      control_profile.py          ← ConstantJerkControl (upstream/downstream belt)
    simulation/
      ksb_simulation.py           ← KSBSimulation (main entry point)
      result.py                   ← SimulationResult dataclass
      utils.py                    ← spawn time generators, get_assigned_slots()
    viewer/                       ← pygame visualizer (may not exist yet)
      __init__.py
      viewer.py                   ← KSBViewer class
  run_sim.py                      ← CLI: run simulation and print summary
  run_viewer.py                   ← CLI: run pygame viewer (may not exist yet)
  pyproject.toml
  README.md
  CLAUDE.md                       ← this file
```

---

## Naming and Mathematical Conventions

These conventions are non-negotiable. Every contributor (human or AI) must
follow them.

### State vector

The kinematic state is a numpy array `x = np.array([p, v, a])` where:
- `p` — position along the buffer axis (metres). **Not `x`** — that name is
  reserved for the full state vector, which may grow in future extensions
  (e.g. adding mass, momentum).
- `v` — velocity (m/s)
- `a` — acceleration (m/s²)

Index constants are defined in `ksb/motion/trajectories.py`:
```python
P, V, A = 0, 1, 2   # indices into state vector x = [p, v, a]
```
Use `x[P]`, `x[V]`, `x[A]` everywhere. Never use `x[0]`, `x[1]`, `x[2]`
directly — the named indices are documentation.

### Bounds array

Kinematic bounds are a numpy array `bounds = np.array([j_max, A_max, V_max, g_min])`.
Index constants are defined in `ksb/planning/contracts.py`:
```python
J_MAX, A_MAX, V_MAX, G_MIN = 0, 1, 2, 3
```

### No State or Bounds dataclasses

There is no `State` class and no `Bounds` class anywhere in this codebase.
If you see one being constructed, it is a bug.

### Trajectory evaluate() convention

All `TrajectoryProfile.evaluate(t)` methods return:
- Scalar `t` → `np.ndarray` shape `(3,)` = `[p, v, a]`
- Array `t` → `np.ndarray` shape `(3, N)` = `[[p...], [v...], [a...]]`

Position is always **relative** (delta from segment start = 0).
`CompositeTrajectory` accumulates absolute position across segments internally.

### Solver interface

All solvers implement:
```python
def solve(self, pi, vi, pf, vf, T, bounds, policy) -> TrajectoryProfile
```
where `pi, vi, pf, vf, T` are plain Python floats and `bounds` is the numpy
bounds array. Raise `InfeasibleError` (from `ksb.planning.contracts`) if no
valid trajectory exists.

---

## Architecture Principles

**Closed-form trajectory representation.** Every trajectory type is an
analytically evaluable function object. Call `.evaluate(t)` at any `t` without
a numerical integrator. This makes gap computation, bounds checking, and
visualization exact and fast.

**Composition over inheritance.** `CompositeTrajectory` sequences primitives.
Adding a new trajectory type means implementing one `TrajectoryProfile` subclass.

**Solvers are stateless strategies.** Each solver is a frozen dataclass
implementing `IProfileSolver`. `KSBSimulation` injects the solver at
construction, allowing easy swapping for comparison experiments.

**No numerical integration at the simulation level.** Trajectories are
analytical; gap curves are computed by evaluating two trajectories at the
same time grid.

**Visualization is a separate concern.** The `viewer/` package depends on the
core `ksb` package but the core has no pygame dependency. Never add a pygame
import to any file outside `ksb/viewer/`.

---

## Current Status

The core simulation pipeline is complete and tested:
- All three solvers: `QuinticTrajectorySolver`, `SCurveSolver`,
  `LinearTrajectorySolver`
- `KSBSimulation.run()` produces `SimulationResult` with trajectories,
  pair records, skip indices, and phase errors
- Three arrival process models: truncated Gaussian, AR(1) log-space, lognormal
- `run_sim.py` CLI works end-to-end

**Not yet implemented (see Active Tasks below):**
- Pygame viewer (`ksb/viewer/`)
- Non-zero initial/final acceleration in solvers
- Upstream/downstream jitter models exposed in `KSBSimulation`
- Stochastic epsilon-vs-sigma analysis

---

## Active Development Tasks

Listed in priority order.

---

### TASK 1 — Pygame viewer  [done]

**Goal:** Real-time pygame visualization of a `SimulationResult`. Used for
debugging and intuition-building. Scrubs through simulation time and shows
inputs moving through upstream → buffer → downstream regions.

**Output files:**
- `ksb/viewer/__init__.py`
- `ksb/viewer/viewer.py`   — main class: `KSBViewer`
- `run_viewer.py`          — CLI entry point

**`KSBViewer` class interface:**
```python
viewer = KSBViewer(result: SimulationResult, cfg: dict, speed: float = 1.0)
viewer.run()
```
Constructed from a `SimulationResult` and the same `cfg` dict used to produce
it. `speed` is the initial playback multiplier.

**Visual layout (horizontal, left-to-right):**
```
|← L_upstream →|←      L_buffer      →|← L_downstream →|
   upstream zone      buffer zone          downstream zone
```

- Draw three labeled zones as horizontal bands, separated by vertical lines.
- Buffer zone gets a slightly different background shade — it is the
  interesting region.
- Total window width scales with total line length × pixels-per-metre.
- Window height: fixed ~120px for the belt lane + ~30px HUD strip at bottom.

**Inputs:**
- Each input is a filled rectangle, width = `input_length`, height = belt lane height − padding.
- Alternating blue shades for easy visual separation (`I1`, `I2`, ...).
- Label `Ii` drawn above the rectangle.
- When two consecutive inputs are closer than `g_min`, both rectangles turn
  red. This is the primary debugging feature — it shows gap compression events
  in real time.
- Inputs only appear once `t >= t_spawn[i]` and disappear once they exit the
  downstream zone.

**Slot separators:**
- Moving vertical lines in the downstream zone, advancing at speed `vd`.
- Represent the departure schedule. Spawned periodically at the buffer/downstream
  boundary at rate `rd`.

**HUD strip (bottom):**
```
t =  3.142 s   |   1.0×   |   solver: scurve   |   inputs: 20   skips: 2   violations: 3
```

**Controls:**
| Key | Action |
|---|---|
| `SPACE` | Pause / resume |
| `R` | Reset to t = 0 |
| `←` / `→` | Step ±1 second |
| `[` / `]` | Step ±1 frame |
| `+` / `-` | Playback speed ×2 / ÷2 |
| `ESC` or close | Quit |

**Gap warning overlay:**
At each frame, for each consecutive pair of inputs both currently visible,
evaluate their instantaneous gap. If gap < `g_min`, draw a faint red
band between them spanning the full belt height.

**Input position at time t:**
Each input has a `CompositeTrajectory` in `result.composite_trajectories[i]`.
The trajectory is defined in the frame of input `i` starting at `t_spawn[i]`.
At simulation time `t`, evaluate:
```python
t_local = t - result.t_spawn[i]
if 0 <= t_local <= traj.T:
    state = traj.evaluate(t_local)   # shape (3,)
    p_lead = state[P]                # leading edge position
```
The trailing edge is at `p_lead - input_length`.

**`run_viewer.py` CLI:**
```
python run_viewer.py [--solver quintic|scurve] [--seed INT] [--batch INT]
                     [--std FLOAT] [--speed FLOAT] [--ppm FLOAT]
```
Runs `KSBSimulation`, then `KSBViewer(result, cfg, speed=args.speed).run()`.

---

### TASK 2 — Upstream belt velocity modulation as a control intervention  [done]

**Goal:** The upstream belt currently runs at constant velocity `vu`. This task
adds an optional, deliberate velocity modulation profile to the upstream belt —
an active control intervention to pre-condition the gap distribution *before*
inputs enter the buffer.

**Motivation (from docs):** Post-skip gap compression is the dominant failure
mode. In the low-variability regime, skips are nearly periodic with interval
`Q = 1/(1-rho)`. This predictability opens the door to a feedforward strategy:
if the upstream belt decelerates slightly before a skip event and re-accelerates
afterward, it can spread inputs more evenly going into the skip, reducing the
compression load on the buffer. The docs identify this as a candidate "Option A"
general solution — one that targets the structural cause (post-skip deceleration)
rather than reacting to it after the fact.

**What already exists:** `ConstantJerkControl` in `ksb/control/control_profile.py`
is already a general piecewise-constant-jerk control profile applied to the
upstream belt. Currently it is always instantiated with `jerks=[0.0]` (zero
jerk, constant velocity). The infrastructure for non-constant upstream motion
already exists — it just needs to be exposed and parameterized.

**The task:**

1. Allow `KSBSimulation` to accept an upstream belt control profile via cfg
   or constructor, instead of hardcoding `ConstantJerkControl(jerks=[0.0], durations=[100])`.

2. Implement a `PeriodicModulationControl` strategy — a control profile that
   applies a repeating acceleration/deceleration pattern synchronized to the
   skip cadence `Q`. The pattern should be parameterizable (amplitude, phase,
   shape). This is the feedforward intervention hypothesized in the docs.

3. The downstream belt remains constant velocity for now (inputs are already
   corrected by the time they reach it).

4. The simulation result and gap metrics are unchanged — the intervention is
   evaluated by comparing `violation_integral` and `epsilon` against the
   baseline constant-velocity case.

**Design note from Marco:** The upstream modulation is computed offline based
on known system parameters (`rho`, `Q`, `vu`, `gap_mean`). It does not require
real-time feedback. Whether it should be a periodic sinusoid, a sawtooth, or a
jerk-limited S-curve shape synchronized to `Q` is an open question to be
explored empirically via the simulation. The `ConstantJerkControl` primitive
is already the right building block — the question is what jerk sequence and
cadence to use.

---

### TASK 3 — Arrival model selection in KSBSimulation  [done]

**Goal:** Expose all three arrival models via cfg:
```python
cfg["arrival_model"] = "gaussian"    # default (truncated Gaussian)
cfg["arrival_model"] = "ar1"         # log-space AR(1), serial correlation
cfg["arrival_model"] = "lognormal"   # shifted lognormal, one-sided floor
```
Dispatch in `KSBSimulation.run()`. Also add a `gap_jitter` key: small
zero-mean Gaussian noise on each input's initial velocity at buffer entry
(models instantaneous belt speed variation, distinct from gap spacing noise).

---

## Key SimulationResult Fields

| Field | Shape | Meaning |
|---|---|---|
| `t_spawn` | `(B,)` | Absolute arrival times at upstream origin |
| `t_control_start` | `(B,)` | Times inputs enter the buffer |
| `assigned_slots` | `(B,)` | Integer slot indices per input |
| `time_horizons` | `(B,)` | Duration of buffer correction per input |
| `skip_indices` | `(K,)` | Input indices after which a slot was skipped |
| `phi_u` | `(B,)` | Phase error at upstream reference (slot periods) |
| `phi_0` | `(B,)` | Phase error at buffer entry (slot periods) |
| `composite_trajectories` | `List[CT]` | Full trajectory per input (upstream+buffer+downstream) |
| `buffer_trajectories` | `List[TP]` | Buffer segment only |
| `pair_records` | `List[PairRecord]` | Gap curves + integrals for each consecutive pair |

### Key PairRecord fields (after `compute_integrals()`)

| Field | Meaning |
|---|---|
| `min_gap` | Worst-case instantaneous gap (m) |
| `violation_integral` | Area below g_min integrated over time (m·s) |
| `violation_duration` | Total time below g_min (s) |
| `average_margin` | Mean gap above g_min over co-occupancy window (m) |
| `g_min_threshold` | The g_min used when integrals were computed |

---

## Environment

This project uses a conda environment named `ksb`. Activate it before running
any Python commands:
```bash
conda activate ksb
```

---

## Smoke Test

After any change, run:
```bash
python run_sim.py --solver scurve --seed 42 --batch 20
```
Must complete without exceptions and print slot assignment + gap metrics.

## Quick Import
```python
import numpy as np
from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.planning.solvers.scurve import SCurveSolver

cfg = {
    "jmax": 50.0, "Vmax": 3.0, "Amax": 8.5,
    "L_upstream": 1.0, "L_buffer": 2.0, "L_downstream": 1.0,
    "input_length": 0.32, "N": 5,
    "slot_length": 0.40,
    "input_gap_mean": 0.80, "input_gap_std": 0.05,
    "arrival_rate_ppm": 175, "slot_rate_ppm": 180,
    "batch": 30,
}
result = KSBSimulation(cfg=cfg, solver=SCurveSolver()).run(seed=42)
```

---

## Rules for Claude Code

1. Read this file in full before starting any task.
2. Read all relevant source files before writing any code.
3. State your plan (files to create/modify, key decisions) before writing code.
4. Follow naming conventions — `p` for position, `x` for state vector,
   `x[P]`/`x[V]`/`x[A]`, bounds as numpy array with index constants.
5. Never introduce a `State` class or `Bounds` class.
6. Never add a pygame import outside `ksb/viewer/`.
7. Preserve solver mathematics exactly. If math changes, say what and why.
8. Run the smoke test after completing any task.