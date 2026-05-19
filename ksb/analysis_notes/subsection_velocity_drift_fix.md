# Fix plan: velocity drift in `subsection`

## Problem recap

`CompositeTrajectory.__post_init__` checks velocity continuity at every junction
with `atol=1e-4`. After the snap guard (which fixed the `seg_remaining < 0` crash),
five seeds still fail with:

```
Velocity or acceleration discontinuity between segment 0 and 1
  prev end: v=1.550381, a=0.049822
  next start: v=1.550551, a=0.000000
```

The discrepancy (~1.6–2.5 × 10⁻⁴ m/s) is between the end velocity of the
upstream `CompositeTrajectory` returned by `subsection` and the initial velocity
of the buffer trajectory, which is set by the caller as
`v_in = state_at(t_in)[V]`.

## Root cause

`subsection` maintains `v_now` and `a_now` by propagating them through
`candidate.eval(seg_remaining)` at each step:

```python
end = candidate.eval(seg_remaining)
v_now = float(end[V])
a_now = float(end[A])
```

`candidate` was created with `x0 = [0, v_now_prev, a_now_prev]` — values that
already carried accumulated FP rounding from earlier iterations. The end state
of the candidate is therefore computed from drifted initial conditions, and
the drift compounds at every segment boundary.

Meanwhile, in `ksb_simulation.py`:

```python
v_in = self._u_control.state_at(t_in)[V]
```

`state_at` integrates the jerk timeline directly from scratch each call — no
accumulated drift. The two quantities diverge from the very first segment they
disagree on, and the divergence can reach ~2 × 10⁻⁴ m/s by the time
`subsection` exits.

## Why the discrepancy is so large

The upstream acceleration profile has a long hold phase (~40 s with a = 0.05 m/s²).
Even a tiny drift in `a_now` (e.g. 5 × 10⁻⁶ m/s² from one bad ramp) compounds
over 40 s of hold: `δv = δa × t_hold ≈ 2 × 10⁻⁴ m/s`. This is above the 1 × 10⁻⁴
tolerance. Conversely, shorter hold durations (larger `a_u_max`) keep the
accumulated drift below the threshold — explaining why the bug is sensitive to
specific `a_u_max` values.

## Fix: re-anchor `v_now` / `a_now` from `state_at` at the start of each iteration

Instead of propagating from a potentially drifted previous state, call
`self.state_at(t_now)` **at the top of every loop body** (after `_segment_at`
returns the correct segment). This resets both `v_now` and `a_now` to the
timeline's authoritative values at `t_now` before constructing each candidate.

```python
while remaining > 1e-10:
    self._ensure_covered(t_now, remaining, v_now)
    seg_t, seg_j, seg_T = self._segment_at(t_now)

    # Re-anchor to the authoritative timeline; eliminates FP drift accumulation
    x_auth = self.state_at(t_now)
    v_now  = float(x_auth[V])
    a_now  = float(x_auth[A])

    elapsed       = t_now - seg_t
    seg_remaining = seg_T - elapsed

    if seg_remaining < 1e-9:
        t_now = seg_t + seg_T
        continue          # next iteration re-anchors automatically

    x0_local  = np.array([0.0, v_now, a_now])
    candidate = ConstantJerkTrajectory(x0=x0_local, T=seg_remaining, jerk=seg_j)
    ...
```

### Why this closes the internal junction continuity too

Because `state_at` and the `ConstantJerkTrajectory` both integrate the same cubic
polynomial over the same time span with the same (jerk, a0, v0), their results
agree to ~10⁻¹⁴ m/s — well within the 1 × 10⁻⁴ atol. Concretely:

- Segment N is created with `x0[V] = state_at(t_N)[V]`.  
- Its end velocity = `state_at(t_N)[V] + state_at(t_N)[A] × T + 0.5 × j × T²`.  
- Segment N+1 is created with `x0[V] = state_at(t_{N+1})[V]`  
  = `state_at(t_N)[V] + state_at(t_N)[A] × T + 0.5 × j × T²`  
  (same formula, same inputs).  

The junction error is purely FP rounding of a single arithmetic expression —
O(10⁻¹⁴), not accumulated drift.

### Effect on the external junction

The last partial segment is created with:
```python
x0_last = np.array([0.0, v_now, a_now])   # v_now = state_at(t_now_last)[V]
```

Its end velocity integrates from the authoritative initial conditions. This
matches `state_at(t_in)[V]` (used by the buffer planner as `v_in`) to within
a single-step rounding error — closing the external junction as well.

## Scope and cost

- Change is entirely inside `subsection`; no caller changes needed.
- `state_at` is O(M) in the number of timeline segments M. Called once per loop
  iteration; a typical upstream trajectory spans 3–5 segments, so M × 5
  integrations per input. Negligible overhead.
- The snap guard (`seg_remaining < 1e-9`) stays in place and needs no update;
  the re-anchor on the next iteration handles state correctly after a snap.
- `_ensure_covered` uses `v_now` only for a rough duration estimate — calling
  it before the re-anchor is fine.
