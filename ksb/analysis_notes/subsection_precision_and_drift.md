# Precision and drift bugs in `subsection`

## Bug 1 — `seg_remaining` goes negative (FP boundary overshoot)

### What it looked like

`ConstantJerkTrajectory(T=seg_remaining, ...)` raised `ValueError("Duration T must be
positive")` for certain `a_u_max` values. The constructor checks `abs(T) < 1e-9`, so
even a tiny positive like `3e-16` crashes it; negative values below `−1e-9` silently
produce a junk segment.

### Root cause

`_segment_at(t_now)` searches in reverse and returns the last segment whose
`seg_t ≤ t_now + 1e-12`. The stored segment start times come from one FP summation
chain (inside `_append_acceleration`), while `t_now` inside `subsection` comes from a
completely independent chain starting at `t0` (spawn time). The two representations of
the same real boundary can diverge by a few ULPs.

At simulation times of 50–100 s, one ULP ≈ `t × 2⁻⁵² ≈ 1–2 × 10⁻¹⁴`. The divergence
is typically `O(10⁻¹⁴)`, well within the 1e-12 tolerance — but with very short ramp
segments (`t_ramp = a_u_max / j_u_max`), even a 2–3 ULP error can push `t_now`
fractionally past the ramp-end boundary, making `seg_remaining` slightly negative.

`a_u_max = 0.05` gives `t_ramp = 0.5 ms`, so any boundary misalignment of `O(1e-14)`
is a meaningful fraction of that window.

### Fix

Guard before constructing the trajectory:

```python
if seg_remaining < 1e-9:
    t_now = seg_t + seg_T   # snap to authoritative boundary
    continue                # next iteration re-anchors v_now/a_now
```

This was added to `subsection` in `ksb/control/upstream_control.py`.

---

## Bug 2 — velocity drift at the upstream/buffer junction

### What it looked like

`CompositeTrajectory.__post_init__` raised:

```
Velocity or acceleration discontinuity between segment 0 and 1
  prev end: v=1.550381, a=0.049822
  next start: v=1.550551, a=0.000000
```

The discrepancy (~1.6–2.5 × 10⁻⁴ m/s) exceeded the 1 × 10⁻⁴ atol. Five specific seeds
failed; the rest were fine.

### Root cause

`subsection` maintained `v_now` and `a_now` by propagating through
`candidate.eval(seg_remaining)` at each iteration:

```python
end = candidate.eval(seg_remaining)
v_now = float(end[V])
a_now = float(end[A])
```

`candidate` was created from `x0 = [0, v_now_prev, a_now_prev]` — already-drifted
values. Meanwhile `ksb_simulation.py` obtains the buffer initial velocity via:

```python
v_in = self._u_control.state_at(t_in)[V]
```

`state_at` integrates the jerk timeline from scratch on each call — no accumulated
drift. After a long hold phase (`a = 0.05 m/s²` for ~40 s), a tiny drift in `a_now`
(`δa ≈ 5 × 10⁻⁶ m/s²`) compounded over the hold: `δv = δa × t_hold ≈ 2 × 10⁻⁴ m/s`
— just above the atol.

### Fix

Re-anchor `v_now` and `a_now` from the authoritative timeline at the top of every
loop body:

```python
x_auth = self.state_at(t_now)
v_now  = float(x_auth[V])
a_now  = float(x_auth[A])
```

Because `state_at` and the candidate `ConstantJerkTrajectory` both integrate the same
cubic polynomial over the same span with the same inputs, their results agree to
`~10⁻¹⁴ m/s` — well within the 1 × 10⁻⁴ atol.

This was added immediately after `_segment_at` in the `subsection` loop.

---

## Combined result

After both fixes, 0/200 seeds fail across `a_u_max ∈ {0.05, 0.10}`.
