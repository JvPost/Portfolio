# Precision bug in `subsection` — analysis

## What the bug looks like

`seg_remaining = seg_T - elapsed` goes slightly negative inside the `while remaining > 1e-10` loop. The next line immediately creates `ConstantJerkTrajectory(T=seg_remaining, ...)`. That constructor checks `if abs(self.T) < 1e-9: raise ValueError`, so:

- If `seg_remaining ∈ (-1e-9, 0)` → the constructor raises (T is non-positive but too small to pass the abs check).
- If `seg_remaining < -1e-9` → the constructor silently succeeds with a negative-duration segment, and `eval` clips its time argument to 0, returning `dp_full ≈ 0`. `remaining` then fails to decrease, and the loop either stalls or appends a junk segment.

## How `seg_remaining` becomes negative

`_segment_at(t_now)` searches in **reverse** and returns the last segment whose `seg_t ≤ t_now + 1e-12`. The `elapsed` it produces can therefore range from `−1e-12` (assigned one segment early) to arbitrarily large positives.

The negative case requires: `t_now > seg_t_B + seg_T_B` (past segment B's end) **but** segment C (which starts at `seg_t_C = seg_t_B + seg_T_B`) is not found by the reverse scan because `seg_t_C > t_now + 1e-12`.

For that last condition to hold while also `seg_t_C ≤ t_now` (we're past it), `seg_t_C` as stored in the timeline must be **slightly larger** than the true boundary — by more than the 1e-12 tolerance.

### Two independent floating-point chains

The stored segment start times are computed inside `_append_acceleration` / `_append_deceleration`:

```python
t_now = t_start
self._append(t_now, +j, t_ramp)   # seg_t_A = t_start
t_now += t_ramp
self._append(t_now,  0, t_hold)   # seg_t_B = t_start ⊕ t_ramp
t_now += t_hold
self._append(t_now, -j, t_ramp)   # seg_t_C = (t_start ⊕ t_ramp) ⊕ t_hold
```

Meanwhile in `subsection`, `t_now` starts at `t0` (spawn time — itself a sum of AR(1) gaps) and advances by:

```python
t_now += seg_remaining   # = seg_T - (t_now - seg_t)
```

Each addition is done in a **different arithmetic context** (different operand magnitudes, different rounding direction). After consuming segments A and B, `t_now` in `subsection` represents `seg_t_C` via one FP path; the stored `seg_t_C` was computed via another. The two representations of the same real value can differ by a few ULPs.

At simulation times of 50–100 s, one ULP is approximately `t × 2^-52 ≈ 1–2 × 10^-14`. So the divergence between the two paths is typically `O(10^-14)`, well within the 1e-12 tolerance. The bug therefore requires accumulated error from **multiple** compounding mis-roundings, not just one.

## Why specific `a_u_max` values matter

`t_ramp = a_u_max / j_u_max`. With the current config (`j_u_max = 100`):

| `a_u_max` | `t_ramp` |
|-----------|---------|
| 0.10      | 1 ms    |
| 0.05      | 0.5 ms  |

Short ramp segments are the **sensitive** ones. A ramp segment is only entered if `t0` lands within its tiny window (`t_ramp ≈ 0.5 ms`). When that happens, `elapsed ≈ t_ramp` and `seg_remaining ≈ 0`. Any FP error that pushes `elapsed` fractionally above `t_ramp` produces a negative `seg_remaining`.

Concretely: the hold segment after the ramp-up is long (~40 s with `a_u_max = 0.05`). The hold's stored `seg_t_B = t_start + t_ramp`. The ramp-down's `seg_t_C = (t_start + t_ramp) + t_hold`. These two large additions each carry a rounding error of `O(t × ε_mach) ≈ 10^-14`. But `subsection`'s `t_now` after consuming the hold follows a path that started from `t0` (spawn time) — a completely independent summation. The two representations can diverge by `2–3 × 10^-14`, which can exceed the ramp's `seg_remaining` if `t0` happens to spawn near the ramp-hold or hold-ramp boundary.

The smaller `a_u_max`, the smaller `t_ramp`, and the smaller the absolute margin before a ULP-sized error makes `seg_remaining` negative.

### Why `_ensure_covered` doesn't rescue this case

`_ensure_covered` extends the timeline only when `t_now >= self._t_end`. If C already exists in the timeline (it was eagerly appended by `_append_acceleration`), `_t_end` is already set to the end of C. `t_now` is within the timeline's covered range, so `_ensure_covered` is a no-op — and the broken lookup in `_segment_at` proceeds unchecked.

## The "not caught early enough" point

The loop guard is `while remaining > 1e-10`. This means we can enter the loop body with `remaining` as small as `1.1e-10` — far below the distance that a single sub-millisecond jerk segment contributes. The actual remaining distance at the point of failure is already negligible; the loop should have stopped, but the threshold wasn't tight enough relative to the FP noise level in `dp_full` for short segments.

The missing guard is on `seg_remaining` itself: there is currently no check of the form `if seg_remaining <= ε: skip/break` **before** the `ConstantJerkTrajectory` constructor is called.

## Summary

| Layer | What's missing |
|-------|---------------|
| Loop entry | `remaining > 1e-10` is loose enough to admit iterations where the true remaining distance is below FP noise |
| Segment lookup | `_segment_at`'s 1e-12 tolerance may be insufficient when the two summation chains (profile construction vs. subsection `t_now` accumulation) diverge by more than 1e-12 |
| Trajectory creation | No guard on `seg_remaining` before calling `ConstantJerkTrajectory(T=seg_remaining, ...)` |
