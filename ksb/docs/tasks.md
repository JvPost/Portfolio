# KSB next tasks

## 1. Rewrite `SCurveSolver` against the new contract

Tear down `ksb/planning/solvers/scurve.py` and rewrite it cleanly, using the
shared `ramp` primitive from `ksb.planning.ramp` and the extended
`IProfileSolver` contract.

**Goals**
- Unify the bisection range to `[v_min, V_max]` (where `v_min` comes from
  `Policy`), so the solver handles peak-shaped profiles (`v_m > max(vi, vf)`)
  and dip-shaped profiles (`v_m < min(vi, vf)`) with the same code path. This
  closes the long-standing dip-case bug that's been masked by `get_next_slot`'s
  retry loop.
- Implement `feasibility_window(...)` returning a real `(T_min, T_max)`:
  `T_min` from `v_m = V_max`, `T_max` from `v_m = v_min`. Both straight from
  the ramp primitive — no bisection needed for the bounds themselves.
- Distinguish T-too-small vs T-too-large failures in `InfeasibleError`
  messages, so callers (i.e. the rewritten `get_next_slot`) can search the
  right direction.
- Eliminate the case-handling loops the current implementation has. The 7-phase
  generation should reduce to: ramp(vi → v_m) + cruise(v_m, T_cruise) +
  ramp(v_m → vf), with phase signs inherited from the ramp primitive.

**Scope**
- `ksb/planning/solvers/scurve.py` only.
- No changes to the contract (already done in prompt 1).
- No changes to consumers (`get_next_slot`, etc) — covered by prompt 3.

**Acceptance**
- `pytest tests/test_segment_geometry_regression.py` passes (this test is
  self-comparing, not snapshot-based, so it should remain green even if
  numerical output shifts slightly).
- `python run_sim.py` with default config runs without error on s-curve.
- `python run_optimize.py --opt-config quick --batch 20` shows substantially
  fewer `SlotAssignmentError` events than the most recent quintic-equivalent
  baseline. Even before `get_next_slot` is updated to use the window, the
  dip-case fix alone should reduce `InfeasibleError` rates inside the slot
  search, which transitively reduces `SlotAssignmentError`.

---

## 2. Rewire `get_next_slot` to use `feasibility_window`

Update `ksb/simulation/utils.py` so that `get_next_slot` queries the solver's
`feasibility_window` once and uses the result to either jump-start the slot
index or pick the search direction, rather than blindly stepping forward 10
times.

**Goals**
- Compute `T_min, T_max` once per call.
- Map the window to a slot-index range
  `[ceil((T_min + t_control_start - t_offset) / slot_period),
    floor((T_max + t_control_start - t_offset) / slot_period)]`.
- If the window is finite (s-curve): assign the lowest slot in range that the
  solver actually accepts. This becomes a one-shot calculation in the common
  case; fall back to a small bounded scan only if numerical edge cases
  intrude.
- If the window is `(0, ∞)` (quintic, linear, default): preserve the current
  iterative search behavior. No regressions for solvers without window info.
- The 10-attempt cap can stay as a defensive backstop, but its trigger should
  become a rare event rather than the dominant failure mode.

**Scope**
- `ksb/simulation/utils.py` — `get_next_slot` only. No changes to
  `KSBSimulation.run()`'s call site.

**Acceptance**
- All current tests pass.
- A run of `run_optimize.py --opt-config quick` on s-curve shows
  `SlotAssignmentError` count near zero in the log. The remaining sentinels
  should be the boundary `AssertionError` (eta_v · v_d > V_max), which is
  expected and benign.
- For quintic, behavior is bit-identical to the current implementation
  (default window = no info = same iterative path).

---

## 3. Discuss: constraining CMA-ES to avoid `eta_v · v_d > V_max`

The optimization log shows ~216 `AssertionError: v_buff_out (X) exceeds Vmax`
events per run. These are the optimizer probing configurations where
`eta_v · v_d > V_max`, which is structurally infeasible (the buffer can't
release faster than the hardware ceiling). Currently the simulation raises
on these, the loss returns `+∞`, and CMA-ES sees a sentinel.

This is parameterization-correct (we want the optimizer to feel the wall) but
wasteful — every sentinel evaluation is a full simulation that returns no
useful gradient information. If we can constrain the search so it never
samples these points, we save compute and give CMA-ES a smoother loss surface.

**Question to think through together (not yet a prompt)**
- Is there a clean way to express `eta_v · eta_r · eta_s ≤ V_max / (l · r_u)`
  as a search-space transformation rather than a constraint?
- Options to explore:
  1. Reparameterize again: replace `eta_v` with a scalar
     `t ∈ [0, 1]` that maps to `[1, V_max / v_d]`. The map is
     state-dependent (depends on the current `eta_r`, `eta_s`), but at
     sample time both are known. Closed-form, no search-space coupling
     surfaces in the optimizer.
  2. Rejection sampling at the CMA-ES level: pre-filter samples that
     violate the constraint before evaluating the loss. CMA libraries
     usually expose hooks for this.
  3. Penalty method: replace the `AssertionError` with a smooth penalty
     proportional to the violation. Lets the optimizer "feel" the
     constraint as a gradient instead of a wall.
  4. Project onto the feasible set: clip `eta_v` to its current
     state-dependent maximum before passing to the simulation. Distorts
     the effective search distribution but eliminates wasted evaluations.

**Trade-offs to weigh**
- Option 1 is the cleanest in spirit (matches what we did with the
  earlier reparameterization), but the state-dependent map means the
  effective range of `eta_v` shrinks when other η's are large — the
  optimizer can't easily *trade* away the other η's to free up `eta_v`.
- Option 3 is what most constrained-optimization literature recommends,
  but it requires picking a penalty coefficient — yet another knob.
- Option 4 is the simplest to implement, but biases the search toward the
  boundary (every clipped sample lands exactly at the wall).
- Decide: are these failures actually costing meaningful optimizer time,
  or is the issue mostly aesthetic? Worth measuring the wasted-compute
  fraction before committing to a fix.

**Out of scope (for this discussion)**
- Any actual implementation. This is a design conversation, not a refactor.