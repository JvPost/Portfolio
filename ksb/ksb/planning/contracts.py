from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List

import numpy as np

if TYPE_CHECKING:
    from ksb.motion.trajectories import CompositeTrajectory, TrajectoryProfile

# ---------- Bounds array indices ----------
J_MAX, A_MAX, V_MAX, PITCH_MIN = 0, 1, 2, 3


# ---------- Exceptions (failure channel) ----------
class PlannerError(Exception):
    """Top-level planning failure (propagate to app if all fallbacks fail)."""


class InfeasibleError(PlannerError):
    """Spec violates physical/bound constraints (e.g., j_max, v_max)."""


class NonConvergenceError(PlannerError):
    """Numeric solver failed to converge within iteration or tolerance limits."""


class InputError(PlannerError):
    """Bad or nonsensical inputs (NaN, negative times, etc.)."""


class SlotAssignmentError(PlannerError):
    """Configuration is such that no feasible slot could be found for an item."""


# ---------- Policy ----------
@dataclass(frozen=True)
class Policy:
    """Planner policy knobs."""
    input_length: float = 0.32
    v_min: float = 0.0   # planner-imposed lower bound on velocity (m/s)


# ---------- Solver protocol ----------
class IProfileSolver(ABC):
    """Strategy for building a jerk profile that satisfies the spec.

    bounds is np.ndarray([j_max, A_max, V_max, gap_min]) — use J_MAX, A_MAX, V_MAX indices.
    """
    def solve(self, pi, vi, pf, vf, T, bounds, policy, ai, af):
        ...

    def feasibility_window(self, pi, vi, pf, vf, bounds, policy, ai, af) -> tuple[float, float]:
        """Return (T_min, T_max) such that solve(..., T, ...) is feasible for T in this range.

        Default implementation returns (0.0, math.inf), meaning the solver has no
        closed-form window and callers should fall back to iterative search.
        Solvers with a tractable window (e.g. SCurveSolver) may override.
        """
        return (0.0, math.inf)