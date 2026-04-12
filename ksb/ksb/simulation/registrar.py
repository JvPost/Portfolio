from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from ksb.motion.trajectories import (
    CompositeTrajectory,
    LinearTrajectory,
    TrajectoryProfile,
)
from ksb.planning.contracts import (
    A_MAX,
    J_MAX,
    V_MAX,
    InfeasibleError,
    Policy,
)
from ksb.planning.solvers.scurve import SCurveSolver


def _ramp(v_a: float, v_b: float, j_max: float, a_max: float) -> tuple[float, float]:
    """Minimum time and displacement for a jerk-limited velocity transition v_a -> v_b.

    Mirrors the private _ramp inside SCurveSolver but exposed at module level
    so RegistrarGeometry can call it during construction for feasibility checks.

    Returns
    -------
    T_ramp : minimum time to complete the transition (s)
    X_ramp : displacement covered during that transition (m)
    """
    dv = abs(v_b - v_a)
    if dv < 1e-12:
        return 0.0, 0.0

    sign = 1.0 if v_b >= v_a else -1.0
    dv_ramp_full = a_max ** 2 / j_max

    if dv >= dv_ramp_full - 1e-9:
        # Trapezoidal: ramp saturates at a_max
        T1 = a_max / j_max
        T2 = (dv - a_max * T1) / a_max
        T3 = T1
    else:
        # Triangular: a_max never reached
        T1 = math.sqrt(dv / j_max)
        T2 = 0.0
        T3 = T1

    j1 = sign * j_max
    x1 = v_a * T1 + (1.0 / 6.0) * j1 * T1 ** 3
    v1 = v_a + 0.5 * j1 * T1 ** 2
    a1 = j1 * T1
    x2 = v1 * T2 + 0.5 * a1 * T2 ** 2
    v2 = v1 + a1 * T2
    j3 = -sign * j_max
    x3 = v2 * T3 + 0.5 * a1 * T3 ** 2 + (1.0 / 6.0) * j3 * T3 ** 3

    return T1 + T2 + T3, x1 + x2 + x3


def segment_t_min(
    Lambda: float,
    v_in: float,
    v_out: float,
    j_max: float,
    a_max: float,
) -> float:
    """Minimum time to traverse Lambda metres while decelerating from v_in to v_out.

    The minimum-time profile for a registrar segment (v_peak capped at v_in)
    is: coast at v_in, then decelerate to v_out. The coast fills whatever
    displacement remains after the ramp.

    Parameters
    ----------
    Lambda : effective control length = L^{R,n} - l_input (m)
    v_in   : segment entry velocity (m/s)
    v_out  : segment exit velocity (m/s); must be <= v_in
    j_max  : maximum jerk (m/s^3)
    a_max  : maximum acceleration magnitude (m/s^2)

    Returns
    -------
    T_min : minimum feasible time horizon for this segment (s).
            If Lambda < X_ramp the geometry is physically infeasible
            (raises ValueError).
    """
    T_ramp, X_ramp = _ramp(v_in, v_out, j_max, a_max)

    if Lambda < X_ramp - 1e-9:
        raise ValueError(
            f"Segment too short: Lambda={Lambda:.4f} m < X_ramp={X_ramp:.4f} m "
            f"for v_in={v_in:.4f} -> v_out={v_out:.4f} m/s. "
            "Increase L_R or reduce N_R or reduce |delta_V|."
        )

    T_coast = (Lambda - X_ramp) / v_in
    return T_ramp + T_coast


@dataclass
class RegistrarGeometry:
    """Pre-computed registrar geometry from design parameters.

    All derived quantities are computed once at construction from the six
    design parameters and held for the lifetime of the object.

    Segment lengths decrease monotonically from segment 1 (fastest, longest)
    to segment N_R (slowest, shortest), chosen to equalise dwell time and
    therefore equalise per-segment correction capacity across all segments.

    Parameters
    ----------
    N_R     : number of registrar segments
    L_R     : total registrar length (m)
    v_BR    : KSB exit velocity = registrar entry velocity (m/s)
    v_d     : downstream velocity = registrar exit velocity (m/s); v_d <= v_BR
    l_input : physical input length (m)
    j_max   : maximum jerk (m/s^3)  — needed for feasibility checks
    a_max   : maximum acceleration magnitude (m/s^2) — needed for feasibility checks
    """

    N_R: int
    L_R: float
    v_BR: float
    v_d: float
    l_input: float
    j_max: float
    a_max: float

    # ---- derived fields (populated in __post_init__) ----
    delta_V: float = field(init=False)
    """Nominal velocity change per segment: (v_d - v_BR) / N_R  (≤ 0)."""

    v_crossing: np.ndarray = field(init=False)
    """Crossing velocities v^{R,0} ... v^{R,N_R}; shape (N_R+1,).
    v_crossing[0] = v_BR, v_crossing[N_R] = v_d."""

    segment_lengths: np.ndarray = field(init=False)
    """Per-segment lengths L^{R,1} ... L^{R,N_R}; shape (N_R,); decreasing."""

    lambda_segments: np.ndarray = field(init=False)
    """Effective control lengths Lambda^{R,n} = L^{R,n} - l_input; shape (N_R,)."""

    tau_internal: np.ndarray = field(init=False)
    """Straddling durations at internal boundaries B^{R,1} ... B^{R,N_R-1};
    shape (N_R-1,).  tau^{R,n} = l_input / v^{R,n}."""

    t_min_segments: np.ndarray = field(init=False)
    """Minimum feasible time horizon per segment; shape (N_R,).
    Computed analytically from Lambda, v_in, v_out, j_max, a_max."""

    x_ramp_segments: np.ndarray = field(init=False)
    """Minimum displacement consumed by the velocity ramp per segment; shape (N_R,)."""

    def __post_init__(self) -> None:
        if self.v_BR < self.v_d - 1e-9:
            raise ValueError(
                f"v_BR={self.v_BR:.4f} < v_d={self.v_d:.4f}. "
                "KSB exit velocity must be >= downstream velocity."
            )
        if self.N_R < 1:
            raise ValueError("N_R must be >= 1.")
        if self.l_input <= 0:
            raise ValueError("l_input must be > 0.")

        self.delta_V = (self.v_d - self.v_BR) / self.N_R

        # v^{R,n} = v_BR + n * delta_V
        self.v_crossing = self.v_BR + np.arange(self.N_R + 1) * self.delta_V

        # Equal-dwell-time segment lengths: L^{R,n} proportional to average
        # transit velocity on that segment.
        v_avg = self.v_BR + (np.arange(self.N_R) + 0.5) * self.delta_V
        self.segment_lengths = self.L_R * v_avg / v_avg.sum()

        # Effective control lengths
        self.lambda_segments = self.segment_lengths - self.l_input
        if np.any(self.lambda_segments <= 1e-9):
            raise ValueError(
                f"All registrar segment lengths must exceed l_input={self.l_input:.4f} m. "
                f"Got segment_lengths={self.segment_lengths}. "
                "Increase L_R or reduce N_R."
            )

        # Straddling durations at internal boundaries
        if self.N_R > 1:
            self.tau_internal = self.l_input / self.v_crossing[1 : self.N_R]
        else:
            self.tau_internal = np.empty(0, dtype=float)

        # Per-segment feasibility: minimum time and ramp displacement.
        # Raises ValueError immediately if any segment is geometrically infeasible.
        t_min_list = []
        x_ramp_list = []
        for n in range(self.N_R):
            v_in = float(self.v_crossing[n])
            v_out = float(self.v_crossing[n + 1])
            Lambda_n = float(self.lambda_segments[n])
            _, X_ramp = _ramp(v_in, v_out, self.j_max, self.a_max)
            T_min_n = segment_t_min(Lambda_n, v_in, v_out, self.j_max, self.a_max)
            t_min_list.append(T_min_n)
            x_ramp_list.append(X_ramp)

        self.t_min_segments = np.array(t_min_list)
        self.x_ramp_segments = np.array(x_ramp_list)

    @property
    def total_straddling_time(self) -> float:
        """Total time consumed by internal straddling windows (s)."""
        return float(self.tau_internal.sum())

    @property
    def total_lambda(self) -> float:
        """Total effective control displacement sum(Lambda^{R,n}) (m)."""
        return float(self.lambda_segments.sum())

    @property
    def t_min_total(self) -> float:
        """Minimum total registrar time window required (s).

        Sum of per-segment T_min plus all internal straddling windows.
        T_R produced by plan() must exceed this for any solve to succeed.
        """
        return float(self.t_min_segments.sum()) + self.total_straddling_time

    def report(self) -> str:
        """Human-readable geometry summary including feasibility diagnostics."""
        lines = [
            "RegistrarGeometry",
            f"  N_R={self.N_R}  L_R={self.L_R:.4f} m  "
            f"v_BR={self.v_BR:.4f}  v_d={self.v_d:.4f}  delta_V={self.delta_V:.4f} m/s",
            f"  l_input={self.l_input:.4f} m  "
            f"j_max={self.j_max:.1f}  a_max={self.a_max:.1f}",
            "",
            f"  {'seg':>4}  {'L (m)':>8}  {'Lambda':>8}  "
            f"{'v_in':>7}  {'v_out':>7}  {'X_ramp':>8}  {'T_min':>8}",
        ]
        for n in range(self.N_R):
            lines.append(
                f"  {n+1:>4}  {self.segment_lengths[n]:>8.4f}  "
                f"{self.lambda_segments[n]:>8.4f}  "
                f"{self.v_crossing[n]:>7.4f}  {self.v_crossing[n+1]:>7.4f}  "
                f"{self.x_ramp_segments[n]:>8.4f}  {self.t_min_segments[n]:>8.4f}"
            )
        lines += [
            "",
            f"  total_straddling_time : {self.total_straddling_time:.4f} s",
            f"  t_min_total           : {self.t_min_total:.4f} s  "
            f"(T_R must exceed this)",
        ]
        return "\n".join(lines)


@dataclass(frozen=True)
class RegistrarResult:
    """Per-input result from RegistrarStage.plan().

    Attributes
    ----------
    segment_trajectories : one CompositeTrajectory per segment; length N_R.
                           Empty tuple when budget_exceeded=True.
    T_R                  : total registrar time window used (s)
    delta_p_i            : position error input to this plan call (m)
    budget_exceeded      : True when |delta_p_i| drove T_R below t_min_total
    residual_error       : position error remaining at handoff (m); 0 when not exceeded
    """

    segment_trajectories: Tuple[CompositeTrajectory, ...]
    T_R: float
    delta_p_i: float
    budget_exceeded: bool
    residual_error: float


@dataclass(frozen=True)
class RegistrarStage:
    """Registrar orchestrator.

    Plans N_R per-segment jerk-limited deceleration profiles for each input,
    distributing the total velocity drop v_BR -> v_d and absorbing the position
    error delta_p_i accumulated on the KSB.

    Position correction mechanism
    -----------------------------
    delta_p_i > 0 means the input crossed B^{BR} earlier than the slot (it is
    ahead in space).  This shifts the effective registrar entry time earlier
    by delta_p_i / v_BR, giving the registrar more time than nominal.  The
    per-segment solver fills this extra time by raising v_peak slightly above
    vf, producing a coast-then-decel profile that takes longer to traverse
    Lambda, arriving at B^{RD} later and letting the slot catch up.

    The correction is automatic: no explicit epsilon offset needed. It falls
    out of computing T_R from the position-error-adjusted entry time.

    Parameters
    ----------
    geometry : pre-computed RegistrarGeometry (shared, constructed once)
    solver   : SCurveSolver instance (stateless, shared)
    bounds   : system bounds array [j_max, a_max, v_max, gap_min]
    """

    geometry: RegistrarGeometry
    solver: SCurveSolver
    bounds: np.ndarray

    def plan(
        self,
        t_in_nominal: float,
        t_slot_RD: float,
        delta_p_i: float,
    ) -> RegistrarResult:
        """Plan the registrar trajectory for a single input.

        Parameters
        ----------
        t_in_nominal : time the trailing edge clears B^{BR} assuming exact KSB
                       landing (s)
        t_slot_RD    : time the assigned slot arrives at B^{RD} (s)
        delta_p_i    : position error at B^{BR} (m); positive = input ahead
        """
        geo = self.geometry

        # Adjust entry time for position error.
        # delta_p_i > 0: input is ahead -> crossed B^{BR} earlier -> more time.
        t_in_actual = t_in_nominal - delta_p_i / geo.v_BR
        T_R = t_slot_RD - t_in_actual

        if T_R < geo.t_min_total - 1e-9:
            return RegistrarResult(
                segment_trajectories=(),
                T_R=T_R,
                delta_p_i=delta_p_i,
                budget_exceeded=True,
                residual_error=delta_p_i,
            )

        # Subtract internal straddling windows to get available control time.
        T_control = T_R - geo.total_straddling_time

        # Distribute control time across segments proportional to T_min per
        # segment so each segment gets at least its minimum.
        T_min_sum = geo.t_min_segments.sum()
        T_slack = T_control - T_min_sum
        T_seg_arr = geo.t_min_segments + T_slack * (geo.t_min_segments / T_min_sum)

        policy = Policy(input_length=geo.l_input)
        segment_trajectories: List[CompositeTrajectory] = []

        for n in range(geo.N_R):
            vi = float(geo.v_crossing[n])
            vf = float(geo.v_crossing[n + 1])
            Lambda_n = float(geo.lambda_segments[n])
            T_seg = float(T_seg_arr[n])

            # Cap V_MAX at the segment entry velocity to prevent upward excursion.
            seg_bounds = self.bounds.copy()
            # seg_bounds[V_MAX] = vi

            try:
                traj = self.solver.solve(
                    pi=0.0,
                    vi=vi,
                    pf=Lambda_n,
                    vf=vf,
                    T=T_seg,
                    bounds=seg_bounds,
                    policy=policy,
                )
            except InfeasibleError as e:
                return RegistrarResult(
                    segment_trajectories=tuple(segment_trajectories),
                    T_R=T_R,
                    delta_p_i=delta_p_i,
                    budget_exceeded=True,
                    residual_error=delta_p_i,
                )

            segment_trajectories.append(traj)

        return RegistrarResult(
            segment_trajectories=tuple(segment_trajectories),
            T_R=T_R,
            delta_p_i=delta_p_i,
            budget_exceeded=False,
            residual_error=0.0,
        )

    def assemble_trajectory(self, result: RegistrarResult) -> CompositeTrajectory:
        """Assemble the full registrar trajectory for one input.

        Chains N_R segment CompositeTrajectories with constant-velocity
        straddling coasts at internal boundaries into a single flat
        CompositeTrajectory.

        Parameters
        ----------
        result : output of plan() for this input; must not have budget_exceeded
        """
        if result.budget_exceeded:
            raise InfeasibleError(
                "Cannot assemble trajectory for a budget-exceeded RegistrarResult."
            )

        geo = self.geometry
        all_segs: List[TrajectoryProfile] = []

        for n, traj in enumerate(result.segment_trajectories):
            for seg in traj.segments:
                all_segs.append(seg)

            # Add straddling coast between segments (not after the last).
            if n < geo.N_R - 1:
                v_coast = float(geo.v_crossing[n + 1])
                tau = float(geo.tau_internal[n])
                all_segs.append(
                    LinearTrajectory(
                        x0=np.array([0.0, v_coast, 0.0]),
                        T=tau,
                    )
                )

        T_total = sum(s.T for s in all_segs)
        return CompositeTrajectory(
            x0=np.array([0.0, float(geo.v_crossing[0]), 0.0]),
            T=T_total,
            segments=tuple(all_segs),
        )
