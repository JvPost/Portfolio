import numpy as np

from ksb.motion.trajectories import LinearTrajectory, V, A
from ksb.planning.contracts import IProfileSolver, InfeasibleError, V_MAX


class LinearTrajectorySolver(IProfileSolver):
    """Solves for a constant-velocity linear trajectory.

    Computes v = (pf - pi) / T and checks consistency with vi and vf.
    Assumes zero acceleration throughout.
    """

    def solve(self, pi, vi, pf, vf, T, bounds, policy) -> LinearTrajectory:
        if T <= 0:
            raise InfeasibleError("Duration T must be positive")

        v_req = (pf - pi) / T

        if not np.allclose(v_req, vi, atol=1e-6) or not np.allclose(v_req, vf, atol=1e-6):
            raise InfeasibleError(
                f"Linear solver requires consistent velocity: "
                f"computed v={v_req}, but vi={vi}, vf={vf}"
            )
        if abs(v_req) > bounds[V_MAX]:
            raise InfeasibleError(f"Required velocity {v_req} exceeds V_max {bounds[V_MAX]}")
        if bounds[0] < 0 or bounds[1] < 0:
            raise InfeasibleError("Bounds must allow zero a/j")

        x0 = np.array([0.0, v_req, 0.0])
        traj = LinearTrajectory(x0=x0, T=T)

        if not traj.check_bounds(bounds):
            raise InfeasibleError("Generated trajectory exceeds bounds (unexpected)")

        return traj
