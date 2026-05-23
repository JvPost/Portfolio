import numpy as np

from ksb.motion.trajectories import PolynomialTrajectory, P, V, A
from ksb.planning.contracts import IProfileSolver, InfeasibleError, J_MAX, A_MAX, V_MAX

_CLIP_TOL = 1e-9


def _clip_bc(val: float, bound: float, name: str) -> float:
    """Clip val to [-bound, bound], tolerating floating-point overshoot up to _CLIP_TOL."""
    if abs(val) > bound + _CLIP_TOL:
        raise InfeasibleError(f"Boundary condition {name}={val:.6g} exceeds bound {bound:.6g}")
    return float(np.clip(val, -bound, bound))


class QuinticSolver(IProfileSolver):
    """Quintic (5th-order position) trajectory solver.

    Solves the 6×6 linear system for boundary conditions:
        p(0)=pi, v(0)=vi, a(0)=ai, p(T)=pf, v(T)=vf, a(T)=af
    """
    def __init__(self, feasibility_dt = .01):
        self.feasibility_dt = feasibility_dt

    def solve(self, pi, vi, pf, vf, T, bounds, policy, ai=0.0, af=0.0) -> PolynomialTrajectory:
        if T <= 0:
            raise InfeasibleError("T must be positive")

        vi = _clip_bc(vi, bounds[V_MAX], "vi")
        vf = _clip_bc(vf, bounds[V_MAX], "vf")
        ai = _clip_bc(ai, bounds[A_MAX], "ai")
        af = _clip_bc(af, bounds[A_MAX], "af")

        # 6×6 system: coefficients [t^5, t^4, t^3, t^2, t, const]
        mat = np.array([
            [0,       0,      0,     0,    0, 1],   # p(0) = pi
            [0,       0,      0,     0,    1, 0],   # v(0) = vi
            [0,       0,      0,     2,    0, 0],   # a(0) = ai
            [T**5,    T**4,   T**3,  T**2, T, 1],  # p(T) = pf
            [5*T**4,  4*T**3, 3*T**2, 2*T, 1, 0], # v(T) = vf
            [20*T**3, 12*T**2, 6*T,  2,   0, 0],  # a(T) = af
        ])
        b = np.array([pi, vi, ai, pf, vf, af])

        coeffs = np.linalg.solve(mat, b)
        poly = np.poly1d(coeffs)

        x0 = np.array([pi, vi, ai])
        traj = PolynomialTrajectory(x0=x0, T=T, poly=poly)

        # Most efficient way of checking is numerically evaluating the function. 
        # This is unavoidable, because the polynomial isn't constructed from 
        # the bounds as, for instance, the scurve is.
        t_arr = np.arange(0, T, self.feasibility_dt)
        states = np.array([traj.poly.deriv(i)(t_arr) for i in range(3, 0, -1)]) # compute jerk, acc, vel
        if np.any(np.greater(states.T, bounds[:3])): # compare with bounds
            raise InfeasibleError("Infeasible")

        return traj
