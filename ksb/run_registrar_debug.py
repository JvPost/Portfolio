#!/usr/bin/env python3
"""Debug script for the Registrar stage.

Builds a RegistrarGeometry from the default config, plans trajectories for a
small set of delta_p_i values, assembles the full registrar CompositeTrajectory
per case, and plots position / velocity / acceleration across all N_R segments.

Usage:
    python run_registrar_debug.py [--config NAME]
"""
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import yaml

from ksb.planning.contracts import V_MAX
from ksb.planning.solvers.scurve import SCurveSolver
from ksb.simulation.registrar import RegistrarGeometry, RegistrarResult, RegistrarStage


_CONFIG_DIR = Path(__file__).parent / "configs"


def build_geometry_and_stage(cfg: dict) -> tuple[RegistrarGeometry, RegistrarStage]:
    bounds = np.array([
        float(cfg["jmax"]),
        float(cfg["Amax"]),
        float(cfg["Vmax"]),
        0.0,
    ])

    v_d = float(cfg["slot_rate_ppm"]) / 60.0 * float(cfg["slot_length"])

    geo = RegistrarGeometry(
        N_R=int(cfg["n_reg_seg"]),
        L_R=float(cfg["L_registrar"]),
        v_BR=float(cfg["V_buff_out"]),
        v_d=v_d,
        l_input=float(cfg["input_length"]),
        j_max=float(cfg["jmax"]),
        a_max=float(cfg["Amax"]),
    )

    stage = RegistrarStage(
        geometry=geo,
        solver=SCurveSolver(),
        bounds=bounds,
    )

    return geo, stage


def make_nominal_times(geo: RegistrarGeometry) -> tuple[float, float]:
    t_in_nominal = 0.0
    t_slot_RD = t_in_nominal + geo.L_R / geo.v_d
    return t_in_nominal, t_slot_RD


def plot_registrar_trajectory(
    geo: RegistrarGeometry,
    stage: RegistrarStage,
    delta_p_cases: list[float],
    n_points: int = 500,
) -> None:
    t_in_nominal, t_slot_RD = make_nominal_times(geo)

    fig, axes = plt.subplots(
        3, len(delta_p_cases),
        figsize=(5 * len(delta_p_cases), 9),
        sharex="col",
    )
    if len(delta_p_cases) == 1:
        axes = axes[:, np.newaxis]

    fig.suptitle(
        f"Registrar debug  |  N_R={geo.N_R}  v_BR={geo.v_BR:.2f}  "
        f"v_d={geo.v_d:.2f}  L_R={geo.L_R:.2f} m",
        fontsize=12,
    )

    for col, dp in enumerate(delta_p_cases):
        result = stage.plan(
            t_in_nominal=t_in_nominal,
            t_slot_RD=t_slot_RD,
            delta_p_i=dp,
        )

        ax_p, ax_v, ax_a = axes[0, col], axes[1, col], axes[2, col]
        title = f"delta_p_i = {dp:+.4f} m"

        if result.budget_exceeded:
            for ax in (ax_p, ax_v, ax_a):
                ax.set_title(f"{title}\n[BUDGET EXCEEDED]", color="red")
                ax.text(0.5, 0.5, "Budget exceeded", ha="center", va="center",
                        transform=ax.transAxes, color="red")
            continue

        traj = stage.assemble_trajectory(result)
        t_eval = np.linspace(0.0, traj.T, n_points)
        state = traj.eval(t_eval)

        p_arr = state[0]
        v_arr = state[1]
        a_arr = state[2]

        boundary_times = _segment_boundary_times(geo, result)

        ax_p.plot(t_eval, p_arr, color="steelblue", lw=1.5)
        for bt in boundary_times:
            ax_p.axvline(bt, color="gray", lw=0.7, ls="--", alpha=0.6)
        ax_p.set_ylabel("position (m)")
        ax_p.set_title(title)
        ax_p.grid(True, alpha=0.3)

        ax_v.plot(t_eval, v_arr, color="darkorange", lw=1.5)
        for bt in boundary_times:
            ax_v.axvline(bt, color="gray", lw=0.7, ls="--", alpha=0.6)
        for n in range(geo.N_R + 1):
            ax_v.axhline(geo.v_crossing[n], color="green", lw=0.6, ls=":", alpha=0.7)
        ax_v.set_ylabel("velocity (m/s)")
        ax_v.grid(True, alpha=0.3)

        ax_a.plot(t_eval, a_arr, color="firebrick", lw=1.5)
        for bt in boundary_times:
            ax_a.axvline(bt, color="gray", lw=0.7, ls="--", alpha=0.6)
        ax_a.axhline(0.0, color="black", lw=0.5)
        ax_a.set_ylabel("acceleration (m/s^2)")
        ax_a.set_xlabel("time (s)")
        ax_a.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def _segment_boundary_times(
    geo: RegistrarGeometry,
    result: RegistrarResult,
) -> list[float]:
    times = []
    t = 0.0
    for n, seg_traj in enumerate(result.segment_trajectories):
        t += seg_traj.T
        if n < geo.N_R - 1:
            times.append(t)
            t += geo.tau_internal[n]
    return times


def main():
    parser = argparse.ArgumentParser(description="Registrar debug plotter")
    parser.add_argument("--config", type=str, default="default")
    args = parser.parse_args()

    with open(_CONFIG_DIR / f"{args.config}.yaml") as f:
        cfg = yaml.safe_load(f)

    geo, stage = build_geometry_and_stage(cfg)
    print(geo.report())

    t_in_nominal, t_slot_RD = make_nominal_times(geo)
    T_R_nominal = t_slot_RD - t_in_nominal
    print(f"\n  t_in_nominal  : {t_in_nominal:.4f} s")
    print(f"  t_slot_RD     : {t_slot_RD:.4f} s")
    print(f"  T_R nominal   : {T_R_nominal:.4f} s")
    print(f"  t_min_total   : {geo.t_min_total:.4f} s")
    margin = T_R_nominal - geo.t_min_total
    print(f"  margin        : {margin:.4f} s  ", end="")
    print("OK" if margin > 0 else "INFEASIBLE — increase L_R or reduce N_R")
    print()

    delta_p_cases = [0.0, 0.01, -0.01]
    plot_registrar_trajectory(geo, stage, delta_p_cases)


if __name__ == "__main__":
    main()
