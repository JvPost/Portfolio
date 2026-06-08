"""Crude grid search over (beta, gamma, L_buffer) maximising the feasible ratio.

Metric: mean fraction of (pair, segment) cells where kinematic_margin >= 0
        (SegmentSyncResponse.feasible), averaged over SEEDS seeds.

Usage:
    conda run -n ksb python grid_search_beta_gamma.py
"""

import itertools

import numpy as np
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

from ksb.simulation.ksb_simulation import KSBSimulation
from ksb.simulation.result import SimulationResult

# ── grid ──────────────────────────────────────────────────────────────────────
BETAS     = [-2.0, 0.0, 2.0]
GAMMAS    = [ 0.0, 1.0, 2.0]
L_BUFFERS = [ 2.0, 2.5, 3.0]
SEEDS     = list(range(5))

# ── config ────────────────────────────────────────────────────────────────────
cfg_path = Path("configs/system/default.yaml")
with open(cfg_path) as f:
    base_cfg = yaml.safe_load(f)


def run_point(beta: float, gamma: float, L_buffer: float) -> float:
    """Return mean feasible ratio (kinematic_margin >= 0) over SEEDS."""
    cfg = {**base_cfg, "beta": beta, "gamma": gamma, "L_buffer": L_buffer}
    sim = KSBSimulation(cfg=cfg)
    ratios = []
    for seed in SEEDS:
        result: SimulationResult = sim.run(seed=seed)
        ratios.append(result.segment_sync_response.feasible.mean())
    return float(np.mean(ratios))


# ── sweep ─────────────────────────────────────────────────────────────────────
grid = np.full((len(L_BUFFERS), len(BETAS), len(GAMMAS)), np.nan)

print(f"{'L_buffer':>9}  {'beta':>6}  {'gamma':>6}  {'feasible':>9}")
print("-" * 38)
for li, L_buffer in enumerate(L_BUFFERS):
    for i, beta in enumerate(BETAS):
        for j, gamma in enumerate(GAMMAS):
            ratio = run_point(beta, gamma, L_buffer)
            grid[li, i, j] = ratio
            print(f"{L_buffer:>9.2f}  {beta:>6.2f}  {gamma:>6.2f}  {ratio:>9.4f}")

# ── best point ────────────────────────────────────────────────────────────────
best = np.unravel_index(np.argmax(grid), grid.shape)
best_L, best_beta, best_gamma = L_BUFFERS[best[0]], BETAS[best[1]], GAMMAS[best[2]]
print(f"\nBest: L_buffer={best_L:.2f}  beta={best_beta:.2f}  gamma={best_gamma:.2f}"
      f"  feasible={grid[best]:.4f}")

# ── heatmaps: one (beta x gamma) panel per L_buffer ───────────────────────────
fig, axes = plt.subplots(1, len(L_BUFFERS), figsize=(6 * len(L_BUFFERS), 5.5), squeeze=False)
axes = axes[0]

vmin, vmax = float(grid.min()), float(grid.max())
im = None
for li, (ax, L_buffer) in enumerate(zip(axes, L_BUFFERS)):
    im = ax.imshow(grid[li], origin="upper", cmap="RdYlGn", vmin=vmin, vmax=vmax)
    ax.set_xticks(range(len(GAMMAS)))
    ax.set_xticklabels([f"{g:.1f}" for g in GAMMAS])
    ax.set_yticks(range(len(BETAS)))
    ax.set_yticklabels([f"{b:.1f}" for b in BETAS])
    ax.set_xlabel("gamma")
    ax.set_ylabel("beta")
    ax.set_title(f"L_buffer = {L_buffer:.2f}")

    for i, j in itertools.product(range(len(BETAS)), range(len(GAMMAS))):
        ax.text(j, i, f"{grid[li, i, j]:.3f}", ha="center", va="center", fontsize=8)

    if li == best[0]:
        ax.plot(best[2], best[1], "k*", markersize=14,
                label=f"best ({best_beta:.1f}, {best_gamma:.1f})")
        ax.legend(loc="upper right", fontsize=9)

fig.suptitle("Feasible ratio  P(kinematic_margin ≥ 0)  —  higher is better")
fig.colorbar(im, ax=list(axes), shrink=0.8, label="feasible ratio")

out = Path("grid_search_beta_gamma.png")
fig.savefig(out, dpi=150)
print(f"\nHeatmap saved to {out}")
plt.show()
