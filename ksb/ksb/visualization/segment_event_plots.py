"""Visualization utilities for SegmentEvents and SegmentSyncResponse budget matrices."""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

from ksb.analysis.events import SegmentEvents

if TYPE_CHECKING:
    from ksb.analysis.sync_response import SegmentSyncResponse


def make_km_norm_and_cmap() -> tuple[mcolors.TwoSlopeNorm, mcolors.LinearSegmentedColormap]:
    """Return a (norm, cmap) pair for kinematic-margin heatmaps.

    cmap: red at -1, white at 0, muted blue-grey at +1.
    norm: TwoSlopeNorm(vcenter=0, vmin=-1, vmax=1).
    """
    cmap = mcolors.LinearSegmentedColormap.from_list(
        "km_diverging",
        [(0.0, "#d73027"), (0.5, "#ffffff"), (1.0, "#6baed6")],
    )
    norm = mcolors.TwoSlopeNorm(vcenter=0, vmin=-1, vmax=1)
    return norm, cmap


def _plot_budget_matrix(
    matrix: np.ndarray,
    label: str,
    heatmap_title: str,
    title_suffix: str = "",
    standardized: bool = False,
) -> plt.Figure:
    """Four-panel figure for any (n_pairs × N_B) budget matrix.

    Top: full heatmap.
    Bottom-left: mean over pair indices i for each segment k — ⟨label⟩_i(k).
    Bottom-right: mean over segment indices k for each pair i — ⟨label⟩_k(i).
    Bars coloured red when the mean is negative.
    """
    n_pairs, N_B = matrix.shape
    mean_over_i = matrix.mean(axis=0)   # (N_B,)    — mean per segment k
    mean_over_k = matrix.mean(axis=1)   # (n_pairs,) — mean per pair i

    suffix = f"  [{title_suffix}]" if title_suffix else ""

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.40, wspace=0.30)
    ax_heat = fig.add_subplot(gs[0, :])
    ax_k    = fig.add_subplot(gs[1, 0])
    ax_i    = fig.add_subplot(gs[1, 1])

    # ── Heatmap ──────────────────────────────────────────────────────────────
    if standardized:
        plot_matrix = np.clip(matrix, -1, 1)
        norm, cmap = make_km_norm_and_cmap()
        im = ax_heat.imshow(plot_matrix, cmap=cmap, norm=norm, aspect="auto", interpolation="nearest")
    else:
        im = ax_heat.imshow(matrix, cmap="RdYlGn", aspect="auto", interpolation="nearest")
    ax_heat.set_xlabel("Segment index k", fontsize=11)
    ax_heat.set_ylabel("Pair index i", fontsize=11)
    ax_heat.set_title(f"{heatmap_title}{suffix}", fontsize=12, fontweight="bold")
    ax_heat.set_xticks(np.arange(N_B))
    ax_heat.set_yticks(np.arange(0, n_pairs, max(1, n_pairs // 10)))
    plt.colorbar(im, ax=ax_heat, label=f"{label}  (s)")

    # ── Mean over pairs → per segment k ──────────────────────────────────────
    colors_k = ["C3" if v < 0 else "C0" for v in mean_over_i]
    ax_k.bar(np.arange(N_B), mean_over_i, color=colors_k, edgecolor="white", linewidth=0.5)
    ax_k.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax_k.set_xlabel("Segment index k", fontsize=11)
    ax_k.set_ylabel(f"⟨{label}⟩ over pairs  (s)", fontsize=11)
    ax_k.set_title(f"Mean per segment  ${label}_i(k){suffix}$", fontsize=11)
    ax_k.set_xticks(np.arange(N_B))
    ax_k.grid(axis="y", alpha=0.3)

    # ── Mean over segments → per pair i ──────────────────────────────────────
    colors_i = ["C3" if v < 0 else "C1" for v in mean_over_k]
    ax_i.bar(np.arange(n_pairs), mean_over_k, color=colors_i, edgecolor="white", linewidth=0.5)
    ax_i.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax_i.set_xlabel("Pair index i", fontsize=11)
    ax_i.set_ylabel(f"⟨{label}⟩ over segments  (s)", fontsize=11)
    ax_i.set_title(f"Mean per pair  ${label}⟩_k(i){suffix}$", fontsize=11)
    ax_i.grid(axis="y", alpha=0.3)

    plt.show()
    return fig


def plot_W_budget(events: SegmentEvents, title_suffix: str = "") -> plt.Figure:
    """Four-panel figure for the budget matrix W = t_in − t_out."""
    return _plot_budget_matrix(
        events.W,
        label="W",
        heatmap_title="Budget matrix  W = t_in − t_out  (s)",
        title_suffix=title_suffix,
    )


def plot_kinematic_margin(
    sync_response: "SegmentSyncResponse",
    title_suffix: str = "",
    standardized: bool = True,
) -> plt.Figure:
    """Four-panel figure for the kinematic margin = W − T_min."""
    return _plot_budget_matrix(
        sync_response.kinematic_margin,
        label="M",
        heatmap_title="Kinematic margin  M = W − T_min  (s)",
        title_suffix=title_suffix,
        standardized=standardized,
    )
