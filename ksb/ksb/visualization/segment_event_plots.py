"""Visualization utilities for SegmentEvents budget matrix W."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt

from ksb.analysis.events import SegmentEvents


def plot_W_budget(events: SegmentEvents, title_suffix: str = "") -> plt.Figure:
    """Four-panel figure: heatmap of W and marginal mean profiles by segment k and pair i.

    Top: heatmap of the full (n_pairs × N_B) budget matrix W = t_in − t_out.
    Bottom-left: mean W over pair indices i for each segment k — ⟨W⟩_i(k).
    Bottom-right: mean W over segment indices k for each pair i — ⟨W⟩_k(i).

    Parameters
    ----------
    events : SegmentEvents
        Segment event times from compute_segment_events().
    title_suffix : str, optional
        Appended to each subplot title for context (e.g. "seed=42").

    Returns
    -------
    fig : matplotlib.figure.Figure
    """
    W = events.W  # (n_pairs, N_B)
    n_pairs, N_B = W.shape

    mean_over_i = W.mean(axis=0)  # (N_B,)  — mean budget per segment k
    mean_over_k = W.mean(axis=1)  # (n_pairs,) — mean budget per pair i

    suffix = f"  [{title_suffix}]" if title_suffix else ""

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 2, hspace=0.40, wspace=0.30)

    ax_heat = fig.add_subplot(gs[0, :])
    ax_k    = fig.add_subplot(gs[1, 0])
    ax_i    = fig.add_subplot(gs[1, 1])

    # ── Heatmap ──────────────────────────────────────────────────────────────
    im = ax_heat.imshow(W, cmap="RdYlGn", aspect="auto", interpolation="nearest")
    ax_heat.set_xlabel("Segment index k", fontsize=11)
    ax_heat.set_ylabel("Pair index i", fontsize=11)
    ax_heat.set_title(f"Budget matrix  W = t_in − t_out  (s){suffix}",
                      fontsize=12, fontweight="bold")
    ax_heat.set_xticks(np.arange(N_B))
    ax_heat.set_yticks(np.arange(0, n_pairs, max(1, n_pairs // 10)))
    plt.colorbar(im, ax=ax_heat, label="W  (s)")

    # ── Mean over pairs → per segment k ──────────────────────────────────────
    colors_k = ["C3" if v < 0 else "C0" for v in mean_over_i]
    ax_k.bar(np.arange(N_B), mean_over_i, color=colors_k, edgecolor="white", linewidth=0.5)
    ax_k.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax_k.set_xlabel("Segment index k", fontsize=11)
    ax_k.set_ylabel("⟨W⟩ over pairs  (s)", fontsize=11)
    ax_k.set_title(f"Mean budget per segment  ⟨W⟩_i(k){suffix}", fontsize=11)
    ax_k.set_xticks(np.arange(N_B))
    ax_k.grid(axis="y", alpha=0.3)

    # ── Mean over segments → per pair i ──────────────────────────────────────
    colors_i = ["C3" if v < 0 else "C1" for v in mean_over_k]
    ax_i.bar(np.arange(n_pairs), mean_over_k, color=colors_i, edgecolor="white", linewidth=0.5)
    ax_i.axhline(0, color="black", linewidth=0.8, linestyle="--")
    ax_i.set_xlabel("Pair index i", fontsize=11)
    ax_i.set_ylabel("⟨W⟩ over segments  (s)", fontsize=11)
    ax_i.set_title(f"Mean budget per pair  ⟨W⟩_k(i){suffix}", fontsize=11)
    ax_i.grid(axis="y", alpha=0.3)

    plt.show()
    return fig
