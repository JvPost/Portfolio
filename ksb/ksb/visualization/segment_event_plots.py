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

    vmin, vmax = -0.1, 0.1
    norm = mcolors.TwoSlopeNorm(vcenter=0, vmin=vmin, vmax=vmax)
    im = ax_heat.imshow(np.clip(matrix, vmin, vmax), cmap="RdYlGn", norm=norm,
                        aspect="auto", interpolation="nearest")

    # # ── Heatmap ──────────────────────────────────────────────────────────────
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
    ax_i.set_title(f"Mean per pair  ${label}_k(i){suffix}$", fontsize=11)
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


def plot_pair_profile(
    values: np.ndarray,
    markers: np.ndarray | None = None,
    *,
    segments=None,
    ax: plt.Axes | None = None,
    ylabel: str = "",
    title: str = "",
    figsize: tuple = (20, 4),
) -> tuple[plt.Figure, plt.Axes]:
    """Line chart of a per-(pair, segment) quantity averaged over seeds.

    Parameters
    ----------
    values:
        Array of shape (n_seeds, n_pairs, n_segments).  Reduced to mean ± std
        over axis 0 before plotting.
    Markers:
        Iterable of pair indices at which to draw red dashed verticals.
    segments:
        Iterable of segment indices k to plot.  Defaults to all segments.
    ax:
        Axes to draw into.  If None, a new figure is created with *figsize*.
    ylabel:
        Y-axis label (LaTeX accepted).
    title:
        Axes title (LaTeX accepted).
    figsize:
        Figure size passed to ``plt.subplots`` when *ax* is None.

    Returns
    -------
    (fig, ax)
    """
    values = np.asarray(values)
    n_segments = values.shape[2]
    if segments is None:
        segments = range(n_segments)

    mean_v = values.mean(axis=0)
    std_v = values.std(axis=0)

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)
    else:
        fig = ax.figure

    x = range(1, mean_v.shape[0]+1)
    for k in segments:
        ax.plot(x, mean_v[:, k], label=f"k={k}", marker=".")
        ax.fill_between(
            x,
            mean_v[:, k] - std_v[:, k],
            mean_v[:, k] + std_v[:, k],
            alpha=0.2,
        )

    if markers is not None:
        for mark in markers:
            mark_i = mark+1
            ax.axvline(x=mark_i, color="red", linestyle="--", linewidth=0.8, alpha=0.7)

            ax.text(mark_i, -.01, str(mark_i), color="red", fontsize=8, ha="center",
                    va="top", transform=ax.get_xaxis_transform())

    ax.set_xlabel("Pair $(i, i+1)$")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()

    return fig, ax


def plot_phase_folded_margin(
    margin: np.ndarray,
    skip_indices,
    *,
    segments=None,
    ax: plt.Axes | None = None,
    band: str | None = "std",
    ylabel: str = "Kinematic margin $M_{i,k}$ (s)",
    title: str = "",
    figsize: tuple = (12, 4),
) -> tuple[plt.Figure, plt.Axes]:
    """Phase-fold the kinematic margin against pairs-since-last-skip.

    Each pair i is assigned a phase ``p = i - (largest skip index <= i)``
    (pairs before the first skip use ``p = i``).  This collapses the
    skip-locked sawtooth in *margin* onto a single mean curve per segment,
    so cycles of different length are overlaid by phase rather than by
    absolute pair index.

    Parameters
    ----------
    margin:
        Array of shape (n_pairs, n_segments) — kinematic margin per pair and segment.
    skip_indices:
        Sorted iterable of pair indices at which a skip occurs.
    segments:
        Iterable of segment indices k to plot.  Defaults to all segments.
    band:
        "std" -> shade mean ± std, "iqr" -> shade 25th-75th percentile,
        None -> no shaded band.  High phase values have fewer samples
        (cycles vary in length) — the band simply reflects that.
    ax:
        Axes to draw into.  If None, a new figure is created with *figsize*.
    ylabel:
        Y-axis label (LaTeX accepted).
    title:
        Axes title (LaTeX accepted).
    figsize:
        Figure size passed to ``plt.subplots`` when *ax* is None.

    Returns
    -------
    (fig, ax)
    """
    margin = np.asarray(margin)
    n_pairs, n_segments = margin.shape
    if segments is None:
        segments = range(n_segments)

    skips = np.asarray(sorted(skip_indices))
    phase = np.empty(n_pairs, dtype=int)
    for i in range(n_pairs):
        prior = skips[skips <= i]
        phase[i] = i - prior[-1] if prior.size else i
    max_phase = phase.max()

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)
    else:
        fig = ax.figure

    for k in segments:
        ps, means, lowers, uppers = [], [], [], []
        for p in range(max_phase + 1):
            values = margin[phase == p, k]
            if values.size == 0:
                continue
            ps.append(p)
            means.append(values.mean())
            if band == "std":
                lowers.append(values.mean() - values.std())
                uppers.append(values.mean() + values.std())
            elif band == "iqr":
                lowers.append(np.percentile(values, 25))
                uppers.append(np.percentile(values, 75))

        line, = ax.plot(ps, means, marker=".", label=f"k={k}")
        if band is not None:
            ax.fill_between(ps, lowers, uppers, color=line.get_color(), alpha=0.2)

    ax.axhline(0, color="black", linewidth=0.8)
    ax.set_xlabel("Pairs since last skip")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()

    return fig, ax


def plot_margin_cdf(
    margin: np.ndarray,
    *,
    segments=None,
    band: bool = True,
    ax: plt.Axes | None = None,
    xlabel: str = "Kinematic margin $M$ (s)",
    ylabel: str = "$P(M < m)$",
    title: str = "",
    figsize: tuple = (8, 5),
) -> tuple[plt.Figure, plt.Axes]:
    """Empirical CDF of the kinematic margin, one curve per segment.

    Negative margin means the buffer window is too tight to correct
    kinematically, so the value of a curve at ``m = 0`` is exactly that
    segment's failure rate ``P(M < 0)``.  Each curve is annotated with this
    crossing value so the failure rate is readable directly off the plot.

    Parameters
    ----------
    margin:
        Array of shape (n_pairs, n_segments) — kinematic margin per pair and segment.
    segments:
        Iterable of segment indices k to plot.  Defaults to all segments.
    band:
        If True, shade a ±1 std band around each ECDF curve, where the std
        at each point is the binomial standard error ``sqrt(F(1-F)/n)`` of
        the empirical CDF estimate.
    ax:
        Axes to draw into.  If None, a new figure is created with *figsize*.
    xlabel, ylabel:
        Axis labels (LaTeX accepted).
    title:
        Axes title (LaTeX accepted).
    figsize:
        Figure size passed to ``plt.subplots`` when *ax* is None.

    Returns
    -------
    (fig, ax)
    """
    margin = np.asarray(margin)
    n_segments = margin.shape[1]
    if segments is None:
        segments = range(n_segments)

    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)
    else:
        fig = ax.figure

    for k in segments:
        values = np.sort(margin[:, k])
        n = values.size
        cdf = np.arange(1, n + 1) / n
        line, = ax.step(values, cdf, where="post", label=f"k={k}")

        if band:
            se = np.sqrt(cdf * (1 - cdf) / n)
            ax.fill_between(
                values,
                np.clip(cdf - se, 0, 1),
                np.clip(cdf + se, 0, 1),
                step="post",
                color=line.get_color(),
                alpha=0.2,
            )

        failure_rate = np.mean(values < 0)
        ax.annotate(
            f"{failure_rate:.2f}",
            xy=(0, failure_rate),
            xytext=(5, -10),
            textcoords="offset points",
            color=line.get_color(),
            fontsize=9,
        )

    ax.axvline(0, color="black", linestyle="--", linewidth=0.8)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()

    return fig, ax
