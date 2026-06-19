"""Headless matplotlib figure builders for the analysis panels.

Kept free of any Qt dependency so they are unit-testable and reusable (reports,
CI thumbnails). Each function takes plain arrays and returns a Figure.
"""
from __future__ import annotations

import numpy as np
import matplotlib
matplotlib.use("Agg")  # safe default; the GUI swaps in the Qt backend canvas
import matplotlib.pyplot as plt


_COLORS = {
    "two_point": "#888888",
    "minmax": "#1f77b4",
    "smoothed": "#2ca02c",
    "global": "#d62728",
}


def deviation_chart(compare: dict, fig=None):
    """Per-station peak deviation (microns) for each strategy."""
    fig = fig or plt.figure(figsize=(5, 3))
    ax = fig.add_subplot(111)
    for name, dev in compare.items():
        ax.plot(np.arange(len(dev)), dev * 1000.0, label=name,
                color=_COLORS.get(name), lw=1.6)
    ax.set_xlabel("ruling (station)")
    ax.set_ylabel("peak deviation (µm)")
    ax.set_title("Flank deviation by strategy")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper center", ncol=2)
    fig.tight_layout()
    return fig


def machinability_chart(delta, dev, fig=None):
    """Distribution parameter |delta| (twist) vs achieved deviation."""
    fig = fig or plt.figure(figsize=(5, 3))
    ax = fig.add_subplot(111)
    u = np.arange(len(delta))
    ad = np.abs(delta)
    ad[~np.isfinite(ad)] = np.nan
    ax.plot(u, ad, color="#9467bd", lw=1.6, label="|δ| (mm)")
    ax.set_xlabel("ruling (station)")
    ax.set_ylabel("|δ|  (mm)  — lower = more twist", color="#9467bd")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(u, dev * 1000.0, color="#d62728", lw=1.4, label="deviation (µm)")
    ax2.set_ylabel("deviation (µm)", color="#d62728")
    ax.set_title("Machinability map")
    fig.tight_layout()
    return fig


def compare_chart(stats: dict, fig=None):
    """Grouped bars comparing strategies on deviation / jerk / cycle.

    Each metric is normalised to its max across strategies so the three
    (different-unit) metrics share an axis; lower is better for all.
    """
    fig = fig or plt.figure(figsize=(5, 3))
    ax = fig.add_subplot(111)
    names = list(stats.keys())
    metrics = [("dev_um", "peak dev"), ("jerk", "jerk"), ("cycle_s", "cycle")]
    x = np.arange(len(metrics))
    w = 0.8 / max(1, len(names))
    for i, name in enumerate(names):
        vals = []
        for key, _ in metrics:
            col = [stats[n][key] for n in names]
            mx = max(col) or 1.0
            vals.append(stats[name][key] / mx)
        ax.bar(x + i*w, vals, w, label=name, color=_COLORS.get(name))
    ax.set_xticks(x + 0.4 - w/2)
    ax.set_xticklabels([m[1] for m in metrics])
    ax.set_ylabel("normalised (lower = better)")
    ax.set_title("Strategy comparison")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    return fig


def chatter_chart(rpm, alim, nlobes, nptsper, feed_rpm=None, fig=None):
    """Stability-lobe diagram: limiting depth a_lim vs spindle speed."""
    fig = fig or plt.figure(figsize=(5, 3))
    ax = fig.add_subplot(111)
    rpm = np.asarray(rpm).reshape(nlobes, nptsper)
    alim = np.asarray(alim).reshape(nlobes, nptsper)
    for k in range(nlobes):
        order = np.argsort(rpm[k])
        ax.plot(rpm[k][order], alim[k][order], color="#1f77b4", lw=1.2)
    amin = float(np.min(alim))
    ax.axhline(amin, color="#d62728", ls="--", lw=1,
               label=f"a_lim,min = {amin:.2f} mm")
    if feed_rpm:
        ax.axvline(feed_rpm, color="green", ls=":", lw=1, label="spindle")
    ax.set_xlabel("spindle speed (rpm)")
    ax.set_ylabel("limiting depth a_lim (mm)")
    ax.set_title("Chatter stability lobes")
    ax.set_ylim(0, min(amin * 8, float(np.percentile(alim, 95))))
    ax.set_xlim(0, float(np.percentile(rpm, 98)))
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def feed_chart(seglen, aprof, fig=None):
    """Time-optimal tool-tip feed (mm/min) along the toolpath."""
    fig = fig or plt.figure(figsize=(5, 3))
    ax = fig.add_subplot(111)
    n = len(aprof)
    ds = 1.0 / (n - 1)
    dLds = np.gradient(seglen, ds)
    feed = np.sqrt(np.clip(aprof, 0, None)) * dLds * 60.0  # mm/min
    ax.plot(seglen, feed, color="#ff7f0e", lw=1.6)
    ax.fill_between(seglen, feed, alpha=0.15, color="#ff7f0e")
    ax.set_xlabel("contact path length (mm)")
    ax.set_ylabel("feed (mm/min)")
    ax.set_title("TOPP time-optimal feed")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig
