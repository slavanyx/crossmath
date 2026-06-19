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
