"""
utils.py

Utility functions for the  temperature fusion notebook
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

def load_recording(path):
    """
    Load temp.csv and meta.json from a temp_logger recording. Returns
    (rec, meta), where rec is a dict of float arrays keyed by column name,
    plus 't' in seconds. Rows without a TMP117 reading are dropped.
    """
    folder = Path(path)
    with open(folder / "meta.json", encoding="utf-8") as f:
        meta = json.load(f)

    # Read the numeric columns ("nan" parses as NaN)
    with open(folder / "temp.csv", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
    cols = [c for c in header if c != "tag"]
    data = np.genfromtxt(folder / "temp.csv", delimiter=",", skip_header=1,
                         usecols=[header.index(c) for c in cols], ndmin=2)
    rec = {c: data[:, i] for i, c in enumerate(cols)}
    rec["t"] = rec["t_us"] * 1e-6

    # Drop rows with no reference reading
    keep = ~np.isnan(rec["tmp117_c"])
    if (~keep).any():
        print(f"Dropped {(~keep).sum()} row(s) with no TMP117 reading")
    rec = {k: v[keep] for k, v in rec.items()}

    return rec, meta

def plot_series(t, series, title):
    """Plot temperature traces. series = [(label, values, color), ...]"""
    fig, ax = plt.subplots(figsize=(10, 4))
    for label, y, color in series:
        ax.plot(t, y, label=label, color=color, lw=2.0 if color == "k" else 1.0)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title(title)
    ax.legend(loc="upper right", ncol=len(series))
    ax.grid(alpha=0.3)
    plt.show()


def plot_histograms(rows, ref_value, title, xlabel, bin_c=0.02):
    """
    Stacked histograms on a shared x axis, with a dashed reference line.
    rows = [(label, values, color), ...]
    """
    values = [v[~np.isnan(v)] for _, v, _ in rows]
    lo = min(min(v.min() for v in values), ref_value)
    hi = max(max(v.max() for v in values), ref_value)
    bins = np.arange(np.floor(lo / bin_c) * bin_c, hi + 2 * bin_c, bin_c)

    fig, axes = plt.subplots(len(rows), 1, figsize=(10, 1.6 * len(rows)),
                             sharex=True)
    for ax, (label, _, color), v in zip(np.atleast_1d(axes), rows, values):
        ax.hist(v, bins=bins, color=color, alpha=0.85)
        ax.axvline(ref_value, color="k", ls="--", lw=1)
        ax.set_ylabel(label, rotation=0, ha="right", va="center")
        ax.set_yticks([])
        ax.text(0.99, 0.9, f"mean = {v.mean():.3f}   σ = {v.std(ddof=1):.3f}",
                transform=ax.transAxes, ha="right", va="top", fontsize=9)
    np.atleast_1d(axes)[-1].set_xlabel(xlabel)
    fig.suptitle(title)
    fig.tight_layout()
    plt.show()