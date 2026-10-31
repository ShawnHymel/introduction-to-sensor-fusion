"""
utils.py

Utility functions for the vertical velocity (barometer + accelerometer)
notebooks
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_recording(path):
    """
    Load acc_baro.csv and meta.json from a baro_accel_logger recording.
    Returns (rec, meta), where rec is a dict of float arrays keyed by column
    name, plus 't' in seconds. Barometer columns are NaN on rows before the
    first barometer sample.
    """
    folder = Path(path)
    with open(folder / "meta.json", encoding="utf-8") as f:
        meta = json.load(f)

    # Read the numeric columns (empty fields parse as NaN)
    with open(folder / "acc_baro.csv", encoding="utf-8") as f:
        header = f.readline().strip().split(",")
    cols = [c for c in header if c != "tag"]
    data = np.genfromtxt(folder / "acc_baro.csv", delimiter=",", skip_header=1,
                         usecols=[header.index(c) for c in cols], ndmin=2)
    rec = {c: data[:, i] for i, c in enumerate(cols)}
    rec["t"] = rec["t_us"] * 1e-6
    rec["baro_new"] = rec["baro_new"] == 1

    return rec, meta


def baro_samples(rec):
    """
    Return (t, alt) for the rows that carry a new barometer sample. The time
    is when the barometer was actually read (row time minus baro_age_us).
    """
    new = rec["baro_new"]
    t = rec["t"][new] - rec["baro_age_us"][new] * 1e-6
    return t, rec["alt_m"][new]


def still_windows(motion, t_start, t_end, margin=0.5):
    """
    Turn a list of (start, end) motion segments into the still windows
    between them, each shrunk by margin seconds on both sides.
    """
    windows = []
    t0 = t_start
    for m0, m1 in motion:
        windows.append((t0, m0 - margin))
        t0 = m1 + margin
    windows.append((t0, t_end))

    return windows


def window_mean(t, x, window):
    """Mean of x over one (t_start, t_end) window, ignoring NaNs."""
    t0, t1 = window
    keep = (t >= t0) & (t <= t1)
    return np.nanmean(x[keep])


def rms_in_windows(t, x, windows):
    """RMS of x over all samples that fall inside the given windows."""
    keep = np.zeros(len(t), dtype=bool)
    for t0, t1 in windows:
        keep |= (t >= t0) & (t <= t1)
    return np.sqrt(np.nanmean(x[keep] ** 2))


def plot_traces(traces, ylabel, title="", windows=None, ylim=None,
                figsize=(12, 4)):
    """
    Plot time series that may have different timestamps.

    traces: list of (t, values, label) or (t, values, label, style), where
            style is a dict of matplotlib keyword arguments
    windows: optional list of (t_start, t_end) spans to shade (still periods)
    """
    fig, ax = plt.subplots(figsize=figsize)
    for trace in traces:
        t, y, label = trace[:3]
        style = trace[3] if len(trace) > 3 else {}
        ax.plot(t, y, label=label, **{"lw": 1.0, **style})

    if windows:
        for i, (t0, t1) in enumerate(windows):
            ax.axvspan(t0, t1, color="tab:green", alpha=0.12,
                       label="still" if i == 0 else None)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    if ylim:
        ax.set_ylim(ylim)
    ax.set_title(title)
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    plt.show()


def plot_histograms(rows, title, xlabel, bins=60):
    """
    Stacked histograms, each with its own x range (the signals have different
    units). rows = [(label, values, color), ...]
    """
    fig, axes = plt.subplots(len(rows), 1, figsize=(10, 2.0 * len(rows)))
    xlabels = xlabel if isinstance(xlabel, (list, tuple)) else [xlabel] * len(rows)
    for ax, (label, v, color), xl in zip(np.atleast_1d(axes), rows, xlabels):
        v = np.asarray(v, dtype=float)
        v = v[~np.isnan(v)]
        ax.hist(v, bins=bins, color=color, alpha=0.85)
        ax.axvline(v.mean(), color="k", ls="--", lw=1)
        ax.set_ylabel(label, rotation=0, ha="right", va="center")
        ax.set_yticks([])
        ax.set_xlabel(xl)
        ax.text(0.99, 0.9, f"mean = {v.mean():.4f}   σ = {v.std(ddof=1):.4f}",
                transform=ax.transAxes, ha="right", va="top", fontsize=9)
    fig.suptitle(title)
    fig.tight_layout()
    plt.show()

def sorted_streams(*streams):
    """
    Interleave several streams into one list of events, in time order.

    streams: (name, t, values) tuples, e.g. ("acc", t_acc, az_rel)
    Returns a list of (t, name, value) tuples:

        [(0.0080, "acc",  0.012),
         (0.0110, "baro", 0.003),
         (0.0160, "acc", -0.008),
         ...]
    """
    events = []
    for name, t, values in streams:
        events.extend((ti, name, vi) for ti, vi in zip(t, values))

    # Sort on the timestamp alone
    events.sort(key=lambda e: e[0])

    return events