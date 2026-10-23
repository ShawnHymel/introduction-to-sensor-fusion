"""
plotter.py

Wraps up the common plots used in the series. Use these functions to generate
raw IMU plots, etc.
"""

import matplotlib.pyplot as plt
import numpy as np


# What to plot for each stream: the labels, the units, and how to get the
# values out of the recording.
STREAM_SPECS = {
    "acc": (("ax", "ay", "az"), "accel (m/s²)", lambda r: r.acc.xyz("a")),
    "gyro": (("gx", "gy", "gz"), "gyro (°/s)",
        lambda r: np.degrees(r.gyro.xyz("g"))),
    "mag": (("mx", "my", "mz"), "mag (µT)", lambda r: r.mag.xyz("m")),
}


def plot_imu(rec, streams=("acc", "gyro"), title=None, figsize=None):
    """
    Plot raw IMU data, one stacked subplot per stream.

    rec: a Recording from loader.load()
    streams: which streams to show, e.g. ("acc",), ("acc", "gyro"),
             or ("acc", "gyro", "mag")
    title: figure title; defaults to the recording's folder name
    """
    if figsize is None:
        figsize = (12, 3 * len(streams))

    fig, axes = plt.subplots(len(streams), 1, figsize=figsize, sharex=True,
                             squeeze=False)
    axes = axes.ravel()

    for ax, name in zip(axes, streams):
        labels, ylabel, getter = STREAM_SPECS[name]
        t = getattr(rec, name).t
        values = getter(rec)

        for i, label in enumerate(labels):
            ax.plot(t, values[:, i], lw=0.8, label=label)

        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right")
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("time (s)")
    fig.suptitle(title if title is not None else rec.path.name)
    fig.tight_layout()


def plot_pitch(traces, title=None, figsize=(12, 5), ylim=None):
    """
    Plot pitch estimates against time.

    traces: list of (t, pitch, label) tuples, or (t, pitch, label, style)
            where style is a dict of matplotlib keyword arguments.
    """
    fig, ax = plt.subplots(figsize=figsize)

    for trace in traces:
        t, pitch, label = trace[:3]
        style = trace[3] if len(trace) > 3 else {}
        ax.plot(t, pitch, label=label, **{"lw": 1.0, **style})

    ax.set_ylabel("pitch (°)")
    ax.set_xlabel("time (s)")
    if ylim:
        ax.set_ylim(ylim)
    ax.legend()
    ax.grid(alpha=0.3)
    if title is not None:
        ax.set_title(title)
    fig.tight_layout()
