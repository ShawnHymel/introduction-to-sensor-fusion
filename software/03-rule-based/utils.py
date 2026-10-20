"""
utils.py

Utility functions for the Bayes Filter notebook
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

def load_recording(path):
    """Load occ.csv and meta.json from a recording folder. Returns (rec, meta)."""
    path = Path(path)

    # Skip the header row and the text "tag" column (column 1)
    data = np.loadtxt(path / "occ.csv", delimiter=",", skiprows=1,
                      usecols=(0, 2, 3, 4, 5, 6), dtype=np.int64)
    rec = {
        "t": data[:, 0] * 1e-6,
        "seq": data[:, 1],
        "pir": data[:, 2],
        "mmwave": data[:, 3],
        "truth": data[:, 4],
        "sens_raw": data[:, 5],
    }

    # Load metadata
    with open(path / "meta.json", encoding="utf-8") as f:
        meta = json.load(f)

    return rec, meta

def plot_timeline(t, rows, title=""):
    """
    Plot stacked traces that share a time axis.

    rows: list of (label, values, color). Labels starting with "P(" are drawn
    as probabilities (0 to 1, with the threshold line); everything else is
    drawn as a 0/1 signal.
    """
    t = np.asarray(t)
    fig, axes = plt.subplots(len(rows), 1, sharex=True,
                             figsize=(12, 1.4 * len(rows) + 0.8))
    axes = np.atleast_1d(axes)

    for ax, (label, values, color) in zip(axes, rows):
        values = np.asarray(values, dtype=float)

        # Probability trace: continuous line with the decision threshold
        if label.startswith("P("):
            ax.plot(t, values, color=color, lw=1.5)
            ax.axhline(THRESHOLD, color="k", ls="--", lw=0.8)
            ax.set_ylim(-0.05, 1.05)

        # 0/1 trace: hold each value until the next sample
        else:
            ax.step(t, values, where="post", color=color, lw=1.5)
            ax.set_ylim(-0.2, 1.2)
            ax.set_yticks([0, 1])

        ax.set_ylabel(label, rotation=0, ha="right", va="center")
        ax.grid(axis="x", alpha=0.3)

    axes[0].set_title(title)
    axes[-1].set_xlabel("Time (s)")
    fig.tight_layout()
    plt.show()

def confusion_matrix(pred, truth):
    """
    2x2 confusion matrix for a 0/1 sensor vs. ground truth.
    Rows = truth (occupied, empty), columns = sensor (1, 0).
    Samples where the sensor is missing (-1) are skipped.
    """
    valid = pred >= 0
    pred = pred[valid]
    truth = truth[valid]
    tp = np.sum((truth == 1) & (pred == 1))
    fn = np.sum((truth == 1) & (pred == 0))
    fp = np.sum((truth == 0) & (pred == 1))
    tn = np.sum((truth == 0) & (pred == 0))

    return np.array([[tp, fn],
                     [fp, tn]])

def print_confusion_matrix(cm):
    """Print a confusion matrix"""
    (tp, fn), (fp, tn) = cm
    print(f"                 sensor=1  sensor=0")
    print(f"  truth=occupied {tp:8d}  {fn:8d}")
    print(f"  truth=empty    {fp:8d}  {tn:8d}")
    print()

def print_error_rates(cm):
    """Print the confusion matrix as rates (each row sums to 1)"""
    (tp, fn), (fp, tn) = cm
    tpr = tp / (tp + fn)
    fnr = fn / (tp + fn)
    fpr = fp / (fp + tn)
    tnr = tn / (fp + tn)
    print(f"                 sensor=1  sensor=0")
    print(f"  truth=occupied {tpr:8.3f}  {fnr:8.3f}    (TPR, FNR)")
    print(f"  truth=empty    {fpr:8.3f}  {tnr:8.3f}    (FPR, TNR)")
    print()