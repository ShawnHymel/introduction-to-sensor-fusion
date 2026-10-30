"""
utils.py

Utility functions for the sensor fusion notebooks:
 * Episode 5: vertical velocity (barometer + accelerometer)
 * Episode 6: horizontal position and velocity (GPS + IMU)
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

################################################################################
# Episode 6: GPS + IMU

# Mean Earth radius for the flat-earth conversion, and the Web Mercator radius
# used by online map tiles (m)
EARTH_RADIUS_M = 6371000.0
WEB_MERCATOR_RADIUS_M = 6378137.0

# Map tiles. OpenStreetMap's volunteer-run tile servers require apps to
# identify themselves (https://operations.osmfoundation.org/policies/tiles/),
# so we send a descriptive User-Agent. Adding a contact (URL or email) is
# recommended, e.g. "SensorFusionNotebook/1.0 (+https://example.com)".
# Tiles are cached in TILE_CACHE_DIR so re-running cells doesn't download
# them again.
TILE_USER_AGENT = "SensorFusionNotebook/1.0 (educational Kalman filter demo)"
TILE_CACHE_DIR = "tile_cache"


def _read_csv(path, skip=("tag", "utc", "date")):
    """Read a logger CSV into a dict of float arrays (empty fields -> NaN)."""
    with open(path, encoding="utf-8") as f:
        header = f.readline().strip().split(",")
    keep = [i for i, c in enumerate(header) if c not in skip]
    data = np.genfromtxt(path, delimiter=",", skip_header=1, usecols=keep,
                         ndmin=2)
    return {header[i]: data[:, j] for j, i in enumerate(keep)}


def load_imu_gps(path):
    """
    Load imu.csv, gps.csv and meta.json (if present) from an imu_gps_logger
    recording. Returns (imu, gps, meta): imu and gps are dicts of float arrays
    keyed by column name, plus 't' in seconds. meta is {} if missing.
    """
    folder = Path(path)
    meta = {}
    if (folder / "meta.json").exists():
        with open(folder / "meta.json", encoding="utf-8") as f:
            meta = json.load(f)

    imu = _read_csv(folder / "imu.csv")
    imu["t"] = imu["t_us"] * 1e-6
    gps = _read_csv(folder / "gps.csv")
    gps["t"] = gps["t_us"] * 1e-6

    return imu, gps, meta


def quat_to_rotmat(qi, qj, qk, qr):
    """
    Convert BNO085 quaternions (arrays) to rotation matrices, shape (n, 3, 3).
    Each matrix rotates a vector from the sensor frame to the world frame.
    """
    R = np.empty((len(qi), 3, 3))
    R[:, 0, 0] = 1 - 2 * (qj**2 + qk**2)
    R[:, 0, 1] = 2 * (qi * qj - qk * qr)
    R[:, 0, 2] = 2 * (qi * qk + qj * qr)
    R[:, 1, 0] = 2 * (qi * qj + qk * qr)
    R[:, 1, 1] = 1 - 2 * (qi**2 + qk**2)
    R[:, 1, 2] = 2 * (qj * qk - qi * qr)
    R[:, 2, 0] = 2 * (qi * qk - qj * qr)
    R[:, 2, 1] = 2 * (qj * qk + qi * qr)
    R[:, 2, 2] = 1 - 2 * (qi**2 + qj**2)
    return R


def lla_to_en(lat, lon, lat0, lon0):
    """
    Flat-earth conversion of latitude/longitude (deg) to East/North (m)
    around an origin. Accurate to well under a metre over a few km.
    """
    east = np.radians(lon - lon0) * np.cos(np.radians(lat0)) * EARTH_RADIUS_M
    north = np.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def en_to_lla(east, north, lat0, lon0):
    """Inverse of lla_to_en: East/North (m) back to latitude/longitude (deg)."""
    lat = lat0 + np.degrees(np.asarray(north) / EARTH_RADIUS_M)
    lon = lon0 + np.degrees(np.asarray(east) /
                            (EARTH_RADIUS_M * np.cos(np.radians(lat0))))
    return lat, lon


def lla_to_web_mercator(lat, lon):
    """Latitude/longitude (deg) to Web Mercator x/y (m), for map tiles."""
    x = WEB_MERCATOR_RADIUS_M * np.radians(lon)
    y = WEB_MERCATOR_RADIUS_M * np.log(np.tan(np.pi / 4 + np.radians(lat) / 2))
    return x, y


def wrap_angle(a):
    """Wrap angles (rad) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def window_average(t, x, t_centers, width=1.0):
    """
    Average x over a window of the given width (s) centered on each of
    t_centers. x can be 1D or 2D (samples along the first axis). Used to
    compare 100 Hz IMU data with 1 Hz GPS data.
    """
    x = np.asarray(x, dtype=float)
    csum = np.concatenate([np.zeros((1,) + x.shape[1:]), np.cumsum(x, axis=0)])
    i0 = np.searchsorted(t, np.asarray(t_centers) - width / 2)
    i1 = np.searchsorted(t, np.asarray(t_centers) + width / 2)
    count = np.maximum(i1 - i0, 1)
    if x.ndim > 1:
        count = count[:, None]
    return (csum[i1] - csum[i0]) / count


def _tile_provider(ctx, name):
    """Look up a contextily tile provider by dotted name, e.g. "CartoDB.Positron"."""
    provider = ctx.providers
    for part in name.split("."):
        provider = provider[part]
    return provider


def plot_tracks(tracks, origin, title="", basemap=True, zoom=None,
                arrows=None, figsize=(9, 9), tiles="OpenStreetMap.Mapnik"):
    """
    Plot East/North tracks, optionally on top of map tiles.

    tracks: list of (east, north, label) or (east, north, label, style),
            where style is a dict of matplotlib keyword arguments
    origin: (lat0, lon0) used for the East/North conversion
    basemap: draw map tiles under the tracks (needs the contextily package
             and an internet connection; falls back to a plain E/N plot)
    tiles: contextily tile provider name, e.g. "OpenStreetMap.Mapnik"
           (default) or "OpenTopoMap" (topographic, also open data)
    zoom: optional (east, north, half_width_m) to zoom in on one spot
    arrows: optional (east_from, north_from, east_to, north_to, label) drawn
            as short segments, e.g. prior -> posterior at each GPS update
    """
    lat0, lon0 = origin
    ctx = None
    if basemap:
        try:
            import contextily as ctx
            import contextily.tile

            # Identify ourselves to the tile server and keep a local cache
            contextily.tile.USER_AGENT = TILE_USER_AGENT
            Path(TILE_CACHE_DIR).mkdir(exist_ok=True)
            ctx.set_cache_dir(TILE_CACHE_DIR)
        except ImportError:
            ctx = None
            print("contextily not installed; plotting without a map "
                  "(pip install contextily)")

    # With a map, plot in Web Mercator; otherwise plain East/North metres
    def to_xy(e, n):
        if ctx is None:
            return np.asarray(e), np.asarray(n)
        lat, lon = en_to_lla(e, n, lat0, lon0)
        return lla_to_web_mercator(lat, lon)

    fig, ax = plt.subplots(figsize=figsize)
    for trace in tracks:
        e, n, label = trace[:3]
        style = trace[3] if len(trace) > 3 else {}
        x, y = to_xy(e, n)
        ax.plot(x, y, label=label, **{"lw": 1.5, **style})

    if arrows is not None:
        e0, n0, e1, n1, label = arrows
        x0, y0 = to_xy(e0, n0)
        x1, y1 = to_xy(e1, n1)
        for i in range(len(x0)):
            ax.plot([x0[i], x1[i]], [y0[i], y1[i]], color="tab:purple",
                    lw=1.5, label=label if i == 0 else None)
        ax.plot(x0, y0, "o", color="tab:purple", ms=3, mfc="none")

    if zoom is not None:
        ze, zn, half = zoom
        (xa, xb), (ya, yb) = (to_xy(np.array([ze - half, ze + half]),
                                    np.array([zn - half, zn + half])))
        ax.set_xlim(xa, xb)
        ax.set_ylim(ya, yb)

    # Equal scale on both axes: shrink the plot box rather than changing the
    # axis limits (the map tiles set the limits too)
    ax.set_aspect("equal", adjustable="box")
    if ctx is not None:
        try:
            ctx.add_basemap(ax, crs="EPSG:3857",
                            source=_tile_provider(ctx, tiles))
        except Exception as e:
            print(f"Could not load map tiles ({e}); plotting without a map")
        ax.set_xticks([])
        ax.set_yticks([])
    else:
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.grid(alpha=0.3)

    ax.set_title(title)
    ax.legend(loc="best")
    fig.tight_layout()
    plt.show()
