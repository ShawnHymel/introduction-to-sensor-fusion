"""
loader.py

Reads recordings produced by log_sensor.py into NumPy arrays.

A recording is a directory containing acc.csv, gyro.csv, mag.csv, an optional
ref.csv, and meta.json. Each stream has its own timestamps and its own rate,
so they are kept separate rather than merged.
"""

from dataclasses import dataclass, field
from pathlib import Path
import json

import numpy as np


@dataclass
class Stream:
    """One sensor stream: timestamps plus its data columns."""
    t: np.ndarray                 # seconds since the start of the recording
    data: dict                    # column name -> 1-D array
    seq: np.ndarray               # per-sample sequence number
    status: np.ndarray            # BNO085 accuracy, 0-3
    delay_us: np.ndarray          # signed; how long before the report the
                                  # sample was taken

    def __len__(self):
        return len(self.t)

    @property
    def rate(self):
        """Mean sample rate in Hz."""
        if len(self.t) < 2:
            return 0.0
        return (len(self.t) - 1) / (self.t[-1] - self.t[0])

    @property
    def gaps(self):
        """Number of breaks in the sequence numbering."""
        return int(np.sum(np.diff(self.seq) != 1))

    def xyz(self, prefix):
        """Stack three named columns into an (N, 3) array."""
        return np.column_stack([self.data[prefix + a] for a in "xyz"])

    def quat(self):
        """Stack the quaternion columns into an (N, 4) array of (w, x, y, z)."""
        missing = [c for c in ("qw", "qx", "qy", "qz") if c not in self.data]
        if missing:
            raise KeyError(f"stream has no quaternion columns (missing {missing})")
        return np.column_stack([self.data[c] for c in ("qw", "qx", "qy", "qz")])


@dataclass
class Recording:
    """A full recording: all streams plus the metadata."""
    path: Path
    meta: dict
    acc: Stream
    gyro: Stream
    mag: Stream
    ref: Stream = None            # absent when recorded with -r N

    @property
    def reference(self):
        """'BNO_GRV', 'BNO_RV', or 'NONE'."""
        return self.meta.get("reference", "NONE")

    @property
    def duration(self):
        return max(s.t[-1] for s in self.streams() if len(s))

    def streams(self):
        return [s for s in (self.acc, self.gyro, self.mag, self.ref)
                if s is not None]

    def summary(self):
        lines = [f"{self.path.name}  ({self.reference}, {self.duration:.1f} s)"]
        for name in ("acc", "gyro", "mag", "ref"):
            s = getattr(self, name)
            if s is None:
                continue
            note = f"  {s.gaps} gap(s)" if s.gaps else ""
            lines.append(f"  {name:<5}{len(s):>7} samples  {s.rate:6.1f} Hz{note}")
        return "\n".join(lines)

    def sorted_streams(self, *names):
        """
        Interleave several streams into one list of events, in time order.

        Returns a list of (t, name, values) tuples, where values holds that
        stream's data columns in header order:

            [(0.2120, "gyro", array([ 0.021, -0.005, -0.007, ...])),
             (0.2163, "acc",  array([ 0.504, -0.112,  9.781])),
             ...]

        Streams with bias columns (gyro, mag) return six values; slice the
        first three for the sensor reading itself.
        """
        events = []

        for name in names:
            stream = getattr(self, name, None)
            if stream is None:
                raise ValueError(f"{self.path.name} has no '{name}' stream")

            # dict preserves insertion order, which came from the CSV header
            values = np.column_stack(list(stream.data.values()))
            events.extend((t, name, v) for t, v in zip(stream.t, values))

        # Sort on the timestamp alone
        events.sort(key=lambda e: e[0])
        
        return events


# Columns present in every stream, before the sensor-specific ones.
_COMMON = ("t_us", "tag", "seq", "bno_delay_us", "status")


def _read_stream(path):
    """Read one CSV into a Stream, or return None if the file is absent."""
    if not path.exists():
        return None

    with open(path, newline="", encoding="utf-8") as f:
        header = f.readline().strip().split(",")

    raw = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=None,
                        encoding="utf-8", names=header)
    if raw.size == 0:
        return None
    raw = np.atleast_1d(raw)

    data = {name: raw[name].astype(float)
            for name in header if name not in _COMMON}

    return Stream(
        t=raw["t_us"].astype(float) * 1e-6,
        data=data,
        seq=raw["seq"].astype(int),
        status=raw["status"].astype(int),
        delay_us=raw["bno_delay_us"].astype(float),
    )


def load(path):
    """Load a recording directory into a Recording."""
    path = Path(path)
    if not path.is_dir():
        raise FileNotFoundError(f"{path} is not a directory")

    meta_path = path / "meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}

    acc  = _read_stream(path / "acc.csv")
    gyro = _read_stream(path / "gyro.csv")
    mag  = _read_stream(path / "mag.csv")
    ref  = _read_stream(path / "ref.csv")

    missing = [n for n, s in (("acc", acc), ("gyro", gyro), ("mag", mag))
               if s is None]
    if missing:
        raise ValueError(f"{path.name} is missing: {', '.join(missing)}")

    return Recording(path=path, meta=meta, acc=acc, gyro=gyro, mag=mag, ref=ref)


def load_all(base, pattern="*"):
    """Load every recording under a base directory, newest last."""
    base = Path(base)
    dirs = sorted(d for d in base.glob(pattern) if (d / "meta.json").exists())
    return [load(d) for d in dirs]