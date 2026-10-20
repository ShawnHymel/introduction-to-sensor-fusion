"""
orientation.py

Conversions between quaternions and Euler angles.

Quaternions are (w, x, y, z). Euler angles follow the aerospace ZYX
convention: yaw about Z, then pitch about Y, then roll about X.
"""

import numpy as np


def quat_to_euler(q):
    """
    Convert quaternions to roll, pitch, yaw in radians.

    q: (4,) or (N, 4) array of (w, x, y, z)
    returns: (3,) or (N, 3) array of (roll, pitch, yaw)
    """
    q = np.atleast_2d(np.asarray(q, dtype=float))
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))

    # Clipped because rounding can push this just outside [-1, 1] near +/-90.
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))

    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    out = np.column_stack([roll, pitch, yaw])
    return out[0] if out.shape[0] == 1 else out


def pitch_from_quat(q):
    """Pitch only, in degrees."""
    return np.degrees(quat_to_euler(q)[..., 1])


def pitch_from_accel(a):
    """
    Pitch from the accelerometer, in degrees.

    Uses the full form so that roll doesn't contaminate the result. Only valid
    when the sensor isn't accelerating -- any linear acceleration is
    indistinguishable from a change in the direction of gravity.

    a: (N, 3) array of (ax, ay, az)
    """
    a = np.atleast_2d(np.asarray(a, dtype=float))
    ax, ay, az = a[:, 0], a[:, 1], a[:, 2]
    return np.degrees(np.arctan2(-ax, np.sqrt(ay * ay + az * az)))