#!/usr/bin/env python3
"""
temp_logger.py

Records a temperature sequence from the temp_logger sketch (NTC, TMP36,
MCP9700A, TMP117) and writes a single CSV (temp.csv), plus a meta.json
describing the recording.

Usage:
    python temp_logger.py --port COM5 --name static --seconds 300 --delay 900
    python temp_logger.py -p /dev/ttyACM0 -n heat_gun -s 600 \
        --notes "foam cover, heat applied at ~60 s"
    python temp_logger.py -p COM5 -n test -s 0      # Ctrl-C to stop
"""

import argparse
import csv
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


# Stream tag and output filename
TAG = "TEMP"
OUT_FILE = "temp.csv"

# Extra time allowed for the sketch to start recording, on top of the
# requested delay (covers the TMP117 start-up wait after boot)
BEGIN_MARGIN_S = 10.0

# Converted temperature columns, and the reference sensor
SENSORS = ["ntc_c", "tmp36_c", "mcp9700_c", "tmp117_c"]
REFERENCE = "tmp117_c"


def parse_args():
    ap = argparse.ArgumentParser(description="Record a temperature sequence.")
    ap.add_argument("-p", "--port", required=True,
                    help="serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("-n", "--name", required=True,
                    help="sequence name; used for the output folder")
    ap.add_argument("-s", "--seconds", type=int, default=300,
                    help="duration; 0 records until Ctrl-C (default: 300)")
    ap.add_argument("-d", "--delay", type=int, default=0,
                    help="delay in seconds before recording starts, e.g. to "
                         "let the block settle (default: 0)")
    ap.add_argument("--notes", default="",
                    help="free-text notes saved in meta.json "
                         "(e.g. 'foam cover, static')")
    ap.add_argument("-o", "--outdir", default="recordings",
                    help="base output directory (default: recordings)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="baud rate, ignored for native USB (default: 115200)")
    return ap.parse_args()


def open_port(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}")
    time.sleep(0.2)
    ser.reset_input_buffer()
    return ser


def send(ser, line):
    ser.write((line + "\n").encode("ascii"))
    ser.flush()


def read_line(ser):
    """Return one decoded line, or None on timeout."""
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").strip()


def parse_value(v):
    """Convert a string to int or float if possible."""
    try:
        return float(v) if "." in v else int(v)
    except ValueError:
        return v


def to_float(v):
    """Parse a float, returning NaN for 'nan' or bad values."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def wait_for_begin(ser, timeout):
    """
    Discard output until '# BEGIN', then collect the config and column specs.
    Returns (columns, cfg, warnings).
    """
    warnings = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith("# ERROR"):
            sys.exit(f"Sketch rejected the command: {line}")
        if line.startswith("# WARN"):
            print(f"  {line}")
            warnings.append(line)
        if line.startswith("# waiting"):
            print(f"  {line[2:]}")
        if line == "# BEGIN":
            break
    else:
        sys.exit("Timed out waiting for '# BEGIN' — is the sketch running?")

    columns, cfg = {}, {}
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line == "# DATA":
            return columns, cfg, warnings

        # Config lines: "# CFG key=value"
        if line.startswith("# CFG "):
            for token in line[len("# CFG "):].split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    cfg[k] = parse_value(v)
            continue

        # Column spec: "# TEMP: t_us,tag,..."
        if line.startswith("#") and ":" in line:
            spec, cols = line.lstrip("# ").split(":", 1)
            columns[spec.strip()] = [c.strip() for c in cols.split(",")]
    sys.exit("Timed out waiting for '# DATA'.")


def parse_end(line):
    """Parse '# END duration_s=... temp=... late=... i2c_err=...'."""
    out = {}
    for token in line[len("# END"):].split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        out[k] = parse_value(v)
    return out


def timestamped(name, when):
    return f"{name}-{when.strftime('%Y%m%d-%H%M%S')}"


def stop_and_drain(ser, writer, cols, stats):
    """Send X and capture any remaining rows until '# END'."""
    send(ser, "X")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith("# END"):
            return parse_end(line)
        if not line.startswith("#"):
            handle_row(line, writer, cols, stats)
    return None


def handle_row(line, writer, cols, stats):
    """Write one data row and update running statistics."""
    parts = line.split(",")
    if len(parts) != len(cols) or parts[1] != TAG:
        stats["malformed"] += 1
        return

    writer.writerow(parts)
    stats["count"] += 1

    # Track sequence gaps
    try:
        s = int(parts[2])
        if stats["last_seq"] >= 0 and s != stats["last_seq"] + 1:
            stats["gaps"] += 1
        stats["last_seq"] = s
    except ValueError:
        pass

    # Keep temperatures for the end-of-run summary (small: ~1 row/s)
    row = dict(zip(cols, parts))
    stats["last"] = row
    for name in SENSORS:
        v = to_float(row.get(name))
        stats["values"][name].append(v)
        if math.isnan(v):
            stats["missing"][name] += 1


def record(ser, cols, outdir):
    """Stream rows to CSV until '# END'. Returns (stats, warnings, end_info)."""
    stats = {
        "count": 0, "last_seq": -1, "gaps": 0, "malformed": 0, "last": None,
        "values": {name: [] for name in SENSORS},
        "missing": {name: 0 for name in SENSORS},
    }
    warnings = []
    end_info = None

    with open(outdir / OUT_FILE, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(cols)

        try:
            while True:
                line = read_line(ser)
                if line is None:
                    continue

                if line.startswith("#"):
                    if line.startswith("# END"):
                        end_info = parse_end(line)
                        break
                    if line.startswith("# WARN") or line.startswith("# ERROR"):
                        print(f"\n  {line}")
                        warnings.append(line)
                    continue

                handle_row(line, writer, cols, stats)

                # Live status line (rows arrive at ~1 Hz)
                r = stats["last"]
                if r:
                    t_s = int(r["t_us"]) * 1e-6
                    temps = "  ".join(
                        f"{n[:-2]}={to_float(r[n]):6.2f}" for n in SENSORS)
                    print(f"  t={t_s:6.0f} s  rows={stats['count']:>5}  "
                          f"{temps}", end="\r", flush=True)

        except KeyboardInterrupt:
            print("\n  stopping...")
            end_info = stop_and_drain(ser, writer, cols, stats)

    return stats, warnings, end_info


def mean_std(xs):
    """Mean and sample standard deviation, ignoring NaNs."""
    xs = [x for x in xs if not math.isnan(x)]
    n = len(xs)
    if n == 0:
        return math.nan, math.nan
    m = sum(xs) / n
    if n < 2:
        return m, math.nan
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, math.sqrt(var)


def summarize(stats):
    """
    Per-sensor stats, plus error vs. the reference (sensor - TMP117).
    The error mean is the calibration offset; the error std is the noise
    sigma for inverse-variance weights. Returns a dict for meta.json.
    """
    ref = stats["values"][REFERENCE]
    summary = {}
    print(f"\n  {'sensor':<11}{'mean':>9}{'std':>8}"
          f"{'offset':>9}{'err std':>9}{'missing':>9}")
    for name in SENSORS:
        vals = stats["values"][name]
        m, s = mean_std(vals)
        entry = {"mean": m, "std": s, "missing": stats["missing"][name]}
        line = f"  {name[:-2]:<11}{m:>9.3f}{s:>8.3f}"
        if name != REFERENCE:
            errs = [v - r for v, r in zip(vals, ref)]
            em, es = mean_std(errs)
            entry.update({"err_mean": em, "err_std": es})
            line += f"{em:>+9.3f}{es:>9.3f}"
        else:
            line += f"{'(ref)':>9}{'':>9}"
        line += f"{stats['missing'][name]:>9}"
        print(line)
        summary[name] = entry
    return summary


def verify(stats, end_info):
    """Print a summary and return True if the recording looks clean."""
    print()
    ok = True

    note = ""
    if stats["gaps"]:
        note = f"  <-- {stats['gaps']} sequence gap(s)"
        ok = False
    print(f"  {TAG:<14}{stats['count']:>7} rows{note}")

    if stats["malformed"]:
        print(f"  malformed lines: {stats['malformed']}")
        ok = False

    if end_info is None:
        print("  WARNING: no '# END' received — the recording was cut short")
        return False

    # Cross-check our row count against the sketch's own tally
    if "temp" in end_info and stats["count"] != end_info["temp"]:
        print(f"  WARNING: sketch sent {end_info['temp']}, "
              f"we captured {stats['count']}")
        ok = False

    # Missing reference readings make error stats unreliable
    if end_info.get("i2c_err", 0):
        print(f"  WARNING: {end_info['i2c_err']} failed TMP117 read(s)")
        ok = False
    if stats["missing"][REFERENCE] == stats["count"] and stats["count"]:
        print("  WARNING: no TMP117 data at all")
        ok = False

    # Timing is reported but doesn't fail the recording
    if end_info.get("late", 0):
        print(f"  note: {end_info['late']} late sample(s)")

    dur = end_info.get("duration_s")
    if dur:
        print(f"  duration: {dur:.3f} s")
        print(f"    {TAG:<14}{stats['count']/dur:>7.2f} Hz")

    return ok


def main():
    args = parse_args()

    if args.delay < 0:
        sys.exit("--delay must be >= 0")

    started = datetime.now(timezone.utc)
    folder = timestamped(args.name, started.astimezone())

    outdir = Path(args.outdir) / folder
    if outdir.exists():
        sys.exit(f"{outdir} already exists")

    ser = open_port(args.port, args.baud)

    cmd = f"S {args.seconds} {args.delay}"
    print(f"Recording '{args.name}' -> {outdir}")
    print(f"  command: {cmd}")
    if args.seconds == 0:
        print("  (press Ctrl-C to stop)")

    # Wait for the sketch to start recording (Ctrl-C cancels cleanly)
    send(ser, cmd)
    try:
        columns, cfg, warnings = wait_for_begin(
            ser, args.delay + BEGIN_MARGIN_S)
    except KeyboardInterrupt:
        send(ser, "X")
        ser.close()
        sys.exit("\nCancelled before recording started.")

    if TAG not in columns:
        ser.close()
        sys.exit(f"Sketch did not describe a {TAG} stream: {columns}")
    cols = columns[TAG]
    print(f"  recording started (tmp117_ok={cfg.get('tmp117_ok', '?')})")

    # Only create the folder once recording has actually started
    outdir.mkdir(parents=True, exist_ok=True)

    stats, rec_warnings, end_info = record(ser, cols, outdir)
    warnings += rec_warnings
    ser.close()

    clean = verify(stats, end_info)
    summary = summarize(stats) if stats["count"] else {}

    # NaN isn't valid JSON, so store it as null
    def clean_nan(d):
        return {k: (clean_nan(v) if isinstance(v, dict)
                    else None if isinstance(v, float) and math.isnan(v)
                    else v) for k, v in d.items()}

    meta = {
        "name": args.name,
        "folder": folder,
        "notes": args.notes,
        "requested_seconds": args.seconds,
        "requested_delay_s": args.delay,
        "started_utc": started.isoformat(),
        "port": args.port,
        "columns": cols,
        "config": cfg,
        "rows": stats["count"],
        "sequence_gaps": stats["gaps"],
        "malformed_lines": stats["malformed"],
        "summary": clean_nan(summary),
        "sketch_end": end_info,
        "warnings": warnings,
        "clean": clean,
    }
    with open(outdir / "meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print(f"\n{'OK' if clean else 'CHECK WARNINGS'} — wrote {outdir}")
    sys.exit(0 if clean else 1)


if __name__ == "__main__":
    main()
