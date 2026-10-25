#!/usr/bin/env python3
"""
baro_accel_logger.py

Records a vertical motion sequence (e.g. an elevator ride) from the
baro_accel_logger sketch (BNO085 z acceleration + BMP581 pressure) and writes
a single CSV (acc_baro.csv), plus a meta.json describing the recording.

Each row is one accelerometer sample (100 Hz). Rows with baro_new = 1 carry a
new barometer sample (50 Hz); run the Kalman update on those rows.

The end-of-run summary prints az and altitude noise. On a recording made at
rest, these are the sigma_a and sigma_b values for tuning the filter.

Usage:
    python baro_accel_logger.py --port COM5 --name rest --seconds 60 \
        --notes "board flat on desk, foam cover"
    python baro_accel_logger.py -p /dev/ttyACM0 -n elevator_up -s 120
    python baro_accel_logger.py -p COM5 -n stairs -s 0      # Ctrl-C to stop
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
TAG = "ACC_BARO"
OUT_FILE = "acc_baro.csv"

# Extra time allowed for the sketch to start recording (covers the settling
# period after the start command)
BEGIN_MARGIN_S = 5.0


def parse_args():
    ap = argparse.ArgumentParser(
        description="Record a vertical acceleration + barometer sequence.")
    ap.add_argument("-p", "--port", required=True,
                    help="serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("-n", "--name", required=True,
                    help="sequence name; used for the output folder")
    ap.add_argument("-s", "--seconds", type=int, default=120,
                    help="duration; 0 records until Ctrl-C (default: 120)")
    ap.add_argument("--notes", default="",
                    help="free-text notes saved in meta.json "
                         "(e.g. 'elevator, floors 1 to 6')")
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
    """Parse a float, returning NaN for empty or bad values."""
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

        # Column spec: "# ACC_BARO: t_us,tag,..."
        if line.startswith("#") and ":" in line:
            spec, cols = line.lstrip("# ").split(":", 1)
            columns[spec.strip()] = [c.strip() for c in cols.split(",")]
    sys.exit("Timed out waiting for '# DATA'.")


def parse_end(line):
    """Parse '# END duration_s=... acc=... baro=... ok=...' into a dict."""
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

    # Keep values for the end-of-run summary (~100 rows/s, small enough)
    row = dict(zip(cols, parts))
    stats["last"] = row
    stats["az"].append(to_float(row.get("az")))

    status = row.get("acc_status", "?")
    stats["acc_status"][status] = stats["acc_status"].get(status, 0) + 1

    if row.get("press_pa", "") == "":
        stats["no_baro"] += 1
    elif row.get("baro_new") == "1":
        stats["baro_new"] += 1
        stats["alt"].append(to_float(row.get("alt_m")))
        age = int(row.get("baro_age_us") or 0)
        stats["max_baro_age_us"] = max(stats["max_baro_age_us"], age)


def record(ser, cols, outdir):
    """Stream rows to CSV until '# END'. Returns (stats, warnings, end_info)."""
    stats = {
        "count": 0, "last_seq": -1, "gaps": 0, "malformed": 0, "last": None,
        "az": [], "alt": [], "baro_new": 0, "no_baro": 0,
        "max_baro_age_us": 0, "acc_status": {},
    }
    warnings = []
    end_info = None
    next_report = time.time() + 1.0

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

                # Live status line, once per second. Altitude is shown
                # relative to the first barometer sample.
                if time.time() >= next_report and stats["last"]:
                    next_report += 1.0
                    r = stats["last"]
                    t_s = int(r["t_us"]) * 1e-6
                    az = to_float(r["az"])
                    if stats["alt"] and r["alt_m"]:
                        rel = to_float(r["alt_m"]) - stats["alt"][0]
                        alt = f"{rel:+7.2f} m"
                    else:
                        alt = "    --  "
                    print(f"  t={t_s:6.1f} s  rows={stats['count']:>6}  "
                          f"az={az:+7.3f}  alt={alt}  "
                          f"baro={stats['baro_new']:>5}",
                          end="\r", flush=True)

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
    Noise and range for az and barometric altitude. On a recording made at
    rest, az std is sigma_a and alt std is sigma_b (R = sigma_b^2).
    Returns a dict for meta.json.
    """
    az_m, az_s = mean_std(stats["az"])
    alt_m, alt_s = mean_std(stats["alt"])
    alts = [a for a in stats["alt"] if not math.isnan(a)]
    alt_range = (max(alts) - min(alts)) if alts else math.nan

    print(f"\n  {'signal':<10}{'mean':>12}{'std':>10}{'range':>10}")
    print(f"  {'az':<10}{az_m:>12.4f}{az_s:>10.4f}{'':>10}   (m/s^2)")
    print(f"  {'alt':<10}{alt_m:>12.3f}{alt_s:>10.4f}{alt_range:>10.3f}   (m)")
    print("  (at rest: az std ~ sigma_a, alt std ~ sigma_b)")

    return {
        "az": {"mean": az_m, "std": az_s},
        "alt": {"mean": alt_m, "std": alt_s, "range": alt_range},
        "acc_status_counts": stats["acc_status"],
        "max_baro_age_us": stats["max_baro_age_us"],
    }


def verify(stats, end_info):
    """Print a summary and return True if the recording looks clean."""
    print()
    ok = True

    note = ""
    if stats["gaps"]:
        note = f"  <-- {stats['gaps']} sequence gap(s)"
        ok = False
    print(f"  {TAG:<14}{stats['count']:>7} rows{note}")
    print(f"  {'  baro_new':<14}{stats['baro_new']:>7} rows")

    if stats["malformed"]:
        print(f"  malformed lines: {stats['malformed']}")
        ok = False

    if end_info is None:
        print("  WARNING: no '# END' received — the recording was cut short")
        return False

    # Cross-check our counts against the sketch's own tally
    if "acc" in end_info and stats["count"] != end_info["acc"]:
        print(f"  WARNING: sketch sent {end_info['acc']} rows, "
              f"we captured {stats['count']}")
        ok = False

    # Every barometer sample should land on exactly one row. The last one can
    # arrive after the final row, so allow a difference of one.
    if "baro" in end_info:
        expected = end_info["baro"] - end_info.get("baro_dropped", 0)
        if abs(stats["baro_new"] - expected) > 1:
            print(f"  WARNING: sketch read {expected} barometer samples, "
                  f"{stats['baro_new']} rows marked baro_new")
            ok = False

    # BNO085 reset or failed barometer reads mean gaps in the data
    if end_info.get("ok", 1) == 0:
        print("  WARNING: BNO085 reset during recording (see warnings)")
        ok = False
    if end_info.get("baro_err", 0):
        print(f"  WARNING: {end_info['baro_err']} failed barometer read(s)")
        ok = False
    if stats["baro_new"] == 0 and stats["count"]:
        print("  WARNING: no barometer data at all")
        ok = False

    # Timing details are reported but don't fail the recording
    if end_info.get("baro_gaps", 0):
        print(f"  note: {end_info['baro_gaps']} barometer gap(s)")
    if end_info.get("baro_dropped", 0):
        print(f"  note: {end_info['baro_dropped']} barometer sample(s) "
              f"overwritten before reaching a row")
    if stats["no_baro"]:
        print(f"  note: {stats['no_baro']} row(s) before the first "
              f"barometer sample")
    print(f"  max baro_age: {stats['max_baro_age_us'] / 1000:.1f} ms")
    print(f"  acc_status counts: {stats['acc_status']}")

    dur = end_info.get("duration_s")
    if dur:
        print(f"  duration: {dur:.3f} s")
        print(f"    {'accel':<14}{stats['count']/dur:>7.1f} Hz")
        print(f"    {'barometer':<14}{stats['baro_new']/dur:>7.1f} Hz")

    return ok


def main():
    args = parse_args()

    started = datetime.now(timezone.utc)
    folder = timestamped(args.name, started.astimezone())

    outdir = Path(args.outdir) / folder
    if outdir.exists():
        sys.exit(f"{outdir} already exists")

    ser = open_port(args.port, args.baud)

    cmd = f"S {args.seconds}"
    print(f"Recording '{args.name}' -> {outdir}")
    print(f"  command: {cmd}")
    if args.seconds == 0:
        print("  (press Ctrl-C to stop)")

    # Wait for the sketch to start recording (Ctrl-C cancels cleanly)
    send(ser, cmd)
    try:
        columns, cfg, warnings = wait_for_begin(ser, BEGIN_MARGIN_S)
    except KeyboardInterrupt:
        send(ser, "X")
        ser.close()
        sys.exit("\nCancelled before recording started.")

    if TAG not in columns:
        ser.close()
        sys.exit(f"Sketch did not describe a {TAG} stream: {columns}")
    cols = columns[TAG]
    print(f"  recording started (bmp_addr={cfg.get('bmp_addr', '?')}, "
          f"baro {cfg.get('bmp_odr_hz', '?')} Hz)")

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
        "started_utc": started.isoformat(),
        "port": args.port,
        "columns": cols,
        "config": cfg,
        "rows": stats["count"],
        "baro_rows": stats["baro_new"],
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
