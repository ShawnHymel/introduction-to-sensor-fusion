#!/usr/bin/env python3
"""
pir_mmwave_logger.py

Records a PIR + mmWave occupancy sequence from the occupancy_logger sketch
and writes a single CSV (occ.csv), plus a meta.json describing the recording.

Usage:
    python pir_mmwave_logger.py --port COM5 --name desk_work --seconds 600
    python pir_mmwave_logger.py -p /dev/ttyACM0 -n empty_room -s 900 -d 30 \
        --notes "door closed, fan off"
    python pir_mmwave_logger.py -p COM5 -n enter_leave -s 0      # Ctrl-C to stop
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


# Stream tag and output filename
TAG = "OCC"
OUT_FILE = "occ.csv"

# Extra time allowed for the sketch to start recording, on top of the
# requested delay (covers PIR warm-up after boot and waiting for mmWave data)
BEGIN_MARGIN_S = 45.0


def parse_args():
    ap = argparse.ArgumentParser(description="Record a PIR + mmWave sequence.")
    ap.add_argument("-p", "--port", required=True,
                    help="serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("-n", "--name", required=True,
                    help="sequence name; used for the output folder")
    ap.add_argument("-s", "--seconds", type=int, default=600,
                    help="duration; 0 records until Ctrl-C (default: 600)")
    ap.add_argument("-d", "--delay", type=int, default=0,
                    help="delay in seconds before recording starts, e.g. to "
                         "leave the room (default: 0)")
    ap.add_argument("--notes", default="",
                    help="free-text notes saved in meta.json "
                         "(e.g. 'fan on, door closed')")
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


def wait_for_begin(ser, timeout):
    """
    Discard output until '# BEGIN', then collect the config and column specs.
    Returns (columns, cfg).
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith("# ERROR"):
            sys.exit(f"Sketch rejected the command: {line}")
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
            return columns, cfg

        # Config lines: "# CFG key=value"
        if line.startswith("# CFG "):
            for token in line[len("# CFG "):].split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    cfg[k] = parse_value(v)
            continue

        # Column spec: "# OCC: t_us,tag,..."
        if line.startswith("#") and ":" in line:
            spec, cols = line.lstrip("# ").split(":", 1)
            columns[spec.strip()] = [c.strip() for c in cols.split(",")]
    sys.exit("Timed out waiting for '# DATA'.")


def parse_end(line):
    """Parse '# END duration_s=... occ=... late=... stale=...' into a dict."""
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

    # Track latest values and truth transitions for the summary
    row = dict(zip(cols, parts))
    stats["last"] = row
    truth = row.get("truth")
    if stats["prev_truth"] is not None and truth != stats["prev_truth"]:
        stats["truth_changes"] += 1
    stats["prev_truth"] = truth
    if truth == "1":
        stats["truth_present"] += 1
    if row.get("mmwave") == "-1":
        stats["mmwave_missing"] += 1


def record(ser, cols, outdir):
    """Stream rows to CSV until '# END'. Returns (stats, warnings, end_info)."""
    stats = {
        "count": 0, "last_seq": -1, "gaps": 0, "malformed": 0,
        "truth_present": 0, "truth_changes": 0, "prev_truth": None,
        "mmwave_missing": 0, "last": None,
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

                # Live status line, once per second
                if time.time() >= next_report and stats["last"]:
                    next_report += 1.0
                    r = stats["last"]
                    t_s = int(r["t_us"]) * 1e-6
                    print(f"  t={t_s:7.1f} s  rows={stats['count']:>6}  "
                          f"pir={r['pir']}  mmwave={r['mmwave']:>2}  "
                          f"truth={r['truth']}", end="\r", flush=True)

        except KeyboardInterrupt:
            print("\n  stopping...")
            end_info = stop_and_drain(ser, writer, cols, stats)

    return stats, warnings, end_info


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
    if "occ" in end_info and stats["count"] != end_info["occ"]:
        print(f"  WARNING: sketch sent {end_info['occ']}, "
              f"we captured {stats['count']}")
        ok = False

    # Timing and mmWave dropouts are reported but don't fail the recording
    if end_info.get("late", 0):
        print(f"  note: {end_info['late']} late sample(s)")
    if stats["mmwave_missing"]:
        print(f"  note: {stats['mmwave_missing']} sample(s) with no mmWave "
              f"data (logged as -1, treat as missing)")

    # Ground-truth summary
    if stats["count"]:
        frac = stats["truth_present"] / stats["count"]
        print(f"  truth: present {frac:.0%} of samples, "
              f"{stats['truth_changes']} change(s)")

    dur = end_info.get("duration_s")
    if dur:
        print(f"  duration: {dur:.3f} s")
        print(f"    {TAG:<14}{stats['count']/dur:>7.1f} Hz")

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
        columns, cfg = wait_for_begin(ser, args.delay + BEGIN_MARGIN_S)
    except KeyboardInterrupt:
        send(ser, "X")
        ser.close()
        sys.exit("\nCancelled before recording started.")

    if TAG not in columns:
        ser.close()
        sys.exit(f"Sketch did not describe a {TAG} stream: {columns}")
    cols = columns[TAG]
    print(f"  recording started (truth={cfg.get('truth_initial', '?')})")

    # Only create the folder once recording has actually started
    outdir.mkdir(parents=True, exist_ok=True)

    stats, warnings, end_info = record(ser, cols, outdir)
    ser.close()

    clean = verify(stats, end_info)

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
        "mmwave_missing": stats["mmwave_missing"],
        "truth_present_fraction": (stats["truth_present"] / stats["count"]
                                   if stats["count"] else None),
        "truth_changes": stats["truth_changes"],
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
