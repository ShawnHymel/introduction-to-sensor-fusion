#!/usr/bin/env python3
"""
sensor_logger.py

Records a sensor sequence from the BNO085 logger sketch and writes one CSV
per stream, plus a meta.json describing the recording.

Usage:
    python sensor_logger.py --port COM5 --name yaw_slow --ref G --seconds 60
    python sensor_logger.py -p /dev/ttyACM0 -o recordings -r N --seconds 0 -n static_long
"""

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import serial


# Map stream tags to output filenames.
TAG_FILES = {
    "BNO_ACC":     "acc.csv",
    "BNO_GYRO_UC": "gyro.csv",
    "BNO_MAG_UC":  "mag.csv",
    "BNO_GRV":     "ref.csv",
    "BNO_RV":      "ref.csv",
}


def parse_args():
    ap = argparse.ArgumentParser(description="Record a BNO085 sensor sequence.")
    ap.add_argument("-p", "--port", required=True,
                    help="serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("-n", "--name", required=True,
                    help="sequence name; used for the output folder")
    ap.add_argument("-r", "--ref", default="G", choices=["G", "R", "N"],
                    help="reference: G=game rotation vector, R=rotation vector, "
                         "N=none (default: G)")
    ap.add_argument("-s", "--seconds", type=int, default=60,
                    help="duration; 0 records until Ctrl-C (default: 60)")
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


def wait_for_begin(ser, timeout=10.0):
    """Discard output until '# BEGIN', then collect the column specs."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith("# ERROR"):
            sys.exit(f"Sketch rejected the command: {line}")
        if line == "# BEGIN":
            break
    else:
        sys.exit("Timed out waiting for '# BEGIN' — is the sketch running?")

    columns = {}
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line == "# DATA":
            return columns
        if line.startswith("#") and ":" in line:
            spec, cols = line.lstrip("# ").split(":", 1)
            columns[spec.strip()] = [c.strip() for c in cols.split(",")]
    sys.exit("Timed out waiting for '# DATA'.")


def parse_end(line):
    """Parse '# END duration_s=... acc=... ok=...' into a dict."""
    out = {}
    for token in line[len("# END"):].split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        try:
            out[k] = float(v) if "." in v else int(v)
        except ValueError:
            out[k] = v
    return out


def timestamped(name, when):
    return f"{name}-{when.strftime('%Y%m%d-%H%M%S')}"


def record(ser, columns, outdir):
    """Stream rows to CSV until '# END'. Returns (counts, warnings, end_info)."""
    files, writers = {}, {}
    for tag, cols in columns.items():
        fname = TAG_FILES.get(tag)
        if fname is None:
            print(f"  note: no output file mapped for {tag}, skipping")
            continue
        fh = open(outdir / fname, "w", newline="", encoding="utf-8")
        files[tag] = fh
        writers[tag] = csv.writer(fh)
        writers[tag].writerow(cols)

    counts = {tag: 0 for tag in writers}
    last_seq = {tag: -1 for tag in writers}
    gaps = {tag: 0 for tag in writers}
    warnings, malformed = [], 0
    end_info = None
    next_report = time.time() + 1.0

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
                    print(f"  {line}")
                    warnings.append(line)
                continue

            parts = line.split(",")
            if len(parts) < 3:
                malformed += 1
                continue
            tag = parts[1]
            w = writers.get(tag)
            if w is None:
                malformed += 1
                continue

            w.writerow(parts)
            counts[tag] += 1

            try:
                s = int(parts[2])
                if last_seq[tag] >= 0 and s != last_seq[tag] + 1:
                    gaps[tag] += 1
                last_seq[tag] = s
            except ValueError:
                pass

            if time.time() >= next_report:
                next_report += 1.0
                rates = "  ".join(f"{t.replace('BNO_',''):<8}{c:>6}"
                                  for t, c in counts.items())
                print(f"  {rates}", end="\r", flush=True)

    except KeyboardInterrupt:
        print("\n  stopping...")
        send(ser, "X")
        deadline = time.time() + 5.0
        while time.time() < deadline:
            line = read_line(ser)
            if line is None:
                continue
            if line.startswith("# END"):
                end_info = parse_end(line)
                break
            if not line.startswith("#"):
                parts = line.split(",")
                if len(parts) >= 3 and parts[1] in writers:
                    writers[parts[1]].writerow(parts)
                    counts[parts[1]] += 1
    finally:
        for fh in files.values():
            fh.close()

    return counts, gaps, warnings, malformed, end_info


def verify(counts, gaps, malformed, end_info):
    """Print a summary and return True if the recording looks clean."""
    print()
    ok = True

    for tag, c in counts.items():
        note = ""
        if gaps[tag]:
            note = f"  <-- {gaps[tag]} sequence gap(s)"
            ok = False
        print(f"  {tag:<14}{c:>7} rows{note}")

    if malformed:
        print(f"  malformed lines: {malformed}")
        ok = False

    if end_info is None:
        print("  WARNING: no '# END' received — the recording was cut short")
        return False

    # Cross-check our row counts against the sketch's own tally.
    expected = {"BNO_ACC": "acc", "BNO_GYRO_UC": "gyro",
                "BNO_MAG_UC": "mag", "BNO_GRV": "ref", "BNO_RV": "ref"}
    for tag, key in expected.items():
        if tag in counts and key in end_info:
            if counts[tag] != end_info[key]:
                print(f"  WARNING: {tag} — sketch sent {end_info[key]}, "
                      f"we captured {counts[tag]}")
                ok = False

    if end_info.get("ok") == 0:
        print("  WARNING: sketch reported ok=0 (the BNO085 reset mid-recording)")
        ok = False

    dur = end_info.get("duration_s")
    if dur:
        print(f"  duration: {dur:.3f} s")
        for tag, c in counts.items():
            print(f"    {tag:<14}{c/dur:>7.1f} Hz")

    return ok


def main():
    args = parse_args()

    started = datetime.now(timezone.utc)
    folder = timestamped(args.name, started.astimezone())

    outdir = Path(args.outdir) / folder
    if outdir.exists():
        sys.exit(f"{outdir} already exists")
    outdir.mkdir(parents=True, exist_ok=True)

    ser = open_port(args.port, args.baud)

    cmd = f"S {args.ref} {args.seconds}"
    print(f"Recording '{args.name}' -> {outdir}")
    print(f"  command: {cmd}")
    if args.seconds == 0:
        print("  (press Ctrl-C to stop)")

    send(ser, cmd)
    columns = wait_for_begin(ser)
    print(f"  streams: {', '.join(sorted(columns))}")

    counts, gaps, warnings, malformed, end_info = record(ser, columns, outdir)
    ser.close()

    clean = verify(counts, gaps, malformed, end_info)

    meta = {
        "name": args.name,
        "folder": folder,
        "reference": {"G": "BNO_GRV", "R": "BNO_RV", "N": "NONE"}[args.ref],
        "requested_seconds": args.seconds,
        "started_utc": started.isoformat(),
        "port": args.port,
        "columns": columns,
        "counts": counts,
        "sequence_gaps": gaps,
        "malformed_lines": malformed,
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