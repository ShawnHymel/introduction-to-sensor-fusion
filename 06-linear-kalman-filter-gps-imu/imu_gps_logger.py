#!/usr/bin/env python3
"""
imu_gps_logger.py

Records an IMU + GPS sequence (e.g. a drive or walk around the neighborhood)
from the imu_gps_logger sketch (BNO085 acceleration + rotation vector,
PA1010D GPS) and writes:

    imu.csv    one row per accelerometer sample (~100 Hz), with the latest
               rotation vector quaternion
    gps.csv    one row per GPS epoch (1 Hz)
    nmea.log   raw NMEA sentences (only if the sketch has LOG_RAW_NMEA = 1)
    meta.json  settings, counts, and a summary of the recording

Before recording, the script waits until the GPS is ready (fix, enough
satellites, low HDOP) and has stayed ready for --fix-settle seconds.

The end-of-run summary prints accelerometer and GPS position noise. On a
recording made at rest, these give sigma_accel and sigma_gps for the filter.

Usage:
    python imu_gps_logger.py --port COM5 --name drive_loop -s 0 \
        --notes "board taped flat on passenger seat, +y forward"
    python imu_gps_logger.py -p /dev/ttyACM0 -n static -s 300
    python imu_gps_logger.py -p COM5 -n indoor_test -s 30 --no-wait
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


# Streams and output files
STREAMS = {"IMU": "imu.csv", "GPS": "gps.csv"}
NMEA_FILE = "nmea.log"

# Extra time allowed for the sketch to start recording (covers settling)
BEGIN_MARGIN_S = 5.0

# Warn if the sketch stops printing idle GPS status for this long
STATUS_TIMEOUT_S = 6.0

# Mean Earth radius for the flat-earth conversion (m)
EARTH_RADIUS_M = 6371000.0


def parse_args():
    ap = argparse.ArgumentParser(
        description="Record an IMU + GPS sequence.")
    ap.add_argument("-p", "--port", required=True,
                    help="serial port (e.g. COM5 or /dev/ttyACM0)")
    ap.add_argument("-n", "--name", required=True,
                    help="sequence name; used for the output folder")
    ap.add_argument("-s", "--seconds", type=int, default=0,
                    help="duration; 0 records until Ctrl-C (default: 0)")
    ap.add_argument("--notes", default="",
                    help="free-text notes saved in meta.json "
                         "(e.g. 'board on dash, +y forward')")
    ap.add_argument("-o", "--outdir", default="recordings",
                    help="base output directory (default: recordings)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="baud rate, ignored for native USB (default: 115200)")

    # GPS readiness
    ap.add_argument("--no-wait", action="store_true",
                    help="start recording without waiting for a GPS fix")
    ap.add_argument("--min-sats", type=int, default=6,
                    help="satellites needed to count as ready (default: 6)")
    ap.add_argument("--max-hdop", type=float, default=2.0,
                    help="max HDOP to count as ready (default: 2.0)")
    ap.add_argument("--fix-settle", type=float, default=30.0,
                    help="seconds the GPS must stay ready before recording "
                         "starts (default: 30)")
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
    for conv in (int, float):
        try:
            return conv(v)
        except ValueError:
            pass
    return v


def to_float(v):
    """Parse a float, returning NaN for empty or bad values."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def to_int(v, default=-1):
    try:
        return int(v)
    except (TypeError, ValueError):
        return default


def parse_kv(text):
    """Parse 'key=value key=value' into a dict."""
    out = {}
    for token in text.split():
        if "=" in token:
            k, v = token.split("=", 1)
            out[k] = parse_value(v)
    return out


def lla_to_en(lat, lon, lat0, lon0):
    """Flat-earth conversion of lat/lon (deg) to East/North (m) around an
    origin. Accurate to well under a metre over a few km."""
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    east = d_lon * math.cos(math.radians(lat0)) * EARTH_RADIUS_M
    north = d_lat * EARTH_RADIUS_M
    return east, north


def stop_if_recording(ser):
    """Stop any recording left running by a previous session."""
    send(ser, "X")
    deadline = time.time() + 1.0
    while time.time() < deadline:
        read_line(ser)
    ser.reset_input_buffer()


def gps_ready(st, args):
    """Readiness test on an idle '# GPS ...' status line."""
    fix = to_int(st.get("fix"), 0)
    sats = to_int(st.get("sats"), 0)
    hdop = to_float(st.get("hdop"))
    return (fix in (1, 2) and sats >= args.min_sats
            and not math.isnan(hdop) and hdop <= args.max_hdop)


def wait_for_fix(ser, args):
    """
    Watch the sketch's idle GPS status lines until the GPS has been ready
    for args.fix_settle seconds. Returns the last status dict.
    """
    print(f"Waiting for GPS: fix 1 or 2, sats >= {args.min_sats}, "
          f"hdop <= {args.max_hdop}, stable for {args.fix_settle:.0f} s")
    print("  (Ctrl-C to cancel; use --no-wait to skip)")

    ready_since = None
    last_status = time.time()
    st = {}
    while True:
        line = read_line(ser)
        now = time.time()

        if now - last_status > STATUS_TIMEOUT_S:
            print("\n  WARNING: no GPS status from the sketch "
                  "(is the GPS connected? is this the right sketch?)")
            last_status = now

        if line is None:
            continue
        if line.startswith("# ERROR"):
            print(f"\n  {line}")
            continue
        if not line.startswith("# GPS "):
            continue

        last_status = now
        st = parse_kv(line[len("# GPS "):])

        if gps_ready(st, args):
            if ready_since is None:
                ready_since = now
            held = now - ready_since
            msg = f"ready, holding {held:4.0f}/{args.fix_settle:.0f} s"
        else:
            ready_since = None
            msg = "not ready"

        print(f"  fix={st.get('fix', '?')} sats={st.get('sats', '?'):>2} "
              f"hdop={st.get('hdop', '?')} pps={st.get('pps', '?')}  "
              f"{msg:<28}", end="\r", flush=True)

        if ready_since is not None and now - ready_since >= args.fix_settle:
            print("\n  GPS ready")
            return st


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
            cfg.update(parse_kv(line[len("# CFG "):]))
            continue

        # Column specs: "# IMU: t_us,tag,..." and "# GPS: t_us,tag,..."
        if line.startswith("#") and ":" in line:
            spec, cols = line.lstrip("# ").split(":", 1)
            columns[spec.strip()] = [c.strip() for c in cols.split(",")]
    sys.exit("Timed out waiting for '# DATA'.")


def timestamped(name, when):
    return f"{name}-{when.strftime('%Y%m%d-%H%M%S')}"


def new_stats():
    return {
        # Per stream
        "count": {tag: 0 for tag in STREAMS},
        "last_seq": {tag: -1 for tag in STREAMS},
        "gaps": {tag: 0 for tag in STREAMS},
        "malformed": 0,

        # IMU
        "ax": [], "ay": [], "az": [],
        "acc_status": {}, "rv_status": {},
        "rv_acc": [], "max_rv_age_us": 0, "no_rv": 0,

        # GPS
        "fix_rows": 0, "fix_counts": {}, "sats": [], "hdop": [],
        "lat": [], "lon": [], "speed": [], "pps_rows": 0,
        "last_gps": None, "origin": None,
    }


def handle_row(line, writers, columns, stats):
    """Write one data row to its stream's CSV and update statistics."""
    parts = line.split(",")
    tag = parts[1] if len(parts) > 1 else ""
    if tag not in writers or len(parts) != len(columns[tag]):
        stats["malformed"] += 1
        return

    writers[tag].writerow(parts)
    stats["count"][tag] += 1

    # Track sequence gaps per stream
    s = to_int(parts[2])
    if stats["last_seq"][tag] >= 0 and s != stats["last_seq"][tag] + 1:
        stats["gaps"][tag] += 1
    stats["last_seq"][tag] = s

    row = dict(zip(columns[tag], parts))
    if tag == "IMU":
        handle_imu(row, stats)
    else:
        handle_gps(row, stats)


def handle_imu(row, stats):
    for axis in ("ax", "ay", "az"):
        stats[axis].append(to_float(row[axis]))

    st = row["acc_status"]
    stats["acc_status"][st] = stats["acc_status"].get(st, 0) + 1

    if row["qr"] == "":
        stats["no_rv"] += 1
        return
    rst = row["rv_status"]
    stats["rv_status"][rst] = stats["rv_status"].get(rst, 0) + 1
    stats["rv_acc"].append(to_float(row["rv_acc_rad"]))
    stats["max_rv_age_us"] = max(stats["max_rv_age_us"],
                                 to_int(row["rv_age_us"], 0))


def handle_gps(row, stats):
    stats["last_gps"] = row
    fix = row["fix"] or "-"
    stats["fix_counts"][fix] = stats["fix_counts"].get(fix, 0) + 1
    if row["pps_t_us"] != "":
        stats["pps_rows"] += 1

    lat, lon = to_float(row["lat_deg"]), to_float(row["lon_deg"])
    if math.isnan(lat) or math.isnan(lon):
        return

    stats["fix_rows"] += 1
    stats["lat"].append(lat)
    stats["lon"].append(lon)
    stats["sats"].append(to_int(row["sats"], 0))
    stats["hdop"].append(to_float(row["hdop"]))
    stats["speed"].append(to_float(row["speed_mps"]))
    if stats["origin"] is None:
        stats["origin"] = (lat, lon)


def status_line(stats):
    """One-line live status, printed once per second."""
    n_imu, n_gps = stats["count"]["IMU"], stats["count"]["GPS"]
    g = stats["last_gps"]
    if g is None:
        return f"  imu={n_imu:>7}  gps={n_gps:>5}  (no GPS rows yet)"

    t_s = to_int(g["t_us"], 0) * 1e-6
    fix = g["fix"] or "-"
    sats = g["sats"] or "-"
    hdop = g["hdop"] or "-"

    # Distance from the first fix of the recording
    where = "  no fix  "
    lat, lon = to_float(g["lat_deg"]), to_float(g["lon_deg"])
    if stats["origin"] and not math.isnan(lat):
        e, n = lla_to_en(lat, lon, *stats["origin"])
        where = f"E={e:+8.1f} N={n:+8.1f} m"
    spd = to_float(g["speed_mps"])
    spd = f"{spd:5.1f} m/s" if not math.isnan(spd) else "   -- m/s"

    return (f"  t={t_s:7.1f} s  imu={n_imu:>7}  gps={n_gps:>5}  "
            f"fix={fix} sats={sats:>2} hdop={hdop:>4}  {where}  {spd}")


def stop_and_drain(ser, writers, columns, stats, nmea):
    """Send X and capture any remaining rows until '# END'."""
    send(ser, "X")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith("# END"):
            return parse_kv(line[len("# END"):])
        if line.startswith("# NMEA "):
            nmea.write(line[len("# NMEA "):] + "\n")
        elif not line.startswith("#"):
            handle_row(line, writers, columns, stats)
    return None


def record(ser, columns, outdir):
    """Stream rows to the CSVs until '# END'.
    Returns (stats, warnings, end_info)."""
    stats = new_stats()
    warnings = []
    end_info = None
    next_report = time.time() + 1.0

    files = {tag: open(outdir / fname, "w", newline="", encoding="utf-8")
             for tag, fname in STREAMS.items()}
    writers = {tag: csv.writer(fh) for tag, fh in files.items()}
    for tag, w in writers.items():
        w.writerow(columns[tag])

    nmea_path = outdir / NMEA_FILE
    nmea = open(nmea_path, "w", encoding="utf-8")

    try:
        while True:
            line = read_line(ser)
            if line is None:
                continue

            if line.startswith("#"):
                if line.startswith("# END"):
                    end_info = parse_kv(line[len("# END"):])
                    break
                if line.startswith("# NMEA "):
                    nmea.write(line[len("# NMEA "):] + "\n")
                elif line.startswith("# WARN") or line.startswith("# ERROR"):
                    print(f"\n  {line}")
                    warnings.append(line)
                continue

            handle_row(line, writers, columns, stats)

            if time.time() >= next_report:
                next_report += 1.0
                print(status_line(stats), end="\r", flush=True)

    except KeyboardInterrupt:
        print("\n  stopping...")
        end_info = stop_and_drain(ser, writers, columns, stats, nmea)

    finally:
        for fh in files.values():
            fh.close()
        nmea.close()

    # Only keep nmea.log if the sketch sent raw sentences
    if nmea_path.stat().st_size == 0:
        nmea_path.unlink()

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


def median(xs):
    xs = sorted(x for x in xs if not math.isnan(x))
    if not xs:
        return math.nan
    mid = len(xs) // 2
    return xs[mid] if len(xs) % 2 else 0.5 * (xs[mid - 1] + xs[mid])


def summarize(stats):
    """
    Noise and range for acceleration and GPS position. On a recording made
    at rest, accel std ~ sigma_accel (per sensor axis) and E/N std
    ~ sigma_gps. Returns a dict for meta.json.
    """
    out = {"accel": {}, "gps": {}}

    print(f"\n  {'signal':<10}{'mean':>12}{'std':>10}")
    for axis in ("ax", "ay", "az"):
        m, s = mean_std(stats[axis])
        out["accel"][axis] = {"mean": m, "std": s}
        print(f"  {axis:<10}{m:>12.4f}{s:>10.4f}   (m/s^2, sensor frame)")

    rv_m, _ = mean_std(stats["rv_acc"])
    out["rv_heading_acc_rad_mean"] = rv_m
    if not math.isnan(rv_m):
        print(f"  heading accuracy (BNO085 estimate): "
              f"{math.degrees(rv_m):.1f} deg mean")

    if stats["fix_rows"]:
        lat0, lon0 = stats["origin"]
        en = [lla_to_en(la, lo, lat0, lon0)
              for la, lo in zip(stats["lat"], stats["lon"])]
        e_m, e_s = mean_std([p[0] for p in en])
        n_m, n_s = mean_std([p[1] for p in en])
        e_rng = max(p[0] for p in en) - min(p[0] for p in en)
        n_rng = max(p[1] for p in en) - min(p[1] for p in en)
        print(f"  {'east':<10}{e_m:>12.2f}{e_s:>10.2f}   "
              f"(m from first fix, range {e_rng:.1f} m)")
        print(f"  {'north':<10}{n_m:>12.2f}{n_s:>10.2f}   "
              f"(m from first fix, range {n_rng:.1f} m)")
        print(f"  sats median {median(stats['sats']):.0f}, "
              f"hdop median {median(stats['hdop']):.2f}")
        print("  (at rest: accel std ~ sigma_accel, east/north std "
              "~ sigma_gps)")
        out["gps"] = {
            "origin_lat_deg": lat0, "origin_lon_deg": lon0,
            "east": {"mean": e_m, "std": e_s, "range": e_rng},
            "north": {"mean": n_m, "std": n_s, "range": n_rng},
            "sats_median": median(stats["sats"]),
            "hdop_median": median(stats["hdop"]),
        }

    out["acc_status_counts"] = stats["acc_status"]
    out["rv_status_counts"] = stats["rv_status"]
    out["fix_counts"] = stats["fix_counts"]
    out["max_rv_age_us"] = stats["max_rv_age_us"]
    return out


def verify(stats, end_info, cfg):
    """Print a summary and return True if the recording looks clean."""
    print()
    ok = True

    for tag in STREAMS:
        note = ""
        if stats["gaps"][tag]:
            note = f"  <-- {stats['gaps'][tag]} sequence gap(s)"
            ok = False
        print(f"  {tag:<14}{stats['count'][tag]:>7} rows{note}")
    print(f"  {'  with fix':<14}{stats['fix_rows']:>7} rows")

    if stats["malformed"]:
        print(f"  malformed lines: {stats['malformed']}")
        ok = False

    if end_info is None:
        print("  WARNING: no '# END' received — the recording was cut short")
        return False

    # Cross-check our counts against the sketch's own tally
    for tag, key in (("IMU", "imu"), ("GPS", "gps")):
        if key in end_info and stats["count"][tag] != end_info[key]:
            print(f"  WARNING: sketch sent {end_info[key]} {tag} rows, "
                  f"we captured {stats['count'][tag]}")
            ok = False

    if end_info.get("ok", 1) == 0:
        print("  WARNING: BNO085 reset during recording (see warnings)")
        ok = False
    if stats["count"]["IMU"] == 0:
        print("  WARNING: no IMU data at all")
        ok = False
    if stats["fix_rows"] == 0:
        print("  WARNING: no GPS fixes (fine for an indoor test)")
        ok = False
    if end_info.get("gps_cksum_err", 0) or end_info.get("gps_overflow", 0):
        print(f"  WARNING: {end_info.get('gps_cksum_err', 0)} NMEA checksum "
              f"error(s), {end_info.get('gps_overflow', 0)} overflow(s)")
        ok = False

    # Details that don't fail the recording
    no_fix = stats["count"]["GPS"] - stats["fix_rows"]
    if stats["fix_rows"] and no_fix:
        print(f"  note: {no_fix} GPS row(s) without a fix")
    if stats["no_rv"]:
        print(f"  note: {stats['no_rv']} IMU row(s) before the first "
              f"rotation vector")
    pps_pin = to_int(cfg.get("gps_pps_pin"), -1)
    if pps_pin >= 0 and stats["fix_rows"] and stats["pps_rows"] == 0:
        print("  note: PPS pin configured but no PPS edges seen "
              "(check the wiring)")
    print(f"  max rv_age: {stats['max_rv_age_us'] / 1000:.1f} ms")
    print(f"  acc_status counts: {stats['acc_status']}")
    print(f"  rv_status counts:  {stats['rv_status']}  "
          f"(3 = magnetometer well calibrated)")
    print(f"  fix counts:        {stats['fix_counts']}")

    dur = end_info.get("duration_s")
    if dur:
        print(f"  duration: {dur:.3f} s")
        print(f"    {'IMU':<14}{stats['count']['IMU']/dur:>7.1f} Hz")
        print(f"    {'GPS':<14}{stats['count']['GPS']/dur:>7.2f} Hz")
        if "pps" in end_info:
            print(f"    {'PPS edges':<14}{end_info['pps']:>7}")

    return ok


def clean_nan(d):
    """NaN isn't valid JSON, so store it as null."""
    if isinstance(d, dict):
        return {k: clean_nan(v) for k, v in d.items()}
    if isinstance(d, float) and math.isnan(d):
        return None
    return d


def main():
    args = parse_args()
    ser = open_port(args.port, args.baud)
    stop_if_recording(ser)

    # Wait for a good fix before recording (Ctrl-C cancels cleanly)
    gps_status = None
    if not args.no_wait:
        try:
            gps_status = wait_for_fix(ser, args)
        except KeyboardInterrupt:
            ser.close()
            sys.exit("\nCancelled while waiting for GPS.")

    started = datetime.now(timezone.utc)
    folder = timestamped(args.name, started.astimezone())
    outdir = Path(args.outdir) / folder
    if outdir.exists():
        sys.exit(f"{outdir} already exists")

    cmd = f"S {args.seconds}"
    print(f"Recording '{args.name}' -> {outdir}")
    print(f"  command: {cmd}")
    if args.seconds == 0:
        print("  (press Ctrl-C to stop)")

    send(ser, cmd)
    try:
        columns, cfg, warnings = wait_for_begin(ser, BEGIN_MARGIN_S)
    except KeyboardInterrupt:
        send(ser, "X")
        ser.close()
        sys.exit("\nCancelled before recording started.")

    missing = [tag for tag in STREAMS if tag not in columns]
    if missing:
        ser.close()
        sys.exit(f"Sketch did not describe stream(s) {missing}: {columns}")
    print(f"  recording started (gps_link={cfg.get('gps_link', '?')}, "
          f"gps {cfg.get('gps_rate_ms', '?')} ms, "
          f"pps pin {cfg.get('gps_pps_pin', '?')})")

    # Only create the folder once recording has actually started
    outdir.mkdir(parents=True, exist_ok=True)

    stats, rec_warnings, end_info = record(ser, columns, outdir)
    warnings += rec_warnings
    ser.close()

    clean = verify(stats, end_info, cfg)
    summary = summarize(stats) if stats["count"]["IMU"] else {}

    meta = {
        "name": args.name,
        "folder": folder,
        "notes": args.notes,
        "requested_seconds": args.seconds,
        "started_utc": started.isoformat(),
        "port": args.port,
        "columns": columns,
        "config": cfg,
        "gps_wait": None if args.no_wait else {
            "min_sats": args.min_sats,
            "max_hdop": args.max_hdop,
            "fix_settle_s": args.fix_settle,
            "status_at_start": gps_status,
        },
        "rows": stats["count"],
        "gps_fix_rows": stats["fix_rows"],
        "sequence_gaps": stats["gaps"],
        "malformed_lines": stats["malformed"],
        "summary": summary,
        "sketch_end": end_info,
        "warnings": warnings,
        "clean": clean,
    }
    with open(outdir / "meta.json", "w", encoding="utf-8") as f:
        json.dump(clean_nan(meta), f, indent=2)

    print(f"\n{'OK' if clean else 'CHECK WARNINGS'} — wrote {outdir}")
    sys.exit(0 if clean else 1)


if __name__ == "__main__":
    main()
