#!/usr/bin/env python3
"""
IMU Performance & Accuracy Test Suite
=====================================
Captures serial data from firmware test modes and generates reports.

Usage:
  # Performance profiling (MODE=1)
  python tools/perf_test.py COM11 --mode perf --duration 30

  # Static drift test (MODE=2) — 30-60 minute run
  python tools/perf_test.py COM11 --mode drift --duration 3600

  # DSP comparison (MODE=3)
  python tools/perf_test.py COM11 --mode dsp --duration 60

  # Saturation test (MODE=4) — firmware automatically cycles through ODR levels
  python tools/perf_test.py COM11 --mode sat --duration 60

  # Interactive capture — save to CSV for manual analysis
  python tools/perf_test.py COM11 --mode capture --output data.csv

Firmware test mode mapping:
  TEST_MODE 0 = NORMAL (interactive data output)
  TEST_MODE 1 = PERF (cycle profiling)
  TEST_MODE 2 = DRIFT (static drift log)
  TEST_MODE 3 = DSP_CMP (float vs DSP comparison)
  TEST_MODE 4 = SAT (saturation, variable ODR)
"""

import serial
import sys
import time
import argparse
import csv
import os
from datetime import datetime
from collections import namedtuple

# ─── Statistics helpers ───

def percentile(sorted_vals, p):
    """p in [0,100]"""
    if not sorted_vals:
        return 0
    k = (len(sorted_vals) - 1) * p / 100.0
    f = int(k)
    c = k - f
    if f + 1 < len(sorted_vals):
        return sorted_vals[f] * (1 - c) + sorted_vals[f + 1] * c
    return sorted_vals[f]

def stats_summary(vals, name=""):
    """Returns dict with min/max/avg/p50/p99/stdev"""
    if not vals:
        return {}
    s = sorted(vals)
    n = len(s)
    avg = sum(s) / n
    var = sum((x - avg) ** 2 for x in s) / n
    return {
        "name": name,
        "n": n,
        "min": min(s),
        "max": max(s),
        "avg": avg,
        "p50": percentile(s, 50),
        "p95": percentile(s, 95),
        "p99": percentile(s, 99),
        "stdev": var ** 0.5,
    }


# ─── Mode: PERF (cycle profiling) ───

PerfRow = namedtuple("PerfRow",
    "mahony_min mahony_avg mahony_max mahony_us_min mahony_us_avg mahony_us_max "
    "frame_min frame_avg frame_max frame_us_min frame_us_avg frame_us_max")

def parse_perf(lines):
    """Parse MODE=1 or MODE=3 output: CSV lines with cycle stats."""
    rows = []
    for line in lines:
        line = line.strip()
        if line.startswith("#") or not line or "mahony_cycles" in line:
            continue
        parts = line.split(",")
        if len(parts) >= 12:
            try:
                rows.append(PerfRow(
                    int(parts[0]), float(parts[1]), int(parts[2]),
                    float(parts[3]), float(parts[4]), float(parts[5]),
                    int(parts[6]), float(parts[7]), int(parts[8]),
                    float(parts[9]), float(parts[10]), float(parts[11]),
                ))
            except (ValueError, IndexError):
                continue
    return rows

def report_perf(rows, title="Performance Report"):
    """Generate perf report from parsed cycle data."""
    if not rows:
        print("No data rows found.")
        return

    mahony_min  = [r.mahony_min for r in rows]
    mahony_avg  = [r.mahony_avg for r in rows]
    mahony_max  = [r.mahony_max for r in rows]
    frame_min   = [r.frame_min for r in rows]
    frame_avg   = [r.frame_avg for r in rows]
    frame_max   = [r.frame_max for r in rows]

    # Overall statistics: min of mins, avg of avgs, max of maxes
    m_min_v  = min(mahony_min)
    m_avg_v  = sum(mahony_avg) / len(mahony_avg)
    m_max_v  = max(mahony_max)
    m_us_min = m_min_v / 480.0
    m_us_avg = m_avg_v / 480.0
    m_us_max = m_max_v / 480.0

    f_min_v  = min(frame_min)
    f_avg_v  = sum(frame_avg) / len(frame_avg)
    f_max_v  = max(frame_max)
    f_us_min = f_min_v / 480.0
    f_us_avg = f_avg_v / 480.0
    f_us_max = f_max_v / 480.0

    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"  CPU: 480MHz  |  Sensor: MPU6050  |  1 cycle = 2.083 ns")
    print(f"{'='*60}")

    print(f"\n  ── Mahony (pure algorithm, no I2C) ──")
    print(f"  Min:     {m_min_v:>8d} cycles  ({m_us_min:>8.2f} μs)")
    print(f"  Avg:     {m_avg_v:>8.0f} cycles  ({m_us_avg:>8.2f} μs)")
    print(f"  Max:     {m_max_v:>8d} cycles  ({m_us_max:>8.2f} μs)")
    print(f"  Samples: {len(rows)} windows of 200 frames")

    print(f"\n  ── Full Frame (ring pop + prep + Mahony) ──")
    print(f"  Min:     {f_min_v:>8d} cycles  ({f_us_min:>8.2f} μs)")
    print(f"  Avg:     {f_avg_v:>8.0f} cycles  ({f_us_avg:>8.2f} μs)")
    print(f"  Max:     {f_max_v:>8d} cycles  ({f_us_max:>8.2f} μs)")

    cpu_pct = f_us_avg / 1000.0 * 100
    print(f"\n  ── CPU Load at 1kHz ODR ──")
    print(f"  Frame budget:           1000 μs")
    print(f"  Avg frame:              {f_us_avg:.2f} μs")
    print(f"  Worst frame:            {f_us_max:.2f} μs")
    print(f"  Safety margin:          {1000-f_us_max:.2f} μs")
    print(f"  CPU utilization:        {cpu_pct:.1f}%")
    print(f"  Max sustainable ODR:    {1e6/f_us_max:.0f} Hz")

    print(f"\n  ── Answer: will interrupt be blocked? ──")
    if f_us_max < 900:
        print(f"  NO — worst frame ({f_us_max:.1f} μs) << 1ms frame budget. Safe.")
    elif f_us_max < 1000:
        print(f"  TIGHT — worst frame ({f_us_max:.1f} μs) near 1ms budget.")
    else:
        print(f"  YES — worst frame ({f_us_max:.1f} μs) exceeds 1ms. Risk of overrun.")

    return {
        "mahony_min": m_min_v, "mahony_avg": m_avg_v, "mahony_max": m_max_v,
        "frame_min": f_min_v, "frame_avg": f_avg_v, "frame_max": f_max_v,
        "cpu_pct": cpu_pct,
    }


# ─── Mode: DRIFT (static drift) ───

DriftRow = namedtuple("DriftRow",
    "t_s roll pitch yaw ax ay az gx gy gz")

def parse_drift(lines):
    rows = []
    for line in lines:
        line = line.strip()
        if line.startswith("#") or not line or "t_s" in line:
            continue
        parts = line.split(",")
        if len(parts) >= 10:
            try:
                rows.append(DriftRow(
                    float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]),
                    float(parts[4]), float(parts[5]), float(parts[6]),
                    float(parts[7]), float(parts[8]), float(parts[9]),
                ))
            except (ValueError, IndexError):
                continue
    return rows

def report_drift(rows, title="Static Drift Test"):
    if not rows:
        print("No data rows found.")
        return

    rolls  = [r.roll for r in rows]
    pitchs = [r.pitch for r in rows]
    yaws   = [r.yaw for r in rows]
    t_s    = [r.t_s for r in rows]
    duration_min = (t_s[-1] - t_s[0]) / 60.0 if len(t_s) > 1 else 0

    # Remove first 10s (settling)
    settle_idx = 0
    for i, t in enumerate(t_s):
        if t > 10.0:
            settle_idx = i
            break

    r_trim = rolls[settle_idx:]
    p_trim = pitchs[settle_idx:]
    y_trim = yaws[settle_idx:]

    r_drift_total = r_trim[-1] - r_trim[0] if len(r_trim) > 1 else 0
    p_drift_total = p_trim[-1] - p_trim[0] if len(p_trim) > 1 else 0
    y_drift_total = y_trim[-1] - y_trim[0] if len(y_trim) > 1 else 0

    r_pp = max(r_trim) - min(r_trim)
    p_pp = max(p_trim) - min(p_trim)
    y_pp = max(y_trim) - min(y_trim)

    r_drift_per_hr = r_drift_total / duration_min * 60 if duration_min > 0 else 0
    p_drift_per_hr = p_drift_total / duration_min * 60 if duration_min > 0 else 0
    y_drift_per_hr = y_drift_total / duration_min * 60 if duration_min > 0 else 0

    r_avg = sum(r_trim) / len(r_trim)
    p_avg = sum(p_trim) / len(p_trim)
    r_stdev = (sum((x - r_avg)**2 for x in r_trim) / len(r_trim))**0.5
    p_stdev = (sum((x - p_avg)**2 for x in p_trim) / len(p_trim))**0.5

    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"  Duration: {duration_min:.1f} min  |  Samples: {len(rows)}")
    print(f"{'='*60}")

    print(f"\n  ── Static Drift (after 10s settling) ──")
    print(f"  Roll:  avg={r_avg:+.3f}°  stdev={r_stdev:.4f}°  pp={r_pp:.3f}°  drift={r_drift_total:+.3f}°  ({r_drift_per_hr:+.2f}°/hr)")
    print(f"  Pitch: avg={p_avg:+.3f}°  stdev={p_stdev:.4f}°  pp={p_pp:.3f}°  drift={p_drift_total:+.3f}°  ({p_drift_per_hr:+.2f}°/hr)")
    print(f"  Yaw:   pp={y_pp:.3f}°  drift={y_drift_total:+.3f}°  ({y_drift_per_hr:+.2f}°/hr)")
    print(f"\n  Note: 6-axis IMU without magnetometer — Yaw drift is inevitable.")
    print(f"           Yaw drift rate reflects gyro bias instability, not algorithm defect.")

    return {
        "duration_min": duration_min,
        "roll": {"avg": r_avg, "stdev": r_stdev, "pp": r_pp, "drift_total": r_drift_total, "drift_per_hr": r_drift_per_hr},
        "pitch": {"avg": p_avg, "stdev": p_stdev, "pp": p_pp, "drift_total": p_drift_total, "drift_per_hr": p_drift_per_hr},
        "yaw": {"pp": y_pp, "drift_total": y_drift_total, "drift_per_hr": y_drift_per_hr},
    }


# ─── Mode: SAT (saturation) ───

SatRow = namedtuple("SatRow", "decim rate_hz mahony_min mahony_avg mahony_max mahony_us_avg")

def parse_sat(lines):
    rows = []
    for line in lines:
        line = line.strip()
        if line.startswith("#") or not line or "decim" in line:
            continue
        parts = line.split(",")
        if len(parts) >= 5:
            try:
                rows.append(SatRow(
                    int(parts[0]), int(parts[1]),
                    int(parts[2]), float(parts[3]), int(parts[4]),
                    float(parts[5]),
                ))
            except (ValueError, IndexError):
                continue
    return rows

def report_sat(rows, title="Saturation Test"):
    if not rows:
        print("No data rows found.")
        return

    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")

    # Group by decimation factor, take last (most stable) measurement
    from collections import defaultdict
    by_decim = defaultdict(list)
    for r in rows:
        by_decim[r.decim].append(r)

    print(f"\n  {'Decim':>6}  {'Rate(Hz)':>8}  {'Min(cyc)':>10}  {'Avg(cyc)':>10}  {'Max(cyc)':>10}  {'Avg(μs)':>8}  {'CPU%':>6}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*6}")

    results = []
    for decim in sorted(by_decim.keys()):
        group = by_decim[decim]
        # Take median of each metric across sweeps
        avg_cycle = sorted(g.mahony_avg for g in group)[len(group)//2]
        min_cycle = sorted(g.mahony_min for g in group)[len(group)//2]
        max_cycle = sorted(g.mahony_max for g in group)[len(group)//2]
        avg_us    = sorted(g.mahony_us_avg for g in group)[len(group)//2]
        rate      = group[0].rate_hz
        cpu_pct   = avg_us / (1e6 / rate) * 100

        print(f"  {decim:>6d}  {rate:>8d}  {min_cycle:>10d}  {avg_cycle:>10.0f}  {max_cycle:>10d}  {avg_us:>8.2f}  {cpu_pct:>5.1f}%")
        results.append({
            "decim": decim, "rate_hz": rate, "min_cyc": min_cycle,
            "avg_cyc": avg_cycle, "max_cyc": max_cycle,
            "avg_us": avg_us, "cpu_pct": cpu_pct,
        })

    # Find saturation point: where CPU% increase >> accuracy gain
    print(f"\n  ── Saturation Analysis ──")
    # Simple heuristic: find rate where doubling halves the marginal benefit
    best_rate = results[0]["rate_hz"]
    for i in range(1, len(results)):
        prev = results[i-1]
        curr = results[i]
        # CPU% increase per 100Hz
        cpu_delta = curr["cpu_pct"] - prev["cpu_pct"]
        rate_delta = curr["rate_hz"] - prev["rate_hz"]
        if rate_delta > 0 and cpu_delta > prev["cpu_pct"] * 0.5:
            best_rate = prev["rate_hz"]
            print(f"  Saturation point: ~{best_rate} Hz "
                  f"(CPU: {prev['cpu_pct']:.1f}% → {curr['cpu_pct']:.1f}%)")
            print(f"  Above {best_rate} Hz: CPU grows disproportionately, marginal accuracy gain.")
            break

    return results


# ─── Serial capture ───

def capture(port, baud=115200, duration=30, output=None):
    """Capture serial data for specified duration, return lines."""
    ser = serial.Serial(port, baud, timeout=1.0)
    ser.reset_input_buffer()

    lines = []
    t0 = time.time()
    print(f"Capturing from {port} for {duration}s...", file=sys.stderr)

    try:
        while time.time() - t0 < duration:
            try:
                line = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
                if line:
                    lines.append(line)
                    if output is None:
                        print(line)
            except (UnicodeDecodeError, serial.SerialException):
                continue
    except KeyboardInterrupt:
        print("\nCapture interrupted.", file=sys.stderr)
    finally:
        ser.close()

    if output:
        with open(output, "w", encoding="utf-8") as f:
            for line in lines:
                f.write(line + "\n")
        print(f"Saved {len(lines)} lines to {output}", file=sys.stderr)

    return lines


# ─── Main ───

def main():
    parser = argparse.ArgumentParser(description="IMU Performance Test Suite")
    parser.add_argument("port", help="Serial port (e.g. COM11)")
    parser.add_argument("--mode", default="capture",
                        choices=["capture", "perf", "drift", "dsp", "sat"],
                        help="Test mode (must match firmware TEST_MODE)")
    parser.add_argument("--duration", type=int, default=30,
                        help="Capture duration in seconds")
    parser.add_argument("--output", help="Save raw data to CSV file")
    parser.add_argument("--input", help="Analyze existing CSV file (skip capture)")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    # Load data
    if args.input:
        with open(args.input, encoding="utf-8") as f:
            lines = [line.rstrip("\r\n") for line in f]
        print(f"Loaded {len(lines)} lines from {args.input}", file=sys.stderr)
    else:
        lines = capture(args.port, args.baud, args.duration, args.output)

    if not lines:
        print("No data captured.")
        return 1

    # Parse & report based on mode
    if args.mode == "perf" or args.mode == "dsp":
        rows = parse_perf(lines)
        title = "DSP Comparison Report" if args.mode == "dsp" else "Cycle Profiling Report"
        report_perf(rows, title)
    elif args.mode == "drift":
        rows = parse_drift(lines)
        report_drift(rows)
    elif args.mode == "sat":
        rows = parse_sat(lines)
        report_sat(rows)
    else:
        # capture mode: just save
        print(f"Captured {len(lines)} lines. Use --input and --mode to analyze.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
