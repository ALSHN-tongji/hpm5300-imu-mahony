#!/usr/bin/env python3
"""
IMU Robustness Boundary Test Suite
===================================
Three stress scenarios to find performance limits:

  1. Shake test  — linear acceleration jitter, compare fixed vs adaptive Kp
  2. High-rate   — max angular velocity before attitude divergence
  3. Long-drift  — 3-5h cumulative drift, verify Mahony I-term stability

Usage:
    # Shake test (record MODE 0 while shaking IMU)
    python stress_analysis.py shake fixed_kp.csv adaptive_kp.csv

    # High-rate test (record MODE 0 while spinning IMU fast)
    python stress_analysis.py highrate spin_test.csv

    # Long-drift test (record MODE 0 for 3-5h, stationary)
    python stress_analysis.py drift long_run.csv
"""

import sys
import os
import csv
import argparse
import numpy as np

if sys.stdout.encoding and sys.stdout.encoding.lower() in ("gbk", "gb2312", "cp936"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def load_mode0_csv(path):
    """Load MODE 0 CSV: ax,ay,az,gx,gy,gz,roll,pitch,yaw,q0,q1,q2,q3"""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 12:
                continue
            try:
                rows.append([float(p) for p in parts[:12]])
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"No valid MODE 0 rows in {path}")
    data = np.array(rows, dtype=np.float64)
    # cols: 0-ax,1-ay,2-az,3-gx,4-gy,5-gz,6-roll,7-pitch,8-yaw,9-q0,10-q1,11-q2
    return data


# ═══════════════════════════════════════════════════════════════
# 1. Shake test — compare fixed vs adaptive Kp
# ═══════════════════════════════════════════════════════════════

def shake_test(fixed_path, adaptive_path, out_dir):
    """Compare attitude jitter under linear acceleration.

    Metrics:
      - Roll/Pitch RMS during shaking
      - Peak-to-peak jitter
      - Accel deviation from 1g (proxy for shake intensity)
    """
    print(f"\n{'='*60}")
    print("  SHAKE TEST — Fixed Kp vs Adaptive Kp")
    print(f"{'='*60}")

    def analyze(path, label):
        print(f"  Loading {path} ({label}) ...")
        data = load_mode0_csv(path)
        roll = data[:, 6]
        pitch = data[:, 7]
        gx, gy, gz = data[:, 3], data[:, 4], data[:, 5]
        ax, ay, az = data[:, 0], data[:, 1], data[:, 2]

        # Accel deviation from 1g
        a_norm = np.sqrt(ax**2 + ay**2 + az**2)
        accel_dev = np.std(a_norm - 1.0)
        gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

        # Remove initial convergence (first 2s)
        skip = 2000
        if len(roll) > skip:
            roll_s = roll[skip:]
            pitch_s = pitch[skip:]
        else:
            roll_s = roll
            pitch_s = pitch

        metrics = {
            "label": label,
            "samples": len(data),
            "roll_rms_deg": float(np.std(roll_s)),
            "pitch_rms_deg": float(np.std(pitch_s)),
            "roll_ptp_deg": float(np.ptp(roll_s)),
            "pitch_ptp_deg": float(np.ptp(pitch_s)),
            "accel_dev_g": float(accel_dev),
            "gyro_max_dps": float(np.max(np.abs(gyro_mag))),
            "gyro_mean_dps": float(np.mean(gyro_mag)),
        }
        return metrics

    m_fixed = analyze(fixed_path, "Fixed Kp")
    m_adapt = analyze(adaptive_path, "Adaptive Kp")

    # Comparison
    print(f"\n  {'Metric':<25s} {'Fixed Kp':>10s} {'Adaptive Kp':>10s} {'Change':>10s}")
    print(f"  {'-'*25} {'-'*10} {'-'*10} {'-'*10}")
    for key, fmt, unit in [
        ("roll_rms_deg", ".3f", "deg"),
        ("pitch_rms_deg", ".3f", "deg"),
        ("roll_ptp_deg", ".2f", "deg"),
        ("pitch_ptp_deg", ".2f", "deg"),
        ("accel_dev_g", ".3f", "g"),
        ("gyro_max_dps", ".1f", "dps"),
    ]:
        fix = m_fixed[key]
        adp = m_adapt[key]
        if fix > 0.001:
            delta = (adp - fix) / fix * 100
        else:
            delta = 0
        tmpl = "  {:<25s} {:>10" + fmt + "} {:>10" + fmt + "} {:+9.1f}%"
        print(tmpl.format(key, fix, adp, delta))

    # Verdict
    rms_improve = (1.0 - m_adapt["roll_rms_deg"] / max(m_fixed["roll_rms_deg"], 1e-6)) * 100
    ptp_improve = (1.0 - m_adapt["roll_ptp_deg"] / max(m_fixed["roll_ptp_deg"], 1e-6)) * 100
    print(f"\n  Adaptive Kp reduces roll RMS by {rms_improve:.1f}%, P-P by {ptp_improve:.1f}%")

    # Save CSV
    with open(os.path.join(out_dir, "shake_comparison.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(m_fixed.keys()))
        w.writeheader()
        w.writerow(m_fixed)
        w.writerow(m_adapt)

    return m_fixed, m_adapt


# ═══════════════════════════════════════════════════════════════
# 2. High-rate test — find divergence boundary
# ═══════════════════════════════════════════════════════════════

def highrate_test(path, out_dir):
    """Analyze high angular velocity tolerance.

    Detects: attitude error growth vs gyro magnitude.
    Reports: max sustainable rate before >5 deg attitude error.
    """
    print(f"\n{'='*60}")
    print("  HIGH-RATE TEST — Angular Velocity Boundary")
    print(f"{'='*60}")
    print(f"  Loading {path} ...")

    data = load_mode0_csv(path)
    roll = data[:, 6]
    pitch = data[:, 7]
    gx, gy, gz = data[:, 3], data[:, 4], data[:, 5]
    gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

    # Skip convergence
    skip = 2000
    gyro_mag = gyro_mag[skip:]
    roll = roll[skip:]
    pitch = pitch[skip:]

    # Bin by gyro magnitude
    bins = [0, 50, 100, 200, 300, 500, 1000]
    bin_labels = ["0-50", "50-100", "100-200", "200-300", "300-500", "500+"]
    results = []

    print(f"\n  {'Gyro Range':>12s}  {'Samples':>8s}  "
          f"{'Roll RMS':>10s}  {'Pitch RMS':>10s}  {'Max Err':>10s}  {'Status':>10s}")
    print(f"  {'-'*12}  {'-'*8}  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*10}")

    max_ok_rate = 0
    divergence_rate = None

    for i in range(len(bins) - 1):
        mask = (gyro_mag >= bins[i]) & (gyro_mag < bins[i+1])
        n = np.sum(mask)
        if n < 10:
            continue

        r_rms = float(np.std(roll[mask]))
        p_rms = float(np.std(pitch[mask]))
        max_err = float(max(np.max(np.abs(roll[mask])), np.max(np.abs(pitch[mask]))))

        status = "OK" if max_err < 10 else ("WARN" if max_err < 30 else "DIVERGE")
        if status == "OK":
            max_ok_rate = bins[i+1]
        if divergence_rate is None and status == "DIVERGE":
            divergence_rate = bins[i]

        print(f"  {bin_labels[i]:>12s}  {n:8d}  {r_rms:10.2f}  {p_rms:10.2f}  {max_err:10.2f}  {status:>10s}")
        results.append({
            "gyro_range": bin_labels[i],
            "samples": int(n),
            "roll_rms_deg": r_rms,
            "pitch_rms_deg": p_rms,
            "max_error_deg": max_err,
            "status": status,
        })

    print(f"\n  Max safe rate: ~{max_ok_rate} °/s")
    if divergence_rate:
        print(f"  Divergence boundary: ~{divergence_rate} °/s")
    else:
        print(f"  No divergence observed up to {bins[-2]} °/s")

    # Save
    with open(os.path.join(out_dir, "highrate_results.csv"), "w", newline="") as f:
        if results:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)

    return results


# ═══════════════════════════════════════════════════════════════
# 3. Long-duration drift test
# ═══════════════════════════════════════════════════════════════

def long_drift_test(path, out_dir):
    """Analyze cumulative drift over multi-hour recording.

    Reports: drift rate per hour, cumulative drift vs time,
             verify Mahony I-term doesn't diverge.
    """
    print(f"\n{'='*60}")
    print("  LONG-DURATION DRIFT TEST")
    print(f"{'='*60}")

    data = load_mode0_csv(path)
    N = data.shape[0]

    # Estimate rate from data
    if N >= 2:
        # MODE 0 outputs at ~50Hz (DOWNSAMPLE_N=20 @1kHz)
        # Estimate from roll/pitch data spacing
        rate = 50.0  # default
    else:
        rate = 50.0

    roll = data[:, 6]
    pitch = data[:, 7]
    yaw = data[:, 8]

    # Skip initial convergence (first 100 samples ~2s)
    skip = 100
    roll = roll[skip:]
    pitch = pitch[skip:]
    yaw = yaw[skip:]
    N = len(roll)

    duration_h = N / rate / 3600.0
    print(f"  {N} samples @ ~{rate:.0f} Hz, {duration_h:.2f} hours")

    # Time vector
    t = np.arange(N) / rate / 3600.0  # hours

    # Linear fit for drift rate
    drift_r, _ = np.polyfit(t, roll, 1)   # deg/hr
    drift_p, _ = np.polyfit(t, pitch, 1)
    drift_y, _ = np.polyfit(t, yaw, 1)

    print(f"\n  {'Axis':>8s}  {'Drift Rate':>12s}  {'Cumul @1h':>10s}  {'Cumul @3h':>10s}")
    print(f"  {'-'*8}  {'-'*12}  {'-'*10}  {'-'*10}")
    for label, d in [("Roll", drift_r), ("Pitch", drift_p), ("Yaw", drift_y)]:
        print(f"  {label:>8s}  {d:+10.2f} deg/hr  "
              f"{d*1:+9.2f} deg  {d*3:+9.2f} deg")

    # Long-term trend: does drift accelerate?
    # Split into thirds and compare slopes
    third = N // 3
    d1, _ = np.polyfit(t[:third], roll[:third], 1)
    d3, _ = np.polyfit(t[-third:], roll[-third:], 1)
    drift_change = (d3 - d1) / max(abs(d1), 0.001) * 100

    print(f"\n  Roll drift change (first vs last third): {drift_change:+.1f}%")
    if abs(drift_change) < 50:
        print("  Stability: I-term is tracking bias, no runaway")
    elif abs(drift_change) < 200:
        print("  Stability: moderate drift acceleration, I-term may be saturating")
    else:
        print("  WARNING: large drift acceleration, check I-term clamping")

    # Save
    with open(os.path.join(out_dir, "long_drift_results.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Axis", "Drift_deg_per_hr", "Cumul_1h_deg", "Cumul_3h_deg",
                     "Drift_trend_pct"])
        w.writerow(["Roll", f"{drift_r:.4f}", f"{drift_r*1:.4f}",
                     f"{drift_r*3:.4f}", f"{drift_change:.1f}"])
        w.writerow(["Pitch", f"{drift_p:.4f}", f"{drift_p*1:.4f}",
                     f"{drift_p*3:.4f}", "—"])
        w.writerow(["Yaw", f"{drift_y:.4f}", f"{drift_y*1:.4f}",
                     f"{drift_y*3:.4f}", "—"])

    return drift_r, drift_p, drift_y


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="IMU Robustness Boundary Test Suite")
    parser.add_argument("mode", choices=["shake", "highrate", "drift"],
                        help="Test mode")
    parser.add_argument("files", nargs="+",
                        help="CSV file(s): shake=fixed,adaptive | "
                             "highrate=file | drift=file")
    parser.add_argument("--out", default="",
                        help="Output directory (default: ../results/)")
    args = parser.parse_args()

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)

    if args.mode == "shake":
        if len(args.files) < 2:
            print("ERROR: shake test needs two files: fixed_kp.csv adaptive_kp.csv")
            sys.exit(1)
        shake_test(args.files[0], args.files[1], out_dir)

    elif args.mode == "highrate":
        highrate_test(args.files[0], out_dir)

    elif args.mode == "drift":
        long_drift_test(args.files[0], out_dir)


if __name__ == "__main__":
    main()
