#!/usr/bin/env python3
"""
Three-way algorithm comparison: Mahony vs Madgwick vs EKF
==========================================================
Runs all three algorithms on the same datasets, computing:
  --static: drift rate + noise (ground truth: angle should not change)
  --shake:  attitude jitter RMS (no ground truth, smoothness only)

Usage:
    python algo_compare.py allan_2h.csv --shake fixed_kp_shake.csv
"""

import sys
import os
import csv
import argparse
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mahony import MahonyAHRS
from madgwick import MadgwickAHRS
from ekf6 import EKF6

if sys.stdout.encoding and sys.stdout.encoding.lower() in ("gbk", "gb2312", "cp936"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def load_raw(path, max_samples=0):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 7:
                continue
            try:
                rows.append([float(p) for p in parts[1:7]])
            except ValueError:
                continue
            if max_samples > 0 and len(rows) >= max_samples:
                break
    return np.array(rows, dtype=np.float64)


def load_mode0_sensor(path):
    """Load MODE 0 CSV, extract sensor columns [ax,ay,az,gx,gy,gz]."""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 6:
                continue
            try:
                rows.append([float(p) for p in parts[:6]])
            except ValueError:
                continue
    return np.array(rows, dtype=np.float64)


def compute_shake(out, skip=200):
    """Compute attitude jitter metrics (no ground truth — smoothness only)."""
    roll = out[skip:, 0]
    pitch = out[skip:, 1]
    return {
        "roll_rms": float(np.std(roll)),
        "pitch_rms": float(np.std(pitch)),
        "roll_ptp": float(np.ptp(roll)),
        "pitch_ptp": float(np.ptp(pitch)),
    }


def compute_drift(angles, rate):
    skip = int(5 * rate)
    a = angles[skip:, 0]
    t = np.arange(len(a)) / rate
    slope, intercept = np.polyfit(t, a, 1)
    detrended = a - (slope * t + intercept)
    noise = float(np.std(detrended))
    return float(slope * 3600), noise


def main():
    parser = argparse.ArgumentParser(description="Mahony vs Madgwick vs EKF")
    parser.add_argument("csv", help="MODE 6 RAW_DATA CSV (static recording)")
    parser.add_argument("--shake", default="",
                        help="MODE 0 shake CSV for dynamic comparison")
    parser.add_argument("--max-samples", type=int, default=500000,
                        help="Max samples (default 500k, ~35min @248Hz)")
    parser.add_argument("--out", default="",
                        help="Output directory (default: ../results/)")
    args = parser.parse_args()

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)

    # ═══════════════════════════════════════════════════════
    # STATIC DRIFT TEST
    # ═══════════════════════════════════════════════════════
    print(f"Loading {args.csv} ...")
    data = load_raw(args.csv, args.max_samples)
    rate = 248.0
    print(f"  {data.shape[0]} samples @ ~{rate} Hz, "
          f"{data.shape[0]/rate/3600:.1f} hours\n")

    static_results = []
    results = []

    print(f"  {'Algorithm':<12s} {'Drift':>10s} {'Noise':>10s} {'Time':>8s}")
    print(f"  {'-'*12} {'-'*10} {'-'*10} {'-'*8}")

    # Mahony
    t0 = time.time()
    m = MahonyAHRS(kp=2.0, ki=0.1)
    om = m.run_dataset(data)
    dm, nm = compute_drift(om, rate)
    tm = time.time() - t0
    print(f"  {'Mahony':<12s} {dm:+9.2f} /hr {nm:8.4f} deg {tm:7.1f}s")
    static_results.append({"algo": "Mahony", "drift_deg_hr": dm, "noise_deg": nm, "time_s": tm})

    # Madgwick
    t0 = time.time()
    mg = MadgwickAHRS(beta=0.05)
    og = mg.run_dataset(data)
    dg, ng = compute_drift(og, rate)
    tg = time.time() - t0
    print(f"  {'Madgwick':<12s} {dg:+9.2f} /hr {ng:8.4f} deg {tg:7.1f}s")
    static_results.append({"algo": "Madgwick", "drift_deg_hr": dg, "noise_deg": ng, "time_s": tg})

    # EKF
    t0 = time.time()
    ekf = EKF6(arw_deg_per_sqrt_hr=0.41, bi_deg_per_hr=6.8)
    oe = ekf.run_dataset(data)
    de, ne = compute_drift(oe, rate)
    te = time.time() - t0
    bx, by, bz = ekf.bias[0] * 57.3, ekf.bias[1] * 57.3, ekf.bias[2] * 57.3
    print(f"  {'EKF':<12s} {de:+9.2f} /hr {ne:8.4f} deg {te:7.1f}s")
    print(f"    EKF bias est: bx={bx:.4f} by={by:.4f} bz={bz:.4f} deg/s")
    static_results.append({"algo": "EKF", "drift_deg_hr": de, "noise_deg": ne, "time_s": te,
                           "bias_bx": bx, "bias_by": by, "bias_bz": bz})

    # ═══════════════════════════════════════════════════════
    # SHAKE TEST (optional)
    # ═══════════════════════════════════════════════════════
    shake_results = []
    if args.shake:
        print(f"\n  SHAKE TEST: {args.shake}")
        shake_data = load_mode0_sensor(args.shake)
        print(f"  {shake_data.shape[0]} samples\n")

        print(f"  {'Algorithm':<12s} {'Roll RMS':>10s} {'Roll P-P':>10s} "
              f"{'Pitch RMS':>10s} {'Pitch P-P':>10s}")
        print(f"  {'-'*12} {'-'*10} {'-'*10} {'-'*10} {'-'*10}")

        for name, algo in [
            ("Mahony", MahonyAHRS(kp=2.0, ki=0.1)),
            ("Madgwick", MadgwickAHRS(beta=0.05)),
            ("EKF", EKF6(arw_deg_per_sqrt_hr=0.41, bi_deg_per_hr=6.8)),
        ]:
            out = algo.run_dataset(shake_data)
            s = compute_shake(out)
            print(f"  {name:<12s} {s['roll_rms']:9.2f} deg {s['roll_ptp']:9.1f} deg "
                  f"{s['pitch_rms']:9.2f} deg {s['pitch_ptp']:9.1f} deg")
            s["algo"] = name
            shake_results.append(s)
        print("\n  Note: no ground truth — RMS is jitter smoothness, not error.")

    # ═══════════════════════════════════════════════════════
    # Save
    # ═══════════════════════════════════════════════════════
    with open(os.path.join(out_dir, "algo_comparison_static.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["algo", "drift_deg_hr", "noise_deg", "time_s"],
                           extrasaction="ignore")
        w.writeheader()
        w.writerows(static_results)

    if shake_results:
        with open(os.path.join(out_dir, "algo_comparison_shake.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["algo", "roll_rms", "roll_ptp",
                                               "pitch_rms", "pitch_ptp"])
            w.writeheader()
            w.writerows(shake_results)

    print(f"\nSaved -> {out_dir}/algo_comparison_static.csv")
    if shake_results:
        print(f"Saved -> {out_dir}/algo_comparison_shake.csv")


if __name__ == "__main__":
    main()
