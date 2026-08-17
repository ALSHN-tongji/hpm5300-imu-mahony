#!/usr/bin/env python3
"""
IMU Parameter Sensitivity Sweep
================================
Sweeps Mahony (Kp, Ki) and Madgwick (beta) across recorded sensor datasets,
computing static drift, static noise, and dynamic smoothness for each combo.

Usage:
    python param_sweep.py static_60s.csv dynamic_60s.csv

Output:
    results/sweep_results.csv       — all raw metrics
    results/mahony_drift.png        — Kp×Ki drift heatmap
    results/mahony_noise.png        — Kp×Ki noise heatmap
    results/mahony_combined.png     — Kp×Ki combined score heatmap
    results/madgwick_beta.png       — beta vs drift/noise line chart
"""

import sys
import os
import csv
import time
import numpy as np

# Add tools dir to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mahony import MahonyAHRS
from madgwick import MadgwickAHRS

# ── Parameter grids ──
MAHONY_KP = [0.1, 0.3, 0.5, 1.0, 2.0, 3.0, 5.0, 10.0]
MAHONY_KI = [0.0, 0.01, 0.05, 0.1, 0.3, 0.5, 1.0]
MADGWICK_BETA = [0.01, 0.03, 0.05, 0.08, 0.1, 0.2, 0.5, 1.0]

# ── Analysis settings ──
WARMUP_SECONDS = 5.0        # skip first N seconds for convergence
SAMPLE_RATE_HZ = 1000.0     # assumed sample rate (Hz)
HPF_ALPHA = 0.95            # high-pass filter coefficient for dynamic smoothness


def load_csv(path):
    """Load recorded CSV: timestamp, ax, ay, az, gx, gy, gz.

    Handles corrupt first lines (partial serial read), comment lines (#),
    and variable column counts gracefully.

    Returns (data_array, sample_rate).  data_array has columns:
        [ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps]
    """
    print(f"Loading {path} ...")
    # Read line-by-line, keeping only valid 7-column data rows
    rows = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) == 7:
                try:
                    rows.append([float(p) for p in parts])
                except ValueError:
                    continue
    if not rows:
        raise ValueError(f"No valid data rows (7 columns) found in {path}")
    raw = np.array(rows, dtype=np.float64)

    n_valid = len(rows)
    total = sum(1 for _ in open(path, "r"))
    if n_valid < total:
        print(f"  (filtered {total - n_valid} bad/header lines, kept {n_valid})")

    # Detect sample rate from timestamps
    if raw.shape[0] >= 2:
        dt_ticks = np.diff(raw[:, 0])
        median_dt = float(np.median(dt_ticks))
        # 24 MHz timer → seconds
        rate = 24_000_000.0 / median_dt if median_dt > 0 else SAMPLE_RATE_HZ
    else:
        rate = SAMPLE_RATE_HZ

    print(f"  {raw.shape[0]} samples, ~{rate:.0f} Hz, "
          f"{raw.shape[0]/rate:.1f}s duration")

    # Extract sensor columns (skip timestamp column 0)
    data = raw[:, 1:7].astype(np.float64)
    return data, rate


def compute_drift_noise(angles, rate, warmup_s):
    """Compute drift (deg/hr) and noise (deg stdev) from angle time series.

    Args:
        angles: 1-D array of Euler angles (degrees).
        rate: Sample rate in Hz.
        warmup_s: Skip first N seconds.

    Returns:
        (drift_deg_per_hr, noise_deg_stdev)
    """
    skip = int(warmup_s * rate)
    if skip >= len(angles) - 10:
        return 0.0, 0.0
    a = angles[skip:]

    # Linear fit for drift
    t = np.arange(len(a), dtype=np.float64) / rate  # seconds
    slope, _ = np.polyfit(t, a, 1)                  # deg/s
    drift = slope * 3600.0                           # deg/hr

    # Detrend for noise
    detrended = a - (slope * t + np.polyfit(t, a, 1)[1])
    noise = np.std(detrended)

    return drift, noise


def compute_dynamic_smoothness(angles, rate, warmup_s):
    """Compute high-frequency RMS (deg) during motion.

    Uses a first-order high-pass filter to isolate jitter from real motion.

    Args:
        angles: 1-D array of Euler angles (degrees).
        rate: Sample rate in Hz.

    Returns:
        RMS of high-pass filtered signal (deg).
    """
    skip = int(warmup_s * rate)
    if skip >= len(angles) - 10:
        return 0.0
    a = angles[skip:]

    # First-order high-pass filter: y[n] = α*(y[n-1] + x[n] - x[n-1])
    hp = np.zeros_like(a)
    for i in range(1, len(a)):
        hp[i] = HPF_ALPHA * (hp[i - 1] + a[i] - a[i - 1])
    return float(np.sqrt(np.mean(hp ** 2)))


def sweep_mahony(static_data, dynamic_data, rate):
    """Sweep Mahony Kp × Ki grid.

    Returns list of dicts with keys:
        kp, ki, drift_roll, drift_pitch, drift_yaw,
        noise_roll, noise_pitch, dyn_roll, dyn_pitch, score
    """
    results = []
    total = len(MAHONY_KP) * len(MAHONY_KI)
    n = 0

    for kp in MAHONY_KP:
        for ki in MAHONY_KI:
            n += 1
            t0 = time.time()
            ahrs = MahonyAHRS(kp=kp, ki=ki)

            # ── Static run ──
            static_out = ahrs.run_dataset(static_data)
            drift_r, noise_r = compute_drift_noise(static_out[:, 0], rate, WARMUP_SECONDS)
            drift_p, noise_p = compute_drift_noise(static_out[:, 1], rate, WARMUP_SECONDS)
            drift_y, _ = compute_drift_noise(static_out[:, 2], rate, WARMUP_SECONDS)

            # ── Dynamic run ──
            ahrs.reset()
            dynamic_out = ahrs.run_dataset(dynamic_data)
            dyn_r = compute_dynamic_smoothness(dynamic_out[:, 0], rate, WARMUP_SECONDS)
            dyn_p = compute_dynamic_smoothness(dynamic_out[:, 1], rate, WARMUP_SECONDS)

            elapsed = time.time() - t0
            print(f"  [{n:3d}/{total}] Kp={kp:.1f} Ki={ki:.3f}  "
                  f"drift_r={drift_r:+.2f}°/h  noise_r={noise_r:.3f}°  "
                  f"dyn_r={dyn_r:.4f}°RMS  ({elapsed:.1f}s)")

            results.append({
                "algo": "Mahony", "kp": kp, "ki": ki,
                "drift_roll": drift_r, "drift_pitch": drift_p, "drift_yaw": drift_y,
                "noise_roll": noise_r, "noise_pitch": noise_p,
                "dyn_roll": dyn_r, "dyn_pitch": dyn_p,
            })

    # Compute combined score (lower = better)
    _compute_scores(results)
    return results


def sweep_madgwick(static_data, dynamic_data, rate):
    """Sweep Madgwick beta values.

    Returns list of dicts.
    """
    results = []
    total = len(MADGWICK_BETA)

    for i, beta in enumerate(MADGWICK_BETA):
        t0 = time.time()
        ahrs = MadgwickAHRS(beta=beta)

        # ── Static run ──
        static_out = ahrs.run_dataset(static_data)
        drift_r, noise_r = compute_drift_noise(static_out[:, 0], rate, WARMUP_SECONDS)
        drift_p, noise_p = compute_drift_noise(static_out[:, 1], rate, WARMUP_SECONDS)
        drift_y, _ = compute_drift_noise(static_out[:, 2], rate, WARMUP_SECONDS)

        # ── Dynamic run ──
        ahrs.reset()
        dynamic_out = ahrs.run_dataset(dynamic_data)
        dyn_r = compute_dynamic_smoothness(dynamic_out[:, 0], rate, WARMUP_SECONDS)
        dyn_p = compute_dynamic_smoothness(dynamic_out[:, 1], rate, WARMUP_SECONDS)

        elapsed = time.time() - t0
        print(f"  [{i+1:3d}/{total}] beta={beta:.3f}  "
              f"drift_r={drift_r:+.2f}°/h  noise_r={noise_r:.3f}°  "
              f"dyn_r={dyn_r:.4f}°RMS  ({elapsed:.1f}s)")

        results.append({
            "algo": "Madgwick", "beta": beta,
            "drift_roll": drift_r, "drift_pitch": drift_p, "drift_yaw": drift_y,
            "noise_roll": noise_r, "noise_pitch": noise_p,
            "dyn_roll": dyn_r, "dyn_pitch": dyn_p,
        })

    _compute_scores(results)
    return results


def _compute_scores(results):
    """Add normalized combined score to results (in-place). Lower = better."""
    # Collect all metrics
    dr = np.array([r["drift_roll"] for r in results])
    dp = np.array([r["drift_pitch"] for r in results])
    nr = np.array([r["noise_roll"] for r in results])
    np_ = np.array([r["noise_pitch"] for r in results])
    dyr = np.array([r["dyn_roll"] for r in results])
    dyp = np.array([r["dyn_pitch"] for r in results])

    # Normalize each metric to [0, 1] using min-max
    def _norm(x):
        mn, mx = np.min(np.abs(x)), np.max(np.abs(x))
        if mx - mn < 1e-12:
            return np.zeros_like(x)
        return (np.abs(x) - mn) / (mx - mn)

    score = (_norm(dr) + _norm(dp) + _norm(nr) + _norm(np_) +
             _norm(dyr) + _norm(dyp)) / 6.0

    for i, r in enumerate(results):
        r["score"] = float(score[i])


def save_csv(results, path):
    """Save all sweep results to CSV. Handles heterogeneous keys (Mahony+Madgwick)."""
    if not results:
        return
    # Collect union of all keys, with priority order
    key_order = ["algo", "kp", "ki", "beta",
                 "drift_roll", "drift_pitch", "drift_yaw",
                 "noise_roll", "noise_pitch",
                 "dyn_roll", "dyn_pitch", "score"]
    all_keys = set()
    for r in results:
        all_keys.update(r.keys())
    # Start with ordered keys that exist, then add any remaining
    keys = [k for k in key_order if k in all_keys]
    keys += sorted(all_keys - set(key_order))
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(results)
    print(f"Saved {len(results)} results → {path}")


def print_best(results, algo_name, top_n=5):
    """Print top-N parameter combos by combined score."""
    print(f"\n{'='*60}")
    print(f"  {algo_name} — Top {top_n} parameter sets (lower score = better)")
    print(f"{'='*60}")
    sorted_r = sorted(results, key=lambda r: r["score"])
    for i, r in enumerate(sorted_r[:top_n]):
        params = ", ".join(f"{k}={v}" for k, v in r.items()
                           if k in ("kp", "ki", "beta"))
        print(f"  #{i+1}  {params:30s}  "
              f"drift_r={r['drift_roll']:+.2f}°/h  "
              f"noise_r={r['noise_roll']:.3f}°  "
              f"dyn_r={r['dyn_roll']:.4f}°RMS  "
              f"score={r['score']:.3f}")


def plot_heatmaps(mahony_results, out_dir):
    """Generate Mahony Kp×Ki heatmaps. Requires matplotlib."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[!] matplotlib not installed — skipping heatmap plots.")
        print("    Install: pip install matplotlib")
        return

    kp_vals = sorted(set(r["kp"] for r in mahony_results))
    ki_vals = sorted(set(r["ki"] for r in mahony_results))

    def _grid(metric):
        g = np.zeros((len(ki_vals), len(kp_vals)))
        for r in mahony_results:
            j = kp_vals.index(r["kp"])
            i = ki_vals.index(r["ki"])
            g[i, j] = abs(getattr(r, metric) if hasattr(r, metric) else r[metric])
        # Fill ki=0 rows that are NaN from dict access
        g = np.nan_to_num(g, nan=0.0)
        return g

    # Use dict-based lookup
    def _grid_dict(metric):
        g = np.zeros((len(ki_vals), len(kp_vals)))
        for r in mahony_results:
            j = kp_vals.index(r["kp"])
            i = ki_vals.index(r["ki"])
            g[i, j] = r[metric]
        return g

    metrics = [
        ("drift_roll", "Mahony: Roll Drift (°/hr)", "coolwarm"),
        ("noise_roll", "Mahony: Roll Noise (° stdev)", "viridis"),
        ("score", "Mahony: Combined Score (lower=better)", "RdYlGn_r"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, (metric, title, cmap) in zip(axes, metrics):
        grid = _grid_dict(metric)
        im = ax.imshow(grid, aspect="auto", origin="lower", cmap=cmap,
                       extent=[min(kp_vals), max(kp_vals), min(ki_vals), max(ki_vals)])
        ax.set_xlabel("Kp")
        ax.set_ylabel("Ki")
        ax.set_title(title)
        plt.colorbar(im, ax=ax)

        # Mark best point
        best = min(mahony_results, key=lambda r: r["score"])
        ax.plot(best["kp"], best["ki"], "k*", markersize=15, markeredgewidth=2,
                markeredgecolor="white")

    plt.tight_layout()
    path = os.path.join(out_dir, "mahony_heatmaps.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved heatmaps → {path}")


def plot_madgwick_beta(madgwick_results, out_dir):
    """Generate Madgwick beta sweep line charts."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return

    betas = [r["beta"] for r in madgwick_results]
    dr = [abs(r["drift_roll"]) for r in madgwick_results]
    dp = [abs(r["drift_pitch"]) for r in madgwick_results]
    nr = [r["noise_roll"] for r in madgwick_results]
    np_ = [r["noise_pitch"] for r in madgwick_results]
    dyr = [r["dyn_roll"] for r in madgwick_results]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    axes[0].plot(betas, dr, "o-", label="Roll drift")
    axes[0].plot(betas, dp, "s-", label="Pitch drift")
    axes[0].set_xlabel("β")
    axes[0].set_ylabel("Drift (°/hr)")
    axes[0].set_title("Madgwick: Static Drift vs β")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(betas, nr, "o-", label="Roll noise")
    axes[1].plot(betas, np_, "s-", label="Pitch noise")
    axes[1].set_xlabel("β")
    axes[1].set_ylabel("Noise (° stdev)")
    axes[1].set_title("Madgwick: Static Noise vs β")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(betas, dyr, "o-", label="Roll dynamic RMS")
    axes[2].set_xlabel("β")
    axes[2].set_ylabel("Dynamic RMS (°)")
    axes[2].set_title("Madgwick: Dynamic Smoothness vs β")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    # Mark best beta
    best = min(madgwick_results, key=lambda r: r["score"])
    for ax in axes:
        ax.axvline(x=best["beta"], color="red", linestyle="--", alpha=0.5,
                   label=f"Best β={best['beta']:.3f}" if ax == axes[0] else "")

    plt.tight_layout()
    path = os.path.join(out_dir, "madgwick_beta.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"Saved beta chart → {path}")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        print("ERROR: Need static and dynamic CSV files.")
        print("Usage: python param_sweep.py static_60s.csv dynamic_60s.csv")
        sys.exit(1)

    static_path = sys.argv[1]
    dynamic_path = sys.argv[2]

    # ── Load data ──
    static_data, rate_s = load_csv(static_path)
    dynamic_data, rate_d = load_csv(dynamic_path)
    rate = min(rate_s, rate_d)
    print(f"Using sample rate: {rate:.0f} Hz\n")

    # ── Create output dir ──
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)

    # ── Sweep Mahony ──
    print("=" * 60)
    print("  Mahony Kp×Ki sweep")
    print("=" * 60)
    t0 = time.time()
    mahony_results = sweep_mahony(static_data, dynamic_data, rate)
    print(f"Mahony sweep done in {time.time()-t0:.1f}s")

    # ── Sweep Madgwick ──
    print(f"\n{'='*60}")
    print("  Madgwick β sweep")
    print("=" * 60)
    t0 = time.time()
    madgwick_results = sweep_madgwick(static_data, dynamic_data, rate)
    print(f"Madgwick sweep done in {time.time()-t0:.1f}s")

    # ── Save results ──
    all_results = mahony_results + madgwick_results
    save_csv(all_results, os.path.join(out_dir, "sweep_results.csv"))

    # ── Print best ──
    print_best(mahony_results, "Mahony")
    print_best(madgwick_results, "Madgwick")

    # ── Plot ──
    plot_heatmaps(mahony_results, out_dir)
    plot_madgwick_beta(madgwick_results, out_dir)

    # ── Summary ──
    best_m = min(mahony_results, key=lambda r: r["score"])
    best_g = min(madgwick_results, key=lambda r: r["score"])
    print(f"\n{'='*60}")
    print(f"  RECOMMENDED PARAMETERS")
    print(f"{'='*60}")
    print(f"  Mahony:   Kp={best_m['kp']:.1f}, Ki={best_m['ki']:.3f}  "
          f"(score={best_m['score']:.3f})")
    print(f"  Madgwick: β={best_g['beta']:.3f}  "
          f"(score={best_g['score']:.3f})")
    print(f"\n  Previous defaults: Kp=1.0, Ki=0.1, β=0.05")
    print(f"  Full results → {out_dir}/sweep_results.csv")


if __name__ == "__main__":
    main()
