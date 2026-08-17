#!/usr/bin/env python3
"""
DLPF Bandwidth Comparison Tool
===============================

Compares noise level across different MPU6050 DLPF settings.
Tests the noise-suppression vs delay trade-off for hardware filter selection.

Usage:
    # MODE 7 single-file (auto-split):
    python dlpf_compare.py dlpf_sweep.csv

    # Manual separate files:
    python dlpf_compare.py dlpf_256.csv:256 dlpf_188.csv:188 ... dlpf_5.csv:5

Output:
    results/dlpf_comparison.png  — noise vs bandwidth + PSD overlay
    results/dlpf_parameters.csv  — numerical comparison table
"""

import sys
import os
import csv
import argparse
import numpy as np

G_TO_MPS2 = 9.80665

# MPU6050 DLPF delay specs (from datasheet, in ms)
DLPF_DELAY = {
    256: 0.98, 188: 1.9, 98: 2.8, 42: 4.8, 20: 8.3, 10: 13.4, 5: 18.6,
}

if sys.stdout.encoding and sys.stdout.encoding.lower() in ("gbk", "gb2312", "cp936"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def load_csv(path):
    """Load MODE 6/7 RAW_DATA CSV (7-column data rows, skips # comments)."""
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
                rows.append([float(p) for p in parts])
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"No valid data in {path}")
    data = np.array(rows, dtype=np.float64)
    if data.shape[0] >= 2:
        dt_ticks = np.median(np.diff(data[:, 0]))
        rate = 24_000_000.0 / dt_ticks if dt_ticks > 0 else 1000.0
    else:
        rate = 1000.0
    return data[:, 1:7], rate


def load_and_split_mode7(path):
    """Load MODE 7 DLPF sweep file, split into per-DLPF sections.

    MODE 7 markers look like:  #DLPF=256
    Returns: list of (bandwidth_hz, data_array, rate)
    """
    print(f"Loading MODE 7 sweep: {path}")
    sections = {}
    current_bw = None
    current_rows = []

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Detect DLPF section marker
            if line.startswith("#DLPF="):
                # Save previous section
                if current_bw is not None and current_rows:
                    sections[current_bw] = current_rows
                # Start new section
                bw_str = line.split("=")[1].strip()
                current_bw = int(bw_str)
                current_rows = []
                print(f"  Found section: DLPF={current_bw} Hz")
                continue
            if line.startswith("#"):
                continue  # skip other headers
            if current_bw is None:
                continue  # skip data before first DLPF marker
            parts = line.split(",")
            if len(parts) != 7:
                continue
            try:
                current_rows.append([float(p) for p in parts])
            except ValueError:
                continue

    # Save last section
    if current_bw is not None and current_rows:
        sections[current_bw] = current_rows

    if not sections:
        raise ValueError(f"No #DLPF= markers found in {path} — not a MODE 7 file?")

    # Convert to arrays
    result = []
    for bw in sorted(sections.keys(), reverse=True):
        rows = sections[bw]
        data = np.array(rows, dtype=np.float64)
        if data.shape[0] >= 2:
            dt_ticks = np.median(np.diff(data[:, 0]))
            rate = 24_000_000.0 / dt_ticks if dt_ticks > 0 else 1000.0
        else:
            rate = 1000.0
        result.append((bw, data[:, 1:7], rate))
        print(f"  DLPF={bw:4d} Hz: {data.shape[0]:6d} samples @ {rate:.0f} Hz "
              f"({data.shape[0]/rate:.1f}s)")

    return result


def welch_psd(x, fs, nperseg=2048, overlap=0.5):
    """Welch PSD estimate."""
    nstep = int(nperseg * (1.0 - overlap))
    n_segs = (len(x) - nperseg) // nstep + 1
    if n_segs < 2:
        return np.array([0]), np.array([0])
    window = np.hanning(nperseg)
    win_power = np.mean(window ** 2)
    psd_sum = np.zeros(nperseg // 2 + 1, dtype=np.float64)
    for i in range(n_segs):
        seg = x[i * nstep: i * nstep + nperseg]
        seg = (seg - np.mean(seg)) * window
        fft = np.fft.rfft(seg)
        psd_sum += np.abs(fft) ** 2
    psd = psd_sum / (n_segs * fs * nperseg * win_power)
    psd[1:-1] *= 2.0
    freq = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return freq, psd


def compute_metrics(data, rate):
    """Compute noise metrics for one recording.
    data columns: ax, ay, az, gx, gy, gz
    """
    metrics = {}
    # Gyro: std dev of each axis (deg/s)
    for i, name in enumerate(["gx", "gy", "gz"]):
        col = data[:, 3 + i]
        metrics[f"{name}_std"] = float(np.std(col))
        f, p = welch_psd(col, rate)
        mask = (f >= 10) & (f <= 80)
        if np.sum(mask) >= 5:
            metrics[f"{name}_psd_floor"] = float(np.mean(p[mask]))
        else:
            metrics[f"{name}_psd_floor"] = float(np.mean(p[len(p)//2:]))

    # Accel: std dev (mg)
    for i, name in enumerate(["ax", "ay", "az"]):
        col = data[:, i]
        metrics[f"{name}_std_mg"] = float(np.std(col)) * 1000.0

    return metrics


def plot_comparison(bandwidths, results, out_path):
    """Plot noise vs DLPF bandwidth."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] matplotlib not installed — skipping plot")
        return

    bw = np.array(bandwidths)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ── Left: Noise std dev vs bandwidth ──
    ax = axes[0]
    colors = ["#1f77b4", "#d62728", "#2ca02c"]
    for i, name in enumerate(["gx", "gy", "gz"]):
        vals = [r[f"{name}_std"] for r in results]
        ax.loglog(bw, vals, "o-", color=colors[i], label=f"{name} std (deg/s)",
                  markersize=6)
    ax.set_xlabel("DLPF Bandwidth (Hz)")
    ax.set_ylabel("Noise Std Dev (deg/s)")
    ax.set_title("Gyro Noise vs DLPF Bandwidth")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    ax.invert_xaxis()

    # Add delay axis on top
    ax2 = ax.twiny()
    delays = [DLPF_DELAY.get(b, 0) for b in bw]
    ax2.set_xscale("log")
    # Map bandwidth to delay
    ax2.set_xlim(ax.get_xlim())
    ax2.set_xlabel("Approx. Delay (ms)")
    # Place delay labels at bandwidth positions
    delay_ticks = [DLPF_DELAY.get(b, 0) for b in sorted(bw, reverse=True)]
    ax2.set_xticks(sorted(bw, reverse=True))
    ax2.set_xticklabels([f"{d:.1f}" for d in delay_ticks], fontsize=7)

    # ── Right: Accel noise (mg) vs bandwidth ──
    ax = axes[1]
    accel_names = ["ax", "ay", "az"]
    for i, name in enumerate(accel_names):
        vals = [r[f"{name}_std_mg"] for r in results]
        ax.loglog(bw, vals, "s-", color=colors[i], label=f"{name} std (mg)",
                  markersize=6)
    ax.set_xlabel("DLPF Bandwidth (Hz)")
    ax.set_ylabel("Noise Std Dev (mg)")
    ax.set_title("Accelerometer Noise vs DLPF Bandwidth")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    ax.invert_xaxis()

    # ── Annotate: lowest noise and current setting ──
    # Select by combined 3-axis gyro noise (DLPF is a shared low-pass, so the
    # axes move together with bandwidth; combined metric keeps Gz in the picture).
    noise_idx = int(np.argmin([r["gx_std"] + r["gy_std"] + r["gz_std"] for r in results]))
    for ax_obj in axes:
        # Lowest noise
        ax_obj.axvline(x=bw[noise_idx], color="green", linestyle="--",
                       alpha=0.4, linewidth=1)
        ax_obj.annotate(f"Lowest noise: {bw[noise_idx]} Hz",
                        xy=(bw[noise_idx], ax_obj.get_ylim()[1] * 0.95),
                        fontsize=8, ha="center", color="green")
        # Current (42 Hz)
        if 42 in bw:
            ax_obj.axvline(x=42, color="gray", linestyle=":", alpha=0.5, linewidth=1)
            ax_obj.annotate("Current: 42 Hz",
                            xy=(42, ax_obj.get_ylim()[0] * 1.2),
                            fontsize=8, ha="center", color="gray")

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved comparison plot -> {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="DLPF bandwidth comparison for MPU6050")
    parser.add_argument("files", nargs="+",
                        help="CSV files with bandwidth: "
                             "dlpf_256.csv:256 dlpf_42.csv:42 ...")
    parser.add_argument("--out", default="",
                        help="Output directory (default: ../results/)")
    args = parser.parse_args()

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)

    # Detect MODE 7 auto-sweep (single file, no :BW suffix on any arg)
    is_mode7 = (len(args.files) == 1 and ":" not in args.files[0])

    bandwidths = []
    all_metrics = []

    if is_mode7:
        # MODE 7: auto-split by #DLPF= markers
        sections = load_and_split_mode7(args.files[0])
        for bw, data, rate in sections:
            m = compute_metrics(data, rate)
            m["dlpf_hz"] = bw
            m["delay_ms"] = DLPF_DELAY.get(bw, 0)
            m["samples"] = data.shape[0]
            bandwidths.append(bw)
            all_metrics.append(m)
            print(f"  DLPF={bw:4d} Hz -> gyro std: "
                  f"{m['gx_std']:.4f}/{m['gy_std']:.4f}/{m['gz_std']:.4f} deg/s")
    else:
        # Manual: file:bandwidth pairs
        for spec in args.files:
            if ":" in spec:
                path, bw_str = spec.rsplit(":", 1)
                bw = int(bw_str)
            else:
                path = spec
                bw = int(os.path.splitext(os.path.basename(path))[0]
                         .replace("dlpf_", "").replace("hz", ""))
            print(f"Loading {path} (DLPF={bw} Hz) ...")
            data, rate = load_csv(path)
            m = compute_metrics(data, rate)
            m["dlpf_hz"] = bw
            m["delay_ms"] = DLPF_DELAY.get(bw, 0)
            m["samples"] = data.shape[0]
            bandwidths.append(bw)
            all_metrics.append(m)
            print(f"  {data.shape[0]} samples @ {rate:.0f} Hz, "
                  f"gyro std: {m['gx_std']:.4f}/{m['gy_std']:.4f}/{m['gz_std']:.4f} deg/s")

    # Sort by bandwidth
    order = np.argsort(bandwidths)[::-1]
    bandwidths = [bandwidths[i] for i in order]
    all_metrics = [all_metrics[i] for i in order]

    # ── Print comparison table ──
    print(f"\n{'='*80}")
    print(f"  DLPF Bandwidth Comparison")
    print(f"{'='*80}")
    print(f"  {'BW(Hz)':>7s}  {'Delay(ms)':>9s}  "
          f"{'Gx std':>8s}  {'Gy std':>8s}  {'Gz std':>8s}  "
          f"{'Ax std':>8s}  {'Ay std':>8s}  {'Az std':>8s}")
    print(f"  {'-'*7}  {'-'*9}  {'-'*8}  {'-'*8}  {'-'*8}  "
          f"{'-'*8}  {'-'*8}  {'-'*8}")
    for m in all_metrics:
        print(f"  {m['dlpf_hz']:7d}  {m['delay_ms']:8.1f}  "
              f"{m['gx_std']:7.4f}  {m['gy_std']:7.4f}  {m['gz_std']:7.4f}  "
              f"{m['ax_std_mg']:7.2f}  {m['ay_std_mg']:7.2f}  {m['az_std_mg']:7.2f}")

    # ── Find best (lowest noise) ──
    # Select by combined 3-axis gyro noise, but report each axis separately —
    # Gz is typically ~2x noisier than Gx/Gy and shouldn't be hidden behind Gx.
    best_noise = min(all_metrics,
                     key=lambda m: m["gx_std"] + m["gy_std"] + m["gz_std"])
    print(f"\n  Lowest noise:  DLPF={best_noise['dlpf_hz']} Hz "
          f"(gx={best_noise['gx_std']:.4f}, gy={best_noise['gy_std']:.4f}, "
          f"gz={best_noise['gz_std']:.4f} deg/s, delay={best_noise['delay_ms']:.1f} ms)")

    # Find current (42 Hz) for comparison
    current = None
    for m in all_metrics:
        if m["dlpf_hz"] == 42:
            current = m
            break
    if current:
        # Compare with two candidates: lowest noise and lowest delay
        best_delay = min(all_metrics, key=lambda m: m["delay_ms"])
        for label, m in [("Lowest noise", best_noise), ("Lowest delay", best_delay)]:
            if m["dlpf_hz"] != 42:
                deltas = []
                for name in ("gx", "gy", "gz"):
                    d = (1.0 - m[f"{name}_std"] / current[f"{name}_std"]) * 100.0
                    deltas.append(f"{name}{'-' if d > 0 else '+'}{abs(d):.0f}%")
                print(f"  {label}: {m['dlpf_hz']} Hz vs 42Hz: "
                      f"noise {' / '.join(deltas)}, "
                      f"delay {m['delay_ms']:.1f}ms vs {current['delay_ms']:.1f}ms")

    # ── Save CSV ──
    keys = ["dlpf_hz", "delay_ms", "samples",
            "gx_std", "gy_std", "gz_std",
            "ax_std_mg", "ay_std_mg", "az_std_mg",
            "gx_psd_floor", "gy_psd_floor", "gz_psd_floor"]
    with open(os.path.join(out_dir, "dlpf_parameters.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(all_metrics)
    print(f"Saved parameters -> {out_dir}/dlpf_parameters.csv")

    # ── Plot ──
    plot_comparison(bandwidths, all_metrics,
                    os.path.join(out_dir, "dlpf_comparison.png"))


if __name__ == "__main__":
    main()
