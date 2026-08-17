#!/usr/bin/env python3
"""
Power Spectral Density (PSD) Analysis for IMU Noise Characterization
====================================================================

Computes PSD via Welch's method from static IMU RAW_DATA (MODE 6).
Cross-validates with Allan variance: the white noise floor from PSD
should match the ARW from Allan deviation.

Usage:
    python psd_analysis.py allan_2h.csv [--rate 248]

Output:
    results/psd_gyro.png    — Gyro PSD (3-axis, dB scale)
    results/psd_accel.png   — Accel PSD (3-axis, dB scale)
    results/psd_parameters.csv — Noise floor comparison

Reference:
    PSD ↔ Allan variance relation:
      σ²(τ) = 4 ∫ PSD(f) · sin⁴(πfτ) / (πfτ)² df
    For white noise: PSD_flat [(deg/s)^2/Hz] → ARW [°/√hr] = √(PSD_flat) × 60
"""

import sys
import os
import csv
import argparse
import numpy as np

# Fix Unicode output on Windows GBK terminals
if sys.stdout.encoding and sys.stdout.encoding.lower() in ("gbk", "gb2312", "cp936"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

G_TO_MPS2 = 9.80665


def load_csv(path):
    """Load MODE 6 RAW_DATA CSV."""
    print(f"Loading {path} ...")
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
        raise ValueError(f"No valid 7-column data in {path}")
    data = np.array(rows, dtype=np.float64)
    if data.shape[0] >= 2:
        dt_ticks = np.median(np.diff(data[:, 0]))
        rate = 24_000_000.0 / dt_ticks if dt_ticks > 0 else 1000.0
    else:
        rate = 1000.0
    print(f"  {data.shape[0]} samples @ {rate:.0f} Hz, "
          f"{data.shape[0]/rate/3600:.2f} hours")
    return data[:, 1:7], rate


def welch_psd(x, fs, nperseg=4096, overlap=0.5):
    """Welch's power spectral density estimate.

    Args:
        x: 1-D signal.
        fs: Sample rate (Hz).
        nperseg: FFT segment length.
        overlap: Overlap fraction (0–1).

    Returns:
        freq: Frequency array (Hz).
        psd: PSD in units²/Hz (same units as x, squared, per Hz).
    """
    nstep = int(nperseg * (1.0 - overlap))
    n_segs = (len(x) - nperseg) // nstep + 1
    if n_segs < 1:
        raise ValueError(f"Signal too short ({len(x)}) for nperseg={nperseg}")

    window = np.hanning(nperseg)
    win_power = np.mean(window ** 2)

    psd_sum = np.zeros(nperseg // 2 + 1, dtype=np.float64)

    for i in range(n_segs):
        seg = x[i * nstep: i * nstep + nperseg]
        seg = (seg - np.mean(seg)) * window  # detrend + window
        fft = np.fft.rfft(seg)
        psd_sum += np.abs(fft) ** 2

    # Average and normalize
    psd = psd_sum / (n_segs * fs * nperseg * win_power)
    # Double the positive frequencies (except DC and Nyquist)
    psd[1:-1] *= 2.0
    freq = np.fft.rfftfreq(nperseg, 1.0 / fs)
    return freq, psd


def estimate_noise_floor(freq, psd, f_range=(10, 100)):
    """Estimate white noise floor from flat region of PSD.

    Returns mean PSD in the specified frequency band.
    """
    mask = (freq >= f_range[0]) & (freq <= f_range[1])
    if np.sum(mask) < 5:
        # Fall back to high-frequency half
        mask = freq > freq[-1] * 0.5
    return float(np.mean(psd[mask]))


def plot_psd(freqs_g, psds_g, labels_g, noise_g,
             freqs_a, psds_a, labels_a, noise_a,
             rate, output_path):
    """Generate two-panel PSD plot."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] matplotlib not installed — skipping plot")
        return

    colors = ["#1f77b4", "#d62728", "#2ca02c"]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # ── Gyro PSD ──
    ax = axes[0]
    for i in range(3):
        ax.loglog(freqs_g[i], psds_g[i], color=colors[i],
                  linewidth=0.5, alpha=0.8, label=labels_g[i])
        if noise_g[i] > 0:
            ax.axhline(y=noise_g[i], color=colors[i], linestyle="--",
                       alpha=0.5, linewidth=0.8)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD [(deg/s)^2/Hz]")
    ax.set_title(f"Gyroscope PSD  (Welch, NFFT=4096, fs={rate:.0f} Hz)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")

    # ── Accel PSD ──
    ax = axes[1]
    for i in range(3):
        ax.loglog(freqs_a[i], psds_a[i], color=colors[i],
                  linewidth=0.5, alpha=0.8, label=labels_a[i])
        if noise_a[i] > 0:
            ax.axhline(y=noise_a[i], color=colors[i], linestyle="--",
                       alpha=0.5, linewidth=0.8)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD [g²/Hz]")
    ax.set_title(f"Accelerometer PSD  (Welch, NFFT=4096, fs={rate:.0f} Hz)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close()
    print(f"Saved PSD plot → {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="IMU Power Spectral Density (Welch PSD) Analysis")
    parser.add_argument("csv", help="MODE 6 RAW_DATA CSV (static recording)")
    parser.add_argument("--rate", type=float, default=0,
                        help="Sample rate override (auto-detect if 0)")
    parser.add_argument("--nperseg", type=int, default=4096,
                        help="Welch segment length (default 4096)")
    parser.add_argument("--out", default="",
                        help="Output directory (default: ../results/)")
    args = parser.parse_args()

    # ── Load ──
    data, rate = load_csv(args.csv)
    if args.rate > 0:
        rate = args.rate

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "results")
    os.makedirs(out_dir, exist_ok=True)

    labels_g = ["Gx", "Gy", "Gz"]
    labels_a = ["Ax", "Ay", "Az"]
    # data columns: ax, ay, az, gx, gy, gz
    accel_idx = [0, 1, 2]
    gyro_idx = [3, 4, 5]

    # ── Compute PSD ──
    print(f"\n{'='*60}")
    print(f"  Welch PSD  (NFFT={args.nperseg}, overlap=50%)")
    print(f"{'='*60}")

    freqs_g, psds_g, noise_g = [], [], []
    freqs_a, psds_a, noise_a = [], [], []

    for i, idx in enumerate(gyro_idx):
        label = labels_g[i]
        print(f"  Gyro {label} ...")
        f, p = welch_psd(data[:, idx], rate, nperseg=args.nperseg)
        freqs_g.append(f)
        psds_g.append(p)
        nf = estimate_noise_floor(f, p)
        noise_g.append(nf)
        arw_from_psd = np.sqrt(nf) * 60.0  # °/√hr
        print(f"    PSD noise floor: {nf:.2e} (deg/s)^2/Hz  "
              f"→ ARW estimate: {arw_from_psd:.3f} °/√hr")

    for i, idx in enumerate(accel_idx):
        label = labels_a[i]
        print(f"  Accel {label} ...")
        f, p = welch_psd(data[:, idx], rate, nperseg=args.nperseg)
        freqs_a.append(f)
        psds_a.append(p)
        nf = estimate_noise_floor(f, p)
        noise_a.append(nf)
        vrw_from_psd = np.sqrt(nf) * G_TO_MPS2 * 60.0  # m/s/√hr
        print(f"    PSD noise floor: {nf:.2e} g²/Hz  "
              f"→ VRW estimate: {vrw_from_psd:.4f} m/s/√hr")

    # ── Cross-validation with Allan ──
    print(f"\n{'='*60}")
    print(f"  PSD ↔ Allan Cross-Validation")
    print(f"{'='*60}")
    for i, label in enumerate(labels_g):
        arw_psd = np.sqrt(noise_g[i]) * 60.0
        print(f"  Gyro {label}:  ARW(PSD)={arw_psd:.3f} °/√hr  "
              f"(Allan ARW from file → run allan_variance.py for comparison)")

    # ── Plot ──
    plot_psd(freqs_g, psds_g, labels_g, noise_g,
             freqs_a, psds_a, labels_a, noise_a,
             rate, os.path.join(out_dir, "psd_analysis.png"))

    # ── CSV ──
    with open(os.path.join(out_dir, "psd_parameters.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Sensor", "Axis", "PSD_floor", "Unit",
                     "ARW_VRW_estimate", "Unit"])
        for i in range(3):
            arw = np.sqrt(noise_g[i]) * 60.0
            w.writerow(["Gyro", labels_g[i], f"{noise_g[i]:.3e}",
                         "(deg/s)^2/Hz", f"{arw:.3f}", "°/√hr"])
        for i in range(3):
            vrw = np.sqrt(noise_a[i]) * G_TO_MPS2 * 60.0
            w.writerow(["Accel", labels_a[i], f"{noise_a[i]:.3e}",
                         "g²/Hz", f"{vrw:.4f}", "m/s/√hr"])
    print(f"Saved parameters → {out_dir}/psd_parameters.csv")

    print(f"\n  High-frequency PSD floor = white noise (ARW/VRW)")
    print(f"  Low-frequency rise = bias drift (1/f noise)")
    print(f"  PSD plot → {out_dir}/psd_analysis.png")


if __name__ == "__main__":
    main()
