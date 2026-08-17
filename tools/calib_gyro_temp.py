#!/usr/bin/env python3
"""
MPU6050 Gyro Temperature Drift Calibration — Linear Fit

Usage:
  1. Set ENABLE_GYRO_TEMP_CALIB_COLLECT = 1 in main.c, compile, flash.
  2. Keep sensor completely STILL (tape it down).
  3. Heat/cool the sensor (hairdryer, ice pack) to cover a wide temp range.
  4. Collect RTT output (format: "T_C, Gx_raw, Gy_raw, Gz_raw").
  5. Save the output lines to a text file, e.g. temp_data.txt.
  6. Run: python calib_gyro_temp.py < temp_data.txt
  7. Or:   python calib_gyro_temp.py temp_data.txt
  8. Copy the printed Ktx, Kty, Ktz into main.c macros.

Model:
  offset_x(T) = Ox0 + Ktx * (T - T0)   [in raw LSB]
  Ktx in raw_LSB/°C.  Convert to °/s/°C:  Ktx_dps = Ktx / 131.0

The script fits Gx_raw = a + b*T  →  Ktx = b (raw_LSB/°C).
"""

import sys
import numpy as np


def fit_axis(name, temps, gyro_vals, gyro_sf=131.0):
    """Linear fit gyro_vals = a + b * temps.  K = b (raw_LSB/°C)."""
    A = np.column_stack([np.ones_like(temps), temps])
    a, b = np.linalg.lstsq(A, gyro_vals, rcond=None)[0]

    # Residual
    pred = a + b * temps
    rmse = np.sqrt(np.mean((gyro_vals - pred) ** 2))
    max_e = np.max(np.abs(gyro_vals - pred))

    K_raw = b                          # raw_LSB / °C
    K_dps = K_raw / gyro_sf            # °/s / °C

    print(f"Axis {name}:")
    print(f"  Offset@T=0: {a:.1f} raw LSB")
    print(f"  K_{name.lower()}  = {K_raw:.4f} raw_LSB/°C  = {K_dps:.6f} °/s/°C")
    print(f"  RMSE: {rmse:.2f} raw  |  Max error: {max_e:.2f} raw")
    print()

    return K_raw, K_dps


def main():
    # Read data
    data = []
    if len(sys.argv) > 1:
        with open(sys.argv[1]) as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split(',')
        if len(parts) < 4:
            continue
        try:
            T = float(parts[0])
            Gx = float(parts[1])
            Gy = float(parts[2])
            Gz = float(parts[3])
            data.append((T, Gx, Gy, Gz))
        except ValueError:
            continue

    if len(data) < 10:
        print("ERROR: Need at least 10 data points. Got", len(data))
        print("Paste RTT output lines (T_C, Gx_raw, Gy_raw, Gz_raw).")
        return 1

    temps = np.array([d[0] for d in data])
    gx = np.array([d[1] for d in data])
    gy = np.array([d[2] for d in data])
    gz = np.array([d[3] for d in data])

    T_min = np.min(temps)
    T_max = np.max(temps)

    print(f"Loaded {len(data)} points | T range: {T_min:.1f} .. {T_max:.1f} °C")
    print()

    Kx_raw, Kx_dps = fit_axis('X', temps, gx)
    Ky_raw, Ky_dps = fit_axis('Y', temps, gy)
    Kz_raw, Kz_dps = fit_axis('Z', temps, gz)

    print("=== Copy into main.c ===")
    print(f"#define K_TX  {Kx_dps:.6f}f   // °/s per °C")
    print(f"#define K_TY  {Ky_dps:.6f}f   // °/s per °C")
    print(f"#define K_TZ  {Kz_dps:.6f}f   // °/s per °C")
    print()
    print(f"// Or in raw LSB/°C (used with int16 offset calc):")
    print(f"// Ktx_raw = {Kx_raw:.4f}f;  Kty_raw = {Ky_raw:.4f}f;  Ktz_raw = {Kz_raw:.4f}f;")

    return 0


if __name__ == '__main__':
    sys.exit(main())
