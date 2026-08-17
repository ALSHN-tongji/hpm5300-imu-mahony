#!/usr/bin/env python3
"""
MPU6050 Accelerometer 6-Point Offline Calibration

Usage:
  1. Set IMU_CAL_DATA_COLLECT = 1 in main.c, compile, flash.
  2. Place sensor in each of the 6 orientations below.
     For each orientation, wait 3s for stabilization,
     then copy ~50 lines of raw output (Ax_raw, Ay_raw, Az_raw),
     compute the mean for each axis.
  3. Fill in the 6 rows below with your measured means.
  4. Run: python calib_accel.py
  5. Copy the printed Ox,Oy,Oz,Sx,Sy,Sz into your firmware.

Orientations (MPU6050 chip on GY-521):
  Face 1: +Z up  — chip face up, flat on table
  Face 2: -Z up  — chip face down, upside down
  Face 3: +X up  — board rotated so +X axis points up
  Face 4: -X up  — board rotated so -X axis points up
  Face 5: +Y up  — board rotated so +Y axis points up
  Face 6: -Y up  — board rotated so -Y axis points up

Model:
  Ax_corr = Sx * (Ax_raw - Ox)
  Ay_corr = Sy * (Ay_raw - Oy)
  Az_corr = Sz * (Az_raw - Oz)

Expected calibrated values at each face:
  Face 1 (+Z up):  [ 0,  0, +16384]
  Face 2 (-Z up):  [ 0,  0, -16384]
  Face 3 (+X up):  [+16384,  0,  0]
  Face 4 (-X up):  [-16384,  0,  0]
  Face 5 (+Y up):  [ 0, +16384,  0]
  Face 6 (-Y up):  [ 0, -16384,  0]

Note: 16384 = 1g in LSB for MPU6050 @ ±2g full scale.
"""

import numpy as np
import sys

# ============================================================
#  FILL IN YOUR MEASURED MEANS HERE (raw int16 values)
# ============================================================
raw = [
    # [Ax_mean,  Ay_mean,  Az_mean]   Orientation
    [ 530, -900, 16640],  # Face 1: +Z up
    [ 800, -420, -16650], # Face 2: -Z up
    [ 16660, -550, 420],   # Face 3: +X up
    [ -16050, -880, -150],  # Face 4: -X up
    [ -330, 15760, 590],   # Face 5: +Y up
    [ 610, -17020, -280],  # Face 6: -Y up
]

# Expected calibrated values (1g = 16384 LSB @ ±2g)
expected = [
    [     0,      0,  16384],  # Face 1: +Z up
    [     0,      0, -16384],  # Face 2: -Z up
    [ 16384,      0,      0],  # Face 3: +X up
    [-16384,      0,      0],  # Face 4: -X up
    [     0,  16384,      0],  # Face 5: +Y up
    [     0, -16384,      0],  # Face 6: -Y up
]
# ============================================================

def calibrate(raw, expected):
    """Solve Ax_corr = S*(Ax_raw - O) for each axis using least squares."""
    raw = np.array(raw, dtype=np.float64)
    exp = np.array(expected, dtype=np.float64)

    for axis, name in enumerate(['X', 'Y', 'Z']):
        r = raw[:, axis]          # measured raw values (6,)
        e = exp[:, axis]          # expected calibrated values (6,)

        # e = S * (r - O) = S*r - S*O
        # Let a = S, b = -S*O
        # e = a*r + b  →  least squares
        A = np.column_stack([r, np.ones_like(r)])
        a, b = np.linalg.lstsq(A, e, rcond=None)[0]

        Sx = a                         # scale
        Ox = -b / a if abs(a) > 1e-9 else 0.0  # offset

        # Verify
        e_calc = Sx * (r - Ox)
        residual = np.max(np.abs(e_calc - e))
        status = "OK" if residual < 500 else "WARN (residual %.0f)" % residual

        print(f"Axis {name}:  O = {Ox:.1f}    S = {Sx:.6f}    ({status})")

    print()
    print("=== Copy into firmware ===")
    for axis, name in enumerate(['X', 'Y', 'Z']):
        r = raw[:, axis]
        e = exp[:, axis]
        A = np.column_stack([r, np.ones_like(r)])
        a, b = np.linalg.lstsq(A, e, rcond=None)[0]
        Sx = a
        Ox = -b / a if abs(a) > 1e-9 else 0.0
        print(f"// Axis {name}:  O{name.lower()} = {Ox:.1f}f;  S{name.lower()} = {Sx:.6f}f;")


if __name__ == '__main__':
    raw_arr = np.array(raw)
    if np.all(raw_arr == 0):
        print("ERROR: Please fill in your measured raw values in the 'raw' list.")
        print("Edit this script and replace the [0,0,0] entries.")
        sys.exit(1)

    print("MPU6050 6-Point Accel Calibration")
    print("=================================")
    print()
    calibrate(raw, expected)
