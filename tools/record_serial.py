#!/usr/bin/env python3
"""
Serial data recorder for IMU RAW_DATA (MODE 6).
Supports short captures and multi-hour Allan variance recordings.

Usage:
    python record_serial.py COM11 static_60s.csv --duration 60
    python record_serial.py COM11 allan_3h.csv --hours 3
    python record_serial.py COM11 allan_6h.csv --hours 6
"""

import sys
import os
import argparse
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def fmt_time(seconds):
    """Format seconds as h:mm:ss."""
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    if h > 0:
        return f"{h}:{m:02d}:{s:02d}"
    return f"{m}:{s:02d}"


def fmt_size(n_lines):
    """Estimate file size from line count."""
    mb = n_lines * 55 / 1e6  # ~55 bytes per line
    if mb > 1000:
        return f"{mb/1000:.1f} GB"
    return f"{mb:.1f} MB"


def main():
    parser = argparse.ArgumentParser(description="Record IMU RAW_DATA from serial")
    parser.add_argument("port", help="Serial port (e.g. COM11)")
    parser.add_argument("output", help="Output CSV file path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--duration", type=int, default=0,
                        help="Record duration in seconds")
    parser.add_argument("--hours", type=float, default=0,
                        help="Record duration in hours (overrides --duration)")
    parser.add_argument("--flush-interval", type=int, default=5000,
                        help="Flush to disk every N lines (default 5000)")
    args = parser.parse_args()

    if args.hours > 0:
        duration = args.hours * 3600.0
    elif args.duration > 0:
        duration = float(args.duration)
    else:
        duration = 60.0

    dur_str = fmt_time(duration)
    print(f"Opening {args.port} @ {args.baud} for {dur_str} ...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    ser.reset_input_buffer()
    print("Waiting for data stream...")
    time.sleep(1)

    lines_written = 0
    start = time.time()
    out_dir = os.path.dirname(os.path.abspath(args.output))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    last_flush = 0

    with open(args.output, "w", buffering=8192) as f:
        while time.time() - start < duration:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as e:
                print(f"\n[ERR] Serial error: {e}")
                break
            if not line:
                continue
            f.write(line + "\n")
            lines_written += 1

            # Periodic flush for crash safety
            if lines_written - last_flush >= args.flush_interval:
                f.flush()
                last_flush = lines_written

            elapsed = time.time() - start
            if lines_written % 1000 == 0:
                rate = lines_written / elapsed if elapsed > 0 else 0
                remaining = duration - elapsed
                size = fmt_size(lines_written)
                print(f"\r  [{fmt_time(elapsed)}/{dur_str}]  "
                      f"{lines_written} lines | ~{rate:.0f} Hz | ~{size}  "
                      f"remain {fmt_time(remaining)}  ", end="")

    ser.close()
    elapsed = time.time() - start
    rate = lines_written / elapsed if elapsed > 0 else 0
    size = fmt_size(lines_written)
    print(f"\rDone: {lines_written} lines in {fmt_time(elapsed)} "
          f"(~{rate:.0f} Hz, ~{size}) → {args.output}")


if __name__ == "__main__":
    main()
