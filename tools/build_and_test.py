#!/usr/bin/env python3
"""
IMU build & test automation
============================
Modifies main.c TEST_MODE, rebuilds, flashes, captures, and analyzes.

Usage:
  # Full performance profiling pipeline
  python tools/build_and_test.py COM11 --mode perf --duration 30

  # Static drift test (1 hour)
  python tools/build_and_test.py COM11 --mode drift --duration 3600

  # DSP comparison (requires CONFIG_HPM_MATH_DSP=1 in CMakeLists.txt)
  python tools/build_and_test.py COM11 --mode dsp --duration 60

  # Saturation sweep
  python tools/build_and_test.py COM11 --mode sat --duration 60

Prerequisites:
  - HPM SDK environment (start_cmd.cmd)
  - OpenOCD with FT2232 probe
  - ninja build tool
"""

import os
import sys
import subprocess
import argparse
import time
import shutil

HPM_SDK_BASE = os.environ.get("HPM_SDK_BASE", r"D:\sdk_env_v1.12.1\hpm_sdk")
PROJ_DIR     = os.path.join(HPM_SDK_BASE, "imu_project")
APP_DIR      = os.path.join(PROJ_DIR, "user_app")
TOOLS_DIR    = os.path.join(PROJ_DIR, "tools")
MAIN_C       = os.path.join(APP_DIR, "src", "main.c")
BUILD_DIR    = os.path.join(HPM_SDK_BASE.replace("hpm_sdk", "hpm_prj"),
                            "imu_app_hpm5300evk_flash_xip_debug")
ELF          = os.path.join(BUILD_DIR, "demo.elf")
OPENOCD      = os.path.join(HPM_SDK_BASE, "..", "tools", "openocd", "openocd.exe")
OPENOCD_CFG  = os.path.join(HPM_SDK_BASE, "boards", "openocd")

# Map test mode names to TEST_MODE values
MODE_MAP = {
    "normal": 0,
    "perf":   1,
    "drift":  2,
    "dsp":    3,
    "sat":    4,
}


def set_test_mode(mode_name):
    """Modify main.c TEST_MODE #define."""
    mode_val = MODE_MAP[mode_name]
    with open(MAIN_C, "r") as f:
        content = f.read()

    import re
    new_content = re.sub(
        r'#define TEST_MODE\s+\d+',
        f'#define TEST_MODE  {mode_val}',
        content
    )

    with open(MAIN_C, "w") as f:
        f.write(new_content)
    print(f"  Set TEST_MODE = {mode_val} ({mode_name})")


def run_cmd(cmd, cwd=None, env=None, check=True):
    """Run command, print output."""
    print(f"  $ {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, env=env or os.environ,
                          capture_output=True, text=True, shell=True)
    if result.stdout:
        for line in result.stdout.splitlines():
            if line.strip():
                print(f"    {line}")
    if result.returncode != 0:
        if result.stderr:
            for line in result.stderr.splitlines():
                if line.strip():
                    print(f"    [ERR] {line}", file=sys.stderr)
        if check:
            raise RuntimeError(f"Command failed: {' '.join(cmd)}")
    return result


def build():
    """Generate project and build with ninja."""
    print("\n── Generating project...")
    gen_cmd = [
        "start_gui.exe",
    ]
    # Actually, we need to use the SDK's generate_project mechanism
    # For now, just build if build dir exists

    if not os.path.isdir(BUILD_DIR):
        print(f"  Build dir not found: {BUILD_DIR}")
        print(f"  Run generate_project first (via start_gui.exe)")
        return False

    print(f"\n── Building in {BUILD_DIR}...")
    run_cmd(["ninja"], cwd=BUILD_DIR)
    print(f"  Build OK")
    return True


def flash():
    """Flash via OpenOCD + GDB."""
    print(f"\n── Flashing {ELF}...")

    if not os.path.isfile(ELF):
        print(f"  ELF not found: {ELF}")
        return False

    # OpenOCD flash command
    cmd = [
        OPENOCD,
        "-c", f"set HPM_SDK_BASE {HPM_SDK_BASE}; set BOARD hpm5300evk; set PROBE ft2232;",
        "-f", os.path.join(OPENOCD_CFG, "hpm5300_all_in_one.cfg"),
        "-c", f"program {ELF} verify reset exit",
    ]
    run_cmd(cmd, cwd=OPENOCD_CFG)
    print(f"  Flash OK")
    time.sleep(2)  # Wait for reset
    return True


def capture_and_analyze(port, mode, duration, output_dir):
    """Run perf_test.py capture + analysis."""
    output_file = os.path.join(output_dir, f"{mode}_{int(time.time())}.csv")

    print(f"\n── Capturing {mode} data from {port} for {duration}s...")
    cmd = [
        sys.executable,
        os.path.join(TOOLS_DIR, "perf_test.py"),
        port,
        "--mode", mode if mode in ("perf", "drift", "dsp", "sat") else "capture",
        "--duration", str(duration),
        "--output", output_file,
    ]
    result = subprocess.run(cmd, capture_output=False)
    print(f"\n  Data saved to: {output_file}")

    # Analyze
    print(f"\n── Analyzing...")
    cmd2 = [
        sys.executable,
        os.path.join(TOOLS_DIR, "perf_test.py"),
        port,
        "--mode", mode if mode in ("perf", "drift", "dsp", "sat") else "capture",
        "--input", output_file,
    ]
    subprocess.run(cmd2, capture_output=False)

    return output_file


def main():
    parser = argparse.ArgumentParser(description="IMU build & test automation")
    parser.add_argument("port", help="Serial port (e.g. COM11)")
    parser.add_argument("--mode", default="perf",
                        choices=["normal", "perf", "drift", "dsp", "sat"],
                        help="Test mode")
    parser.add_argument("--duration", type=int, default=30,
                        help="Capture duration (s)")
    parser.add_argument("--skip-build", action="store_true",
                        help="Skip build (use existing binary)")
    parser.add_argument("--skip-flash", action="store_true",
                        help="Skip flash (use existing firmware)")
    parser.add_argument("--output-dir", default=os.path.join(PROJ_DIR, "test_results"),
                        help="Output directory for test data")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    if not args.skip_build:
        set_test_mode(args.mode)
        if not build():
            print("Build failed. Abort.", file=sys.stderr)
            return 1

    if not args.skip_flash:
        if not flash():
            print("Flash failed. Abort.", file=sys.stderr)
            return 1

    output = capture_and_analyze(args.port, args.mode, args.duration, args.output_dir)

    print(f"\n{'='*60}")
    print(f"  Test complete: {args.mode}")
    print(f"  Output: {output}")
    print(f"{'='*60}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
