#!/usr/bin/env python3
"""
Speed calibration test script for Embedded_Platform_DaDas.
Runs on Raspberry Pi 5 (with GUI); connects to STM32 via serial. Enables
encoder-only output (#calibOutput:1;;) so the STM32 sends short @enc:... frames
instead of full @imu. Sends VCD commands (speed, 0 steer, duration), collects
@enc for speed_ema and totalTick, prompts for real distance, computes empirical
scale, and displays per-run plots on the monitor (and saves them as PNG).
Disables encoder-only mode (#calibOutput:0;;) at end.
"""

import argparse
import math
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)
try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Install matplotlib: pip install matplotlib", file=sys.stderr)
    sys.exit(1)

# Protocol
BAUD = 115200
MSG_END = ";;\r\n"
ENCODER_MM_PER_TICK = (math.pi * 62.0) / 32768.0   # same as firmware (wheel D=62mm, 32768 ticks/rev)
EPSILON_MM = 1.0   # minimum encoder_mm to include run in overall scale
EMA_VS_COMMANDED_TOLERANCE_MM_S = 20.0   # flag if mean EMA differs from commanded by more than this


def send_command(ser, key, payload):
    """Send #key:payload;;\\r\\n and return True. Does not wait for response."""
    msg = f"#{key}:{payload}{MSG_END}"
    ser.write(msg.encode("ascii"))
    return True


def read_line(ser, timeout_s=0.5):
    """Read one line (ending with \\n). Returns str or None on timeout."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        if ser.in_waiting:
            b = ser.read(1)
            if b == b"\n":
                return (buf + b).decode("ascii", errors="replace").strip()
            buf += b
        else:
            time.sleep(0.01)
    return None


def parse_enc_line(line):
    """
    Parse @enc:... line (encoder-only, used during calibration test).
    Format: @enc:speed_ema;totalTick;md;agc;;\\r\\n
    Returns (speed_ema, total_tick) or None.
    """
    if not line.startswith("@enc:"):
        return None
    rest = line[5:].rstrip("\r\n")
    if rest.endswith(";;"):
        rest = rest[:-2]
    parts = rest.split(";")
    if len(parts) < 4:
        return None
    try:
        speed_ema = int(parts[0])
        total_tick = int(parts[1])
        return (speed_ema, total_tick)
    except (ValueError, IndexError):
        return None


def run_one_test(ser, speed_mm_s, duration_sec, run_index, out_dir, timeout_extra_s=5):
    """
    Send one VCD (speed, 0 steer, duration), collect @imu until @vcd:0;0;0;;,
    then prompt for real distance. Returns dict with all run results for this run,
    or None if aborted/timeout.
    """
    time_ds = int(round(duration_sec * 10))
    send_command(ser, "vcd", f"{speed_mm_s};0;{time_ds}")
    deadline = time.monotonic() + duration_sec + timeout_extra_s
    samples = []   # list of (index, speed_ema, total_tick)
    start_tick = None
    end_tick = None
    run_ended = False
    sample_idx = 0

    while time.monotonic() < deadline:
        line = read_line(ser, timeout_s=1.0)
        if line is None:
            continue
        if "@vcd:0;0;0;;" in line or line.strip() == "@vcd:0;0;0;;":
            run_ended = True
            break
        parsed = parse_enc_line(line)
        if parsed is not None:
            speed_ema, total_tick = parsed
            if start_tick is None:
                start_tick = total_tick
            end_tick = total_tick
            samples.append((sample_idx, speed_ema, total_tick))
            sample_idx += 1

    if not run_ended:
        print("  [Timeout waiting for @vcd:0;0;0;;]", file=sys.stderr)
        return None
    if start_tick is None or end_tick is None or len(samples) == 0:
        print("  [No @enc samples collected]", file=sys.stderr)
        return None

    delta_ticks = end_tick - start_tick
    encoder_mm = delta_ticks * ENCODER_MM_PER_TICK
    mean_ema = sum(s[1] for s in samples) / len(samples) if samples else 0
    median_ema = sorted(s[1] for s in samples)[len(samples) // 2] if samples else 0

    # Prompt for real distance
    while True:
        try:
            inp = input(f"  Enter real distance (mm) for this run (or 'skip'): ").strip()
            if inp.lower() == "skip":
                real_mm = None
                break
            real_mm = float(inp)
            if real_mm < 0:
                print("  Enter a non-negative number or 'skip'.", file=sys.stderr)
                continue
            break
        except EOFError:
            real_mm = None
            break
        except ValueError:
            print("  Enter a number (mm) or 'skip'.", file=sys.stderr)

    scale_i = (real_mm / encoder_mm) if (real_mm is not None and encoder_mm >= EPSILON_MM) else None

    # Validation
    ema_warn = ""
    if abs(mean_ema - speed_mm_s) > EMA_VS_COMMANDED_TOLERANCE_MM_S:
        ema_warn = f"  [Warning: mean EMA {mean_ema:.0f} mm/s vs commanded {speed_mm_s} mm/s]"

    result = {
        "run_index": run_index,
        "speed_mm_s": speed_mm_s,
        "duration_sec": duration_sec,
        "start_tick": start_tick,
        "end_tick": end_tick,
        "delta_ticks": delta_ticks,
        "encoder_mm": encoder_mm,
        "real_mm": real_mm,
        "scale_i": scale_i,
        "mean_ema": mean_ema,
        "median_ema": median_ema,
        "samples": samples,
        "ema_warn": ema_warn,
    }

    # Per-run plot
    if out_dir and samples:
        _save_run_plot(result, out_dir)

    return result


def _save_run_plot(result, out_dir):
    """Save one PNG and display on the Pi's monitor: subplot 1 EMA vs time, subplot 2 encoder vs real distance."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    r = result
    fname = f"calib_run_{r['run_index']:03d}_{r['speed_mm_s']}mms_{r['duration_sec']}s.png"
    path = out_dir / fname

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    # EMA speed vs time (sample index; period ~150ms)
    indices = [s[0] for s in r["samples"]]
    ema_vals = [s[1] for s in r["samples"]]
    ax1.plot(indices, ema_vals, label="EMA speed (mm/s)")
    ax1.axhline(y=r["speed_mm_s"], color="gray", linestyle="--", label=f"Commanded {r['speed_mm_s']} mm/s")
    ax1.set_xlabel("Sample index")
    ax1.set_ylabel("Speed (mm/s)")
    ax1.set_title(f"Run {r['run_index']}: EMA speed vs time")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Distance comparison
    ax2.bar([0], [r["encoder_mm"]], width=0.35, label="Encoder (mm)")
    if r["real_mm"] is not None:
        ax2.bar([0.4], [r["real_mm"]], width=0.35, label="Real (mm)")
    ax2.set_xticks([0.175] if r["real_mm"] is not None else [0])
    ax2.set_xticklabels(["Encoder vs Real"] if r["real_mm"] is not None else ["Encoder"])
    ax2.set_ylabel("Distance (mm)")
    ax2.set_title(f"Run {r['run_index']}: Distance (scale_i = {r['scale_i']:.4f})" if r["scale_i"] is not None else f"Run {r['run_index']}: Distance (skipped)")
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plt.savefig(path, dpi=100)
    plt.show()
    plt.close()
    print(f"  Saved plot: {path}")


def compute_overall_scale(results):
    """Ratio of totals over valid runs. Returns (scale, n_valid)."""
    valid = [r for r in results if r.get("real_mm") is not None and r.get("encoder_mm", 0) >= EPSILON_MM]
    if not valid:
        return None, 0
    total_real = sum(r["real_mm"] for r in valid)
    total_encoder = sum(r["encoder_mm"] for r in valid)
    if total_encoder <= 0:
        return None, 0
    return total_real / total_encoder, len(valid)


def main():
    parser = argparse.ArgumentParser(description="Speed calibration: run VCD tests, collect encoder data, compute scale.")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (e.g. /dev/ttyAMA0, /dev/serial0)")
    parser.add_argument("--baud", type=int, default=BAUD, help="Baud rate")
    parser.add_argument("--out", default="calib_results", help="Output directory for plots and log")
    parser.add_argument("--tests", default="200,10 300,8 100,15", help="Space-separated speed_mm_s,duration_sec (e.g. '200,10 300,8')")
    parser.add_argument("--no-kl", action="store_true", help="Skip sending #kl:30;; (e.g. if already at KL 30)")
    parser.add_argument("--no-push", action="store_true", help="Do not send #speedCalib to STM32 at the end")
    args = parser.parse_args()

    # Parse test list
    tests = []
    for part in args.tests.split():
        part = part.strip()
        if not part:
            continue
        if "," in part:
            a, b = part.split(",", 1)
            try:
                tests.append((int(a), float(b)))
            except ValueError:
                print(f"Invalid test '{part}'; use speed_mm_s,duration_sec", file=sys.stderr)
                sys.exit(1)
        else:
            print(f"Invalid test '{part}'; use speed_mm_s,duration_sec", file=sys.stderr)
            sys.exit(1)
    if not tests:
        tests = [(200, 10), (300, 8), (100, 15)]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} at {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Serial open failed: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # Drain and sync
        time.sleep(0.3)
        while ser.in_waiting:
            ser.read(ser.in_waiting)
            time.sleep(0.05)

        if not args.no_kl:
            send_command(ser, "kl", "30")
            time.sleep(0.2)
            while True:
                line = read_line(ser, timeout_s=2.0)
                if line and ("@kl:30" in line or "kl 30" in line.lower() or "30" in line):
                    break
                if line:
                    print(f"  [{line}]")
            print("KL 30 set.")

        # Use encoder-only output for calibration (short @enc:... instead of full @imu)
        send_command(ser, "calibOutput", "1")
        time.sleep(0.15)

        results = []
        for i, (speed_mm_s, duration_sec) in enumerate(tests):
            print(f"\n--- Run {i + 1}/{len(tests)}: speed={speed_mm_s} mm/s, duration={duration_sec} s ---")
            r = run_one_test(ser, speed_mm_s, duration_sec, i + 1, args.out)
            if r is not None:
                results.append(r)
                if r.get("ema_warn"):
                    print(r["ema_warn"])
                enc = r["encoder_mm"]
                real = r["real_mm"]
                scale = r.get("scale_i")
                print(f"  Encoder distance: {enc:.1f} mm  Real: {real} mm  Scale_i: {scale}")
            else:
                print("  Run skipped or failed.")

        # Summary table
        print("\n" + "=" * 80)
        print("Summary")
        print("=" * 80)
        print(f"{'Run':>4} {'Speed':>6} {'Dur':>5} {'Encoder_mm':>10} {'Real_mm':>10} {'Scale_i':>10} {'Mean_EMA':>8}")
        print("-" * 80)
        for r in results:
            real_s = str(r["real_mm"]) if r["real_mm"] is not None else "skip"
            scale_s = f"{r['scale_i']:.4f}" if r.get("scale_i") is not None else "-"
            print(f"{r['run_index']:>4} {r['speed_mm_s']:>6} {r['duration_sec']:>5.1f} {r['encoder_mm']:>10.1f} {real_s:>10} {scale_s:>10} {r['mean_ema']:>8.1f}")

        scale_overall, n_valid = compute_overall_scale(results)
        if scale_overall is not None:
            print("-" * 80)
            print(f"Overall scale (ratio of totals, n={n_valid}): {scale_overall:.6f}")
            if not args.no_push:
                send_command(ser, "speedCalib", f"{scale_overall:.6f}")
                print("Sent #speedCalib to STM32 (if firmware supports it).")
        else:
            print("No valid runs for overall scale.")

    finally:
        try:
            send_command(ser, "calibOutput", "0")
        except Exception:
            pass
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
