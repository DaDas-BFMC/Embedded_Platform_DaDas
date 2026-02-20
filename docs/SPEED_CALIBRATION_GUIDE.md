# Speed calibration guide

This guide explains how to run the **speed calibration test** on the car: the script drives the STM32 over serial, runs the vehicle at known speed and duration, and uses measured real distance to compute an empirical scale factor for the encoder-based speed.

## Purpose

- Run the car at a **known speed** for a **known time** (no steering).
- Measure **real distance** travelled (e.g. with a tape or marked track).
- Compare with **encoder-derived distance** from the AS5600 (reported in **encoder-only** `@enc` frames during the test).
- Compute a **scale factor** so that `real_distance = scale × encoder_distance`. The same scale corrects reported speed.
- Optionally **send the scale to the STM32** via `#speedCalib:...;;` so the firmware applies it to future speed/distance values.

**Encoder-only mode for this test:** The script sends **`#calibOutput:1;;`** before the runs so the STM32 sends short **`@enc:speed_ema;totalTick;md;agc;;`** lines instead of the full `@imu` frame. This reduces serial load and latency during calibration. At the end it sends **`#calibOutput:0;;`** to restore normal `@imu` output.

## Where it runs

The script runs **on the Raspberry Pi 5**. The STM32 is connected to the Pi via **serial (UART)**. The script opens the Pi’s serial port to send commands and receive **`@enc`** (during calibration) and **`@vcd`** messages.

**Typical serial ports on the Pi:**

- `/dev/ttyAMA0` or `/dev/serial0` – hardware UART (often used for the car).
- `/dev/ttyUSB0` – if you use a USB–serial adapter.

Confirm the port (e.g. `ls /dev/serial*` or check which device appears when the car is connected).

## Prerequisites

- **Python 3**
- **pyserial:** `pip install pyserial`
- **matplotlib** (for plots): `pip install matplotlib`

On the Pi you can use:

```bash
pip install pyserial matplotlib
```

or install via your system package manager if you prefer.

## Hardware setup

- Car on a **flat, straight path** with enough space for the longest test run.
- A way to **measure real distance** (tape measure, marked track, etc.).
- STM32 **powered** and **connected to the Pi UART**.
- The firmware must accept **KL 30** and **VCD** commands (script sends `#kl:30;;` then `#vcd:speed;0;time_ds;;`).

## How to run

From the project root (or with paths adjusted):

```bash
python3 tools/speed_calib_test.py --port /dev/ttyAMA0 --baud 115200
```

**Optional arguments:**

| Argument   | Default           | Description |
|-----------|-------------------|-------------|
| `--port`  | `/dev/ttyAMA0`    | Serial port (e.g. `/dev/serial0` on Pi). |
| `--baud`  | `115200`          | Baud rate. |
| `--out`   | `calib_results`   | Directory for per-run plots and output. |
| `--tests` | `200,10 300,8 100,15` | Space-separated list of `speed_mm_s,duration_sec` (e.g. `200,10` = 200 mm/s for 10 s). |
| `--no-kl` | (off)             | Skip sending `#kl:30;;` (use if already at KL 30). |
| `--no-push` | (off)           | Do not send `#speedCalib:...;;` to the STM32 at the end. |

**Examples:**

```bash
# Default port and test list
python3 tools/speed_calib_test.py

# Custom port and two tests only
python3 tools/speed_calib_test.py --port /dev/serial0 --tests "200,10 250,8"

# Save plots to a specific folder and do not push scale to STM32
python3 tools/speed_calib_test.py --out my_calib --no-push
```

## Test flow

1. Script opens the serial port and (unless `--no-kl`) sends **`#kl:30;;`** and waits for a response.
2. Script sends **`#calibOutput:1;;`** so the STM32 sends only **`@enc:speed_ema;totalTick;md;agc;;`** (no full IMU data).
3. For **each test** in the list (e.g. 200 mm/s for 10 s):
   - Sends **`#vcd:speed;0;time_deciseconds;;`** (e.g. `#vcd:200;0;100;;` for 10 s).
   - Collects every **`@enc`** line until it receives **`@vcd:0;0;0;;`** (run end).
   - From `@enc` lines it records **speed_ema** and **totalTick** (encoder).
   - **Prompts:** `Enter real distance (mm) for this run (or 'skip'):`
   - You measure the distance the car travelled and enter the value in **mm**, or type **skip** to exclude the run from the scale fit.
3. After each run, the script prints a short summary and saves a **per-run plot** (see below).
4. After **all runs**, it prints a **summary table** and the **overall scale** (ratio of total real distance to total encoder distance over valid runs).
5. Unless you used **`--no-push`**, it sends **`#speedCalib:<scale>;;`** to the STM32. This only has an effect if the firmware implements the `#speedCalib` command and applies the scale to encoder-based speed.
6. Script sends **`#calibOutput:0;;`** to restore full `@imu` output (and in `finally` so it runs even on error).

## Plots

- **Location:** In the directory given by `--out` (default `calib_results/`).
- **Per-run file name:** `calib_run_001_200mms_10s.png` (run index, speed, duration).
- **Content:**
  - **Top:** EMA speed (mm/s) vs sample index during the run, with a horizontal line at the commanded speed. Use this to check that the reported speed tracked the command and to spot ripple or dropouts.
  - **Bottom:** Bar comparison of encoder-derived distance vs real distance (mm) for that run, and the per-run scale.
- **Interpretation:** If **scale > 1**, the encoder is **underestimating** distance (real distance is larger than encoder distance); the firmware will multiply raw speed by this scale so that reported speed and distance match reality better.

Plots are displayed on the Pi's monitor (GUI expected) and also saved as PNG to the output directory. Close each plot window to continue to the next run.

## Result and pushing to STM32

- The script prints the **overall scale** (e.g. `1.023456`) and, unless `--no-push` is set, sends **`#speedCalib:1.023456;;`** to the STM32.
- The STM32 must have the **`#speedCalib`** command implemented and must apply the received scale to the encoder-based speed (and distance) in the IMU+encoder task. If the firmware does not support this command yet, the send has no effect; you can still use the printed scale for offline correction or later firmware update.

## Troubleshooting

| Issue | What to try |
|-------|-------------|
| **Permission denied** on serial port | Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and back in. Or run the script with appropriate permissions. |
| **Port busy** | Close any other serial monitor or terminal connected to the same port. |
| **No response / timeout** | Check wiring (TX/RX and GND between Pi and STM32). Confirm baud rate (115200). Ensure the STM32 is running the firmware that sends `@imu` and responds to `#vcd`. |
| **KL 30 required** | The script sends `#kl:30;;` by default. If you see “kl 30 is required” from the car, do not use `--no-kl` so the script sets KL 30. |
| **No @enc lines** | Ensure IMU+encoder is active (e.g. `#imu:1;;` and KL 15 or 30). The script sends `#calibOutput:1;;` so you should see `@enc:...` lines. The combined task runs at 150 ms period (~6.7 Hz). |

## Summary

- Run the script **on the Pi 5** with the car connected via serial.
- Use **`--port`** to match your UART (e.g. `/dev/ttyAMA0` or `/dev/serial0`).
- Enter **real distance in mm** after each run (or **skip**).
- Check the **plots** in the output directory and the **overall scale** in the summary.
- If the firmware supports it, the script will **push the scale** to the STM32 with `#speedCalib:...;;`.
