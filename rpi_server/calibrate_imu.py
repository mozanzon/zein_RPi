#!/usr/bin/env python3
"""
calibrate_imu.py — Hard-iron magnetometer calibration tool.

Runs on the Raspberry Pi inside the project venv.
Communicates with the Arduino over Serial to collect raw magnetometer
readings, then computes hard-iron offsets (center of the sphere).

Usage:
    cd rpi_server
    source venv/bin/activate
    python calibrate_imu.py            # uses /dev/ttyACM0
    python calibrate_imu.py /dev/ttyUSB0   # specify a different port

Procedure:
    1. Run this script.
    2. When prompted, slowly rotate the robot/sensor 360° in all
       orientations (yaw + tilt in all directions, like a figure-8).
    3. The script collects data for 30 seconds, then computes and
       prints the hard-iron offsets.
    4. Copy the printed values into imu_telemetry.ino:
         const float MAG_OFFSET_X = <value>;
         const float MAG_OFFSET_Y = <value>;
         const float MAG_OFFSET_Z = <value>;
    5. Re-upload the sketch to the Arduino.

Output:
    - Prints offsets to the terminal.
    - Saves full results to calibration.json in this directory.
"""

import sys
import json
import time
import serial

# ── Configuration ──────────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
COLLECTION_DURATION = 30  # seconds


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    print("=" * 60)
    print("  IMU Hard-Iron Magnetometer Calibration")
    print("=" * 60)
    print(f"  Serial port : {port}")
    print(f"  Baud rate   : {BAUD_RATE}")
    print(f"  Duration    : {COLLECTION_DURATION}s")
    print("=" * 60)
    print()

    # ── Open Serial connection ─────────────────────────────────────
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {port}: {e}")
        sys.exit(1)

    print(f"[INFO] Connected to {port}. Waiting 2s for Arduino to settle...")
    time.sleep(2)

    # Flush any boot-up / calibration messages from the Arduino
    ser.reset_input_buffer()

    # ── Send 'c' to put Arduino into calibration mode ──────────────
    print("[INFO] Sending 'c' to enter calibration mode...")
    ser.write(b'c')
    time.sleep(0.5)
    ser.reset_input_buffer()  # Discard the status ack line

    print()
    print("  >>> SLOWLY ROTATE the robot/sensor in ALL directions <<<")
    print("  >>> (yaw full 360°, tilt forward/back/left/right)     <<<")
    print()

    # ── Collect magnetometer data ──────────────────────────────────
    samples_mx = []
    samples_my = []
    samples_mz = []

    start_time = time.time()
    last_print = start_time

    while True:
        elapsed = time.time() - start_time
        if elapsed >= COLLECTION_DURATION:
            break

        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue

        # Parse JSON lines with mx, my, mz keys
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        if "mx" in data and "my" in data and "mz" in data:
            samples_mx.append(data["mx"])
            samples_my.append(data["my"])
            samples_mz.append(data["mz"])

        # Progress update every 2 seconds
        now = time.time()
        if now - last_print >= 2.0:
            remaining = COLLECTION_DURATION - elapsed
            print(f"  Collecting... {len(samples_mx)} samples | "
                  f"{remaining:.0f}s remaining")
            last_print = now

    # ── Send 'n' to return Arduino to normal mode ──────────────────
    ser.write(b'n')
    ser.close()
    print()

    # ── Validate ───────────────────────────────────────────────────
    if len(samples_mx) < 50:
        print(f"[ERROR] Only collected {len(samples_mx)} samples. "
              "That is too few for reliable calibration.")
        print("        Make sure the Arduino is connected and the sensor")
        print("        is responding. Try again.")
        sys.exit(1)

    print(f"[INFO] Collected {len(samples_mx)} samples.")
    print()

    # ── Compute hard-iron offsets (center of min-max bounding box) ─
    # Hard-iron offset = midpoint of the min/max range on each axis.
    # This centers the magnetometer sphere at the origin.
    offset_x = (min(samples_mx) + max(samples_mx)) / 2.0
    offset_y = (min(samples_my) + max(samples_my)) / 2.0
    offset_z = (min(samples_mz) + max(samples_mz)) / 2.0

    # ── Display results ────────────────────────────────────────────
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print()
    print(f"  X range: {min(samples_mx):.4f}  to  {max(samples_mx):.4f}")
    print(f"  Y range: {min(samples_my):.4f}  to  {max(samples_my):.4f}")
    print(f"  Z range: {min(samples_mz):.4f}  to  {max(samples_mz):.4f}")
    print()
    print("  Hard-iron offsets:")
    print(f"    MAG_OFFSET_X = {offset_x:.4f}")
    print(f"    MAG_OFFSET_Y = {offset_y:.4f}")
    print(f"    MAG_OFFSET_Z = {offset_z:.4f}")
    print()
    print("  Copy these into imu_telemetry.ino and re-upload:")
    print(f"    const float MAG_OFFSET_X = {offset_x:.4f};")
    print(f"    const float MAG_OFFSET_Y = {offset_y:.4f};")
    print(f"    const float MAG_OFFSET_Z = {offset_z:.4f};")
    print()
    print("=" * 60)

    # ── Save to calibration.json ───────────────────────────────────
    result = {
        "mag_offset_x": round(offset_x, 4),
        "mag_offset_y": round(offset_y, 4),
        "mag_offset_z": round(offset_z, 4),
        "samples_collected": len(samples_mx),
        "x_range": [round(min(samples_mx), 4), round(max(samples_mx), 4)],
        "y_range": [round(min(samples_my), 4), round(max(samples_my), 4)],
        "z_range": [round(min(samples_mz), 4), round(max(samples_mz), 4)],
    }

    out_path = "calibration.json"
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)

    print(f"[INFO] Results saved to {out_path}")


if __name__ == "__main__":
    main()
