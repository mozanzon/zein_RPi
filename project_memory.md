# Project Memory: Road Inspector Bot (PID Branch)

## Overview
This repository contains the software stack for the Road Inspector Bot. 
The system uses an **Arduino** hooked up to an **MPU9250 IMU** to capture high-speed, accurate orientation data. This Arduino communicates via Serial USB to a **Raspberry Pi**, which hosts a Python WebSocket server. A standalone **Web UI** on the laptop displays the data with a clean, high-end white HUD interface.

## Tech Stack
- **Hardware**: Arduino, Raspberry Pi, MPU9250 IMU.
- **Backend (RPi)**: Python, `pyserial`, `websockets`, `asyncio`.
- **Frontend (Laptop)**: HTML5, CSS3 (Light Mode HUD), Vanilla Javascript.

## Current Architecture
1. **Arduino (`arduino/imu_telemetry/`)**: 
   - Uses the **Hideaki Tai MPU9250** library (optimized for 8-bit AVR boards like Mega 2560).
   - **Heading (Yaw)**: Complementary filter blending gyro Z integration (fast/smooth) with tilt-compensated magnetometer (absolute correction). Formula: `yaw = 0.98*(yaw+gz*dt) + 0.02*mag_yaw`. Handles 0°/360° wraparound correctly.
   - **Pitch & Roll**: Computed via the **Madgwick filter** for smooth, responsive values.
   - **Hard-iron offsets**: `MAG_OFFSET_X/Y/Z` constants at top of .ino — fill in after running `calibrate_imu.py`.
   - **Calibration mode**: Send `'c'` over Serial → Arduino outputs `{"mx":..,"my":..,"mz":..}` for 30s. Send `'n'` to exit early.
   - Outputs `{"yaw": 123.45, "pitch": ..., "roll": ...}` over Serial @ 115200 baud (yaw = compass heading 0-360°).
2. **RPi Server (`rpi_server/`)**: 
   - `server.py` — reads Serial data, runs WebSocket server (port 8765), broadcasts JSON. **Not modified.**
   - `calibrate_imu.py` — standalone script to collect raw mag data and compute hard-iron offsets. Saves to `calibration.json`.
   - Uses a virtual environment: `python3 -m venv venv`, `source venv/bin/activate`, `pip install -r requirements.txt`.
3. **Web Client (`client/`)**: 
   - Client manually enters the RPi's IP address. 
   - UI connects via WebSockets and manipulates SVG/CSS transforms to simulate a robot HUD (white theme).
   - Displays 16-point cardinal direction (N, NNE, NE, ENE, E, ...) derived from the 0-360° heading.

## Project Progress
- [x] Wiped previous stale branch state.
- [x] Initialized architecture and pushed to `PID` branch.
- [x] Integrated MPU9250 telemetry stack (Arduino -> DB/Server -> UI).
- [x] Implemented complementary filter for yaw (gyro + tilt-compensated mag).
- [x] Added calibration mode ('c' command) and calibrate_imu.py script.

## Future Capabilities
- Add dual-mode USB Web Serial / Network connectivity.
- Add camera feed overlay underneath the HUD.
- Control motor speeds via PID.
