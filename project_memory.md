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
   - **Heading (Yaw)**: Computed via **tilt-compensated magnetometer** heading — gives absolute compass direction (N/E/S/W) that updates even when stationary. The Madgwick filter's gyro-based yaw is NOT used for heading.
   - **Pitch & Roll**: Computed via the **Madgwick filter** for smooth, responsive values.
   - Outputs `{"yaw": 123.45, "pitch": ..., "roll": ...}` over Serial @ 115200 baud (yaw = compass heading 0-360°).
2. **RPi Server (`rpi_server/`)**: 
   - Python script reads Serial data. 
   - Uses a virtual environment: `python3 -m venv venv`, `source venv/bin/activate`, `pip install -r requirements.txt`.
   - Runs a WebSocket server (port 8765) and broadcasts JSON payloads.
3. **Web Client (`client/`)**: 
   - Client manually enters the RPi's IP address. 
   - UI connects via WebSockets and manipulates SVG/CSS transforms to simulate a robot HUD (white theme).
   - Displays 16-point cardinal direction (N, NNE, NE, ENE, E, ...) derived from the 0-360° heading.

## Project Progress
- [x] Wiped previous stale branch state.
- [x] Initialized architecture and pushed to `PID` branch.
- [x] Integrated MPU9250 telemetry stack (Arduino -> DB/Server -> UI).
- [x] Switched yaw from Madgwick gyro-fusion to tilt-compensated magnetometer heading for absolute compass direction.

## Future Capabilities
- Add dual-mode USB Web Serial / Network connectivity.
- Add camera feed overlay underneath the HUD.
- Control motor speeds via PID.
