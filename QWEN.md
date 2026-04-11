# Road Inspector Bot — Project Memory

## Project Overview
Autonomous road inspection robot with real-time video streaming, IMU-based orientation sensing, and web-based remote control interface.

## Architecture
- **Arduino** (`motor_imu_controller.ino`): Motor control + MPU-9250 IMU via serial
- **Raspberry Pi** (`vision_control_server.py`): Camera streaming + Flask web server + serial bridge to Arduino

## Key Serial Commands (Pi → Arduino)
- `FORWARD <0-255>`, `BACKWARD <0-255>` — directional movement with ramping
- `SET_SPEED <0-255>` — live speed adjustment
- `TURN_LEFT_270 <speed>`, `TURN_RIGHT_180 <speed>` — compass-based turns
- `STOP`, `S` (emergency) — stopping
- `IMU_STREAM [ms]`, `IMU_STOP`, `IMU_READ` — IMU data control

## Key Serial Responses (Arduino → Pi)
- `IMU,ax,ay,az,gx,gy,gz,heading` — CSV telemetry line
- Status messages (logged, not parsed by server)

## IMU Data Flow
Arduino reads MPU-9250 → sends CSV over serial → Pi `serial_reader` thread parses `IMU,` lines → updates shared `imu_data` dict → broadcasts to SSE subscribers → web dashboard renders compass + accel chart

## Web Dashboard
- `http://<ip>:8080/` — main UI (stream + motor control + IMU panel)
- `/stream` — MJPEG video
- `/imu` — SSE IMU telemetry
- `/cmd` POST — send serial commands
- `/status` — system health

## Branches
- `main` — stable
- `feature/encoder-imu-gps-control` — active development (encoder, IMU, GPS integration)

## Hardware
- Arduino Uno + MPU-9250 (I2C) + L298N/L293D dual H-bridge + 2 DC motors
- Raspberry Pi 3B+/4 + USB camera (V4L2)
- Serial: USB @ 115200 baud, `/dev/ttyACM0`

## Notes
- Original `motor_imu_controller.ino` is manual control (no feedback loop)
- New work on `feature/encoder-imu-gps-control` adds encoder feedback + GPS
- `.github/` workflows (Qwen Code internal) are gitignored
