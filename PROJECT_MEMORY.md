# Project Memory

## Project

Arduino Mega robot controller for a differential-drive RoboScan-style robot with:

- Two BTS7960-style drive motor drivers.
- One BTS7960-style plotter/sprayer motor driver.
- QMC5883L compass/IMU heading sensor over I2C.
- Left and right quadrature wheel encoders.
- Serial Monitor command control.
- Optional Raspberry Pi or laptop bridge control from a React UI.

Current main combined sketch:

- `combined_robot_plotter/combined_robot_plotter.ino`

Bridge code in this workspace:

- `pi_fast_bridge/robot_fast_bridge.py` - Raspberry Pi bridge for wireless UI control and camera streaming.
- `pi_fast_bridge/requirements.txt`
- `local_com_bridge/local_com_bridge.py` - laptop-side COM bridge with real COM-port listing/dropdown support.
- `local_com_bridge/requirements.txt`

React UI project that has been wired to this robot protocol:

- `C:/Users/Mohamed Zein/Downloads/Uni_projects/RoboScan/road_inspector_bot/RoboScanV2`

Important edited UI files:

- `src/services/RobotBridgeService.ts`
- `src/app/context/RobotContext.tsx`
- `src/app/components/tabs/ConnectionTab.tsx`
- `src/app/components/tabs/ControlTab.tsx`

Original reference sketches:

- `serial_monitor_robot/serial_monitor_robot.ino`
- `24Battery/24Battery.ino`

## Serial Monitor

Use Arduino Serial Monitor with:

- Baud: `115200`
- Line ending: `Newline`

The sketch declares:

- `const long SERIAL_BAUD_RATE = 115200;`

On boot it prints:

```text
READY:Combined_RoboScan_controller|baud=115200
```

## Current Commands

Drive:

- `W` - drive forward.
- `X` - drive backward.
- `A` - turn left 90 degrees from current heading.
- `D` - turn right 90 degrees from current heading.
- `S` or `STOP` - stop drive and plotter.
- `SPEED <0-255>` - set forward/backward drive PWM speed.
- `TURN SPEED <0-255>` - set the maximum PWM used during 90-degree turns.

Plotter:

- `PLOT CONT` or `CONT` - arm continuous plotting while the robot is moving.
- `PLOT DASH` or `DASH` - arm dashed plotting while the robot is moving.
- `PLOT DASH DIST <dash_m> <gap_m>` - set dashed plot/gap lengths by measured encoder travel.
- `PLOT OFF` - turn plotter mode off.
- `PLOT SPEED <0-255>` - set plotter motor PWM speed.
- `PLOT DIST <meters>` - plot for a measured travel distance, then stop the plotter. `0` means unlimited.
- `PLOT TICKS <ticks>` - plot for a measured encoder tick distance, converted to meters using wheel radius. `0` means unlimited.
- `WHEEL RADIUS <meters>` - set wheel radius for encoder distance math.

Info:

- `STATUS` - print heading, encoder ticks, distance, drive speed, turn speed, plot mode, plot speed, plot target, and plot done state.
- `HELP` - print command menu.

Automatic telemetry:

- The Arduino now streams `STATUS|...` packets every `100 ms`.
- This is required for the React UI and bridge telemetry cards.
- Packet prefix is `STATUS`, not `DATA`, so bridge parsers should treat it as the live sensor packet.

Typical packet fields:

```text
STATUS|heading=...|yaw=...|e1=...|e2=...|de1=...|de2=...|left_m=...|right_m=...|speed=...|lspeed=...|rspeed=...|battery=0|drive_speed=...|turn_speed=...|drive_moving=...|wheel_radius_m=...|plot_mode=...|plot_speed=...|spraying=...|dash_m=...|gap_m=...|plot_target_m=...|plot_done=...
```

## Key Behavior Decisions

- The plotter should run only while the robot is moving.
- `PLOT CONT` and `PLOT DASH` only arm the plotter mode; they should not start the plotter motor while the robot is stopped.
- When drive motion stops, the plotter motor should stop too.
- The robot may keep moving after `PLOT DIST` or `PLOT TICKS` finishes; only the plotter turns off for that distance target.
- `A` and `D` perform automatic 90-degree turns from the current compass heading.
- Compass heading is the final authority for completing a 90-degree turn.
- Encoder ticks are used during turns to estimate turn progress, slow down near the end, and stop if the robot exceeds a safety limit.
- Turning has its own speed setting, separate from forward/backward drive speed.
- The UI should use the combined robot plotter command set because this has been the best working Arduino attempt.
- For fastest response, prefer the new fast bridge and low-res camera defaults instead of heavy Pi-side inference.
- YOLO should run on the laptop/local machine through `yolo_inference_server.py`, not on the Raspberry Pi.
- The UI sends camera frames from the bridge to the local YOLO server when YOLO is enabled.

## Calibration Values

In `combined_robot_plotter.ino`:

- `float wheelRadiusM = 0.16;`
- `const float TICKS_PER_REV = 2400.0;`
- `const float ENCODER_PPR = 600.0;`
- `const float ROBOT_TRACK_WIDTH_M = 0.50;`
- `const int DEFAULT_DRIVE_SPEED = 160;`
- `const int DEFAULT_TURN_SPEED = 90;`
- `const int MIN_TURN_PWM = 45;`
- `const int DEFAULT_PLOTTER_SPEED = 180;`

Important:

- Wheel diameter is 32 cm, so wheel radius is 16 cm / `0.16 m`.
- The 600 PPR quadrature encoder is counted on both A and B edges, so `600 * 4 = 2400 ticks/rev`.
- Wheel circumference is about `1.005 m`, so each encoder tick is about `0.419 mm`.
- `ROBOT_TRACK_WIDTH_M` must be measured and tuned. It is the center-to-center distance between the left and right wheels.
- If 90-degree turns overshoot, lower `TURN SPEED` first, then tune `ROBOT_TRACK_WIDTH_M` and `HEADING_TOLERANCE_DEG`.

## Pin Map

Left drive motor:

- `M1_RPWM = 5`
- `M1_LPWM = 6`
- `M1_R_EN = 7`
- `M1_L_EN = 8`

Right drive motor:

- `M2_RPWM = 44`
- `M2_LPWM = 45`
- `M2_R_EN = 46`
- `M2_L_EN = 47`

Plotter motor:

- `PLOTTER_RPWM = 38`
- `PLOTTER_LPWM = 39`
- `PLOTTER_R_EN = 40`
- `PLOTTER_L_EN = 41`

Encoders:

- `ENC_LEFT_A = 2`
- `ENC_LEFT_B = 18`
- `ENC_RIGHT_A = 3`
- `ENC_RIGHT_B = 19`

Compass:

- QMC5883L over I2C via `Wire`.

## Verification Note

This machine did not have `arduino-cli` available during development, so compile verification could not be run locally. Verify in Arduino IDE using board:

- Arduino Mega / Mega 2560

Required libraries:

- `QMC5883LCompass`
- `Wire` built-in library

Verified during integration:

- React production build passed with `npm run build`.
- Python syntax check passed for `pi_fast_bridge/robot_fast_bridge.py`.
- Python syntax check passed for `local_com_bridge/local_com_bridge.py`.
- Python syntax check passed for the YOLO server in the RoboScan workspace.
- Existing RoboScan bridge tests passed.

## Typical Serial Workflows

Continuous plotting for 2 meters:

```text
WHEEL RADIUS 0.16
TURN SPEED 70
SPEED 150
PLOT DIST 2.0
PLOT CONT
W
S
```

Dashed plotting while moving:

```text
PLOT DIST 0
PLOT DASH
W
S
```

Turn slowly:

```text
TURN SPEED 60
A
D
```

## UI Connection Modes

The React UI now supports three connection modes in the Connection tab:

```text
RASPBERRY PI
DIRECT COM
LOCAL COM
```

### Raspberry Pi Mode

Use for wireless control and camera streaming.

Data path:

```text
React UI -> WebSocket ws://<pi-ip>:8765 -> pi_fast_bridge -> Arduino serial
```

Run on Raspberry Pi:

```bash
cd /path/to/Akrmna_ya_rbna
pip install -r pi_fast_bridge/requirements.txt
python pi_fast_bridge/robot_fast_bridge.py --host 0.0.0.0
```

Default fast camera settings:

- `424x240`
- `12 fps`
- JPEG quality `45`

Reason:

- Lower bandwidth and CPU load keeps the stream responsive.
- Pi bridge should not run YOLO.

### Direct COM Mode

Uses browser Web Serial.

Limitations:

- Works only in Chrome or Edge.
- Requires `localhost` or HTTPS.
- Browser opens its own port picker; the app cannot force a normal COM dropdown.
- Camera streaming is not available.

Use `115200` baud to match the Arduino.

### Local COM Mode

Use this if the browser cannot show/select the Arduino COM port directly.

Data path:

```text
React UI -> ws://localhost:8787/ws -> local_com_bridge.py -> Arduino COM port
```

Run on the laptop:

```bash
cd "C:\Users\Mohamed Zein\Downloads\Uni_projects\Akrmna_ya_rbna"
pip install -r local_com_bridge\requirements.txt
python local_com_bridge\local_com_bridge.py
```

Then in the UI:

1. Open `Connection`.
2. Select `LOCAL COM`.
3. Click `REFRESH`.
4. Pick the Arduino COM port.
5. Set baud to `115200`.
6. Click `CONNECT`.

This mode gives a real COM-port dropdown by calling:

```text
GET http://localhost:8787/ports
```

and communicates through:

```text
ws://localhost:8787/ws
```

Camera streaming is not available in Local COM mode. Use Raspberry Pi mode when the camera is needed.

## YOLO

YOLO inference is intended to run on the laptop/local machine, not the Raspberry Pi.

YOLO server location:

- `C:/Users/Mohamed Zein/Downloads/Uni_projects/RoboScan/road_inspector_bot/yolo_inference_server.py`

Model files:

- `model/best.pt`
- `model/best.onnx`

The YOLO server now falls back to `model/best.onnx` if the requested `.pt` path is missing.

Run on the laptop:

```bash
cd "C:\Users\Mohamed Zein\Downloads\Uni_projects\RoboScan\road_inspector_bot"
python yolo_inference_server.py --model model/best.pt --host localhost --port 5000
```

The React UI uses:

```text
http://localhost:5000
```

for local YOLO inference.
