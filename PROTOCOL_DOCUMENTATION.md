# Robot Bridge Communication Protocol

## Overview

The Robot Bridge uses a WebSocket-based protocol for real-time communication between the React app and Arduino through a Raspberry Pi 5 bridge.

```
React App <--WebSocket--> Raspberry Pi Bridge <--Serial--> Arduino
                                    |
                                    +-- YOLO AI Model
                                    +-- Camera Capture
```

## Serial Protocol (Arduino ↔ Raspberry Pi)

### Command Format

Commands sent from bridge to Arduino use simple text format:

```
COMMAND [parameters]\n
```

### Available Commands

#### Motor Control

```
MOTOR L <speed> R <speed>
  - Left speed: -255 to 255 (negative = backward)
  - Right speed: -255 to 255
  - Example: MOTOR L 200 R 150

FORWARD <speed>
  - Speed: 0-255
  - Example: FORWARD 200

BACKWARD <speed>
  - Speed: 0-255
  - Example: BACKWARD 150

LEFT <speed>
  - Rotation speed: 0-255
  - Example: LEFT 180

RIGHT <speed>
  - Rotation speed: 0-255
  - Example: RIGHT 180

STOP
  - Stops all motors
```

#### Status Requests

```
STATUS
  - Requests current status
  
S
  - Short form of STOP
```

### Arduino Response Format

All responses from Arduino follow this pattern:

#### Data Messages

**Compass Data:**
```
DATA:COMPASS|x=<value>|y=<value>|z=<value>|heading=<value>
  - x, y, z: Magnetic field values (float)
  - heading: Calculated heading in degrees (float)
  - Sent every 100ms (10Hz)
  - Example: DATA:COMPASS|x=12.34|y=-5.67|z=89.01|heading=156.89
```

**Encoder Data:**
```
DATA:ENCODER|e1=<count>|e2=<count>|lspeed=<speed>|rspeed=<speed>|ldir=<dir>|rdir=<dir>
  - e1, e2: Encoder pulse counts (int)
  - lspeed, rspeed: Motor speeds (0-255)
  - ldir, rdir: Motor direction (-1, 0, or 1)
  - Sent every 50ms (20Hz)
  - Example: DATA:ENCODER|e1=1234|e2=1289|lspeed=200|rspeed=180|ldir=1|rdir=1
```

#### Acknowledgment Messages

```
ACK:<COMMAND>|[parameters]
  - Example: ACK:MOTOR|L=200|R=150
```

#### Status Messages

```
STATUS:|lspeed=<speed>|rspeed=<speed>|e1=<count>|e2=<count>
  - Example: STATUS:|lspeed=200|rspeed=150|e1=5000|e2=4890
```

#### Error Messages

```
ERROR:<message>
  - Example: ERROR:Unknown_command
```

#### System Messages

```
READY:Arduino_initialized
  - Sent on startup
  
ERROR:Compass_not_found
  - Sent if HMC5883 compass fails to initialize
```

---

## WebSocket Protocol (React ↔ Raspberry Pi Bridge)

### Connection

```
ws://[host]:[port]
  - Default: ws://localhost:8765
  - Message format: JSON
```

### Client → Server Messages

#### Motor Commands

```json
{
  "type": "motor",
  "left": -255,
  "right": 255
}
```

#### Movement Commands

```json
{
  "type": "movement",
  "action": "forward",
  "speed": 200
}
```

Supported actions: `"forward"`, `"backward"`, `"left"`, `"right"`

#### Stop Command

```json
{
  "type": "stop"
}
```

#### Status Request

```json
{
  "type": "status"
}
```

### Server → Client Messages

#### Data Stream

Sent continuously (~20 FPS):

```json
{
  "timestamp": "2024-01-15T10:30:45.123Z",
  "arduino": {
    "type": "compass",
    "x": 12.34,
    "y": -5.67,
    "z": 89.01,
    "heading": 156.89
  },
  "detections": [
    {
      "class": 0,
      "class_name": "person",
      "confidence": 0.95,
      "bbox": [100, 50, 200, 300]
    }
  ],
  "frame": "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==",
  "inference_time_ms": 45.23,
  "stats": {
    "connected_clients": 1,
    "loop_fps": 19.8,
    "inference_fps": 22.1
  }
}
```

#### Connection Events

These are not sent as messages but trigger event listeners:

```
- "connected": Connection established
- "disconnected": Connection lost
- "error": Error occurred
```

---

## Data Types Reference

### ArduinoData

```typescript
interface ArduinoData {
  type: 'compass' | 'encoder' | 'ack' | 'error' | 'status';
  
  // Compass fields
  x?: number;        // X magnetic field
  y?: number;        // Y magnetic field
  z?: number;        // Z magnetic field
  heading?: number;  // Calculated heading (0-360°)
  
  // Encoder fields
  e1?: number;       // Encoder 1 count
  e2?: number;       // Encoder 2 count
  lspeed?: number;   // Left motor speed
  rspeed?: number;   // Right motor speed
  ldir?: number;     // Left motor direction (-1, 0, 1)
  rdir?: number;     // Right motor direction (-1, 0, 1)
  
  // General fields
  message?: string;  // Status/error message
}
```

### Detection

```typescript
interface Detection {
  class: number;              // Class index (0-based)
  class_name: string;         // Human readable class name
  confidence: number;         // 0.0 to 1.0
  bbox: [number, number, number, number];  // [x1, y1, x2, y2]
}
```

### BridgeStats

```typescript
interface BridgeStats {
  connected_clients: number;  // Number of WebSocket clients
  loop_fps: number;          // Main loop FPS
  inference_fps: number;     // AI inference FPS
}
```

---

## Timing & Frequencies

| Component | Frequency | Interval | Purpose |
|-----------|-----------|----------|---------|
| Compass | 10 Hz | 100 ms | IMU updates |
| Encoder | 20 Hz | 50 ms | Odometry tracking |
| WebSocket Broadcast | 20 Hz | 50 ms | Real-time data to clients |
| Camera Capture | 30 Hz | 33 ms | Continuous capture |
| YOLO Inference | ~15-25 Hz | Variable | AI detection (depends on model) |

---

## Error Handling

### Arduino Errors

```
ERROR:Compass_not_found
  - Action: Disable compass data, continue other operations

ERROR:Unknown_command
  - Action: Log error, resend valid command

ERROR:Invalid_speed
  - Action: Clamp speed to valid range
```

### WebSocket Errors

```
Connection Timeout
  - Action: Auto-reconnect with exponential backoff
  - Max attempts: 10
  - Initial delay: 3 seconds

Invalid JSON
  - Action: Log warning, ignore message

Sensor Disconnected
  - Action: Notify UI, continue operating without sensor
```

---

## Example Communication Flow

### Start-up Sequence

```
1. React App connects via WebSocket
2. Bridge accepts connection
3. Bridge sends status: "READY"
4. Bridge starts streaming sensor data
5. React App ready to send commands
```

### Command Execution

```
React: {"type": "movement", "action": "forward", "speed": 200}
  ↓
Bridge: Sends "FORWARD 200" to Arduino
  ↓
Arduino: Acknowledges "ACK:FORWARD|speed=200"
  ↓
Arduino: Starts motor, begins streaming encoder data
  ↓
Bridge: Receives encoder data, forwards via WebSocket
  ↓
React: Updates UI with motor status
```

### Data Streaming

```
Every 50ms:
  Arduino → Bridge: "DATA:COMPASS|x=12.34|..."
  Arduino → Bridge: "DATA:ENCODER|e1=1234|..."
  Bridge → React: 
    {
      "arduino": {compass data OR encoder data},
      "detections": [{...}, ...],
      "frame": "base64_image",
      "stats": {...}
    }
```

---

## Performance Considerations

### Bandwidth

- **Sensor Data Only**: ~1-5 KB/s
- **With Frame Data (JPEG, 70% quality)**: ~200-500 KB/s
- **Full Stream (Detections + Frame)**: ~300-600 KB/s

### Latency

- Serial (Arduino → Bridge): ~1-2 ms
- WebSocket (Bridge → React): ~5-20 ms
- Total end-to-end: ~10-30 ms

### Optimization Tips

1. **Reduce Frame Frequency**: Lower FPS = lower bandwidth
2. **JPEG Quality**: Lower quality = smaller size
3. **Model Size**: Use `yolov8n.pt` for faster inference
4. **Skip Frames**: Process only every Nth frame
5. **Compress Detections**: Only send relevant detections

---

## Testing

### Arduino Serial Test

```bash
# Listen to Arduino output
screen /dev/ttyUSB0 115200

# Send command
# Ctrl+A, Shift+X to exit
```

### WebSocket Test (JavaScript)

```javascript
const ws = new WebSocket('ws://localhost:8765');

ws.onopen = () => {
  console.log('Connected');
  ws.send(JSON.stringify({
    type: 'movement',
    action: 'forward',
    speed: 200
  }));
};

ws.onmessage = (event) => {
  console.log('Received:', JSON.parse(event.data));
};
```

### Bridge Logs

```bash
# Monitor logs in real-time
tail -f robot_bridge.log

# Filter specific messages
grep "DATA:ENCODER" robot_bridge.log
```

---

## Future Extensions

### Possible Protocol Additions

1. **Autonomous Navigation**
   - Waypoint commands
   - Path planning

2. **Advanced Sensor Data**
   - Temperature
   - Distance sensors
   - IMU acceleration/gyro

3. **Recording & Playback**
   - Save sensor data
   - Replay missions

4. **Multi-Robot Support**
   - Robot ID in messages
   - Swarm coordination
