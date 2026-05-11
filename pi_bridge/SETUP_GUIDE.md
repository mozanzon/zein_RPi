# Raspberry Pi 5 Robot Bridge Setup Guide

## Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS (64-bit recommended)
- Python 3.9 or higher
- Arduino connected via USB serial
- USB Camera or CSI camera connected

## Installation Steps

### 1. System Dependencies

```bash
sudo apt-get update
sudo apt-get upgrade -y

# Install required system packages
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  python3-dev \
  libopenjp2-7 \
  libtiff6 \
  libatlas-base-dev \
  libjasper-dev \
  libharfbuzz0b \
  libwebp6 \
  libtiff5 \
  libjasper1 \
  libharfbuzz0b \
  libwebpdemux0
```

### 2. Python Virtual Environment

```bash
# Navigate to project directory
cd ~/robot_bridge

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate
```

### 3. Install Python Dependencies

```bash
# Upgrade pip
pip install --upgrade pip

# Install requirements
pip install -r requirements.txt

# For Raspberry Pi specific: lightweight PyTorch
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### 4. Arduino Connection

**Find Arduino Port:**
```bash
ls /dev/tty* | grep -E 'USB|ACM'
# Usually: /dev/ttyUSB0 or /dev/ttyACM0
```

**Add User to Serial Group:**
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### 5. Camera Setup

**For USB Camera:**
```bash
ls /dev/video*
# Should show /dev/video0 or /dev/video1
```

**For CSI Camera:**
```bash
# Enable camera in raspi-config
sudo raspi-config
# Interface Options > Camera > Enable
```

### 6. GPU Acceleration (Optional but Recommended)

```bash
# Install OpenGL support for better performance
sudo apt-get install -y libgles2-mesa libgles2-mesa-dev
```

## Running the Bridge

### Manual Start

```bash
# Activate virtual environment
source venv/bin/activate

# Run the bridge
python robot_bridge_rpi5.py \
  --arduino-port /dev/ttyUSB0 \
  --host 0.0.0.0 \
  --port 8765 \
  --model yolov8n.pt \
  --camera 0
```

### Systemd Service (Automatic Start)

Create `/etc/systemd/system/robot-bridge.service`:

```ini
[Unit]
Description=Robot Bridge Service
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=always
RestartSec=10
User=pi
WorkingDirectory=/home/pi/robot_bridge
Environment="PATH=/home/pi/robot_bridge/venv/bin"
ExecStart=/home/pi/robot_bridge/venv/bin/python robot_bridge_rpi5.py \
  --arduino-port /dev/ttyUSB0 \
  --host 0.0.0.0 \
  --port 8765 \
  --model yolov8n.pt \
  --camera 0

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable robot-bridge
sudo systemctl start robot-bridge
sudo systemctl status robot-bridge
```

## Configuration

### Environment Variables

Create `.env` file:

```env
ARDUINO_PORT=/dev/ttyUSB0
WEBSOCKET_HOST=0.0.0.0
WEBSOCKET_PORT=8765
YOLO_MODEL=yolov8n.pt
CAMERA_ID=0
CONFIDENCE_THRESHOLD=0.5
```

### Arduino Configuration

Adjust these settings in `robot_bridge_rpi5.py`:

```python
# Serial configuration
arduino = ArduinoReader(port='/dev/ttyUSB0', baudrate=115200)

# Camera configuration
camera = CameraCapture(camera_id=0, width=640, height=480, fps=30)

# Model configuration
ai_model = AIModel(model_path='yolov8n.pt', device=0)
```

## Connect from React App

In React component:

```tsx
import { useRobotBridge } from '@/hooks/useRobotBridge';

export function RobotControl() {
  const {
    isConnected,
    frame,
    detections,
    stats,
    moveForward,
    moveBackward,
    stop,
  } = useRobotBridge({
    host: 'your-raspberry-pi-ip',
    port: 8765,
    autoConnect: true,
  });

  return (
    <div>
      <p>Connected: {isConnected ? 'Yes' : 'No'}</p>
      {frame && <img src={`data:image/jpeg;base64,${frame}`} alt="Camera" />}
      <div>Detections: {detections.length}</div>
      <div>FPS: {stats?.loop_fps.toFixed(1)}</div>
      
      <button onClick={() => moveForward(200)}>Forward</button>
      <button onClick={() => moveBackward(200)}>Backward</button>
      <button onClick={() => stop()}>Stop</button>
    </div>
  );
}
```

## Troubleshooting

### Can't Connect to Arduino
```bash
# Check serial port
dmesg | grep tty

# Test connection
cat /dev/ttyUSB0
```

### Camera Issues
```bash
# List cameras
v4l2-ctl --list-devices

# Test camera
libcamera-hello
```

### YOLO Model Too Slow
- Use smaller model: `yolov8n.pt` (nano)
- Reduce frame resolution
- Enable GPU acceleration if available
- Disable inference if not needed

### Memory Issues
```bash
# Monitor memory
free -h

# Use lightweight model
python robot_bridge_rpi5.py --model yolov8n.pt
```

### Logs

```bash
# View service logs
sudo journalctl -u robot-bridge -f

# View custom logs
tail -f robot_bridge.log
```

## Performance Tips

1. **Reduce frame resolution**: Lower resolution = faster processing
2. **Lower FPS**: 20 FPS instead of 30 FPS for faster AI inference
3. **Use nano model**: `yolov8n.pt` is fastest for RPi
4. **Enable hardware acceleration**: Use GPU if available
5. **Optimize encoding**: Reduce JPEG quality for faster transfer

## Network Configuration

### For Local Network Access

```bash
# Get Raspberry Pi IP
hostname -I

# Connect from React app using IP:
useRobotBridge({ host: '192.168.x.x' })
```

### For Remote Access (SSH Tunnel)

```bash
# On your local machine
ssh -L 8765:localhost:8765 pi@your-rpi-ip

# Then connect locally
useRobotBridge({ host: 'localhost' })
```

## Next Steps

1. Test Arduino communication
2. Test camera capture
3. Test YOLO inference
4. Connect React app
5. Tune performance settings
6. Deploy as systemd service
