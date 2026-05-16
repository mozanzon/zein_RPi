#!/usr/bin/env python3
"""
Fast RoboScan bridge for Raspberry Pi.

Streams Arduino STATUS packets and a low-latency JPEG camera feed to the React UI.
YOLO is intentionally not run here; run it on the laptop with yolo_inference_server.py.
"""

import argparse
import asyncio
import base64
import json
import logging
import threading
import time
from collections import deque
from datetime import datetime

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import serial
except ImportError:
    serial = None

import websockets


DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyAMA0", "/dev/ttyAMA10"]


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger("robot_fast_bridge")


def parse_ports(value):
    return [part.strip() for part in str(value).split(",") if part.strip()]


def parse_value(value):
    value = value.strip()
    if value in ("true", "false"):
      return value == "true"
    try:
        parsed = float(value)
        return int(parsed) if parsed.is_integer() else parsed
    except ValueError:
        return value


def parse_arduino_line(line):
    if not line.startswith("STATUS|"):
        if line.startswith("ACK:"):
            return {"type": "ack", "message": line[4:]}
        if line.startswith("ERROR:"):
            return {"type": "error", "message": line[6:]}
        return None

    data = {"type": "status"}
    for part in line.split("|")[1:]:
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        data[key] = parse_value(value)

    if "heading" in data:
        data["yaw"] = data["heading"]
    if "drive_moving" in data:
        data["moving"] = bool(data["drive_moving"])
    return data


class ArduinoLink:
    def __init__(self, ports, baudrate):
        self.ports = ports
        self.baudrate = baudrate
        self.serial = None
        self.running = True
        self.latest_sensor = None
        self.latest_raw = ""
        self.connected_port = None
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _connect(self):
        if serial is None:
            logger.error("pyserial is not installed")
            time.sleep(2)
            return

        for port in self.ports:
            try:
                logger.info("Trying Arduino port %s", port)
                self.serial = serial.Serial(port, self.baudrate, timeout=0.02, write_timeout=0.05)
                self.connected_port = port
                time.sleep(1.5)
                logger.info("Connected Arduino on %s", port)
                return
            except Exception as exc:
                logger.warning("Arduino port %s failed: %s", port, exc)
                self.serial = None

        time.sleep(2)

    def _loop(self):
        while self.running:
            if self.serial is None:
                self._connect()
                continue

            try:
                raw = self.serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parsed = parse_arduino_line(line)
                with self.lock:
                    self.latest_raw = line
                    if parsed and parsed.get("type") == "status":
                        self.latest_sensor = parsed
            except Exception as exc:
                logger.warning("Arduino read failed, reconnecting: %s", exc)
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
                self.connected_port = None

    def send(self, command):
        if self.serial is None:
            return False
        try:
            self.serial.write((command.strip() + "\n").encode("utf-8"))
            return True
        except Exception as exc:
            logger.warning("Arduino write failed: %s", exc)
            return False

    def snapshot(self):
        with self.lock:
            return self.latest_sensor, self.latest_raw, self.connected_port


class CameraLink:
    def __init__(self, source, width, height, fps, quality):
        self.source = int(source) if str(source).isdigit() else source
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self.frame_b64 = None
        self.error = None
        self.frames = 0
        self.running = True
        self.lock = threading.Lock()

    def start(self):
        if cv2 is None:
            self.error = "opencv-python is not installed"
            logger.error(self.error)
            return
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        delay = 1.0 / max(1, self.fps)
        while self.running:
            cap = cv2.VideoCapture(self.source, cv2.CAP_V4L2)
            if not cap.isOpened():
                self.error = f"camera {self.source} not available"
                logger.warning(self.error)
                time.sleep(2)
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.error = None

            while self.running:
                start = time.time()
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.error = "camera frame read failed"
                    break

                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    with self.lock:
                        self.frame_b64 = base64.b64encode(encoded).decode("ascii")
                        self.frames += 1

                elapsed = time.time() - start
                if elapsed < delay:
                    time.sleep(delay - elapsed)

            cap.release()
            time.sleep(0.5)

    def snapshot(self):
        with self.lock:
            return self.frame_b64, self.frames, self.error


class RobotBridge:
    def __init__(self, args):
        self.arduino = ArduinoLink(parse_ports(args.arduino_ports), args.baudrate)
        self.camera = CameraLink(args.camera, args.width, args.height, args.fps, args.jpeg_quality)
        self.host = args.host
        self.port = args.port
        self.clients = set()
        self.command_history = deque(maxlen=20)

    def start_background(self):
        self.arduino.start()
        self.camera.start()

    async def handle_client(self, websocket, path=None):
        self.clients.add(websocket)
        logger.info("client connected: %d", len(self.clients))
        try:
            async for message in websocket:
                await self.handle_message(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            logger.info("client disconnected: %d", len(self.clients))

    async def handle_message(self, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return

        command = self.to_arduino_command(data)
        if not command:
            return

        sent = self.arduino.send(command)
        self.command_history.append({"command": command, "sent": sent, "time": time.time()})
        logger.info("command %s sent=%s", command, sent)

    def to_arduino_command(self, data):
        msg_type = data.get("type")

        if msg_type == "raw":
            return str(data.get("command", "")).strip()

        if msg_type == "movement":
            action = str(data.get("action", "")).lower()
            speed = int(max(0, min(255, int(data.get("speed", 160)))))
            return {
                "forward": "W",
                "backward": "X",
                "left": "A",
                "right": "D",
            }.get(action)

        if msg_type == "stop":
            return "S"

        if msg_type == "status":
            return "STATUS"

        if msg_type == "speed":
            return f"SPEED {int(max(0, min(255, int(data.get('speed', 160)))))}"

        if msg_type == "turn_speed":
            return f"TURN SPEED {int(max(0, min(255, int(data.get('speed', 70)))))}"

        if msg_type == "plot":
            mode = str(data.get("mode", "")).lower()
            if mode == "cont":
                return "PLOT CONT"
            if mode == "dash":
                return "PLOT DASH"
            if mode == "dash_dist":
                dash_m = max(0.01, float(data.get("dash_m", 0.5)))
                gap_m = max(0.01, float(data.get("gap_m", 0.3)))
                return f"PLOT DASH DIST {dash_m:.3f} {gap_m:.3f}"
            if mode == "off":
                return "PLOT OFF"
            if mode == "speed":
                return f"PLOT SPEED {int(max(0, min(255, int(data.get('speed', 180)))))}"
            if mode == "dist":
                return f"PLOT DIST {float(data.get('meters', 0)):.3f}"
            if mode == "ticks":
                return f"PLOT TICKS {int(max(0, int(data.get('ticks', 0))))}"

        return None

    async def broadcast_loop(self):
        while True:
            sensor, raw, port = self.arduino.snapshot()
            frame, frame_count, camera_error = self.camera.snapshot()
            payload = {
                "timestamp": datetime.now().isoformat(),
                "arduino": sensor,
                "raw": raw,
                "stats": {
                    "connected_clients": len(self.clients),
                    "arduino_port": port,
                    "camera_connected": frame is not None and camera_error is None,
                    "camera_error": camera_error,
                    "camera_frames_encoded": frame_count,
                    "stream_fps": self.camera.fps,
                    "jpeg_quality": self.camera.quality,
                },
            }
            if frame is not None:
                payload["frame"] = frame

            if self.clients:
                message = json.dumps(payload, separators=(",", ":"))
                results = await asyncio.gather(
                    *[asyncio.wait_for(client.send(message), timeout=1.0) for client in list(self.clients)],
                    return_exceptions=True,
                )
                for client, result in zip(list(self.clients), results):
                    if isinstance(result, Exception):
                        self.clients.discard(client)

            await asyncio.sleep(1.0 / max(1, self.camera.fps))

    async def run(self):
        self.start_background()
        logger.info("WebSocket listening on ws://%s:%s", self.host, self.port)
        async with websockets.serve(
            self.handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            max_size=2_000_000,
        ):
            await self.broadcast_loop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arduino-ports", default=",".join(DEFAULT_PORTS))
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=180)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--jpeg-quality", type=int, default=35)
    args = parser.parse_args()

    asyncio.run(RobotBridge(args).run())


if __name__ == "__main__":
    main()
