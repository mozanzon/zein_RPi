#!/usr/bin/env python3
import cv2
import socket
import threading
import time
import logging

from flask import Flask, Response, request, jsonify

try:
    import serial
except ImportError:
    serial = None

try:
    import simplejpeg
    USE_SIMPLEJPEG = True
except ImportError:
    USE_SIMPLEJPEG = False

CAMERA_INDEX  = 0
WIDTH         = 320
HEIGHT        = 240
FPS           = 20
JPEG_QUALITY  = 65
PORT          = 8080
HOST          = "0.0.0.0"

SERIAL_PORT   = "/dev/ttyACM0"   # adjust if needed (/dev/ttyUSB0 etc)
SERIAL_BAUD   = 115200           # default requested

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)
app = Flask(__name__)

ser = None

def open_serial():
    global ser
    if serial is None:
        log.warning("pyserial not installed. Install with: pip3 install pyserial")
        return
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.2)
        time.sleep(2.0)
        log.info(f"Serial connected: {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        ser = None
        log.warning(f"Serial unavailable: {e}")

class Camera:
    def __init__(self):
        self._raw_frame = None
        self._jpeg_frame = None
        self._raw_lock = threading.Lock()
        self._jpeg_lock = threading.Lock()
        self.running = False
        self.fps_capture = 0.0
        self.fps_encode = 0.0

        self._open_camera()
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._encode_loop, daemon=True).start()

    def _open_camera(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open /dev/video{CAMERA_INDEX}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _capture_loop(self):
        count = 0
        t0 = time.time()
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._raw_lock:
                self._raw_frame = frame
            count += 1
            elapsed = time.time() - t0
            if elapsed >= 2.0:
                self.fps_capture = round(count / elapsed, 1)
                count = 0
                t0 = time.time()

    def _encode_loop(self):
        count = 0
        t0 = time.time()
        interval = 1.0 / FPS

        while self.running:
            t_start = time.time()
            with self._raw_lock:
                frame = self._raw_frame

            if frame is None:
                time.sleep(0.01)
                continue

            jpeg = self._encode(frame)
            if jpeg:
                with self._jpeg_lock:
                    self._jpeg_frame = jpeg
                count += 1

            sleep_time = interval - (time.time() - t_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

            elapsed = time.time() - t0
            if elapsed >= 2.0:
                self.fps_encode = round(count / elapsed, 1)
                count = 0
                t0 = time.time()

    def _encode(self, frame):
        if USE_SIMPLEJPEG:
            try:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return simplejpeg.encode_jpeg(rgb, quality=JPEG_QUALITY, colorspace="RGB")
            except Exception:
                pass
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return buf.tobytes() if ok else None

    def get_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg_frame

camera = Camera()
open_serial()

def mjpeg_generator():
    interval = 1.0 / FPS
    while True:
        jpeg = camera.get_jpeg()
        if jpeg:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        time.sleep(interval)

@app.route("/")
def index():
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        ip = "localhost"
    return f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Pi Stream + Control</title>
  <style>
    body {{ background:#111; color:#eee; font-family:Arial; text-align:center; }}
    img {{ width:90%; max-width:700px; border:2px solid #333; margin-top:10px; }}
    .row {{ margin:10px; }}
    button {{ font-size:16px; padding:10px 14px; margin:4px; }}
    input {{ font-size:16px; width:90px; text-align:center; }}
    #log {{ margin-top:10px; color:#8f8; min-height:24px; }}
  </style>
</head>
<body>
  <h2>Pi Stream + Motor Control</h2>
  <div>{ip}:{PORT}</div>
  <img src="/stream" alt="Live stream"/>

  <div class="row">
    Speed: <input id="spd" type="number" min="1" max="255" value="140">
  </div>

  <div class="row">
    <button onclick="sendCmd('FORWARD ' + spd())">Forward</button>
    <button onclick="sendCmd('BACKWARD ' + spd())">Backward</button>
    <button onclick="sendCmd('STOP')">Stop</button>
    <button onclick="sendCmd('S')">EMERGENCY</button>
  </div>

  <div class="row">
    <button onclick="sendCmd('TURN_LEFT_180 ' + spd())">Left 180°</button>
    <button onclick="sendCmd('TURN_RIGHT_180 ' + spd())">Right 180°</button>
  </div>

  <div id="log"></div>

  <script>
    function spd() {{
      const v = parseInt(document.getElementById('spd').value || "140", 10);
      if (isNaN(v)) return 140;
      return Math.max(1, Math.min(255, v));
    }}

    async function sendCmd(cmd) {{
      const log = document.getElementById('log');
      log.textContent = "Sending: " + cmd;
      try {{
        const r = await fetch('/cmd', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify({{ cmd }})
        }});
        const j = await r.json();
        log.textContent = j.ok ? ("OK: " + j.sent) : ("ERR: " + (j.error || "unknown"));
      }} catch (e) {{
        log.textContent = "ERR: " + e;
      }}
    }}
  </script>
</body>
</html>"""

@app.route("/cmd", methods=["POST"])
def cmd():
    global ser
    data = request.get_json(silent=True) or {}
    c = str(data.get("cmd", "")).strip()
    if not c:
        return jsonify({"ok": False, "error": "missing cmd"}), 400

    if ser is None:
        open_serial()
    if ser is None:
        return jsonify({"ok": False, "error": "serial not connected"}), 503

    try:
        ser.write((c + "\n").encode("utf-8"))
        return jsonify({"ok": True, "sent": c})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.route("/stream")
def stream():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/snapshot")
def snapshot():
    jpeg = camera.get_jpeg()
    if jpeg is None:
        return "Camera not ready", 503
    return Response(jpeg, mimetype="image/jpeg")

@app.route("/status")
def status():
    return {
        "running": camera.running,
        "fps_capture": camera.fps_capture,
        "fps_encode": camera.fps_encode,
        "fps_target": FPS,
        "resolution": f"{WIDTH}x{HEIGHT}",
        "quality": JPEG_QUALITY,
        "encoder": "simplejpeg" if USE_SIMPLEJPEG else "opencv",
        "camera": CAMERA_INDEX,
        "port": PORT,
        "serial_port": SERIAL
