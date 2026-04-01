#!/usr/bin/env python3
import cv2
import socket
import threading
import time
import logging

from flask import Flask, Response, request, jsonify

# Optional serial (for Arduino control)
try:
    import serial
except ImportError:
    serial = None

# Optional faster JPEG encoder
try:
    import simplejpeg
    USE_SIMPLEJPEG = True
except ImportError:
    USE_SIMPLEJPEG = False

# ---------------- CONFIG ----------------
CAMERA_INDEX  = 0
WIDTH         = 320
HEIGHT        = 240
FPS           = 20
JPEG_QUALITY  = 65
PORT          = 8080
HOST          = "0.0.0.0"

SERIAL_PORT   = "/dev/ttyACM0"   # change to /dev/ttyUSB0 if needed
SERIAL_BAUD   = 115200           # requested default
# ---------------------------------------

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger(__name__)
app = Flask(__name__)

ser = None
camera = None


def get_local_ip():
    try:
        return socket.gethostbyname(socket.gethostname())
    except Exception:
        return "localhost"


def open_serial():
    global ser
    if serial is None:
        log.warning("pyserial not installed. Run: pip3 install pyserial")
        return

    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.2)
        time.sleep(2.0)  # allow Arduino reset on serial open
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

        self.cap = None
        self._open_camera()
        self.running = True

        threading.Thread(target=self._capture_loop, daemon=True, name="capture").start()
        threading.Thread(target=self._encode_loop, daemon=True, name="encode").start()

    def _open_camera(self):
        log.info(f"Opening /dev/video{CAMERA_INDEX} with V4L2...")
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open /dev/video{CAMERA_INDEX}")

        # Set desired resolution/fps
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Try safer format negotiation: MJPG first, then YUYV fallback
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        time.sleep(0.05)
        ok, test = self.cap.read()
        if not ok or test is None:
            log.warning("MJPG read failed, trying YUYV...")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
            time.sleep(0.05)
            ok2, test2 = self.cap.read()
            if not ok2 or test2 is None:
                raise RuntimeError("Camera opened but cannot read frames (MJPG/YUYV failed).")

        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        enc = "simplejpeg ⚡" if USE_SIMPLEJPEG else "opencv"
        log.info(f"Camera ready: {w}x{h} @ {fps:.0f}fps | encoder: {enc}")

    def _capture_loop(self):
        count = 0
        t0 = time.time()

        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                log.warning("Frame grab failed — retrying...")
                time.sleep(0.03)
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
        interval = 1.0 / FPS if FPS > 0 else 0.05

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

    def stop(self):
        self.running = False
        try:
            if self.cap:
                self.cap.release()
        except Exception:
            pass


def mjpeg_generator():
    interval = 1.0 / FPS if FPS > 0 else 0.05
    while True:
        if camera is None:
            time.sleep(0.1)
            continue

        jpeg = camera.get_jpeg()
        if jpeg:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpeg +
                b"\r\n"
            )
        time.sleep(interval)


@app.route("/")
def index():
    ip = get_local_ip()
    return f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Pi Stream + Control</title>
  <style>
    body {{ background:#111; color:#eee; font-family:Arial,sans-serif; text-align:center; margin:0; }}
    header {{ padding:14px; background:#1a1a1a; border-bottom:1px solid #333; }}
    img {{ width:92%; max-width:760px; border:2px solid #333; margin-top:14px; border-radius:6px; }}
    .row {{ margin:12px; }}
    button {{ font-size:16px; padding:10px 14px; margin:4px; border-radius:6px; border:1px solid #444; cursor:pointer; }}
    input {{ font-size:16px; width:95px; text-align:center; }}
    #log {{ margin:12px; color:#8f8; min-height:22px; }}
    .links a {{ color:#65c3ff; margin:0 8px; }}
  </style>
</head>
<body>
  <header>
    <h2>Pi Stream + Motor Control</h2>
    <div>{ip}:{PORT}</div>
    <div class="links">
      <a href="/stream" target="_blank">/stream</a>
      <a href="/snapshot" target="_blank">/snapshot</a>
      <a href="/status" target="_blank">/status</a>
    </div>
  </header>

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
          body: JSON.stringify({{ cmd: cmd }})
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
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/snapshot")
def snapshot():
    if camera is None:
        return "Camera not ready", 503

    jpeg = camera.get_jpeg()
    if jpeg is None:
        return "Camera not ready", 503
    return Response(jpeg, mimetype="image/jpeg")


@app.route("/status")
def status():
    return {
        "running": camera.running if camera else False,
        "fps_capture": camera.fps_capture if camera else 0.0,
        "fps_encode": camera.fps_encode if camera else 0.0,
        "fps_target": FPS,
        "resolution": f"{WIDTH}x{HEIGHT}",
        "quality": JPEG_QUALITY,
        "encoder": "simplejpeg" if USE_SIMPLEJPEG else "opencv",
        "camera": CAMERA_INDEX,
        "port": PORT,
        "host": HOST,
        "serial_port": SERIAL_PORT,
        "serial_baud": SERIAL_BAUD,
        "serial_connected": ser is not None
    }


if __name__ == "__main__":
    try:
        open_serial()
        camera = Camera()
    except Exception as e:
        log.error(f"Startup camera error: {e}")
        camera = None

    ip = get_local_ip()
    print()
    print("  Open:")
    print(f"    http://{ip}:{PORT}/")
    print(f"    http://{ip}:{PORT}/stream")
    print(f"    http://{ip}:{PORT}/snapshot")
    print(f"    http://{ip}:{PORT}/status")

