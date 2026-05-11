import socket
import serial
import threading
import time
import cv2
import json
from flask import Flask, Response, jsonify
from flask_cors import CORS
from ultralytics import YOLO
import os

# --- Configuration ---
TCP_IP = '0.0.0.0'
TCP_PORT = 5005
HTTP_PORT = 5000
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
CAMERA_INDEX = 0

client_socket = None
latest_detections = []
detection_lock = threading.Lock()

# Load YOLO model
model_path = os.path.join(os.path.dirname(__file__), '..', 'model', 'best.pt')
try:
    model = YOLO(model_path)
except Exception as e:
    print(f"Failed to load model: {e}")
    model = None

app = Flask(__name__)
CORS(app)
camera = cv2.VideoCapture(CAMERA_INDEX)

def generate_frames():
    global latest_detections
    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(0.1)
            continue
        
        if model:
            results = model.predict(frame, verbose=False)
            dets = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    label = model.names[cls_id]
                    dets.append({
                        'x': x1, 'y': y1, 'w': x2 - x1, 'h': y2 - y1,
                        'label': label, 'conf': conf
                    })
            with detection_lock:
                latest_detections = dets

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detections')
def get_detections():
    with detection_lock:
        return jsonify(latest_detections)

def run_flask_server():
    print(f"Starting Webcam HTTP server on {TCP_IP}:{HTTP_PORT} (/video_feed)...")
    app.run(host=TCP_IP, port=HTTP_PORT, threaded=True, use_reloader=False)

def serial_to_tcp_thread(ser):
    global client_socket
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if client_socket:
                    try:
                        client_socket.sendall((line + '\n').encode('utf-8'))
                    except Exception as e:
                        pass
            else:
                time.sleep(0.01)
        except Exception as e:
            time.sleep(1)

def start_tcp_server():
    global client_socket
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        t = threading.Thread(target=serial_to_tcp_thread, args=(ser,), daemon=True)
        t.start()
    except Exception as e:
        ser = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_IP, TCP_PORT))
    server.listen(1)

    while True:
        client, addr = server.accept()
        client_socket = client
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                cmd = data.decode('utf-8').strip()
                if ser and cmd:
                    ser.write((cmd + '\n').encode('utf-8'))
        except Exception as e:
            pass
        finally:
            client_socket.close()
            client_socket = None

if __name__ == '__main__':
    flask_thread = threading.Thread(target=run_flask_server, daemon=True)
    flask_thread.start()
    start_tcp_server()
