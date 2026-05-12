import socket
import serial
import threading
import time
import cv2
import json
from flask import Flask, Response, jsonify
from flask_cors import CORS

# --- Configuration ---
TCP_IP = '0.0.0.0'
TCP_PORT = 5005
HTTP_PORT = 5000
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
CAMERA_INDEX = 0

client_socket = None

app = Flask(__name__)
CORS(app)
camera = cv2.VideoCapture(CAMERA_INDEX)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(0.1)
            continue

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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
