import socket
import serial
import threading
import time
import cv2
from flask import Flask, Response

# --- Configuration ---
TCP_IP = '0.0.0.0'            # Listen on all interfaces
TCP_PORT = 5005               # Port for TCP connection (Motors/Compass)
HTTP_PORT = 5000              # Port for the Webcam HTTP stream
SERIAL_PORT = '/dev/ttyUSB0'  # Update this to your Arduino's serial port (e.g. /dev/ttyACM0)
BAUD_RATE = 115200
CAMERA_INDEX = 0              # Usually 0 for the default USB webcam

# Global connection socket
client_socket = None

# --- Flask App for Webcam ---
app = Flask(__name__)
camera = cv2.VideoCapture(CAMERA_INDEX)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(0.1)
            continue
        else:
            # Re-size if needed for performance
            # frame = cv2.resize(frame, (640, 480))
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            # Yield as MJPEG
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask_server():
    print(f"Starting Webcam HTTP server on {TCP_IP}:{HTTP_PORT} (/video_feed)...")
    app.run(host=TCP_IP, port=HTTP_PORT, threaded=True, use_reloader=False)

# --- TCP/Serial Bridge for Motors & Compass ---
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
    
    # Initialize Serial connection to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        # Start thread to read from Arduino and send to TCP client
        t = threading.Thread(target=serial_to_tcp_thread, args=(ser,), daemon=True)
        t.start()
    except Exception as e:
        print(f"Failed to connect to Arduino: {e} (Continuing without serial...)")
        ser = None

    # Setup TCP Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server.bind((TCP_IP, TCP_PORT))
    server.listen(1)
    print(f"TCP Server listening on {TCP_IP}:{TCP_PORT} (Motors/Sensors)")

    while True:
        client, addr = server.accept()
        print(f"Accepted TCP connection from {addr}")
        client_socket = client
        
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break # Client disconnected
                
                cmd = data.decode('utf-8').strip()
                print(f"[TCP Client] {cmd}")
                
                # Forward to Arduino
                if ser and cmd:
                    ser.write((cmd + '\n').encode('utf-8'))
                    
        except Exception as e:
            print(f"Client connection error: {e}")
        finally:
            print(f"Client {addr} disconnected.")
            client_socket.close()
            client_socket = None

if __name__ == '__main__':
    # Start the Flask webcam server in a daemon thread
    flask_thread = threading.Thread(target=run_flask_server, daemon=True)
    flask_thread.start()
    
    # Run the TCP socket server on the main thread
    start_tcp_server()
