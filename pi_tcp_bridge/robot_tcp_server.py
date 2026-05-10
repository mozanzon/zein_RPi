import socket
import serial
import threading
import time

# --- Configuration ---
TCP_IP = '0.0.0.0'       # Listen on all interfaces
TCP_PORT = 5005          # Port for TCP connection
SERIAL_PORT = '/dev/ttyUSB0'  # Update this to your Arduino's serial port (e.g. /dev/ttyACM0)
BAUD_RATE = 115200

# Global connection socket
client_socket = None

def serial_to_tcp_thread(ser):
    global client_socket
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(f"[Arduino] {line}")
                if client_socket:
                    try:
                        client_socket.sendall((line + '\n').encode('utf-8'))
                    except Exception as e:
                        print(f"Error sending to TCP: {e}")
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Serial reading error: {e}")
            time.sleep(1)

def start_server():
    global client_socket
    
    # Initialize Serial connection to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return

    # Start thread to read from Arduino and send to TCP client
    t = threading.Thread(target=serial_to_tcp_thread, args=(ser,), daemon=True)
    t.start()

    # Setup TCP Server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Allow port reuse
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server.bind((TCP_IP, TCP_PORT))
    server.listen(1)
    print(f"TCP Server listening on {TCP_IP}:{TCP_PORT}")

    while True:
        print("Waiting for a client connection...")
        client, addr = server.accept()
        print(f"Accepted connection from {addr}")
        client_socket = client
        
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break # Client disconnected
                
                cmd = data.decode('utf-8').strip()
                print(f"[TCP Client] {cmd}")
                
                # Forward to Arduino
                if cmd:
                    ser.write((cmd + '\n').encode('utf-8'))
                    
        except Exception as e:
            print(f"Client connection error: {e}")
        finally:
            print(f"Client {addr} disconnected.")
            client_socket.close()
            client_socket = None

if __name__ == '__main__':
    start_server()
