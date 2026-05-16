#!/usr/bin/env python3
"""
Local COM bridge for controlling Arduino from the browser UI.

Run this on the laptop. It exposes:
  - GET http://localhost:8787/ports
  - WebSocket ws://localhost:8787/ws

The React UI uses this when direct browser Web Serial is unavailable or awkward.
"""

import asyncio
import json
import threading
import time
from datetime import datetime

from aiohttp import web
import serial
from serial.tools import list_ports


class SerialSession:
    def __init__(self):
        self.serial = None
        self.port = ""
        self.baudrate = 115200
        self.latest_raw = ""
        self.latest_arduino = None
        self.lock = threading.Lock()
        self.running = True

    def open(self, port, baudrate):
        self.close()
        self.port = port
        self.baudrate = baudrate
        self.serial = serial.Serial(port, baudrate, timeout=0.02, write_timeout=0.05)
        time.sleep(1.5)
        threading.Thread(target=self._read_loop, daemon=True).start()

    def close(self):
        old = self.serial
        self.serial = None
        if old:
            try:
                old.close()
            except Exception:
                pass

    def write(self, command):
        if not self.serial:
            return False
        self.serial.write((command.strip() + "\n").encode("utf-8"))
        return True

    def snapshot(self):
        with self.lock:
            return self.latest_raw, self.latest_arduino

    def _read_loop(self):
        active_serial = self.serial
        while self.running and self.serial is active_serial and active_serial:
            try:
                raw = active_serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parsed = parse_arduino_line(line)
                with self.lock:
                    self.latest_raw = line
                    if parsed:
                        self.latest_arduino = parsed
            except Exception:
                self.close()
                return


def parse_value(value):
    value = value.strip()
    if value == "true":
        return True
    if value == "false":
        return False
    try:
        parsed = float(value)
        return int(parsed) if parsed.is_integer() else parsed
    except ValueError:
        return value


def parse_arduino_line(line):
    if line.startswith("STATUS|") or line.startswith("DATA|"):
        parsed = {"type": "status"}
        for part in line.split("|")[1:]:
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            parsed[key] = parse_value(value)
        if "heading" in parsed:
            parsed["yaw"] = parsed["heading"]
        return parsed
    if line.startswith("ACK:"):
        return {"type": "ack", "message": line[4:]}
    if line.startswith("ERROR:"):
        return {"type": "error", "message": line[6:]}
    return None


def serial_command(data):
    kind = data.get("type")
    if kind == "raw":
        return str(data.get("command", "")).strip()
    if kind == "stop":
        return "S"
    if kind == "status":
        return "STATUS"
    if kind == "speed":
        return f"SPEED {clamp_int(data.get('speed', 160))}"
    if kind == "turn_speed":
        return f"TURN SPEED {clamp_int(data.get('speed', 70))}"
    if kind == "movement":
        return {
            "forward": "W",
            "backward": "X",
            "left": "A",
            "right": "D",
        }.get(str(data.get("action", "")).lower())
    if kind == "plot":
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
            return f"PLOT SPEED {clamp_int(data.get('speed', 180))}"
        if mode == "dist":
            return f"PLOT DIST {max(0.0, float(data.get('meters', 0))):.3f}"
        if mode == "ticks":
            return f"PLOT TICKS {max(0, int(data.get('ticks', 0)))}"
    return None


def clamp_int(value, low=0, high=255):
    return max(low, min(high, int(value)))


def cors(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return response


session = SerialSession()


async def ports_handler(request):
    ports = [
        {
            "device": item.device,
            "name": item.name,
            "description": item.description,
            "hwid": item.hwid,
        }
        for item in list_ports.comports()
    ]
    return cors(web.json_response({"ports": ports}))


async def options_handler(request):
    return cors(web.Response(status=204))


async def websocket_handler(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    async def sender():
        while not ws.closed:
            raw, arduino = session.snapshot()
            await ws.send_json(
                {
                    "timestamp": datetime.now().isoformat(),
                    "raw": raw,
                    "arduino": arduino,
                    "stats": {
                        "connected_clients": 1,
                        "loop_fps": 0,
                        "transport": "local-com-bridge",
                        "serial_port": session.port,
                        "baudrate": session.baudrate,
                        "camera_connected": False,
                        "camera_error": "Local COM bridge does not stream camera.",
                    },
                }
            )
            await asyncio.sleep(0.1)

    sender_task = asyncio.create_task(sender())

    try:
        async for message in ws:
            if message.type != web.WSMsgType.TEXT:
                continue
            data = json.loads(message.data)
            if data.get("type") == "connect_serial":
                session.open(data["port"], int(data.get("baudrate", 115200)))
                await ws.send_json({"type": "connected_serial", "port": session.port})
                continue
            if data.get("type") == "disconnect_serial":
                session.close()
                await ws.send_json({"type": "disconnected_serial"})
                continue
            command = serial_command(data)
            if command:
                session.write(command)
    finally:
        sender_task.cancel()

    return ws


def create_app():
    app = web.Application()
    app.router.add_get("/ports", ports_handler)
    app.router.add_route("OPTIONS", "/{tail:.*}", options_handler)
    app.router.add_get("/", websocket_handler)
    app.router.add_get("/ws", websocket_handler)
    return app


if __name__ == "__main__":
    web.run_app(create_app(), host="127.0.0.1", port=8787)
