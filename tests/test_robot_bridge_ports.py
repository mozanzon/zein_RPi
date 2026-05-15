import importlib.util
import asyncio
import sys
import types
import unittest
from pathlib import Path


BRIDGE_MODULE = None


def load_bridge_module():
    global BRIDGE_MODULE
    if BRIDGE_MODULE is not None:
        return BRIDGE_MODULE

    for name in ("cv2", "numpy", "serial", "websockets"):
        sys.modules.setdefault(name, types.SimpleNamespace())

    bridge_path = Path(__file__).resolve().parents[1] / "pi_bridge" / "robot_bridge_rpi5.py"
    spec = importlib.util.spec_from_file_location("robot_bridge_rpi5", bridge_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    BRIDGE_MODULE = module
    return BRIDGE_MODULE


class SerialPortConfigTests(unittest.TestCase):
    def test_default_ports_try_ama10_before_ama0_before_usb_fallbacks(self):
        bridge = load_bridge_module()

        self.assertEqual(
            bridge.DEFAULT_ARDUINO_PORTS,
            ["/dev/ttyAMA10", "/dev/ttyAMA0", "/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyACM10"],
        )

        reader = bridge.ArduinoReader()
        self.assertEqual(reader.ports, bridge.DEFAULT_ARDUINO_PORTS)
        self.assertEqual(reader.port, "/dev/ttyAMA10")

    def test_parse_serial_ports_accepts_comma_separated_overrides(self):
        bridge = load_bridge_module()

        self.assertEqual(
            bridge.parse_serial_ports(" /dev/ttyAMA0, /dev/ttyUSB0 ,, "),
            ["/dev/ttyAMA0", "/dev/ttyUSB0"],
        )


class ArduinoTelemetryParsingTests(unittest.TestCase):
    def test_status_packet_parses_sensor_fields(self):
        bridge = load_bridge_module()
        reader = bridge.ArduinoReader()

        parsed = reader._parse_line(
            "STATUS|heading=91.25|yaw=91.25|lat=30.123456|lng=31.654321|fix=1|"
            "e1=120|e2=118|speed=0.340|battery=87|spraying=0|dash_cm=25.5|gap_cm=12.0"
        )

        self.assertEqual(parsed["type"], "status")
        self.assertEqual(parsed["heading"], 91.25)
        self.assertEqual(parsed["yaw"], 91.25)
        self.assertEqual(parsed["lat"], 30.123456)
        self.assertEqual(parsed["lng"], 31.654321)
        self.assertEqual(parsed["fix"], 1)
        self.assertEqual(parsed["e1"], 120)
        self.assertEqual(parsed["e2"], 118)
        self.assertEqual(parsed["speed"], 0.34)
        self.assertEqual(parsed["battery"], 87)
        self.assertEqual(parsed["spraying"], 0)
        self.assertEqual(parsed["dash_cm"], 25.5)
        self.assertEqual(parsed["gap_cm"], 12)

    def test_latest_sensor_data_skips_plain_text_serial_lines(self):
        bridge = load_bridge_module()
        reader = bridge.ArduinoReader()
        reader.data_buffer.extend(
            [
                {
                    "timestamp": "2026-05-15T15:00:00",
                    "raw": "STATUS|heading=45.0|e1=10|e2=11|speed=0.1",
                    "parsed": {"type": "status", "heading": 45.0, "e1": 10, "e2": 11, "speed": 0.1},
                },
                {
                    "timestamp": "2026-05-15T15:00:01",
                    "raw": "Already stopped.",
                    "parsed": {},
                },
            ]
        )

        self.assertEqual(
            reader.get_latest_sensor_data(),
            {"type": "status", "heading": 45.0, "e1": 10, "e2": 11, "speed": 0.1},
        )


class WebSocketCompatibilityTests(unittest.TestCase):
    def test_handle_client_accepts_single_connection_argument(self):
        bridge = load_bridge_module()
        robot_bridge = bridge.RobotBridge()
        websocket = EmptyWebSocket()

        asyncio.run(robot_bridge.handle_client(websocket))

        self.assertNotIn(websocket, robot_bridge.clients)


class EmptyWebSocket:
    def __aiter__(self):
        return self

    async def __anext__(self):
        raise StopAsyncIteration


if __name__ == "__main__":
    unittest.main()
