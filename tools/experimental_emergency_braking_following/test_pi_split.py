import json
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

import numpy as np

from pi_planner import validate_boxes, validate_ego, validate_lane


class PiPlannerIntegrationTest(unittest.TestCase):
    def test_rejects_malformed_sensor_messages(self):
        for lane in ({"valid": True, "offset": float("nan"), "heading": 0},
                     {"valid": True, "offset": 0, "heading": 0, "curv": None}):
            with self.assertRaises(ValueError):
                validate_lane(lane)
        with self.assertRaises(ValueError):
            validate_boxes({"w": 640, "boxes": [{"cls": "vehicle", "u0": "4", "u1": 5}]})
        with self.assertRaises(ValueError):
            validate_ego({"session": "x", "seq": 0, "speed": "1"})

    def test_udp_planning_debug_and_dropout(self):
        sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(6)]
        for sock in sockets:
            sock.bind(("127.0.0.1", 0))
        ports = [sock.getsockname()[1] for sock in sockets]
        for sock in sockets[:4]:
            sock.close()
        command_sock, debug_sock = sockets[4:]
        command_sock.settimeout(0.1)
        debug_sock.settimeout(1)
        script = Path(__file__).with_name("pi_planner.py")
        process = subprocess.Popen([sys.executable, str(script), "--desktop-host", "127.0.0.1",
                                    "--ego-port", str(ports[0]), "--lidar-port", str(ports[1]),
                                    "--lane-port", str(ports[2]), "--boxes-port", str(ports[3]),
                                    "--command-port", str(ports[4]), "--debug-port", str(ports[5])],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ego = {"session": "test", "seq": 0, "mode": "shadow", "speed": 1.0, "yaw_rate": 0.0,
               "wheelbase": 2.4, "max_steer": 0.6, "kart_width": 1.2, "front_m": 1.0,
               "rear_m": 1.2, "max_range": 25.0, "target_speed": 2.0, "cam_fov": 70.4}
        lane = {"valid": True, "offset": 0.0, "heading": 0.0, "width": 3.5}
        scan = np.array([[500, -5], [500, 0], [500, 5], [505, 0]], dtype="<i2").tobytes()
        messages = []
        try:
            end = time.monotonic() + 2.5
            while time.monotonic() < end:
                ego["seq"] += 1
                sender.sendto(json.dumps(ego).encode(), ("127.0.0.1", ports[0]))
                sender.sendto(json.dumps(lane).encode(), ("127.0.0.1", ports[2]))
                if time.monotonic() < end - 0.8:
                    sender.sendto(scan, ("127.0.0.1", ports[1]))
                try:
                    messages.append(json.loads(command_sock.recv(65535)))
                except socket.timeout:
                    if process.poll() is not None:
                        self.fail(process.stderr.read().decode())
                time.sleep(0.03)
            self.assertTrue(any(m["state"] != "LIDAR-LOST" for m in messages))
            self.assertTrue(any(m["state"] == "LIDAR-LOST" for m in messages[-10:]))
            self.assertTrue(any(m["objects"] for m in messages))
            self.assertTrue(all(m["session"] == "test" and m["ego_seq"] <= ego["seq"] for m in messages))
            debug = json.loads(debug_sock.recv(65535))
            self.assertEqual(debug["mode"], "shadow")
            self.assertIn("steer", debug)
            time.sleep(0.7)
            command_sock.setblocking(False)
            while True:
                try:
                    command_sock.recv(65535)
                except BlockingIOError:
                    break
            command_sock.settimeout(0.2)
            with self.assertRaises(socket.timeout):
                command_sock.recv(65535)
        finally:
            process.terminate()
            process.communicate(timeout=5)
            sender.close()
            command_sock.close()
            debug_sock.close()


if __name__ == "__main__":
    unittest.main()
