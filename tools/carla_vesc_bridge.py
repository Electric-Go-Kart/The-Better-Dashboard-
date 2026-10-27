#!/usr/bin/env python3
"""Drive the dashboard from CARLA by emitting VESC-style STATUS frames on SocketCAN.

Frame format (matches Backend/cancontroller.cpp):
  ID = (9 << 8) | controller_id, extended
  payload = int32 ERPM, int16 current*10, int16 voltage*10 (big-endian)
The dashboard divides ERPM by 7, so ERPM = wheel_rpm * gear_ratio * POLE_PAIRS.

Usage:
  python3 carla_vesc_bridge.py --iface vcan0            # needs a running CARLA server
  python3 carla_vesc_bridge.py --iface vcan0 --mock     # no CARLA, synthetic drive cycle
Pip: carla (matching your server), python-can
"""
import argparse
import math
import struct
import time

import can

CAN_PACKET_STATUS = 9
POLE_PAIRS = 7


class Pack:
    """Simple battery model: voltage sags with current."""

    def __init__(self, cells=13, capacity_ah=20.0, r_internal=0.05):
        self.v_full = cells * 4.2
        self.v_empty = cells * 3.0
        self.capacity_ah = capacity_ah
        self.r = r_internal
        self.used_ah = 0.0

    def voltage(self, current_a):
        soc = max(0.0, 1.0 - self.used_ah / self.capacity_ah)
        open_circuit = self.v_empty + (self.v_full - self.v_empty) * soc
        return open_circuit - current_a * self.r

    def step(self, current_a, dt):
        self.used_ah += current_a * dt / 3600.0


def encode(erpm, current_a, voltage_v):
    clamp16 = lambda x: max(-32768, min(32767, int(x)))
    return struct.pack(">ihh", int(erpm), clamp16(current_a * 10), clamp16(voltage_v * 10))


class WheelSide:
    def __init__(self, controller_id):
        self.controller_id = controller_id

    def frame(self, wheel_rpm, current_a, voltage_v, gear_ratio):
        erpm = wheel_rpm * gear_ratio * POLE_PAIRS
        return can.Message(
            arbitration_id=(CAN_PACKET_STATUS << 8) | self.controller_id,
            is_extended_id=True,
            data=encode(erpm, current_a, voltage_v),
        )


def motor_current(throttle, brake, speed_ms, max_current):
    """Positive when driving, negative (regen) when braking."""
    if brake > 0.05 and speed_ms > 0.2:
        return -brake * max_current * 0.4
    return throttle * max_current


def read_carla(vehicle, track_m, wheel_radius_m):
    v = vehicle.get_velocity()
    fwd = vehicle.get_transform().get_forward_vector()
    speed = v.x * fwd.x + v.y * fwd.y + v.z * fwd.z
    yaw_rate = math.radians(vehicle.get_angular_velocity().z)
    # Left/right speeds from yaw rate give a differential between sides (CARLA yaw is clockwise-positive).
    v_left = speed + yaw_rate * track_m / 2
    v_right = speed - yaw_rate * track_m / 2
    rpm = lambda s: s / (2 * math.pi * wheel_radius_m) * 60.0
    ctl = vehicle.get_control()
    return rpm(v_left), rpm(v_right), speed, ctl.throttle, ctl.brake


def mock_state(t, wheel_r=0.14):
    throttle = max(0.0, math.sin(t / 4.0))
    brake = max(0.0, -math.sin(t / 4.0)) * 0.3
    speed = 8.0 * (1 - math.cos(t / 4.0)) / 2
    steer = 0.3 * math.sin(t / 7.0)
    rpm = lambda s: s / (2 * math.pi * wheel_r) * 60.0
    return rpm(speed * (1 - steer)), rpm(speed * (1 + steer)), speed, throttle, brake


def find_vehicle(world, role, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for a in world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == role:
                return a
        time.sleep(1.0)
    raise SystemExit(f"No vehicle with role_name='{role}' found. Start CARLA's manual_control.py "
                     "(it spawns 'hero') or spawn a vehicle with that role_name.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="vcan0")
    ap.add_argument("--left-id", type=int, default=1)
    ap.add_argument("--right-id", type=int, default=2)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--role", default="hero", help="role_name of the vehicle to follow")
    ap.add_argument("--track", type=float, default=0.9, help="kart track width, m")
    ap.add_argument("--wheel-radius", type=float, default=0.1207, help="m (9.5 in wheel, as in Speedometer.ui.qml)")
    ap.add_argument("--gear-ratio", type=float, default=30 / 9, help="motor rev per wheel rev (30:9 teeth, as in Speedometer.ui.qml)")
    ap.add_argument("--max-current", type=float, default=60.0, help="A at full throttle")
    ap.add_argument("--cells", type=int, default=13)
    ap.add_argument("--capacity-ah", type=float, default=20.0)
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    bus = can.Bus(channel=args.iface, interface="socketcan")
    left, right = WheelSide(args.left_id), WheelSide(args.right_id)
    pack = Pack(args.cells, args.capacity_ah)

    vehicle = None
    if not args.mock:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        vehicle = find_vehicle(client.get_world(), args.role)

    period = 1.0 / args.hz
    t0 = nxt = time.time()
    print(f"Sending VESC status frames on {args.iface} (left={args.left_id}, right={args.right_id}) at {args.hz} Hz")
    while True:
        if args.mock:
            l_rpm, r_rpm, speed, thr, brk = mock_state(time.time() - t0, args.wheel_radius)
        else:
            l_rpm, r_rpm, speed, thr, brk = read_carla(vehicle, args.track, args.wheel_radius)

        base = motor_current(thr, brk, speed, args.max_current)
        skew = 0.05 * (l_rpm - r_rpm) / max(abs(l_rpm) + abs(r_rpm), 1.0)
        i_left, i_right = base * (1.0 + skew), base * (1.0 - skew)
        volts = pack.voltage(i_left + i_right)
        pack.step(i_left + i_right, period)

        bus.send(left.frame(l_rpm, i_left, volts, args.gear_ratio))
        bus.send(right.frame(r_rpm, i_right, volts, args.gear_ratio))

        nxt += period
        time.sleep(max(0.0, nxt - time.time()))


if __name__ == "__main__":
    main()
