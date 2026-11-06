#!/usr/bin/env python3
"""Send CARLA sensor data to a remote Pi and optionally apply Pi commands to CARLA only."""
import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

import carla

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_vesc_bridge import CameraStreamer, LidarStreamer, OdomStreamer, attach_camera, read_carla
from eb_drive import LidarSink, SCENARIOS, Tee, spawn_scenario
from split_protocol import CommandGate, ControlMode


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pi-host", required=True)
    ap.add_argument("--mode", choices=("shadow", "simulated"), default="shadow")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--ego-port", type=int, default=5612)
    ap.add_argument("--lidar-port", type=int, default=5611)
    ap.add_argument("--command-port", type=int, default=5613)
    ap.add_argument("--dashboard-port", type=int, default=5602)
    ap.add_argument("--no-dashboard-detections", action="store_true")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--hz", type=float, default=20)
    ap.add_argument("--speed", type=float, default=2.24, help="target speed in m/s")
    ap.add_argument("--duration", type=float, default=120)
    ap.add_argument("--scenario", choices=SCENARIOS, default="none")
    ap.add_argument("--dist", type=float, default=45)
    ap.add_argument("--lead-speed", type=float, default=1.5)
    ap.add_argument("--ped-speed", type=float, default=1.4)
    ap.add_argument("--ped-trigger", type=float, default=16)
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=360)
    ap.add_argument("--cam-fov", type=float, default=70.4)
    ap.add_argument("--cam-x", type=float, default=0.8)
    ap.add_argument("--cam-z", type=float, default=1.5)
    ap.add_argument("--cam-port", type=int, default=5600)
    ap.add_argument("--vision-cam-port", type=int, default=5606)
    ap.add_argument("--odom-port", type=int, default=5603)
    ap.add_argument("--wheel-radius", type=float, default=0.1207)
    ap.add_argument("--track", type=float, default=0.9)
    ap.add_argument("--lidar-x", type=float, default=1.0)
    ap.add_argument("--lidar-z", type=float, default=0.5)
    ap.add_argument("--lidar-range", type=float, default=25.0)
    ap.add_argument("--lidar-pps", type=int, default=21600)
    ap.add_argument("--lidar-hz", type=float, default=10)
    ap.add_argument("--lidar-noise", type=float, default=0.02)
    ap.add_argument("--lidar-drop", type=float, default=0.0)
    a = ap.parse_args()
    if min(a.hz, a.timeout, a.duration, a.lidar_hz) <= 0:
        ap.error("rates, duration and timeout must be positive")

    client = carla.Client(a.host, a.port)
    client.set_timeout(20)
    world = client.get_world()
    heroes = [v for v in world.get_actors().filter("vehicle.*") if v.attributes.get("role_name") == "hero"]
    if len(heroes) != 1:
        raise SystemExit(f"Expected exactly one existing hero vehicle, found {len(heroes)}")
    hero = heroes[0]
    bb = hero.bounding_box
    wheels = hero.get_physics_control().wheels
    if len(wheels) < 4:
        raise SystemExit("Vehicle must expose four wheels for steering geometry")
    wheelbase = math.dist((wheels[0].position.x, wheels[0].position.y),
                          (wheels[2].position.x, wheels[2].position.y)) / 100
    max_steer = math.radians(max(w.max_steer_angle for w in wheels))
    if not 0.8 < wheelbase < 5 or max_steer <= 0:
        raise SystemExit("Invalid CARLA vehicle steering geometry")
    gate = CommandGate(a.timeout)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", a.command_port))
    sock.setblocking(False)
    ego_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    streams = [CameraStreamer(a.pi_host, p, 70, 15) for p in (a.cam_port, a.vision_cam_port)]
    odom = OdomStreamer(a.pi_host, a.odom_port, a.wheel_radius, 0.02)
    sink = LidarSink(a.lidar_noise, a.lidar_drop, LidarStreamer(a.pi_host, a.lidar_port, a.lidar_z))
    sensors, actors = [], []
    tm = client.get_trafficmanager()
    control = ControlMode(a.mode, hero, tm.get_port(), carla.VehicleControl)
    dashboard_live = False
    try:
        control.update(gate)
        sensors.append(attach_camera(world, hero, Tee(streams), a.cam_width, a.cam_height, a.cam_fov, a.cam_x, a.cam_z))
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        for key, value in (("channels", 1), ("range", a.lidar_range), ("points_per_second", a.lidar_pps),
                           ("rotation_frequency", a.lidar_hz), ("upper_fov", 0), ("lower_fov", 0),
                           ("dropoff_general_rate", 0), ("dropoff_intensity_limit", 1),
                           ("dropoff_zero_intensity", 0), ("noise_stddev", 0)):
            if bp.has_attribute(key):
                bp.set_attribute(key, str(value))
        lidar = world.spawn_actor(bp, carla.Transform(carla.Location(x=a.lidar_x, z=a.lidar_z)), attach_to=hero)
        sensors.append(lidar)
        lidar.listen(sink.on_carla_scan)
        actors, _, drivers = spawn_scenario(world, world.get_map(), hero, a.scenario, a)
        for driver in drivers:
            if not isinstance(driver, tuple):
                driver.set_autopilot(True, tm.get_port())
                if hasattr(tm, "set_desired_speed"):
                    tm.set_desired_speed(driver, a.lead_speed * 3.6)
        start = time.monotonic()
        seq = 0
        pedestrian_started = False
        print(f"CARLA-only {a.mode} mode; remote Pi {a.pi_host}", flush=True)
        while time.monotonic() - start < a.duration:
            tick = time.monotonic()
            seq += 1
            left, right, speed, _, _ = read_carla(hero, a.track, a.wheel_radius)
            odom.send(left, right)
            ego = {"session": gate.session, "seq": seq, "mode": a.mode,
                   "control": "autopilot" if control.autopilot else a.mode, "speed": speed,
                   "yaw_rate": math.radians(hero.get_angular_velocity().z), "wheelbase": wheelbase,
                   "max_steer": max_steer, "kart_width": 2 * bb.extent.y,
                   "front_m": bb.extent.x - a.lidar_x + 0.05, "rear_m": bb.extent.x + a.lidar_x,
                   "max_range": a.lidar_range, "target_speed": a.speed, "cam_fov": a.cam_fov}
            gate.record_ego(seq)
            ego_sock.sendto(json.dumps(ego, allow_nan=False).encode(), (a.pi_host, a.ego_port))
            sink.drain()
            while True:
                try:
                    packet = sock.recv(65535)
                except BlockingIOError:
                    break
                try:
                    message = json.loads(packet)
                    if gate.accept(message) and not a.no_dashboard_detections and isinstance(message.get("objects"), list):
                        ego_sock.sendto(json.dumps({"objects": message["objects"]}, allow_nan=False).encode(),
                                        ("127.0.0.1", a.dashboard_port))
                        dashboard_live = True
                except (ValueError, UnicodeDecodeError) as exc:
                    print(f"Invalid command packet: {exc}", flush=True)
            control.update(gate)
            if dashboard_live and not gate.fresh():
                ego_sock.sendto(b'{"objects":[]}', ("127.0.0.1", a.dashboard_port))
                dashboard_live = False
            for driver in drivers:
                if isinstance(driver, tuple):
                    pedestrian, direction = driver
                    if a.scenario == "ped-sidewalk" or pedestrian.get_location().distance(hero.get_location()) < a.ped_trigger or pedestrian_started:
                        pedestrian_started = True
                        pedestrian.apply_control(carla.WalkerControl(direction=direction, speed=a.ped_speed))
            time.sleep(max(0, 1 / a.hz - (time.monotonic() - tick)))
    finally:
        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        for actor in actors:
            actor.destroy()
        if dashboard_live:
            ego_sock.sendto(b'{"objects":[]}', ("127.0.0.1", a.dashboard_port))
        control.close()
        sock.close()
        ego_sock.close()


if __name__ == "__main__":
    main()
