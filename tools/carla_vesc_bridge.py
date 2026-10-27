#!/usr/bin/env python3
"""Drive the dashboard from CARLA by emitting VESC-style STATUS frames on SocketCAN.

Frame format (matches Backend/cancontroller.cpp):
  ID = (9 << 8) | controller_id, extended
  payload = int32 ERPM, int16 current*10, int16 voltage*10 (big-endian)
The dashboard divides ERPM by 7, so ERPM = wheel_rpm * gear_ratio * POLE_PAIRS.

Usage:
  python3 carla_vesc_bridge.py --iface vcan0            # needs a running CARLA server
  python3 carla_vesc_bridge.py --iface vcan0 --mock     # no CARLA, synthetic drive cycle
Pip: carla (matching your server), python-can, pillow, numpy

A single-plane lidar modelled on the STL-27L (1 channel, 21600 points/s, 25 m) is also attached and streamed as compact
x/y scans over UDP (--no-lidar disables it).

A front camera is attached to the followed vehicle and streamed as low-res JPEG
datagrams over UDP to the dashboard's "What The Kart Sees" page (--no-camera disables it).
"""
import argparse
import io
import json
import math
import socket
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


class CameraStreamer:
    """Encodes frames as small JPEGs and sends one per UDP datagram."""

    def __init__(self, host, port, quality, max_fps):
        self.addr = (host, port)
        self.quality = quality
        self.min_interval = 1.0 / max_fps
        self.last_sent = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_rgb(self, rgb):
        now = time.time()
        if now - self.last_sent < self.min_interval:
            return
        self.last_sent = now
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=self.quality)
        data = buf.getvalue()
        if len(data) < 65000:
            self.sock.sendto(data, self.addr)

    def on_carla_image(self, image):
        import numpy as np
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        self.send_rgb(bgra[:, :, 2::-1].copy())


def attach_camera(world, vehicle, streamer, width, height, fov, x, z):
    import carla
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    bp.set_attribute("sensor_tick", str(1.0 / 15))
    camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=x, z=z)), attach_to=vehicle)
    camera.listen(streamer.on_carla_image)
    return camera


class LidarStreamer:
    """Sends each scan as little-endian int16 (x forward, y right) pairs in cm."""

    def __init__(self, host, port, mount_z, max_points=2200):
        self.addr = (host, port)
        self.max_points = max_points
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_xyz(self, xyz):
        import numpy as np
        if len(xyz) > self.max_points:
            xyz = xyz[:: -(-len(xyz) // self.max_points)]
        cm = np.clip(xyz[:, :2] * 100.0, -32768, 32767).astype("<i2")
        self.sock.sendto(cm.tobytes(), self.addr)

    def on_carla_scan(self, scan):
        import numpy as np
        pts = np.frombuffer(scan.raw_data, dtype=np.float32).reshape((-1, 4))
        self.send_xyz(pts[:, :3])


def attach_lidar(world, vehicle, streamer, x, z, lidar_range, pps, hz):
    import carla
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("channels", "1")
    bp.set_attribute("range", str(lidar_range))
    bp.set_attribute("points_per_second", str(pps))
    bp.set_attribute("rotation_frequency", str(hz))
    bp.set_attribute("upper_fov", "0")
    bp.set_attribute("lower_fov", "0")
    lidar = world.spawn_actor(bp, carla.Transform(carla.Location(x=x, z=z)), attach_to=vehicle)
    lidar.listen(streamer.on_carla_scan)
    return lidar


def mock_lidar_scan(t, lidar_range, n=360):
    import numpy as np
    ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
    half_w, half_l = 6.0, 12.0 + 3.0 * math.sin(t)
    r = np.minimum(half_l / np.maximum(np.abs(np.cos(ang)), 1e-3),
                   half_w / np.maximum(np.abs(np.sin(ang)), 1e-3))
    r = np.minimum(r, lidar_range)
    return np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros(n)], axis=1).astype(np.float32)


def classify_actor(actor):
    tid = actor.type_id
    if tid.startswith("walker."):
        return "person"
    if tid.startswith("vehicle."):
        return "bike" if actor.attributes.get("number_of_wheels") == "2" else "vehicle"
    if tid.startswith("static.prop."):
        name = tid.lower()
        if "cone" in name:
            return "cone"
        if any(k in name for k in ("barrier", "pole", "bollard", "fence", "guardrail", "streetsign", "trafficlight")):
            return "pole/barrier"
    return "unknown"


# Map objects (parked cars, poles, fences...) are not actors, so they are read from the map once.
ENV_LABELS = {
    "Car": "vehicle", "Truck": "vehicle", "Bus": "vehicle", "Train": "vehicle", "Vehicles": "vehicle",
    "Motorcycle": "bike", "Bicycle": "bike", "Rider": "bike",
    "Pedestrians": "person",
    "Poles": "pole/barrier", "Fences": "pole/barrier", "Guardrail": "pole/barrier",
    "Walls": "pole/barrier", "TrafficSigns": "pole/barrier", "TrafficLight": "pole/barrier",
    "Static": "unknown", "Dynamic": "unknown", "Other": "unknown",
}
ENV_MAX_EXTENT_M = 6.0


def load_env_objects(world):
    import carla
    items = []
    for name, cls in ENV_LABELS.items():
        label = getattr(carla.CityObjectLabel, name, None)
        if label is None:
            continue
        for o in world.get_environment_objects(label):
            ext = o.bounding_box.extent
            if max(ext.x, ext.y) > ENV_MAX_EXTENT_M:
                continue
            c = o.bounding_box.location
            items.append((o.id, cls, c.x, c.y, max(ext.x, ext.y)))
    print(f"Loaded {len(items)} static map objects for detection")
    return items


def env_objects_near(env_items, ego, lidar_x, max_range, fov_deg):
    tf = ego.get_transform()
    fwd, right = tf.get_forward_vector(), tf.get_right_vector()
    ox = tf.location.x + fwd.x * lidar_x
    oy = tf.location.y + fwd.y * lidar_x
    half_fov = math.radians(fov_deg) / 2
    objects = []
    for oid, cls, cx, cy, radius in env_items:
        dx, dy = cx - ox, cy - oy
        if abs(dx) > max_range + 6 or abs(dy) > max_range + 6:
            continue
        x = dx * fwd.x + dy * fwd.y
        y = dx * right.x + dy * right.y
        dist = max(0.0, math.hypot(x, y) - radius)
        if dist > max_range:
            continue
        in_view = x > 0 and abs(math.atan2(y, x)) <= half_fov
        objects.append({
            "id": int(oid) % 2147483647,
            "class": cls if in_view else "unknown",
            "x": round(x, 2), "y": round(y, 2), "d": round(dist, 2),
            "src": "fused" if in_view and cls != "unknown" else "lidar",
            "vx": 0.0, "vy": 0.0,
        })
    return objects


def ground_truth_objects(world, ego, lidar_x, max_range, fov_deg):
    """Nearby CARLA actors in the kart frame (x forward, y right, metres), relative to the lidar.

    Actors inside the camera FOV keep their class (src 'fused'); everything else is reported as
    'unknown' from lidar only, like the real pipeline will.
    """
    tf = ego.get_transform()
    fwd, right = tf.get_forward_vector(), tf.get_right_vector()
    ox = tf.location.x + fwd.x * lidar_x
    oy = tf.location.y + fwd.y * lidar_x
    half_fov = math.radians(fov_deg) / 2
    objects = []
    for actor in world.get_actors():
        if actor.id == ego.id or not actor.type_id.startswith(("vehicle.", "walker.", "static.prop.")):
            continue
        loc = actor.get_location()
        dx, dy = loc.x - ox, loc.y - oy
        x = dx * fwd.x + dy * fwd.y
        y = dx * right.x + dy * right.y
        centre = math.hypot(x, y)
        if centre > max_range + 3.0:
            continue
        ext = actor.bounding_box.extent
        dist = max(0.0, centre - max(ext.x, ext.y))
        if dist > max_range:
            continue
        in_view = x > 0 and abs(math.atan2(y, x)) <= half_fov
        v = actor.get_velocity()
        obj = {
            "id": actor.id,
            "class": classify_actor(actor) if in_view else "unknown",
            "x": round(x, 2), "y": round(y, 2), "d": round(dist, 2),
            "src": "fused" if in_view else "lidar",
            "vx": round(v.x * fwd.x + v.y * fwd.y, 2),
            "vy": round(v.x * right.x + v.y * right.y, 2),
        }
        objects.append(obj)
    return objects


def mock_objects(t):
    """A few moving fake objects for --mock."""
    def obj(i, cls, x, y, src, vx=0.0, vy=0.0):
        return {"id": i, "class": cls, "x": round(x, 2), "y": round(y, 2),
                "d": round(math.hypot(x, y), 2), "src": src, "vx": vx, "vy": vy}
    return [
        obj(1, "person", 7 + 2 * math.sin(t * 0.5), -2 + 1.5 * math.sin(t * 0.8), "fused",
            math.cos(t * 0.5), 1.2 * math.cos(t * 0.8)),
        obj(2, "vehicle", 15, 4 + math.sin(t * 0.3), "fused"),
        obj(3, "cone", 4, 1.5, "fused"),
        obj(4, "bike", 11 + 3 * math.cos(t * 0.4), -5, "fused"),
        obj(5, "pole/barrier", 9, 6, "fused"),
        obj(6, "unknown", -6 + 1.5 * math.sin(t * 0.6), 3, "lidar"),
        obj(7, "unknown", 1, -8, "lidar"),
        obj(8, "unknown", -14, -4 + 2 * math.cos(t * 0.5), "lidar"),
    ]


class DetectionStreamer:
    """Sends one JSON datagram per tick listing the objects the dashboard should draw."""

    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, objects):
        self.sock.sendto(json.dumps({"objects": objects}).encode(), self.addr)


def mock_camera_frame(t, width, height):
    import numpy as np
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[: height // 2] = (90, 140, 210)
    img[height // 2:] = (60, 60, 60)
    x = int((math.sin(t) * 0.4 + 0.5) * width)
    img[height // 2:, max(0, x - 2): x + 2] = (240, 240, 240)
    return img


def spawn_traffic(client, world, hero, n_cars, n_walkers, radius_m=60.0):
    """Spawn autopilot cars and walking pedestrians near the hero. Returns actors to destroy later."""
    import random
    import carla
    lib = world.get_blueprint_library()
    hero_loc = hero.get_location()
    actors = []

    def near_points(points):
        pts = [p for p in points if p.location.distance(hero_loc) < radius_m and p.location.distance(hero_loc) > 6.0]
        random.shuffle(pts)
        return pts

    car_bps = [b for b in lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
    tm = client.get_trafficmanager()
    for tf in near_points(world.get_map().get_spawn_points())[:n_cars]:
        car = world.try_spawn_actor(random.choice(car_bps), tf)
        if car is not None:
            car.set_autopilot(True, tm.get_port())
            actors.append(car)

    walker_ctrl_bp = lib.find("controller.ai.walker")
    walkers = []
    for _ in range(n_walkers * 40):
        if len(walkers) >= n_walkers:
            break
        loc = world.get_random_location_from_navigation()
        if loc is None or loc.distance(hero_loc) > radius_m or loc.distance(hero_loc) < 4.0:
            continue
        bp = random.choice(lib.filter("walker.pedestrian.*"))
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        w = world.try_spawn_actor(bp, carla.Transform(loc))
        if w is not None:
            walkers.append(w)
    for w in walkers:
        ctrl = world.spawn_actor(walker_ctrl_bp, carla.Transform(), attach_to=w)
        world.tick() if world.get_settings().synchronous_mode else time.sleep(0.05)
        ctrl.start()
        ctrl.go_to_location(world.get_random_location_from_navigation())
        ctrl.set_max_speed(1.0 + random.random() * 0.8)
        actors.append(ctrl)
        actors.append(w)
    print(f"Spawned {len([a for a in actors if a.type_id.startswith('vehicle.')])} cars and {len(walkers)} pedestrians near '{hero.type_id}'")
    return actors


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
    ap.add_argument("--no-camera", action="store_true", help="do not stream the front camera")
    ap.add_argument("--no-lidar", action="store_true", help="do not stream the lidar scan")
    ap.add_argument("--lidar-port", type=int, default=5601, help="matches DASH_LIDAR_PORT")
    ap.add_argument("--lidar-range", type=float, default=25.0, help="m")
    ap.add_argument("--lidar-pps", type=int, default=21600, help="points per second (STL-27L: 21600)")
    ap.add_argument("--lidar-hz", type=float, default=10.0, help="rotations per second")
    ap.add_argument("--lidar-x", type=float, default=0.3, help="forward offset from vehicle origin, m")
    ap.add_argument("--lidar-z", type=float, default=0.4, help="height above vehicle origin, m")
    ap.add_argument("--spawn-cars", type=int, default=0, help="spawn N autopilot cars near the hero")
    ap.add_argument("--spawn-walkers", type=int, default=0, help="spawn N walking pedestrians near the hero")
    ap.add_argument("--no-detections", action="store_true", help="do not send detected objects")
    ap.add_argument("--det-port", type=int, default=5602, help="matches DASH_DETECTION_PORT")
    ap.add_argument("--det-hz", type=float, default=25.0)
    ap.add_argument("--det-range", type=float, default=20.0, help="m")
    ap.add_argument("--det-fov", type=float, default=160.0, help="combined camera FOV in degrees; objects outside become 'unknown'")
    ap.add_argument("--cam-host", default="127.0.0.1", help="dashboard UDP host")
    ap.add_argument("--cam-port", type=int, default=5600, help="matches DASH_CAMERA_PORT")
    ap.add_argument("--cam-width", type=int, default=320)
    ap.add_argument("--cam-height", type=int, default=180)
    ap.add_argument("--cam-fov", type=float, default=90.0)
    ap.add_argument("--cam-fps", type=float, default=15.0)
    ap.add_argument("--cam-quality", type=int, default=50, help="JPEG quality")
    ap.add_argument("--cam-x", type=float, default=0.8, help="forward offset from vehicle origin, m")
    ap.add_argument("--cam-z", type=float, default=1.5, help="height above vehicle origin, m")
    args = ap.parse_args()

    bus = can.Bus(channel=args.iface, interface="socketcan")
    left, right = WheelSide(args.left_id), WheelSide(args.right_id)
    pack = Pack(args.cells, args.capacity_ah)

    vehicle = None
    if not args.mock:
        import carla
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        vehicle = find_vehicle(world, args.role)

    spawned = []
    if not args.mock and (args.spawn_cars or args.spawn_walkers):
        spawned = spawn_traffic(client, world, vehicle, args.spawn_cars, args.spawn_walkers)

    streamer = camera = None
    if not args.no_camera:
        streamer = CameraStreamer(args.cam_host, args.cam_port, args.cam_quality, args.cam_fps)
        if vehicle is not None:
            camera = attach_camera(world, vehicle, streamer, args.cam_width, args.cam_height,
                                   args.cam_fov, args.cam_x, args.cam_z)
            print(f"Streaming front camera {args.cam_width}x{args.cam_height} to {args.cam_host}:{args.cam_port}")

    lidar_streamer = lidar = None
    if not args.no_lidar:
        lidar_streamer = LidarStreamer(args.cam_host, args.lidar_port, args.lidar_z)
        if vehicle is not None:
            lidar = attach_lidar(world, vehicle, lidar_streamer, args.lidar_x, args.lidar_z,
                                 args.lidar_range, args.lidar_pps, args.lidar_hz)
            print(f"Streaming lidar to {args.cam_host}:{args.lidar_port}")

    det_streamer = None if args.no_detections else DetectionStreamer(args.cam_host, args.det_port)
    world_ref = None if args.mock else world

    period = 1.0 / args.hz
    t0 = nxt = time.time()
    print(f"Sending VESC status frames on {args.iface} (left={args.left_id}, right={args.right_id}) at {args.hz} Hz")
    try:
        run_loop(args, bus, left, right, pack, vehicle, streamer, lidar_streamer, det_streamer, world_ref, period, t0, nxt)
    finally:
        for actor in spawned:
            if actor.type_id == "controller.ai.walker":
                actor.stop()
            actor.destroy()
        for sensor in (camera, lidar):
            if sensor is not None:
                sensor.stop()
                sensor.destroy()


def run_loop(args, bus, left, right, pack, vehicle, streamer, lidar_streamer, det_streamer, world, period, t0, nxt):
    last_scan = 0.0
    last_det = 0.0
    last_report = 0.0
    env_items = load_env_objects(world) if (det_streamer is not None and world is not None) else []
    while True:
        if det_streamer is not None and time.time() - last_det >= 1.0 / args.det_hz:
            last_det = time.time()
            if args.mock:
                det_streamer.send(mock_objects(last_det - t0))
            else:
                objs = ground_truth_objects(world, vehicle, args.lidar_x, args.det_range, args.det_fov)
                objs += env_objects_near(env_items, vehicle, args.lidar_x, args.det_range, args.det_fov)
                det_streamer.send(objs)
                if last_det - last_report > 2.0:
                    last_report = last_det
                    nearest = min((o["d"] for o in objs), default=None)
                    print(f"detections: {len(objs)} objects within {args.det_range:.0f} m, nearest {nearest}")
        if lidar_streamer is not None and args.mock and time.time() - last_scan > 0.1:
            last_scan = time.time()
            lidar_streamer.send_xyz(mock_lidar_scan(last_scan - t0, args.lidar_range))
        if streamer is not None and args.mock:
            streamer.send_rgb(mock_camera_frame(time.time() - t0, args.cam_width, args.cam_height))
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
