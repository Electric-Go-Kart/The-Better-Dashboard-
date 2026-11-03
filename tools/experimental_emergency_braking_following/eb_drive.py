#!/usr/bin/env python3
"""Emergency braking + following in CARLA: lidar first, YOLO for the class, lane node for steering (.venv-carla).

Three processes (see README.md):
    .venv-carla:  python eb_drive.py --scenario ped-cross --out runs/ped1
    .venv-ml:     python ../experimental_lane_assist/seg_lane_node.py --no-display      (lane geometry, UDP 5604)
    .venv-ml:     python eb_vision_node.py --device cuda                                (YOLO classes, UDP 5607)
Add --truth-lane to steer from the map instead of the lane node (so you can test braking on its own), and
--no-vision to run lidar-only. The kart only steers by the lane estimate (plus a small in-lane nudge away from
things beside it) and brakes / follows by lidar. CARLA ground truth is used only to score the run.

Outputs in --out: drive.csv, summary.txt (also printed).
"""
import argparse
import csv
import json
import math
import socket
import statistics as st
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "experimental_lane_assist"))
sys.path.insert(0, str(HERE))
import carla  # noqa: E402
from carla_vesc_bridge import CameraStreamer, LidarStreamer, OdomStreamer, attach_camera, read_carla, spawn_traffic  # noqa: E402
from lane_controllers import make_controller  # noqa: E402
from lane_drive import LaneState, truth  # noqa: E402
from eb_core import (Odometry, Params, Planner, PointWindow, SpeedController, Tracker, apply_boxes,  # noqa: E402
                     cluster)

SCENARIOS = ("none", "lead-stop", "lead-slow", "ped-cross", "ped-sidewalk")


class Tee:
    """Sends each camera frame to several UDP ports (lane node on 5600, vision node on 5606)."""

    def __init__(self, streamers):
        self.streamers = streamers

    def on_carla_image(self, image):
        for s in self.streamers:
            s.on_carla_image(image)


class LidarSink:
    """In-process lidar listener; optionally adds range noise and random dropouts to make the scan harder."""

    def __init__(self, noise, drop, forward_udp=None):
        self.q, self.noise, self.drop, self.fwd = deque(maxlen=8), noise, drop, forward_udp
        self.rng = np.random.default_rng(0)

    def on_carla_scan(self, scan):
        pts = np.frombuffer(scan.raw_data, dtype=np.float32).reshape((-1, 4))[:, :2].copy()  # x forward, y right
        if self.drop > 0:
            pts = pts[self.rng.random(len(pts)) > self.drop]
        if self.noise > 0 and len(pts):
            r = np.hypot(pts[:, 0], pts[:, 1])
            pts *= ((r + self.rng.normal(0, self.noise, len(pts))) / np.maximum(r, 1e-3))[:, None]
        self.q.append((time.time(), pts))
        if self.fwd is not None:
            self.fwd.send_xyz(np.c_[pts, np.zeros(len(pts))])

    def drain(self):
        out = []
        while self.q:
            out.append(self.q.popleft())
        return out


def debug_msg(a, p, now, speed, d, nudge, lane, win, odo, trk, boxes_rx, box_seen):
    """Everything eb_view.py draws: points, tracks with class votes, the camera boxes as bearing wedges, the decision."""
    pts = win.current(now, odo)
    if len(pts) > 700:
        pts = pts[:: -(-len(pts) // 700)]
    tracks = []
    for tr in trk.tracks:
        if tr.pts_k is None or len(tr.pts_k) == 0 or tr.hits < 2:
            continue
        sub = tr.pts_k[:: max(1, len(tr.pts_k) // 30)]
        th = np.arctan2(tr.pts_k[:, 1], tr.pts_k[:, 0])
        tracks.append({"id": tr.id, "cls": tr.cls, "votes": {k: round(v, 2) for k, v in tr.votes.items()}, "hits": tr.hits,
                       "c": [round(float(x), 2) for x in tr.pts_k.mean(axis=0)], "v": [round(float(x), 2) for x in tr.vel_k],
                       "pts": np.round(sub, 2).tolist(), "bear": [round(float(th.min()), 3), round(float(th.max()), 3)],
                       "threat": d.threat is tr})
    boxes = []
    msg = boxes_rx.msg
    if msg and msg.get("w") and time.time() - boxes_rx.recv < 1.0:
        w = msg["w"]
        f = (w / 2) / math.tan(math.radians(a.cam_fov) / 2)
        boxes = [{"cls": b["cls"], "conf": b.get("conf", 0), "lo": round(math.atan((b["u0"] - w / 2) / f), 3),
                  "hi": round(math.atan((b["u1"] - w / 2) / f), 3)} for b in msg["boxes"]]
    return json.dumps({"t": now, "v": round(speed, 2), "state": d.state, "v_target": round(d.v_target, 2), "a_req": d.a_req,
                       "gap": d.gap, "nudge": round(nudge, 2), "lane": lane, "fov": a.cam_fov, "pts": np.round(pts, 2).tolist(),
                       "tracks": tracks, "boxes": boxes, "kart": [p.kart_width, p.front_m, p.rear_m, p.margin]}).encode()


def spawn_scenario(world, wmap, hero, name, a):
    """Place the scenario actors relative to the hero's lane. Returns (actors, objects) where objects are scored."""
    lib = world.get_blueprint_library()
    wp = wmap.get_waypoint(hero.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
    actors, objs, drivers = [], [], []

    def ahead(d):
        n = wp.next(float(d))
        return n[0] if n else None

    if name in ("lead-stop", "lead-slow"):
        w = ahead(a.dist)
        bp = lib.find("vehicle.tesla.model3")
        tf = w.transform
        tf.location.z += 0.4
        car = world.try_spawn_actor(bp, tf)
        if car is None:
            raise SystemExit("could not place the lead car (something is in the way); try another --spawn or --dist")
        actors.append(car)
        objs.append(car)
        if name == "lead-stop":
            car.apply_control(carla.VehicleControl(hand_brake=True))
        else:
            drivers.append(car)
    elif name in ("ped-cross", "ped-sidewalk"):
        w = ahead(a.dist)
        right = w.transform.get_right_vector()
        fwd = w.transform.get_forward_vector()
        off = w.lane_width / 2 + (1.2 if name == "ped-cross" else 0.8)
        loc = carla.Location(w.transform.location.x + right.x * off, w.transform.location.y + right.y * off,
                             w.transform.location.z + 0.8)
        bp = lib.filter("walker.pedestrian.*")[0]
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        ped = world.try_spawn_actor(bp, carla.Transform(loc, w.transform.rotation))
        if ped is None:
            raise SystemExit("could not place the pedestrian; try another --spawn or --dist")
        actors.append(ped)
        objs.append(ped)
        if name == "ped-cross":
            d = carla.Vector3D(-right.x, -right.y, 0)
        else:
            d = carla.Vector3D(fwd.x, fwd.y, 0)
        drivers.append((ped, d))
    return actors, objs, drivers


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map")
    ap.add_argument("--blueprint", default="vehicle.audi.a2")
    ap.add_argument("--spawn", type=int, default=0)
    ap.add_argument("--keep-pose", action="store_true")
    ap.add_argument("--speed", type=float, default=4.0, help="cruise m/s")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--out", default="run")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--max-throttle", type=float, default=0.6)
    ap.add_argument("--scenario", choices=SCENARIOS, default="none")
    ap.add_argument("--dist", type=float, default=45.0, help="m ahead of the kart where the scenario object is placed")
    ap.add_argument("--lead-speed", type=float, default=1.5, help="lead-slow: lead car m/s")
    ap.add_argument("--ped-trigger", type=float, default=16.0, help="ped-cross: starts walking when the kart is this close, m")
    ap.add_argument("--ped-speed", type=float, default=1.4)
    ap.add_argument("--spawn-cars", type=int, default=0)
    ap.add_argument("--spawn-walkers", type=int, default=0)
    ap.add_argument("--no-vision", action="store_true", help="lidar only: ignore the YOLO boxes")
    ap.add_argument("--no-nudge", action="store_true")
    ap.add_argument("--truth-lane", action="store_true", help="steer from the map instead of seg_lane_node.py")
    ap.add_argument("--lidar-x", type=float, default=1.0, help="lidar mount, m forward of the vehicle centre")
    ap.add_argument("--lidar-z", type=float, default=0.5)
    ap.add_argument("--lidar-range", type=float, default=25.0)
    ap.add_argument("--lidar-pps", type=int, default=21600)
    ap.add_argument("--lidar-hz", type=float, default=10.0)
    ap.add_argument("--lidar-noise", type=float, default=0.02, help="range noise sigma, m")
    ap.add_argument("--lidar-drop", type=float, default=0.0, help="fraction of points randomly dropped")
    ap.add_argument("--lidar-udp", type=int, default=0, help="also send scans to this UDP port (the bridge's lidar format)")
    ap.add_argument("--kp", type=float, default=0.775)
    ap.add_argument("--ld-time", type=float, default=1.1)
    ap.add_argument("--steer-rate", type=float, default=2.5)
    ap.add_argument("--lost-brake", type=float, default=0.5)
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=360)
    ap.add_argument("--cam-fov", type=float, default=70.4)
    ap.add_argument("--cam-x", type=float, default=0.8)
    ap.add_argument("--cam-z", type=float, default=1.5)
    ap.add_argument("--cam-port", type=int, default=5600, help="lane node")
    ap.add_argument("--vision-cam-port", type=int, default=5606, help="vision node")
    ap.add_argument("--boxes-port", type=int, default=5607)
    ap.add_argument("--state-port", type=int, default=5604)
    ap.add_argument("--odom-port", type=int, default=5603)
    ap.add_argument("--wheel-radius", type=float, default=0.1207)
    ap.add_argument("--track", type=float, default=0.9)
    ap.add_argument("--keep-lights", action="store_true")
    ap.add_argument("--pi-host", default="127.0.0.1", help="where the lane / vision nodes run (the Pi's IP); camera and odometry are sent there")
    ap.add_argument("--debug-port", type=int, default=5608, help="UDP port for eb_view.py (0 = off)")
    a = ap.parse_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    client = carla.Client(a.host, a.port)
    client.set_timeout(20.0)
    world = client.load_world(a.map) if a.map else client.get_world()
    wmap = world.get_map()
    print("map", wmap.name)

    hero, spawned = None, False
    for v in world.get_actors().filter("vehicle.*"):
        if v.attributes.get("role_name") == "hero":
            hero = v
    pts = wmap.get_spawn_points()
    if hero is None:
        bp = world.get_blueprint_library().find(a.blueprint)
        bp.set_attribute("role_name", "hero")
        hero = world.spawn_actor(bp, pts[a.spawn % len(pts)])
        spawned = True
    elif not a.keep_pose:
        hero.set_transform(pts[a.spawn % len(pts)])
        hero.set_target_velocity(carla.Vector3D(0, 0, 0))
        time.sleep(1.0)
    if not a.keep_lights:
        for tl in world.get_actors().filter("traffic.traffic_light"):
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)

    bb = hero.bounding_box
    p = Params(v_set=a.speed, kart_width=2 * bb.extent.y, front_m=bb.extent.x - a.lidar_x + 0.05,
               rear_m=bb.extent.x + a.lidar_x, max_range=a.lidar_range, nudge=not a.no_nudge)
    print(f"vehicle {2 * bb.extent.x:.2f} x {2 * bb.extent.y:.2f} m, lidar {a.lidar_x:.1f} m ahead of centre: "
          f"bumper {p.front_m:.2f} m ahead of / {p.rear_m:.2f} m behind the lidar")
    wheels = hero.get_physics_control().wheels
    wheelbase = math.dist((wheels[0].position.x, wheels[0].position.y), (wheels[2].position.x, wheels[2].position.y)) / 100.0 \
        if len(wheels) >= 4 else 2.5
    if not 0.8 < wheelbase < 5.0:
        wheelbase = 2.5
    max_steer = math.radians(max(w.max_steer_angle for w in wheels))

    streamers = [CameraStreamer(a.pi_host, a.cam_port, 70, 15)]
    if not a.no_vision:
        streamers.append(CameraStreamer(a.pi_host, a.vision_cam_port, 70, 15))
    camera = attach_camera(world, hero, Tee(streamers), a.cam_width, a.cam_height, a.cam_fov, a.cam_x, a.cam_z)
    sink = LidarSink(a.lidar_noise, a.lidar_drop, LidarStreamer("127.0.0.1", a.lidar_udp, a.lidar_z) if a.lidar_udp else None)
    lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    for k, v in (("channels", 1), ("range", a.lidar_range), ("points_per_second", a.lidar_pps),
                 ("rotation_frequency", a.lidar_hz), ("upper_fov", 0), ("lower_fov", 0), ("dropoff_general_rate", 0.0),
                 ("dropoff_intensity_limit", 1.0), ("dropoff_zero_intensity", 0.0), ("noise_stddev", 0.0)):
        if lidar_bp.has_attribute(k):
            lidar_bp.set_attribute(k, str(v))
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(x=a.lidar_x, z=a.lidar_z)), attach_to=hero)
    lidar.listen(sink.on_carla_scan)

    collisions = []
    col = world.spawn_actor(world.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=hero)
    col.listen(lambda e: collisions.append((time.time(), e.other_actor.type_id)))

    dbg_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if a.debug_port else None
    odom = OdomStreamer(a.pi_host, a.odom_port, a.wheel_radius, 0.02) if a.odom_port else None
    lane_rx = LaneState(a.state_port)
    boxes_rx = LaneState(a.boxes_port)
    controller = make_controller("pure-pursuit", a)

    traffic = spawn_traffic(client, world, hero, a.spawn_cars, a.spawn_walkers) if (a.spawn_cars or a.spawn_walkers) else []
    actors, objs, drivers = spawn_scenario(world, wmap, hero, a.scenario, a)
    tm = client.get_trafficmanager() if a.scenario == "lead-slow" else None
    for d in drivers:
        if tm is not None and not isinstance(d, tuple):
            d.set_autopilot(True, tm.get_port())
            tm.ignore_lights_percentage(d, 100)
            if hasattr(tm, "set_desired_speed"):
                tm.set_desired_speed(d, a.lead_speed * 3.6)
            else:
                tm.vehicle_percentage_speed_difference(d, 85.0)
    ped_go = False

    odo, win, trk, planner, spd = Odometry(), PointWindow(p), Tracker(), Planner(p), SpeedController(p.a_max, a.max_throttle)
    log = csv.writer(open(out / "drive.csv", "w", newline=""))
    log.writerow(["t", "speed", "steer", "throttle", "brake", "state", "v_target", "a_req", "gap", "nudge", "tracks", "classes",
                  "lidar_age", "lane_valid", "true_offset", "dist_true", "collision"])
    rows = []
    if not a.truth_lane:
        print("waiting for lane state (seg_lane_node.py) ...")
        t_wait = time.time()
        while lane_rx.msg is None and time.time() - t_wait < 120:
            lane_rx.poll()
            time.sleep(0.1)
    steer, last_valid, nudge_f = 0.0, time.time(), 0.0
    t0 = last = time.time()
    period = 1.0 / a.hz
    box_seen, box_n = 0.0, 0
    n_col = 0
    try:
        nxt = time.time()
        while time.time() - t0 < a.duration:
            now = time.time()
            dt = max(1e-3, now - last)
            last = now
            lane_rx.poll()
            boxes_rx.poll()
            t_off, t_head, wp = truth(wmap, hero)
            l_rpm, r_rpm, speed, _, _ = read_carla(hero, a.track, a.wheel_radius)
            yaw_rate = math.radians(hero.get_angular_velocity().z)  # CARLA: clockwise positive, like Odometry
            if odom:
                odom.send(l_rpm, r_rpm)
            odo.step(speed, yaw_rate, dt)

            for ts, pts_k in sink.drain():
                win.add(ts, pts_k, odo)
            trk.update(cluster(win.current(now, odo), p.cell_m, p.min_points), now, odo)
            if not a.no_vision and boxes_rx.msg and boxes_rx.recv != box_seen and now - boxes_rx.recv < 1.0:
                box_seen = boxes_rx.recv
                apply_boxes(trk.tracks, boxes_rx.msg, a.cam_fov)
                box_n += 1

            m = lane_rx.msg
            fresh = m is not None and now - lane_rx.recv < 0.6
            if a.truth_lane:
                lane = {"valid": True, "offset": t_off, "heading": t_head, "curv": 0.0, "width": wp.lane_width}
            elif fresh and m["valid"]:
                lane = dict(m)
            else:
                lane = None
            ok = lane is not None
            if ok:
                last_valid = now
            lost_for = now - last_valid
            p.v_set = a.speed if (ok or lost_for < a.lost_brake) else 0.0

            d = planner.plan(now, speed, yaw_rate, trk.tracks, lane, now - win.last_t)
            throttle, brake = spd.update(speed, d.v_target, d.a_brake, dt)

            nudge_f += max(-0.5 * dt, min(0.5 * dt, d.nudge - nudge_f))
            if ok:
                ld = max(6.0, a.ld_time * speed)
                cmd = controller.update(lane["offset"] + nudge_f, lane["heading"], ld, lane.get("curv", 0.0), speed,
                                        wheelbase, max_steer, period)
            else:
                cmd = steer
            step = a.steer_rate * period
            steer += max(-step, min(step, cmd - steer))
            hero.apply_control(carla.VehicleControl(throttle=throttle, steer=float(steer), brake=brake))

            # scenario actors
            hl = hero.get_location()
            for dr in drivers:
                if isinstance(dr, tuple):
                    ped, direction = dr
                    if a.scenario == "ped-sidewalk" or ped.get_location().distance(hl) < a.ped_trigger or ped_go:
                        ped_go = True
                        ped.apply_control(carla.WalkerControl(direction=direction, speed=a.ped_speed))
            dist_true = min((hl.distance(o.get_location()) - bb.extent.x - max(o.bounding_box.extent.x, o.bounding_box.extent.y)
                             for o in objs), default=float("nan"))
            new_col = len(collisions) > n_col
            n_col = len(collisions)

            if dbg_sock:
                dbg_sock.sendto(debug_msg(a, p, now, speed, d, nudge_f, lane, win, odo, trk, boxes_rx, box_seen), ("127.0.0.1", a.debug_port))

            t = now - t0
            lane_ok = int(ok)
            classes = ",".join(sorted({tr.cls for tr in trk.tracks if tr.hits > 3}))
            log.writerow([f"{t:.2f}", f"{speed:.2f}", f"{steer:.3f}", f"{throttle:.2f}", f"{brake:.2f}", d.state, f"{d.v_target:.2f}",
                          "" if d.a_req is None else f"{d.a_req:.2f}", "" if d.gap is None else f"{d.gap:.2f}", f"{nudge_f:.2f}",
                          len(trk.tracks), classes, f"{now - win.last_t:.2f}", lane_ok, f"{t_off:.3f}", f"{dist_true:.2f}",
                          int(new_col)])
            rows.append(dict(t=t, speed=speed, state=d.state, true=t_off, dist=dist_true, valid=bool(ok)))
            if int(t) != int(t - period):
                print(f"t={t:5.1f}s v={speed:4.1f} tgt={d.v_target:4.1f} {d.state:10s} dist={dist_true:6.1f} m "
                      f"tracks={len(trk.tracks)} [{classes}] lane_err={t_off:+.2f} cam-boxes={box_n}")
            if new_col:
                print(f"COLLISION with {collisions[-1][1]} at t={t:.1f}s v={speed:.1f}")
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
    finally:
        hero.apply_control(carla.VehicleControl(brake=1.0))
        for s in (camera, lidar, col):
            s.stop()
            s.destroy()
        for x in actors + traffic:
            try:
                x.destroy()
            except RuntimeError:
                pass
        if spawned:
            hero.destroy()

    summarize(rows, collisions, out / "summary.txt", a)


def summarize(rows, collisions, path, a):
    moving = [r for r in rows if r["t"] > 3]
    lines = [f"scenario {a.scenario}, cruise {a.speed} m/s, vision {'off' if a.no_vision else 'on'}",
             f"duration {rows[-1]['t']:.0f} s" if rows else "no samples"]
    if rows:
        lines.append(f"collisions: {len(collisions)} {sorted({c[1] for c in collisions})}")
        dists = [r["dist"] for r in rows if not math.isnan(r["dist"])]
        if dists:
            lines.append(f"closest approach to the scenario object (bbox-ish): {min(dists):.2f} m")
        lines.append(f"AEB time: {sum(r['state'] == 'AEB' for r in rows) / a.hz:.1f} s, "
                     f"states: { {s: sum(r['state'] == s for r in rows) for s in sorted({r['state'] for r in rows})} }")
        lines.append(f"min speed {min(r['speed'] for r in rows):.2f} m/s, final {rows[-1]['speed']:.2f} m/s")
    if moving:
        tr = [r["true"] for r in moving if r["speed"] > 0.5]
        if tr:
            lines.append(f"lane: rms {math.sqrt(st.fmean(x * x for x in tr)):.2f} m  max {max(abs(x) for x in tr):.2f} m")
        lines.append(f"lane estimate missing {100 * sum(not r['valid'] for r in moving) / len(moving):.1f} % of the time")
    text = "\n".join(lines)
    print("\n--- summary ---\n" + text)
    Path(path).write_text(text + "\n")


if __name__ == "__main__":
    main()
