#!/usr/bin/env python3
"""Closed-loop lane keeping in CARLA from seg_lane_node.py's output, with a log scored against the map.

Run in .venv-carla (needs carla + the bridge's helpers). Two processes:
    terminal 1 (.venv-carla):  python lane_drive.py --duration 120 --out runs/run1
    terminal 2 (.venv-ml):     python seg_lane_node.py --no-display --save-dir runs/run1/frames
The driver spawns (or finds) the 'hero' vehicle, streams its camera JPEGs on UDP 5600 and wheel speeds on
5603 exactly like the bridge, reads the lane state on UDP 5604 and steers with pure pursuit. The steering
input is ONLY the lane node's estimate. CARLA's map is used for the log: true lane offset / heading are
written next to the estimate so the estimator and the drive can be scored afterwards.

Outputs in --out: drive.csv, summary.txt (also printed).
"""
import argparse
import csv
import json
import math
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla  # noqa: E402
from carla_vesc_bridge import CameraStreamer, OdomStreamer, attach_camera, read_carla  # noqa: E402
from lane_controllers import make_controller  # noqa: E402


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


class LaneState:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.msg, self.recv = None, 0.0

    def poll(self):
        while True:
            try:
                self.msg = json.loads(self.sock.recv(4096))
                self.recv = time.time()
            except BlockingIOError:
                return
            except ValueError:
                continue


def truth(world_map, vehicle):
    """Lane centre offset (m, + = centre is to the right of the kart) and heading (rad, + = lane turns right)."""
    tf = vehicle.get_transform()
    wp = world_map.get_waypoint(tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    r = wp.transform.get_right_vector()
    d = tf.location - wp.transform.location
    lateral = d.x * r.x + d.y * r.y  # kart position right of the lane centre
    return -lateral, math.radians(wrap(wp.transform.rotation.yaw - tf.rotation.yaw)), wp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", help="load this map first, e.g. Town03")
    ap.add_argument("--blueprint", default="vehicle.audi.a2", help="spawned when no 'hero' exists")
    ap.add_argument("--spawn", type=int, default=0, help="spawn point index")
    ap.add_argument("--keep-pose", action="store_true", help="do not move an existing hero to the spawn point")
    ap.add_argument("--speed", type=float, default=3.0, help="target m/s")
    ap.add_argument("--ld-time", type=float, default=1.5, help="s of travel to look ahead when --lookahead is 0")
    ap.add_argument("--max-throttle", type=float, default=0.6)
    ap.add_argument("--duration", type=float, default=120.0, help="s")
    ap.add_argument("--out", default="run")
    ap.add_argument("--lookahead", type=float, default=0.0, help="m, 0 = take it from the lane node")
    ap.add_argument("--kp", type=float, default=1.0, help="gain on the pure-pursuit steering")
    ap.add_argument("--controller", choices=("pure-pursuit", "stanley", "pid", "mpc"),
                    default="pure-pursuit", help="lane steering controller")
    ap.add_argument("--stanley-k", type=float, default=1.0, help="Stanley cross-track gain")
    ap.add_argument("--stanley-softening", type=float, default=0.5, help="Stanley low-speed softening (m/s)")
    ap.add_argument("--pid-kp", type=float, default=0.7, help="PID proportional gain (rad/m)")
    ap.add_argument("--pid-ki", type=float, default=0.03, help="PID integral gain (rad/(m s))")
    ap.add_argument("--pid-kd", type=float, default=0.15, help="PID derivative gain (rad s/m)")
    ap.add_argument("--mpc-horizon", type=int, default=8, help="MPC prediction steps")
    ap.add_argument("--mpc-step", type=float, default=0.2, help="MPC prediction timestep (s)")
    ap.add_argument("--mpc-beam", type=int, default=24, help="MPC beam-search width")
    ap.add_argument("--steer-rate", type=float, default=2.5, help="max steering change per second (-1..1 units)")
    ap.add_argument("--lost-brake", type=float, default=0.5, help="s of lost lane before braking")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=360)
    ap.add_argument("--cam-fov", type=float, default=70.4)
    ap.add_argument("--cam-x", type=float, default=0.8)
    ap.add_argument("--cam-z", type=float, default=1.5)
    ap.add_argument("--cam-port", type=int, default=5600)
    ap.add_argument("--odom-port", type=int, default=5603)
    ap.add_argument("--odom-noise", type=float, default=0.02)
    ap.add_argument("--wheel-radius", type=float, default=0.1207)
    ap.add_argument("--track", type=float, default=0.9)
    ap.add_argument("--state-port", type=int, default=5604)
    ap.add_argument("--truth-drive", action="store_true", help="steer by the lane node (default); this flag instead drives with the map truth to check the setup")
    ap.add_argument("--keep-lights", action="store_true", help="do not force traffic lights green")
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
    if hero is None:
        bp = world.get_blueprint_library().find(a.blueprint)
        bp.set_attribute("role_name", "hero")
        pts = wmap.get_spawn_points()
        tf = pts[a.spawn % len(pts)]
        hero = world.spawn_actor(bp, tf)
        spawned = True
        print(f"spawned {a.blueprint} at spawn point {a.spawn % len(pts)}")
    if not spawned and not a.keep_pose:
        pts = wmap.get_spawn_points()
        hero.set_transform(pts[a.spawn % len(pts)])
        hero.set_target_velocity(carla.Vector3D(0, 0, 0))
        hero.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        print(f"existing hero moved to spawn point {a.spawn % len(pts)} (--keep-pose to leave it)")
        time.sleep(1.0)
    t_off0, _, wp0 = truth(wmap, hero)
    ahead = next((d for d in range(5, 60, 5) if any(w.is_junction for w in wp0.next(float(d)))), None)
    print(f"start: road {wp0.road_id} lane {wp0.lane_id}, offset from the driving-lane centre {t_off0:+.2f} m, "
          f"junction {'in ' + str(ahead) + ' m' if ahead else 'not within 60 m'}")
    if abs(t_off0) > 1.0:
        print("WARNING: the kart does not start on a driving lane (parking/bike lane?). Pick another --spawn.")
    if not a.keep_lights:
        for tl in world.get_actors().filter("traffic.traffic_light"):
            tl.set_state(carla.TrafficLightState.Green)
            tl.freeze(True)

    pc = hero.get_physics_control()
    wheels = pc.wheels
    def dist(p, q):  # positions are world-space cm: use the full distance, not one axis
        return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2) / 100.0

    wheelbase = dist(wheels[0].position, wheels[2].position) if len(wheels) >= 4 else 2.5
    if not 0.8 < wheelbase < 5.0:
        wheelbase = 2.5
    max_steer = math.radians(max(w.max_steer_angle for w in wheels))
    print(f"wheelbase {wheelbase:.2f} m, max steer {math.degrees(max_steer):.0f} deg")

    cam_streamer = CameraStreamer("127.0.0.1", a.cam_port, 70, 15)
    camera = attach_camera(world, hero, cam_streamer, a.cam_width, a.cam_height, a.cam_fov, a.cam_x, a.cam_z)
    odom = OdomStreamer("127.0.0.1", a.odom_port, a.wheel_radius, a.odom_noise) if a.odom_port else None
    lane = LaneState(a.state_port)
    controller = make_controller(a.controller, a)

    log = csv.writer(open(out / "drive.csv", "w", newline=""))
    log.writerow(["controller", "t", "speed", "steer", "throttle", "brake", "valid", "est_offset", "est_heading", "look_y", "conf", "crossing",
                  "held", "true_offset", "true_heading", "lane_id", "road_id", "is_junction", "lane_age_s", "state"])
    rows = []
    print("waiting for lane state from seg_lane_node.py ...")
    wait0 = time.time()
    while lane.msg is None and time.time() - wait0 < 120:
        lane.poll()
        time.sleep(0.1)
    steer, last_valid, t0, period = 0.0, time.time(), time.time(), 1.0 / a.hz
    sp_int = 0.0
    silent_warned = 0.0
    was_ok = False
    try:
        nxt = time.time()
        while time.time() - t0 < a.duration:
            lane.poll()
            now = time.time()
            m = lane.msg
            age = now - lane.recv if m else 1e9
            fresh = m is not None and age < 0.6
            if not fresh and age > 2.0 and now - silent_warned > 5.0:
                silent_warned = now
                print(f"WARNING: no lane state for {age:.0f} s (frames not reaching seg_lane_node.py, or it crashed: check its terminal)")
            t_off, t_head, wp = truth(wmap, hero)
            l_rpm, r_rpm, speed, _, _ = read_carla(hero, a.track, a.wheel_radius)
            if odom:
                odom.send(l_rpm, r_rpm)

            if a.truth_drive:  # truth drive, for checking the harness itself
                ld = 6.0
                target, ok = t_off + math.tan(t_head) * ld, True
            elif fresh and m["valid"]:
                ld = a.lookahead or max(6.0, a.ld_time * speed)  # look further ahead as speed rises
                target, ok = m["offset"] + math.tan(m["heading"]) * ld + 0.5 * m.get("curv", 0.0) * ld * ld, True
                last_valid = now
            else:
                ld, target, ok = 6.0, 0.0, False

            state = "drive"
            if ok:
                if a.truth_drive:
                    steer_offset, steer_heading, curvature = t_off, t_head, 0.0
                else:
                    steer_offset, steer_heading = m["offset"], m["heading"]
                    curvature = m.get("curv", 0.0)
                cmd = controller.update(steer_offset, steer_heading, ld, curvature, speed,
                                        wheelbase, max_steer, period)
            else:
                cmd = steer  # hold the last steering while the lane is lost
                state = "lost"
                if was_ok and hasattr(controller, "reset"):
                    controller.reset()
            was_ok = ok
            step = a.steer_rate * period
            steer += max(-step, min(step, cmd - steer))

            lost_for = now - last_valid
            want = a.speed if (ok or lost_for < a.lost_brake) else 0.0
            if want == 0.0:
                state = "stopped(lost)"
            err = want - speed
            # P alone settled at ~1.8 m/s against a 3 m/s target; the integral term closes that gap
            sp_int = max(0.0, min(0.6, sp_int + 0.15 * err * period)) if want > 0 else 0.0
            throttle = max(0.0, min(a.max_throttle, 0.3 * err + sp_int))
            brake = max(0.0, min(1.0, -0.4 * err)) if want == 0.0 or err < -1.0 else 0.0
            hero.apply_control(carla.VehicleControl(throttle=throttle, steer=float(steer), brake=brake))

            t = now - t0
            row = [a.controller, f"{t:.2f}", f"{speed:.2f}", f"{steer:.3f}", f"{throttle:.2f}", f"{brake:.2f}",
                   int(bool(m and fresh and m['valid'])), f"{m['offset']:.3f}" if m and fresh else "",
                   f"{m['heading']:.4f}" if m and fresh else "", f"{m['look_y']:.3f}" if m and fresh else "",
                   f"{m['conf']:.2f}" if m and fresh else "", int(bool(m and m.get('crossing'))), int(bool(m and m.get('held'))),
                   f"{t_off:.3f}", f"{t_head:.4f}", wp.lane_id, wp.road_id, int(wp.is_junction), f"{min(age, 99):.2f}", state]
            log.writerow(row)
            rows.append(dict(t=t, speed=speed, valid=bool(m and fresh and m["valid"]), est=m["offset"] if m and fresh else None,
                             est_h=m["heading"] if m and fresh else None, true=t_off, true_h=t_head, junction=wp.is_junction,
                             state=state))
            if int(t) != int(t - period) and int(t) % 5 == 0:
                est = f"{m['offset']:+.2f}" if m and fresh else "  n/a"
                print(f"t={t:5.0f}s v={speed:4.1f} steer={steer:+.2f} est={est} true={t_off:+.2f} {state}"
                      f"{' [junction]' if wp.is_junction else ''}")
            if abs(t_off) > 3.0 and t > 3:
                print(f"left the lane (true offset {t_off:+.2f} m), stopping")
                break
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
    finally:
        hero.apply_control(carla.VehicleControl(brake=1.0))
        camera.stop()
        camera.destroy()
        if spawned:
            hero.destroy()
        else:
            hero.apply_control(carla.VehicleControl(brake=1.0))

    summarize(rows, out / "summary.txt", a.controller)


def summarize(rows, path, controller_name):
    import statistics as st
    moving = [r for r in rows if r["t"] > 3 and r["speed"] > 0.5]
    lines = [f"controller {controller_name}",
             f"samples {len(rows)}, duration {rows[-1]['t']:.0f} s" if rows else "no samples"]
    if moving:
        tr = [r["true"] for r in moving]
        lines.append(f"drive: kart lateral error vs lane centre  rms {math.sqrt(st.fmean(x * x for x in tr)):.2f} m  max {max(abs(x) for x in tr):.2f} m")
        lines.append(f"lane departures (|true| > 1.5 m): {sum(abs(x) > 1.5 for x in tr)} samples ({100 * sum(abs(x) > 1.5 for x in tr) / len(tr):.1f} %)")
        both = [r for r in moving if r["valid"] and r["est"] is not None]
        lines.append(f"lane node valid {100 * len(both) / len(moving):.1f} % of moving samples")
        if both:
            e = [r["est"] - r["true"] for r in both]
            h = [math.degrees(r["est_h"] - r["true_h"]) for r in both]
            lines.append(f"estimator offset error  mean {st.fmean(e):+.2f} m  rms {math.sqrt(st.fmean(x * x for x in e)):.2f} m  "
                         f"p95 {sorted(abs(x) for x in e)[int(0.95 * len(e))]:.2f} m")
            lines.append(f"estimator heading error mean {st.fmean(h):+.1f} deg  rms {math.sqrt(st.fmean(x * x for x in h)):.1f} deg")
            bad = [r for r in both if abs(r["est"] - r["true"]) > 0.5]
            lines.append(f"estimator off by > 0.5 m: {100 * len(bad) / len(both):.1f} %  (in junctions: {sum(r['junction'] for r in bad)} of {len(bad)})")
            lines.append(f"estimator sign: corr(est, true) = {corr([r['est'] for r in both], [r['true'] for r in both]):+.2f}")
        lines.append(f"stopped for lost lane: {100 * sum(r['state'].startswith('stopped') for r in moving) / len(moving):.1f} % of moving samples")
    text = "\n".join(lines)
    print("\n--- summary ---\n" + text)
    Path(path).write_text(text + "\n")


def corr(x, y):
    import statistics as st
    if len(x) < 3 or st.pstdev(x) == 0 or st.pstdev(y) == 0:
        return float("nan")
    mx, my = st.fmean(x), st.fmean(y)
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / (len(x) * st.pstdev(x) * st.pstdev(y))


if __name__ == "__main__":
    main()
