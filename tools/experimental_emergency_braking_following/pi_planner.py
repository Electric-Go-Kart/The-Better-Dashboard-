#!/usr/bin/env python3
"""Experimental CARLA-only Pi planner. Receives sensors; never opens an actuator interface."""
import argparse
import json
import math
import socket
import sys
import time
import uuid
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experimental_lane_assist"))
from lane_controllers import PurePursuit
from eb_core import Odometry, Params, Planner, PointWindow, SpeedController, Tracker, apply_boxes, cluster


def receiver(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.setblocking(False)
    return sock


def drain(sock):
    packets = []
    while True:
        try:
            packets.append(sock.recv(65535))
        except BlockingIOError:
            return packets


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_ego(m):
    if not isinstance(m, dict) or not isinstance(m.get("session"), str) or not m["session"] or type(m.get("seq")) is not int or m["seq"] < 0:
        raise ValueError("invalid ego session or sequence")
    keys = ("speed", "yaw_rate", "wheelbase", "max_steer", "kart_width", "front_m",
            "rear_m", "max_range", "target_speed", "cam_fov")
    if any(not finite_number(m.get(key)) for key in keys):
        raise ValueError("invalid ego numeric field")
    if (m["wheelbase"] <= 0 or m["max_steer"] <= 0 or m["kart_width"] <= 0 or m["front_m"] < 0
            or m["rear_m"] < 0 or m["max_range"] <= 0 or m["target_speed"] < 0 or not 0 < m["cam_fov"] < 180):
        raise ValueError("invalid ego geometry or target speed")


def validate_lane(m):
    if not isinstance(m, dict) or type(m.get("valid")) is not bool:
        raise ValueError("invalid lane state")
    if m["valid"] and any(not finite_number(m.get(k)) for k in ("offset", "heading")):
        raise ValueError("invalid lane geometry")
    if "curv" in m and not finite_number(m["curv"]):
        raise ValueError("invalid lane curv")
    if "width" in m and m["width"] is not None and not finite_number(m["width"]):
        raise ValueError("invalid lane width")


def validate_boxes(m):
    if not isinstance(m, dict) or not finite_number(m.get("w")) or m["w"] <= 0 or not isinstance(m.get("boxes"), list):
        raise ValueError("invalid boxes")
    for box in m["boxes"]:
        if (not isinstance(box, dict) or not isinstance(box.get("cls"), str)
                or any(not finite_number(box.get(k)) for k in ("u0", "u1"))
                or ("conf" in box and not finite_number(box["conf"]))):
            raise ValueError("invalid box")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--desktop-host", required=True)
    ap.add_argument("--ego-port", type=int, default=5612)
    ap.add_argument("--lidar-port", type=int, default=5611)
    ap.add_argument("--lane-port", type=int, default=5604)
    ap.add_argument("--boxes-port", type=int, default=5607)
    ap.add_argument("--command-port", type=int, default=5613)
    ap.add_argument("--debug-port", type=int, default=5608)
    ap.add_argument("--hz", type=float, default=20)
    ap.add_argument("--input-timeout", type=float, default=0.5)
    args = ap.parse_args()
    if not math.isfinite(args.hz) or not math.isfinite(args.input_timeout) or args.hz <= 0 or args.input_timeout <= 0:
        ap.error("hz and input timeout must be positive")

    ego_sock, lidar_sock, lane_sock, boxes_sock = [receiver(p) for p in
                                                   (args.ego_port, args.lidar_port, args.lane_port, args.boxes_port)]
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    odo, tracker, controller = Odometry(), Tracker(), PurePursuit(0.775)
    params = Params()
    window, planner, speed_controller = PointWindow(params), Planner(params), SpeedController(params.a_max, 0.6)
    ego = lane = boxes = None
    ego_at = lane_at = boxes_at = 0.0
    last = time.monotonic()
    last_lane = last
    last_box_seq = 0.0
    command_seq = 0
    instance = uuid.uuid4().hex
    steer = nudge = 0.0
    session = None
    retired_sessions = set()
    print("Waiting for CARLA sensor data; Pi planner has no actuator output.", flush=True)
    try:
        while True:
            tick = time.monotonic()
            dt, last = min(max(tick - last, 0.001), 0.5), tick
            for packet in drain(ego_sock):
                try:
                    m = json.loads(packet)
                    validate_ego(m)
                    if m["session"] in retired_sessions:
                        continue
                    if session == m["session"] and ego is not None and m["seq"] <= ego["seq"]:
                        continue
                    if session != m["session"]:
                        if session is not None:
                            retired_sessions.add(session)
                        session = m["session"]
                        odo, tracker = Odometry(), Tracker()
                        params = Params()
                        window, planner = PointWindow(params), Planner(params)
                        speed_controller = SpeedController(params.a_max, 0.6)
                        lane = boxes = None
                        steer = nudge = 0.0
                        last_lane = tick
                    ego, ego_at = m, tick
                except (ValueError, KeyError, TypeError, UnicodeDecodeError) as exc:
                    print(f"Invalid ego packet: {exc}", file=sys.stderr, flush=True)
            for sock, kind in ((lane_sock, "lane"), (boxes_sock, "boxes")):
                for packet in drain(sock):
                    try:
                        msg = json.loads(packet)
                        if not isinstance(msg, dict):
                            raise ValueError("expected JSON object")
                        if kind == "lane":
                            validate_lane(msg)
                            lane, lane_at = msg, tick
                        else:
                            validate_boxes(msg)
                            boxes, boxes_at = msg, tick
                    except (ValueError, UnicodeDecodeError) as exc:
                        print(f"Invalid {kind} packet: {exc}", file=sys.stderr, flush=True)
            if ego is None or tick - ego_at > args.input_timeout:
                drain(lidar_sock)
                time.sleep(max(0, 1 / args.hz - (time.monotonic() - tick)))
                continue
            params.kart_width, params.front_m, params.rear_m = ego["kart_width"], ego["front_m"], ego["rear_m"]
            params.max_range = ego["max_range"]
            speed, yaw = ego["speed"], ego["yaw_rate"]
            odo.step(speed, yaw, dt)
            for packet in drain(lidar_sock):
                if len(packet) % 4:
                    print("Invalid lidar datagram length", file=sys.stderr, flush=True)
                    continue
                pts = np.frombuffer(packet, dtype="<i2").reshape(-1, 2).astype(np.float32) / 100
                window.add(tick, pts, odo)
            tracker.update(cluster(window.current(tick, odo), params.cell_m, params.min_points), tick, odo)
            if boxes and boxes_at != last_box_seq and tick - boxes_at < 1:
                apply_boxes(tracker.tracks, boxes, ego["cam_fov"])
                last_box_seq = boxes_at
            active_lane = lane if lane and lane.get("valid") and tick - lane_at < 0.6 else None
            if active_lane:
                last_lane = tick
            params.v_set = ego["target_speed"] if active_lane or tick - last_lane < 0.5 else 0.0
            decision = planner.plan(tick, speed, yaw, tracker.tracks, active_lane, tick - window.last_t)
            throttle, brake = speed_controller.update(speed, decision.v_target, decision.a_brake, dt)
            nudge += max(-0.5 * dt, min(0.5 * dt, decision.nudge - nudge))
            if active_lane:
                lookahead = max(6.0, 1.1 * speed)
                desired = controller.update(active_lane["offset"] + nudge, active_lane["heading"], lookahead,
                                            active_lane.get("curv", 0), speed, ego["wheelbase"],
                                            ego["max_steer"], dt)
                steer += max(-2.5 * dt, min(2.5 * dt, desired - steer))
            command_seq += 1
            command = {"session": session, "instance": instance, "seq": command_seq, "ego_seq": ego["seq"], "steer": steer,
                       "throttle": throttle, "brake": brake, "state": decision.state,
                       "v_target": decision.v_target, "a_req": decision.a_req, "gap": decision.gap}
            points = window.current(tick, odo)
            if len(points) > 700:
                points = points[:: -(-len(points) // 700)]
            tracks = []
            for track in tracker.tracks:
                if track.pts_k is None or len(track.pts_k) == 0 or track.hits < 2:
                    continue
                angles = np.arctan2(track.pts_k[:, 1], track.pts_k[:, 0])
                tracks.append({"id": track.id, "cls": track.cls, "votes": track.votes,
                               "c": track.pts_k.mean(axis=0).tolist(), "v": track.vel_k.tolist(),
                               "pts": track.pts_k[::max(1, len(track.pts_k) // 30)].tolist(),
                               "bear": [float(angles.min()), float(angles.max())],
                               "threat": decision.threat is track})
            command["objects"] = [
                {"id": tr["id"], "class": tr["cls"], "x": tr["c"][0], "y": tr["c"][1],
                 "d": math.hypot(*tr["c"]), "vx": tr["v"][0], "vy": tr["v"][1],
                 "src": "fused" if tr["cls"] != "unknown" else "lidar",
                 "conf": min(1.0, max(tr["votes"].values(), default=0.0))}
                for tr in tracks]
            out.sendto(json.dumps(command, allow_nan=False).encode(), (args.desktop_host, args.command_port))
            wedges = []
            if boxes and tick - boxes_at < 1 and boxes.get("w"):
                width = boxes["w"]
                focal = width / 2 / math.tan(math.radians(ego["cam_fov"]) / 2)
                wedges = [{"cls": b["cls"], "conf": b.get("conf", 0),
                           "lo": math.atan((b["u0"] - width / 2) / focal),
                           "hi": math.atan((b["u1"] - width / 2) / focal)} for b in boxes["boxes"]]
            debug = {"t": time.time(), "v": speed, "state": decision.state, "v_target": decision.v_target,
                     "a_req": decision.a_req, "gap": decision.gap, "nudge": nudge, "lane": active_lane,
                     "fov": ego["cam_fov"], "pts": points.tolist(), "tracks": tracks, "boxes": wedges,
                     "kart": [params.kart_width, params.front_m, params.rear_m, params.margin],
                     "steer": steer, "throttle": throttle, "brake": brake,
                     "mode": ego.get("mode", "unknown"), "control": ego.get("control", "unknown"),
                     "seq": command_seq}
            out.sendto(json.dumps(debug, allow_nan=False).encode(), (args.desktop_host, args.debug_port))
            time.sleep(max(0, 1 / args.hz - (time.monotonic() - tick)))
    finally:
        for sock in (ego_sock, lidar_sock, lane_sock, boxes_sock, out):
            sock.close()


if __name__ == "__main__":
    main()
