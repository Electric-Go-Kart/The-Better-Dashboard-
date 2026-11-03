#!/usr/bin/env python3
"""Offline check of the emergency braking / following logic, no CARLA needed.

Simulates a kart on a straight 3 m lane (x forward, y right) with a 2D lidar that returns half a rotation
per 50 ms step (noise included), wheel odometry, delayed camera boxes and moving obstacles, and runs the
same eb_core code as eb_drive.py. Each scenario is scored on contact, gap and (for the false-alarm cases)
whether the kart kept moving.

    python eb_sim.py --selftest            # all scenarios, exit code 1 if any fails
    python eb_sim.py --scenario ped-cross  # one scenario, per-second trace
"""
import argparse
import math
import sys
from collections import deque

import numpy as np

from eb_core import Odometry, Params, Planner, PointWindow, SpeedController, Tracker, apply_boxes, cluster

DT = 0.05
LIDAR_X, CAM_X = 0.3, 0.8
FOV, IMG_W = 70.4, 640


class Shape:
    def __init__(self, cls, kind, size, pos, vel, start=0.0, trigger=None):
        self.cls, self.kind, self.size = cls, kind, size  # circle: radius, box: (half length, half width)
        self.p, self.v = np.array(pos, float), np.array(vel, float)
        self.start, self.trigger, self.moving = start, trigger, trigger is None and start <= 0.0
        self.hist = deque(maxlen=200)

    def step(self, t, kart_x, kart_v):
        if not self.moving:
            if self.trigger is not None:
                dist = self.p[0] - kart_x
                self.moving = dist / max(kart_v, 0.5) <= self.trigger
            else:
                self.moving = t >= self.start
        if self.moving:
            self.p = self.p + self.v * DT
        self.hist.append((t, self.p.copy()))

    def pos_at(self, t):
        return min(self.hist, key=lambda h: abs(h[0] - t))[1] if self.hist else self.p

    def heading(self):
        return math.atan2(self.v[1], self.v[0]) if np.hypot(*self.v) > 0.05 else 0.0


def raycast(shapes, origin, angles):
    u = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    best = np.full(len(angles), np.inf)
    for s in shapes:
        if s.kind == "circle":
            d = s.p - origin
            b = u @ d
            disc = b * b - (d @ d - s.size ** 2)
            tt = np.where((disc > 0) & (b > 0), b - np.sqrt(np.maximum(disc, 0.0)), np.inf)
        else:
            th = s.heading()
            c, sn = math.cos(th), math.sin(th)
            o = origin - s.p
            ob = np.array([o[0] * c + o[1] * sn, -o[0] * sn + o[1] * c])
            ub = np.stack([u[:, 0] * c + u[:, 1] * sn, -u[:, 0] * sn + u[:, 1] * c], axis=1)
            ub = np.where(np.abs(ub) < 1e-12, 1e-12, ub)
            half = np.array(s.size)
            t1, t2 = (-half - ob) / ub, (half - ob) / ub
            tn, tf = np.minimum(t1, t2).max(axis=1), np.maximum(t1, t2).min(axis=1)
            tt = np.where((tf >= tn) & (tn > 0), tn, np.inf)
        best = np.minimum(best, tt)
    return best


def clearance(shape, kart_x, p):
    """Distance from the object's outline to the kart's rectangle (negative = contact)."""
    lo = np.array([kart_x - p.rear_m, -p.kart_width / 2])
    hi = np.array([kart_x + p.front_m, p.kart_width / 2])
    if shape.kind == "circle":
        q = np.clip(shape.p, lo, hi)
        return float(np.hypot(*(shape.p - q))) - shape.size
    th = shape.heading()
    c, sn = math.cos(th), math.sin(th)
    hl, hw = shape.size
    pts = []
    for a in np.linspace(-hl, hl, 15):
        for b in (-hw, hw):
            pts.append((a, b))
    for b in np.linspace(-hw, hw, 5):
        for a in (-hl, hl):
            pts.append((a, b))
    world = np.array([[shape.p[0] + a * c - b * sn, shape.p[1] + a * sn + b * c] for a, b in pts])
    inside = ((world >= lo) & (world <= hi)).all(axis=1)
    if inside.any():
        return -0.1
    q = np.clip(world, lo, hi)
    return float(np.hypot(*(world - q).T).min())


def camera_msg(shapes, t, kart_x, latency, vision_on):
    if not vision_on:
        return None
    f = (IMG_W / 2) / math.tan(math.radians(FOV) / 2)
    boxes = []
    for s in shapes:
        p = s.pos_at(t - latency) - np.array([kart_x + CAM_X, 0.0])
        if p[0] < 1.0 or math.hypot(*p) > 30:
            continue
        if s.kind == "circle":
            r = s.size
        else:
            r = max(s.size)
        centre = math.atan2(p[1], p[0])
        half = math.asin(min(1.0, r / math.hypot(*p)))
        lo, hi = centre - half, centre + half
        if hi < -math.radians(FOV / 2) or lo > math.radians(FOV / 2):
            continue
        boxes.append({"cls": s.cls, "conf": 0.8, "u0": IMG_W / 2 + f * math.tan(max(lo, -1.2)),
                      "u1": IMG_W / 2 + f * math.tan(min(hi, 1.2))})
    return {"w": IMG_W, "boxes": boxes}


def run(name, shapes, v_set=3.0, duration=20.0, vision=True, lidar_dropout=None, verbose=False, seed=1):
    rng = np.random.default_rng(seed)
    p = Params(v_set=v_set)
    odo, win, trk, plan, spd = Odometry(), PointWindow(p), Tracker(), Planner(p), SpeedController(p.a_max)
    x, v, a_act, phi = 0.0, v_set, 0.0, 0.0
    brake_cmd = throttle_cmd = 0.0
    t, next_cam, msg_buf = 0.0, 0.0, deque()
    min_clear, min_speed, aeb_time, mean_v, n = 1e9, 1e9, 0.0, 0.0, 0
    states = {}
    lane = {"valid": True, "offset": 0.0, "heading": 0.0, "curv": 0.0, "width": 3.0}
    while t < duration:
        t += DT
        for s in shapes:
            s.step(t, x, v)
        # lidar: half a rotation per step with range noise
        if lidar_dropout is None or t < lidar_dropout:
            ang = phi + np.linspace(0, math.pi, 1080, endpoint=False)
            phi = (phi + math.pi) % (2 * math.pi)
            origin = np.array([x + LIDAR_X, 0.0])
            r = raycast(shapes, origin, ang)
            ok = r < 25.0
            rr = r[ok] + rng.normal(0, 0.015, ok.sum())
            pts = np.stack([rr * np.cos(ang[ok]), rr * np.sin(ang[ok])], axis=1)
            win.add(t, pts, odo)
        # camera: ~10 Hz, 0.25 s old
        if t >= next_cam:
            next_cam += 0.1
            m = camera_msg(shapes, t, x, 0.25, vision)
            if m is not None:
                msg_buf.append(m)
        odo.step(v + rng.normal(0, 0.02), 0.0, DT)
        trk.update(cluster(win.current(t, odo), p.cell_m, p.min_points), t, odo)
        while msg_buf:
            apply_boxes(trk.tracks, msg_buf.popleft(), FOV)
        d = plan.plan(t, v, 0.0, trk.tracks, lane, t - win.last_t)
        throttle_cmd, brake_cmd = spd.update(v, d.v_target, d.a_brake, DT)
        a_target = 3.5 * throttle_cmd - 7.0 * brake_cmd - 0.05 * v
        a_act += (a_target - a_act) * min(1.0, DT / 0.15)  # actuator lag
        v = max(0.0, v + a_act * DT)
        x += v * DT
        for s in shapes:
            min_clear = min(min_clear, clearance(s, x, p))
        if t > 3.0:
            min_speed, mean_v, n = min(min_speed, v), mean_v + v, n + 1
        states[d.state] = states.get(d.state, 0) + 1
        aeb_time += DT if d.state == "AEB" else 0.0
        if verbose and abs(t - round(t)) < DT / 2:
            print(f"  t={t:4.0f} x={x:6.1f} v={v:4.2f} tgt={d.v_target:4.2f} {d.state:9s} "
                  f"gap={'-' if d.gap is None else f'{d.gap:5.1f}'} tracks={len(trk.tracks)} "
                  f"cls={[tr.cls for tr in trk.tracks if tr.hits > 3][:3]}")
    return {"name": name, "clear": min_clear, "min_v": min_speed, "mean_v": mean_v / max(n, 1),
            "aeb": aeb_time, "final_v": v, "states": states}


def scenarios():
    return {
        "clear": dict(shapes=lambda: [], v_set=3.0, duration=15, check=lambda r: r["mean_v"] > 2.7),
        "static-car": dict(shapes=lambda: [Shape("vehicle", "box", (2.25, 0.9), (45, 0), (0, 0))], v_set=5.0, duration=20,
                           check=lambda r: r["clear"] > 0.3 and r["final_v"] < 0.2),
        "static-car-novision": dict(shapes=lambda: [Shape("vehicle", "box", (2.25, 0.9), (45, 0), (0, 0))], v_set=5.0,
                                    duration=20, vision=False, check=lambda r: r["clear"] > 0.3 and r["final_v"] < 0.2),
        "lead-slow": dict(shapes=lambda: [Shape("vehicle", "box", (2.25, 0.9), (30, 0), (1.5, 0))], v_set=4.0, duration=30,
                          check=lambda r: r["clear"] > 1.0 and r["aeb"] == 0 and 1.0 < r["final_v"] < 2.0),
        "ped-cross": dict(shapes=lambda: [Shape("person", "circle", 0.25, (30, 5.0), (0, -1.4), trigger=2.5)], v_set=4.0,
                          duration=15, check=lambda r: r["clear"] > 0.1),
        "ped-cross-novision": dict(shapes=lambda: [Shape("person", "circle", 0.25, (30, 5.0), (0, -1.4), trigger=2.5)],
                                   v_set=4.0, duration=15, vision=False, check=lambda r: r["clear"] > 0.1),
        "ped-late": dict(shapes=lambda: [Shape("person", "circle", 0.25, (14, 3.0), (0, -1.5), trigger=2.0)], v_set=4.0,
                         duration=12, check=lambda r: r["clear"] > 0.0),
        "ped-sidewalk": dict(shapes=lambda: [Shape("person", "circle", 0.25, (20, 3.5), (1.3, 0))], v_set=3.0, duration=15,
                             check=lambda r: r["aeb"] == 0 and r["mean_v"] > 2.4),
        "oncoming": dict(shapes=lambda: [Shape("vehicle", "box", (2.25, 0.9), (70, -3.4), (-5, 0))], v_set=3.0, duration=15,
                         check=lambda r: r["aeb"] == 0 and r["mean_v"] > 2.4),
        "lidar-dropout": dict(shapes=lambda: [], v_set=3.0, duration=10, lidar_dropout=3.0,
                              check=lambda r: r["final_v"] < 0.1),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--scenario", choices=list(scenarios()))
    a = ap.parse_args()
    table = scenarios()
    names = list(table) if a.selftest or not a.scenario else [a.scenario]
    failed = 0
    print(f"{'scenario':<22} {'result':<6} {'clearance m':>11} {'min v':>6} {'mean v':>7} {'final v':>8} {'AEB s':>6}  states")
    for name in names:
        sc = table[name]
        res = run(name, sc["shapes"](), sc["v_set"], sc["duration"], sc.get("vision", True), sc.get("lidar_dropout"),
                  verbose=bool(a.scenario))
        ok = sc["check"](res)
        failed += not ok
        print(f"{name:<22} {'PASS' if ok else 'FAIL':<6} {res['clear']:>11.2f} {res['min_v']:>6.2f} {res['mean_v']:>7.2f} "
              f"{res['final_v']:>8.2f} {res['aeb']:>6.2f}  {res['states']}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
