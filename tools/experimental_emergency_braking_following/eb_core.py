"""Lidar-first obstacle tracking, class-aware motion prediction and a braking/following planner.

numpy only (no CARLA, no torch), so the same code runs in eb_drive.py and in the offline eb_sim.py.
Frames: kart frame is x forward, y right (m), like the lidar. The odom frame is the kart frame at start,
integrated from wheel speeds (what the real kart has), so tracks survive the kart's own motion.
"""
import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Params:
    v_set: float = 3.0            # cruise speed, m/s
    kart_width: float = 1.1       # m
    margin: float = 0.3           # clearance added to each side of the kart's swept path, m
    front_m: float = 0.7          # lidar to front bumper, m
    rear_m: float = 1.2           # lidar to rear bumper, m
    min_range: float = 0.25
    max_range: float = 25.0
    window_s: float = 0.15        # lidar returns newer than this are clustered together
    cell_m: float = 0.35          # clustering grid cell
    min_points: int = 3
    min_age_s: float = 0.2        # a track must be this old before it can trigger anything
    horizon_s: float = 5.0
    step_s: float = 0.1
    d_min: float = 1.5            # standstill gap behind a leader, m
    stop_margin: float = 0.6      # gap kept when braking to avoid a predicted collision, m
    headway_s: float = 1.2
    k_gap: float = 0.5            # 1/s, gap error -> speed
    a_comf: float = 1.5           # comfortable decel, m/s^2
    a_aeb: float = 3.0            # required decel above which emergency braking starts
    a_max: float = 6.0            # decel the kart can really do
    v_creep: float = 0.7          # speed floor for "caution" slowing near pedestrians
    lidar_timeout_s: float = 0.4  # no lidar for this long -> stop
    conv_m: float = 6.0           # the kart is assumed to rejoin the lane centre over this distance
    max_nudge: float = 0.4        # sideways shift inside the lane away from an obstacle, m
    edge_margin: float = 0.15
    nudge: bool = True


# (base radius growth, growth per second) of the zone an object may reach, by class
CLASS_INFL = {"person": (0.25, 0.7), "bike": (0.2, 0.4), "vehicle": (0.15, 0.15),
              "pole/barrier": (0.1, 0.0), "unknown": (0.2, 0.5)}


class Odometry:
    def __init__(self):
        self.x = self.y = self.psi = 0.0

    def step(self, v, yaw_rate, dt):
        self.psi += yaw_rate * dt  # clockwise positive, like CARLA and the lane node
        self.x += v * math.cos(self.psi) * dt
        self.y += v * math.sin(self.psi) * dt

    def to_odom(self, p):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return np.stack([self.x + p[:, 0] * c - p[:, 1] * s, self.y + p[:, 0] * s + p[:, 1] * c], axis=1)

    def to_kart(self, q):
        c, s = math.cos(self.psi), math.sin(self.psi)
        dx, dy = q[:, 0] - self.x, q[:, 1] - self.y
        return np.stack([dx * c + dy * s, -dx * s + dy * c], axis=1)

    def vec_to_kart(self, v):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return np.array([v[0] * c + v[1] * s, -v[0] * s + v[1] * c])


class PointWindow:
    """Recent lidar returns kept in the odom frame, so the kart's motion between batches is removed."""

    def __init__(self, p):
        self.p, self.buf, self.last_t = p, deque(), 0.0

    def add(self, t, pts_kart, odo):
        p = self.p
        if len(pts_kart):
            x, y, r = pts_kart[:, 0], pts_kart[:, 1], np.hypot(pts_kart[:, 0], pts_kart[:, 1])
            body = (x > -p.rear_m - 0.15) & (x < p.front_m + 0.1) & (np.abs(y) < p.kart_width / 2 + 0.15)
            pts_kart = pts_kart[(r > p.min_range) & (r < p.max_range) & ~body]
        self.buf.append((t, odo.to_odom(pts_kart) if len(pts_kart) else np.zeros((0, 2))))
        self.last_t = t

    def current(self, now, odo):
        while self.buf and now - self.buf[0][0] > self.p.window_s:
            self.buf.popleft()
        if not self.buf:
            return np.zeros((0, 2))
        return odo.to_kart(np.vstack([b[1] for b in self.buf]))


def cluster(pts, cell, min_points):
    """Connected components of occupied grid cells (8-neighbourhood). Returns a list of (n, 2) arrays."""
    if len(pts) == 0:
        return []
    keys = np.floor(pts / cell).astype(np.int64)
    cells = {}
    for i in range(len(pts)):
        cells.setdefault((int(keys[i, 0]), int(keys[i, 1])), []).append(i)
    seen, out = set(), []
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack, idx = [start], []
        while stack:
            c = stack.pop()
            idx.extend(cells[c])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (c[0] + dx, c[1] + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        stack.append(nb)
        if len(idx) >= min_points:
            out.append(pts[idx])
    return out


class Track:
    def __init__(self, tid, centre_odom, pts_odom, now):
        self.id, self.t0, self.last_hit, self.hits = tid, now, now, 1
        self.pos, self.vel = np.array(centre_odom, float), np.zeros(2)
        self.P = np.array([[0.5, 0.0], [0.0, 4.0]])  # position/velocity covariance, same on both axes
        self.pts_odom = pts_odom
        self.votes = {}
        self.pts_k, self.vel_k, self.extent = None, np.zeros(2), 0.0

    def predict(self, dt, sigma_a=1.5):
        self.pos = self.pos + self.vel * dt
        self.pts_odom = self.pts_odom + self.vel * dt
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = sigma_a ** 2 * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]])
        self.P = F @ self.P @ F.T + Q

    def update(self, z, r=0.2):
        s = self.P[0, 0] + r * r
        k = np.array([self.P[0, 0], self.P[1, 0]]) / s
        innov = z - self.pos
        self.pos = self.pos + k[0] * innov
        self.vel = self.vel + k[1] * innov
        self.P = self.P - np.outer(k, self.P[0])

    @property
    def cls(self):
        if not self.votes:
            return "unknown"
        name, score = max(self.votes.items(), key=lambda kv: kv[1])
        return name if score >= 0.6 else "unknown"

    def effective_velocity(self):
        speed = float(np.hypot(*self.vel))
        if speed < 0.3 or self.extent > 6.0:  # below the noise floor, or a wall-sized structure
            return np.zeros(2)
        return self.vel


class Tracker:
    def __init__(self):
        self.tracks, self.next_id, self.t_prev = [], 1, None

    def update(self, clusters, now, odo):
        dt = 0.05 if self.t_prev is None else min(max(now - self.t_prev, 0.005), 0.5)
        self.t_prev = now
        for t in self.tracks:
            t.predict(dt)
        cents = [odo.to_odom(c.mean(axis=0, keepdims=True))[0] for c in clusters]
        exts = [float(max(np.ptp(c[:, 0]), np.ptp(c[:, 1]))) for c in clusters]
        pairs = []
        for ti, t in enumerate(self.tracks):
            for ci, c in enumerate(cents):
                d = float(np.hypot(*(t.pos - c)))
                if d < 1.0 + 0.5 * max(exts[ci], t.extent):
                    pairs.append((d, ti, ci))
        used_t, used_c = set(), set()
        for d, ti, ci in sorted(pairs):
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            t = self.tracks[ti]
            t.update(cents[ci])
            t.pts_odom, t.extent, t.last_hit = odo.to_odom(clusters[ci]), exts[ci], now
            t.hits += 1
        for ci, c in enumerate(clusters):
            if ci not in used_c:
                t = Track(self.next_id, cents[ci], odo.to_odom(c), now)
                t.extent = exts[ci]
                self.next_id += 1
                self.tracks.append(t)
        self.tracks = [t for t in self.tracks if now - t.last_hit < 0.5]
        for t in self.tracks:
            t.pts_k = odo.to_kart(t.pts_odom)
            t.vel_k = odo.vec_to_kart(t.effective_velocity())
        return self.tracks


def apply_boxes(tracks, msg, fov_deg, decay=0.9):
    """Give lidar tracks a class from camera boxes by matching bearing intervals (one box per track)."""
    if not msg or not msg.get("w"):
        return
    w = msg["w"]
    f = (w / 2) / math.tan(math.radians(fov_deg) / 2)
    half = math.radians(fov_deg) / 2 - 0.02
    pad = math.radians(1.0)
    boxes = [(math.atan((b["u0"] - w / 2) / f) - pad, math.atan((b["u1"] - w / 2) / f) + pad, b) for b in msg["boxes"]]
    cands = []
    for t in tracks:
        if t.pts_k is None or len(t.pts_k) == 0 or t.pts_k[:, 0].mean() <= 0.5:
            continue
        th = np.arctan2(t.pts_k[:, 1], t.pts_k[:, 0])
        lo, hi = float(th.min()) - pad, float(th.max()) + pad
        if abs((lo + hi) / 2) > half:
            continue
        t.in_view = True
        for bl, br, b in boxes:
            inter = min(hi, br) - max(lo, bl)
            if inter > 0:
                cands.append((inter / (max(hi, br) - min(lo, bl)), id(t), t, id(b), b))
    for t in tracks:
        if getattr(t, "in_view", False):
            for k in t.votes:
                t.votes[k] *= decay
            t.in_view = False
    used_t, used_b = set(), set()
    for iou, tid, t, bid, b in sorted(cands, key=lambda c: -c[0]):
        if iou < 0.25 or tid in used_t or bid in used_b:
            continue
        used_t.add(tid)
        used_b.add(bid)
        t.votes[b["cls"]] = t.votes.get(b["cls"], 0.0) + b.get("conf", 0.5)


class Decision:
    def __init__(self):
        self.v_target, self.a_brake, self.state = 0.0, None, "cruise"
        self.gap = self.a_req = None
        self.threat = None
        self.nudge = 0.0


class Planner:
    def __init__(self, p):
        self.p = p
        self.T = np.arange(p.step_s, p.horizon_s + 1e-9, p.step_s)
        self.aeb_until, self.aeb_a = 0.0, 0.0

    def _path(self, v, yaw_rate, lane):
        p = self.p
        if lane and lane.get("valid"):
            o, h, k = lane["offset"], lane["heading"], lane.get("curv", 0.0)
            return lambda x: (o + math.tan(h) * x + 0.5 * k * x * x) * np.minimum(np.maximum(x, 0.0) / p.conv_m, 1.0)
        kappa = float(np.clip(yaw_rate / max(v, 0.5), -0.3, 0.3))
        return lambda x: 0.5 * kappa * x * x

    def _grid(self, pts, vel, v, path, w, cls):
        p, T = self.p, self.T
        P = pts[None, :, :] + vel[None, None, :] * T[:, None, None]
        x, y = P[..., 0], P[..., 1]
        lat = np.abs(y - path(x))
        lon = x - v * T[:, None]  # point position relative to the kart's centre line of travel
        in_back = lon > -p.rear_m
        a, b = CLASS_INFL.get(cls, CLASS_INFL["unknown"])
        infl = np.minimum(a + b * T, 1.5)[:, None]
        hard = in_back & (lon < p.front_m) & (lat < w)
        soft = in_back & (lon < p.front_m + 0.5) & (lat < w + infl)
        return x, hard, soft

    @staticmethod
    def _first(mask):
        rows = mask.any(axis=1)
        return int(np.argmax(rows)) if rows.any() else None

    def plan(self, now, v, yaw_rate, tracks, lane, lidar_age):
        p = self.p
        d = Decision()
        if lidar_age > p.lidar_timeout_s:
            d.state, d.v_target, d.a_brake = "LIDAR-LOST", 0.0, p.a_comf * 1.5
            return d
        w = p.kart_width / 2 + p.margin
        path = self._path(v, yaw_rate, lane)
        lead = None          # (gap, speed along path, track)
        a_req, threat = 0.0, None
        caps = []            # (speed cap, track, hard?)
        side = None          # (x, lateral) of the nearest object next to the corridor
        for t in tracks:
            if t.pts_k is None or len(t.pts_k) == 0 or t.hits < 3 or now - t.t0 < p.min_age_s or now - t.last_hit > 0.35:
                continue
            pts = t.pts_k
            if len(pts) > 40:
                pts = pts[np.argsort(pts[:, 1])][np.linspace(0, len(pts) - 1, 40).astype(int)]
            pts = pts[pts[:, 0] > p.front_m - 0.1]  # only what is ahead of the bumper can be braked for
            if len(pts) == 0:
                continue
            lat0 = pts[:, 1] - path(pts[:, 0])
            inside = np.abs(lat0) < w
            near = pts[:, 0] < p.front_m + 12.0
            if inside.any():
                gap = float(pts[inside, 0].min()) - p.front_m
                if lead is None or gap < lead[0]:
                    lead = (gap, float(t.vel_k[0]), t)
                continue
            i = int(np.argmin(np.where(near, np.abs(lat0), 1e9)))
            if near[i] and abs(lat0[i]) < w + 0.8 and (side is None or pts[i, 0] < side[0]):
                side = (float(pts[i, 0]), float(lat0[i]))
            x, hard, soft = self._grid(pts, t.vel_k, v, path, w, t.cls)
            ih = self._first(hard)
            if ih is not None:
                s = float(x[ih][hard[ih]].min()) - p.front_m
                a = v * v / (2 * max(s - p.stop_margin, 0.05))
                caps.append((math.sqrt(2 * p.a_comf * max(s - p.stop_margin, 0.0)), t, True))
                if a > a_req:
                    a_req, threat = a, t
                continue
            isf = self._first(soft)
            if isf is not None and t.cls != "pole/barrier":
                s = float(x[isf][soft[isf]].min()) - p.front_m
                caps.append((max(math.sqrt(2 * p.a_comf * max(s - p.stop_margin, 0.0)), p.v_creep), t, False))

        v_target = p.v_set
        if lead is not None:
            gap, v_l, t = lead
            d.gap = gap
            d_des = p.d_min + p.headway_s * v
            v_target = min(v_target, min(max(v_l, 0.0) + p.k_gap * (gap - d_des), p.v_set))
            if v > v_l:
                a_rel = (v - v_l) ** 2 / (2 * max(gap - p.stop_margin, 0.05))
                if a_rel > a_req:
                    a_req, threat = a_rel, t
        for cap, t, hard in caps:
            v_target = min(v_target, cap)
            if threat is None:
                threat = t
        v_target = max(v_target, 0.0)

        d.a_req, d.threat, d.v_target = a_req, threat, v_target
        if a_req >= p.a_aeb:
            self.aeb_until, self.aeb_a = now + 0.5, min(p.a_max, max(1.25 * a_req, p.a_aeb))
        if (a_req >= p.a_aeb or now < self.aeb_until) and v > 0.1:
            d.a_brake, d.state = self.aeb_a, "AEB"
        elif lead is not None and v_target < p.v_set - 0.2:
            d.state = "follow"
        elif caps and v_target < p.v_set - 0.2:
            d.state = "brake" if any(h for _, _, h in caps) else "caution"

        if p.nudge and side is not None and lane and lane.get("valid") and lane.get("width"):
            room = lane["width"] / 2 - p.kart_width / 2 - p.edge_margin
            if abs(side[1]) > 0.25 and room > 0.05:
                d.nudge = -math.copysign(min(p.max_nudge, room), side[1])
        return d


class SpeedController:
    """Maps the planner's target speed / emergency decel to throttle and brake."""

    def __init__(self, a_max=6.0, max_throttle=0.6):
        self.a_max, self.max_throttle, self.integ = a_max, max_throttle, 0.0

    def update(self, v, v_target, a_brake, dt):
        if a_brake:
            self.integ = 0.0
            return 0.0, min(1.0, a_brake / self.a_max)
        if v_target < 0.05:
            self.integ = 0.0
            return 0.0, 0.35 if v < 0.3 else min(1.0, 1.5 * v / self.a_max + 0.15)
        err = v_target - v
        a_des = max(-self.a_max, min(1.2, 1.5 * err))
        if a_des < -0.2:
            self.integ = 0.0
            return 0.0, min(1.0, -a_des / self.a_max)
        self.integ = max(0.0, min(0.6, self.integ + 0.15 * err * dt))
        return max(0.0, min(self.max_throttle, 0.3 * err + self.integ)), 0.0
