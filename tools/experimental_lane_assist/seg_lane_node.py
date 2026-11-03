#!/usr/bin/env python3
"""TwinLiteNet lane/drivable masks -> ground-plane homography -> lateral offset and heading.

Input is camera frames only (bridge UDP JPEG stream, a folder, or a ROS 2 image topic).
No CARLA ground truth is used. Output: lane-centre offset and heading error in the kart
frame (x forward, y right, metres, like the lidar), shown in a window and optionally
published on ROS 2 as std_msgs/Float32MultiArray on /lane/state:
    [valid, offset_m, heading_rad, lookahead_y_m, confidence]
offset_m > 0 means the lane centre is to the right of the kart.

Run (bridge as in the README, 640x360, same --cam-fov/--cam-x/--cam-z as given here):
    python seg_lane_node.py                      # UDP 5600, torch TwinLiteNet
    python seg_lane_node.py --folder frames --loop
    python seg_lane_node.py --selftest           # geometry + estimator check, no model needed
    python seg_lane_node.py --ros                # also publish /lane/state (needs rclpy)
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_camera_harness lives in tools/

SRC_COLORS = {"line": (0, 255, 255), "line+": (0, 165, 255), "road": (255, 120, 0)}


class Ground:
    """Flat-ground camera model. Camera at (cam_x, 0, height), pitch positive = nose up (CARLA)."""

    def __init__(self, width, height, fov_deg, cam_x, cam_height, pitch_deg=0.0):
        self.w, self.h = width, height
        self.f = (width / 2) / math.tan(math.radians(fov_deg) / 2)
        self.cx, self.cy = width / 2, height / 2
        self.cam_x, self.hgt = cam_x, cam_height
        self.phi = math.radians(-pitch_deg)  # downward tilt
        self.sp, self.cp = math.sin(self.phi), math.cos(self.phi)

    def pix_to_ground(self, u, v):
        """Pixel -> (X forward, Y right) on the road plane. NaN at or above the horizon."""
        u, v = np.asarray(u, float), np.asarray(v, float)
        a, b = (u - self.cx) / self.f, (v - self.cy) / self.f
        den = self.sp + b * self.cp
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(den > 1e-6, self.hgt / den, np.nan)
        return self.cam_x + t * (self.cp + b * self.sp), t * a

    def ground_to_pix(self, X, Y):
        dx = np.asarray(X, float) - self.cam_x
        zc = dx * self.cp + self.hgt * self.sp
        yc = dx * self.sp + self.hgt * self.cp
        with np.errstate(divide="ignore", invalid="ignore"):
            ok = zc > 0.05
            u = np.where(ok, self.cx + self.f * np.asarray(Y, float) / zc, np.nan)
            v = np.where(ok, self.cy + self.f * yc / zc, np.nan)
        return u, v


def runs(row):
    """(start, end) index pairs of True runs in a 1-D bool array."""
    d = np.diff(np.concatenate(([0], row.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1))


class LaneEstimator:
    """Tracks the kart's own lane as a left and a right boundary (each a line y = a + b*x).

    Right-hand traffic: the left boundary is the nearest lane line left of the kart (the dashed centre
    line); the right boundary is the nearest lane line to the right, else the right edge of the
    drivable mask. The drivable mask is never used for its middle (it contains the oncoming lane).
    A boundary that disappears (dashed gap) is remembered for --line-hold s, after which the lane
    centre comes from the other boundary and the learned lane width.
    """

    def __init__(self, ground, a):
        self.g, self.a = ground, a
        self.rows_x = [float(s) for s in a.rows.split(",")]
        self.res, self.bx0, self.by = 0.05, 1.0, 6.0
        self.bn = int((max(self.rows_x) + 2.0 - self.bx0) / self.res)
        self.build_bev()
        self.visible = sum(1 for X in self.rows_x if 2 <= self.x_row(X) < self.bn - 2 and
                           np.isfinite(self.g.ground_to_pix(X, 0.0)[1]) and 3 <= self.g.ground_to_pix(X, 0.0)[1] < self.g.h - 3) or 1
        self.crossing = False
        self.bev_keep = self.bev_removed = None
        self.width = a.lane_width
        self.mem = {"L": None, "R": None}  # (a0, b, time)
        self.last_ok = 0.0
        self.state = None  # smoothed [offset, heading, lookahead_y]
        self.curv = 0.0  # smoothed lane curvature, 1/m (+ = bends right)
        self.line_hold = a.line_hold

    def build_bev(self):
        xs = self.bx0 + (self.bn - 1 - np.arange(self.bn)) * self.res  # row 0 = far
        ys = -self.by + np.arange(int(2 * self.by / self.res)) * self.res
        Yg, Xg = np.meshgrid(ys, xs)
        u, v = self.g.ground_to_pix(Xg, Yg)
        ok = np.isfinite(u) & np.isfinite(v)
        self.map_u = np.where(ok, u, -1).astype(np.float32)
        self.map_v = np.where(ok, v, -1).astype(np.float32)
        self.bev_ys = ys

    def to_bev(self, m):
        return cv2.remap(m, self.map_u, self.map_v, cv2.INTER_NEAREST, borderValue=0)

    def x_row(self, X):
        return int(round((self.bx0 + (self.bn - 1) * self.res - X) / self.res))

    def clean_lines(self, ll_bev):
        """Drop transverse marks (stop lines, crosswalk bars) and crosswalk stripe groups.

        Works on the top-down line mask. Returns (kept mask, removed mask, crossing flag)."""
        a, res = self.a, self.res
        mask = cv2.morphologyEx(ll_bev, cv2.MORPH_CLOSE, np.ones((5, 3), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        keep = np.zeros_like(mask)
        removed = np.zeros_like(mask)
        comps = []
        for k in range(1, n):
            if st[k, cv2.CC_STAT_AREA] < a.min_blob_px:
                continue
            r, c = np.nonzero(lab == k)
            x = self.bx0 + (self.bn - 1 - r) * res
            y = self.bev_ys[c]
            lx, ly = np.ptp(x), np.ptp(y)
            cov = np.cov(np.stack([x, y])) if len(x) > 2 else np.eye(2)
            w, vec = np.linalg.eigh(cov)
            ang = math.degrees(math.atan2(abs(vec[1, 1]), abs(vec[0, 1])))  # principal axis angle from forward
            comps.append(dict(k=k, x0=x.min(), x1=x.max(), yc=float(y.mean()), lx=lx, ly=ly, ang=ang, ok=True))
            cp = comps[-1]
            if ang > a.max_line_angle or (ly > a.max_blob_width and ly > 0.6 * lx) or (lx < a.min_line_len and ly > 0.6 * lx):
                cp["ok"] = False
        # crosswalk: three or more short stripes side by side (similar x range, small lateral spacing)
        short = [c for c in comps if c["ok"] and c["lx"] < a.max_stripe_len]
        short.sort(key=lambda c: c["yc"])
        crossing = any(c["ok"] is False and c["ly"] > a.max_blob_width for c in comps)
        for i in range(len(short)):
            grp = [short[i]]
            for c in short[i + 1:]:
                last = grp[-1]
                if c["yc"] - last["yc"] <= a.stripe_gap and min(c["x1"], last["x1"]) - max(c["x0"], last["x0"]) > 0.3 * min(c["lx"], last["lx"]):
                    grp.append(c)
            if len(grp) >= 3:
                for c in grp:
                    c["ok"] = False
                crossing = True
        for c in comps:
            keep[lab == c["k"]] = 1 if c["ok"] else 0
            if not c["ok"]:
                removed[lab == c["k"]] = 1
        crossing = crossing or any(not c["ok"] for c in comps)
        return keep, removed, crossing

    def row_points(self, da, ll, now):
        """Per row: nearest line left/right of the kart (y, m) and the right edge of the road (y, m).

        Candidates must lie within --gate of a still-remembered boundary."""
        a = self.a
        da_b = self.to_bev(da)
        keep, removed, self.crossing = self.clean_lines(self.to_bev(ll))
        self.bev_keep, self.bev_removed = keep, removed
        rows = []
        ys = self.bev_ys
        for X in self.rows_x:
            r = self.x_row(X)
            if r < 2 or r >= self.bn - 2:
                continue
            band = slice(r - 2, r + 3)
            left = right = edge = None
            dirty = bool(removed[band].any())  # row overlaps a stop line / crosswalk: take no line from it
            lc, rc = [], []
            for s, e in ([] if dirty else runs(keep[band].any(axis=0))):
                y = float((ys[s] + ys[e]) / 2)
                if abs(y) > a.max_y or (ys[e] - ys[s]) > 0.8:
                    continue
                side = "L" if y < 0 else "R"
                m = self.mem[side]
                if m is not None and now - m[2] <= self.line_hold and abs(y - (m[0] + m[1] * X)) > a.gate:
                    continue
                (lc if side == "L" else rc).append(y)
            # a lane has a left and a right line about one lane width apart: pick that pair, not just the nearest lines
            lo, hi = 0.6 * self.width, 1.5 * self.width
            pairs = [(l, r) for l in lc for r in rc if lo <= r - l <= hi]
            if pairs:
                left, right = min(pairs, key=lambda p: abs((p[1] - p[0]) - self.width) + 0.15 * abs(p[0] + p[1]))
            elif lc or rc:
                nearest = min(lc + rc, key=abs)  # no consistent pair: trust only the line nearest the kart
                left, right = (nearest, None) if nearest < 0 else (None, nearest)
            rr = [q for q in runs(da_b[band].any(axis=0)) if (ys[q[1]] - ys[q[0]]) > 0.5]
            if rr:
                c0 = int(np.searchsorted(ys, 0.0))
                s, e = min(rr, key=lambda q: 0 if q[0] <= c0 <= q[1] else min(abs(q[0] - c0), abs(q[1] - c0)))
                ye = float(ys[e])
                if 0 < ye <= a.max_edge:
                    edge = ye
            rows.append((X, left, right, edge))
        return rows

    def fit_side(self, pts):
        """pts: (x, y, w). Robust line fit, or None."""
        if len(pts) < self.a.min_side:
            return None
        X = np.array([p[0] for p in pts]); Y = np.array([p[1] for p in pts]); W = np.array([p[2] for p in pts])
        keep = np.ones(len(pts), bool)
        for _ in range(2):
            if keep.sum() < self.a.min_side:
                return None
            b, a0 = np.polyfit(X[keep], Y[keep], 1, w=W[keep])
            keep = np.abs(Y - (a0 + b * X)) < self.a.outlier
        if keep.sum() < self.a.min_side:
            return None
        return a0, b, float(W[keep].sum())

    def fit_quad(self, pts):
        """Same points as fit_side, but y = a0 + b*x + c*x^2/2 with c ridge-regularised. Returns (a0, b, c) or None."""
        if len(pts) < 4:
            return None
        X = np.array([p[0] for p in pts]); Y = np.array([p[1] for p in pts]); W = np.array([p[2] for p in pts])
        if X.max() - X.min() < 5.0:
            return None
        keep = np.ones(len(pts), bool)
        coef = None
        for _ in range(2):
            if keep.sum() < 4:
                return None
            A = np.stack([np.ones(keep.sum()), X[keep], X[keep] ** 2 / 2], 1) * W[keep, None]
            A = np.vstack([A, [0.0, 0.0, self.a.curv_ridge]])
            coef = np.linalg.lstsq(A, np.append(Y[keep] * W[keep], 0.0), rcond=None)[0]
            keep = np.abs(Y - (coef[0] + coef[1] * X + coef[2] * X ** 2 / 2)) < self.a.outlier
        if keep.sum() < 4 or abs(coef[2]) > self.a.max_curv:
            return None
        return float(coef[0]), float(coef[1]), float(coef[2])

    @staticmethod
    def move_line(a0, b, pose):
        """Re-express the line y = a0 + b*x after the kart moved by pose (x, y, theta) in its old frame."""
        px, py, th = pose
        c, s_ = math.cos(th), math.sin(th)
        pts = []
        for x in (0.0, 10.0):
            qx, qy = x - px, a0 + b * x - py
            pts.append((qx * c + qy * s_, -qx * s_ + qy * c))
        if abs(pts[1][0] - pts[0][0]) < 1e-3:
            return a0, b
        nb = (pts[1][1] - pts[0][1]) / (pts[1][0] - pts[0][0])
        return pts[0][1] - nb * pts[0][0], nb

    def advance(self, pose):
        """Carry the remembered boundaries and the smoothed state along with the kart's motion."""
        for name, m in self.mem.items():
            if m is not None:
                self.mem[name] = (*self.move_line(m[0], m[1], pose), m[2])
        if self.state is not None:
            a0, b = self.move_line(self.state[0], math.tan(self.state[1]), pose)
            self.state = np.array([a0, math.atan(b), a0 + b * self.a.lookahead])

    def update(self, da, ll, now, pose=None):
        if pose is not None:
            self.line_hold = self.a.line_hold_odom
            self.advance(pose)
        else:
            self.line_hold = self.a.line_hold
        rows = self.row_points(da, ll, now)
        L = [(X, l, 1.0) for X, l, r, e in rows if l is not None]
        R = [(X, r, 1.0) for X, l, r, e in rows if r is not None]
        if len(R) < self.a.min_side:  # no right line: the road edge stands in, at lower weight
            R = R + [(X, e, 0.5) for X, l, r, e in rows if r is None and e is not None]
        fits, fresh, quads = {}, {}, {}
        for name, pts in (("L", L), ("R", R)):
            f = self.fit_side(pts)
            q = self.fit_quad(pts) if f is not None else None
            if q is not None:
                quads[name] = q
            if f is not None:
                self.mem[name] = (f[0], f[1], now)
                fits[name], fresh[name] = (f[0], f[1]), min(1.0, f[2] / self.visible)
            elif self.mem[name] is not None and now - self.mem[name][2] <= self.line_hold:
                fits[name], fresh[name] = self.mem[name][:2], 0.0

        out = {"pts": [], "valid": False, "offset": 0.0, "heading": 0.0, "look_y": 0.0, "curv": 0.0, "conf": 0.0, "keep": None, "rows": rows, "crossing": self.crossing}
        xl = self.a.lookahead
        if "L" in fits and "R" in fits and fresh["L"] > 0 and fresh["R"] > 0:
            w = (fits["R"][0] + fits["R"][1] * xl) - (fits["L"][0] + fits["L"][1] * xl)
            if not self.crossing and 0.7 * self.a.lane_width <= w <= 1.3 * self.a.lane_width:
                self.width += 0.05 * (w - self.width)
        center = None
        if "L" in fits and "R" in fits and abs(fits["L"][1] - fits["R"][1]) > self.a.slope_gap:
            # boundaries not parallel: one is a turn-lane edge or a stray fragment. Keep the one that agrees with the last heading.
            ref = math.tan(self.state[1]) if self.state is not None else 0.0
            keep_side = min("LR", key=lambda k: abs(fits[k][1] - ref))
            drop = "R" if keep_side == "L" else "L"
            fits.pop(drop)
            fresh.pop(drop)
        curv = 0.0
        if "L" in fits and "R" in fits:
            center = ((fits["L"][0] + fits["R"][0]) / 2, (fits["L"][1] + fits["R"][1]) / 2)
            if "L" in quads and "R" in quads:
                qa, qb, qc = [(quads["L"][i] + quads["R"][i]) / 2 for i in range(3)]
                center, curv = (qa, qb), qc
            conf = 1.0 * min(fresh["L"], fresh["R"]) + 0.3 * (fresh["L"] == 0 or fresh["R"] == 0) + 0.3 * max(fresh["L"], fresh["R"])
        elif "L" in fits:
            center, conf = (fits["L"][0] + self.width / 2, fits["L"][1]), 0.6 * fresh["L"] + 0.2
            if "L" in quads:
                center, curv = (quads["L"][0] + self.width / 2, quads["L"][1]), quads["L"][2]
        elif "R" in fits:
            center, conf = (fits["R"][0] - self.width / 2, fits["R"][1]), 0.6 * fresh["R"] + 0.2
            if "R" in quads:
                center, curv = (quads["R"][0] - self.width / 2, quads["R"][1]), quads["R"][2]
        if center is not None:
            conf = min(1.0, conf)
            cur = np.array([center[0], math.atan(center[1]), center[0] + center[1] * xl + curv * xl * xl / 2])
            if abs(cur[0]) > self.a.max_offset or abs(math.degrees(cur[1])) > self.a.max_heading:
                conf = 0.0  # implausible: the kart is not 2 m off the middle of its own lane
            if conf >= self.a.min_conf:
                self.state = cur if self.state is None else self.a.alpha * cur + (1 - self.a.alpha) * self.state
                self.curv = curv if self.last_ok == 0.0 else self.a.alpha * curv + (1 - self.a.alpha) * self.curv
                self.last_ok = now
                out.update(valid=True, conf=conf)
                for X, l, r, e in rows:
                    yc = center[0] + center[1] * X + curv * X * X / 2
                    src = "line" if (l is not None and r is not None) else "line+" if (l is not None or r is not None) else "road"
                    out["pts"].append((X, yc, 1.0, src, l, r if r is not None else e))
        if not out["valid"]:
            if self.state is not None and now - self.last_ok <= self.a.hold:
                out.update(valid=True, held=True, conf=0.0)
            else:
                self.state = None
        if out["valid"] and self.state is not None:
            out.update(offset=float(self.state[0]), heading=float(self.state[1]), look_y=float(self.state[2]), curv=float(self.curv))
        return out


def render(img, da, ll, est, res, fps, ms):
    g = est.g
    out = img.copy()
    if da is not None:
        ov = out.copy()
        ov[da == 1] = (0, 200, 0)
        ov[ll == 1] = (0, 0, 255)
        out = cv2.addWeighted(ov, 0.4, out, 0.6, 0)
    for X in est.rows_x:
        _, v = g.ground_to_pix(X, 0.0)
        if np.isfinite(v) and 0 <= v < g.h:
            cv2.line(out, (0, int(v)), (12, int(v)), (255, 255, 255), 1)
            cv2.putText(out, f"{X:g}m", (14, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    for i, (X, Yc, _, src, l, r) in enumerate(res["pts"]):
        u, v = g.ground_to_pix(X, Yc)
        if np.isfinite(u) and np.isfinite(v):
            ok = res["keep"] is None or res["keep"][i]
            cv2.circle(out, (int(u), int(v)), 4, SRC_COLORS[src] if ok else (128, 128, 128), -1)
    if res["valid"]:
        xs = np.linspace(est.rows_x[0], est.rows_x[-1], 12)
        ys = res["offset"] + math.tan(res["heading"]) * xs
        u, v = g.ground_to_pix(xs, ys)
        for k in range(len(xs) - 1):
            if np.isfinite(u[k]) and np.isfinite(u[k + 1]):
                cv2.line(out, (int(u[k]), int(v[k])), (int(u[k + 1]), int(v[k + 1])), (255, 0, 255), 2)

    bev = np.full((g.h, 360, 3), 30, np.uint8)
    xmax, ymax = 16.0, 5.0
    sc = g.h / xmax

    def tp(X, Y):
        return int(bev.shape[1] / 2 + Y * sc * 0.9), int(g.h - X * sc)

    for X in range(0, int(xmax) + 1, 2):
        cv2.line(bev, tp(X, -ymax), tp(X, ymax), (60, 60, 60), 1)
        cv2.putText(bev, str(X), (2, tp(X, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
    for Y in range(-4, 5, 2):
        cv2.line(bev, tp(0, Y), tp(xmax, Y), (60, 60, 60), 1)
    cv2.rectangle(bev, tp(0.3, -0.5), tp(-0.8, 0.5), (200, 200, 200), -1)
    if est.bev_removed is not None:
        for mask, col in ((est.bev_keep, (0, 0, 200)), (est.bev_removed, (0, 140, 255))):
            rr, cc = np.nonzero(mask[::2, ::2])
            for r_, c_ in zip(rr * 2, cc * 2):
                X = est.bx0 + (est.bn - 1 - r_) * est.res
                if X <= xmax:
                    cv2.circle(bev, tp(X, est.bev_ys[c_]), 1, col, -1)
    for X, Yc, _, src, l, r in res["pts"]:
        for y in (l, r):
            if y is not None:
                cv2.circle(bev, tp(X, y), 3, (0, 0, 255), -1)
        cv2.circle(bev, tp(X, Yc), 4, SRC_COLORS[src], -1)
    if res["valid"]:
        cv2.line(bev, tp(0, res["offset"]), tp(xmax, res["offset"] + math.tan(res["heading"]) * xmax), (255, 0, 255), 2)
        cv2.circle(bev, tp(est.a.lookahead, res["look_y"]), 6, (0, 255, 0), 2)

    txt = (f"offset {res['offset'] * 100:+.0f} cm  heading {math.degrees(res['heading']):+.1f} deg  conf {res['conf']:.2f}"
           if res["valid"] else "LANE LOST")
    if res.get("held"):
        txt += "  (held)"
    if res.get("crossing"):
        txt += "  [marks ignored]"
    cv2.putText(out, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if res["valid"] else (0, 0, 255), 2)
    cv2.putText(out, f"{fps:.1f} fps  seg {ms:.0f} ms", (8, g.h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(bev, "top view (+y right)", (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return np.hstack([out, bev])


def synth(g, a, off, yaw_deg, left_on=True, right_on=True, marks=()):
    """Masks for a kart at lateral offset `off` in a lane of width a.lane_width, with an oncoming lane to the left."""
    half = a.lane_width / 2
    t = math.tan(math.radians(yaw_deg))
    da, ll = np.zeros((g.h, g.w), np.uint8), np.zeros((g.h, g.w), np.uint8)
    xs = np.linspace(2.0, 25.0, 200)
    for on, y0 in ((left_on, -half), (right_on, half)):
        if on:
            u, v = g.ground_to_pix(xs, off + t * xs + y0)
            m = np.isfinite(u) & np.isfinite(v)
            cv2.polylines(ll, [np.stack([u[m], v[m]], 1).astype(np.int32)], False, 1, 3)
    for kind, x0 in marks:
        polys = []
        if kind == "stop":  # transverse bar across the kart's lane
            polys = [(x0, x0 + 0.4, off - half, off + half)]
        elif kind == "zebra":  # stripes along the travel direction, across the whole road
            polys = [(x0, x0 + 3.5, yy, yy + 0.5) for yy in np.arange(-5.0, 4.0, 1.0)]
        elif kind == "ladder":  # bars across the road
            polys = [(xx, xx + 0.5, -4.5, 3.0) for xx in np.arange(x0, x0 + 4.0, 1.0)]
        for xa, xb, ya, yb in polys:
            qx, qy = np.array([xa, xb, xb, xa]), np.array([ya, ya, yb, yb])
            u, v = g.ground_to_pix(qx, qy)
            if np.isfinite(u).all():
                cv2.fillPoly(ll, [np.stack([u, v], 1).astype(np.int32)], 1)
    lu, lv = g.ground_to_pix(xs, off + t * xs - half - a.lane_width)  # oncoming lane is drivable too
    ru, rv = g.ground_to_pix(xs, off + t * xs + half)
    m = np.isfinite(lu) & np.isfinite(ru)
    cv2.fillPoly(da, [np.concatenate([np.stack([lu[m], lv[m]], 1), np.stack([ru[m], rv[m]], 1)[::-1]]).astype(np.int32)], 1)
    return da, ll


def selftest(a):
    """Synthetic masks through the homography with known lane geometry."""
    g = Ground(a.cam_width, a.cam_height, a.cam_fov, a.cam_x, a.cam_height_m, a.cam_pitch)
    bad = False
    for off, yaw_deg, lon, ron in [(0.0, 0.0, 1, 1), (0.5, 0.0, 1, 1), (-0.7, 5.0, 1, 1), (0.4, -8.0, 0, 1),
                                   (-0.3, 3.0, 1, 0), (0.2, 0.0, 0, 0)]:
        est = LaneEstimator(g, a)
        res = est.update(*synth(g, a, off, yaw_deg, lon, ron), 0.0)
        e_off = res["offset"] - off
        e_hd = math.degrees(res["heading"]) - yaw_deg
        ok = res["valid"] and abs(e_off) < 0.12 and abs(e_hd) < 2.0
        bad |= not ok
        print(f"off {off:+.2f} yaw {yaw_deg:+.1f} left={lon} right={ron}: est off {res['offset']:+.3f} "
              f"hd {math.degrees(res['heading']):+.2f} conf {res['conf']:.2f} err {e_off * 100:+.1f} cm  {'ok' if ok else 'FAIL'}")

    # dashed centre line: line visible 4 frames, gone 12, visible again. Offset must not move toward the road middle.
    est = LaneEstimator(g, a)
    worst = 0.0
    off = 0.2
    for k in range(40):
        gap = 4 <= k % 16 < 16
        res = est.update(*synth(g, a, off, 0.0, left_on=not gap, right_on=True), k * 0.1)
        if k >= 2:
            worst = max(worst, abs(res["offset"] - off))
    ok = worst < 0.12
    bad |= not ok
    print(f"dashed centre line, right edge only in gaps: worst error {worst * 100:.1f} cm  {'ok' if ok else 'FAIL'}")
    for kind in ("stop", "zebra", "ladder"):
        est = LaneEstimator(g, a)
        worst, lost = 0.0, 0
        off = 0.3
        for k in range(60):
            x0 = 14.0 - 0.3 * k  # mark approaches the kart at ~3 m/s, 10 Hz
            gap = 4 <= k % 16 < 16
            res = est.update(*synth(g, a, off, 0.0, left_on=not gap or k < 4, right_on=True, marks=[(kind, x0)]), k * 0.1)
            if k >= 3:
                worst = max(worst, abs(res["offset"] - off)) if res["valid"] else worst
                lost += not res["valid"]
        ok = worst < 0.15 and lost == 0
        bad |= not ok
        print(f"{kind:6s} approaching + dashed centre line: worst error {worst * 100:.1f} cm, lost frames {lost}  {'ok' if ok else 'FAIL'}")
    # odometry: pure rotation of the forward axis, then a blind stretch with the kart yawed to the lane
    b = LaneEstimator.move_line(0.0, 0.0, (0.0, 0.0, math.radians(10)))[1]
    ok = abs(b + math.tan(math.radians(10))) < 1e-6
    bad |= not ok
    print(f"rotate 10 deg right: forward line slope {b:+.4f} (want {-math.tan(math.radians(10)):+.4f})  {'ok' if ok else 'FAIL'}")
    t = WheelOdom.__new__(WheelOdom)
    t.track, t.max_age, t.pose, t.last, t.v, t.w, t.recv = a.track_width, 0.5, [0.0, 0.0, 0.0], None, 0.0, 0.0, 0.0
    import threading
    t.lock = threading.Lock()
    for k in range(101):
        t.feed(2.0 + 0.2 * a.track_width / 2 * 0, 2.0, k * 0.01)  # straight, 2 m/s for 1 s
    ok = abs(t.pose[0] - 2.0) < 0.05 and abs(t.pose[2]) < 1e-9
    bad |= not ok
    print(f"wheel odometry straight 1 s at 2 m/s: x {t.pose[0]:.3f} m  {'ok' if ok else 'FAIL'}")
    t.pose, t.last = [0.0, 0.0, 0.0], None
    for k in range(101):
        t.feed(2.0, 1.8, k * 0.01)  # left faster: turning right, w = 0.2 / track
    want = 0.2 / a.track_width
    ok = abs(t.pose[2] - want) < 1e-6 and t.pose[1] > 0
    bad |= not ok
    print(f"wheel odometry right turn: yaw {math.degrees(t.pose[2]):.2f} deg (want {math.degrees(want):.2f}), y {t.pose[1]:.3f} m  {'ok' if ok else 'FAIL'}")
    for use_odom in (False, True):
        est = LaneEstimator(g, a)
        off0, yaw, v, dt = -0.5, 3.0, 3.0, 0.1
        worst, lost = 0.0, 0
        for k in range(30):
            true_off = off0 + math.tan(math.radians(yaw)) * v * dt * k
            blind = k >= 4
            da, ll = (np.zeros((g.h, g.w), np.uint8),) * 2 if blind else synth(g, a, true_off, yaw)
            res = est.update(da, ll, k * dt, (v * dt, 0.0, 0.0) if use_odom and k else None)
            if k >= 4:
                lost += not res["valid"]
                if res["valid"]:
                    worst = max(worst, abs(res["offset"] - true_off))
        ok = (not use_odom) or (worst < 0.1 and lost == 0)
        bad |= not ok
        print(f"2.6 s blind, kart yawed {yaw:.0f} deg to lane, odometry {'on ' if use_odom else 'off'}: worst error {worst * 100:.1f} cm, "
              f"lost frames {lost}  {'ok' if ok else 'FAIL'}")
    # a curved line (bend) is wide but long: it must not be taken for a stop line
    ll = np.zeros((g.h, g.w), np.uint8)
    xs = np.linspace(3.0, 16.0, 60)
    for y0 in (-1.5, 1.5):
        u, v = g.ground_to_pix(xs, y0 + 0.008 * xs ** 2)  # bend of ~60 m radius
        m = np.isfinite(u)
        cv2.polylines(ll, [np.stack([u[m], v[m]], 1).astype(np.int32)], False, 1, 3)
    est = LaneEstimator(g, a)
    res = est.update(np.zeros((g.h, g.w), np.uint8), ll, 0.0)
    ok = res["valid"] and not res["crossing"] and abs(res["look_y"] - (0.008 * 36)) < 0.1 and abs(res["offset"]) < 0.1
    bad |= not ok
    print(f"curved lane lines kept: valid={res['valid']} crossing={res['crossing']} offset {res['offset']:+.2f} (want 0) look_y {res['look_y']:+.2f} (want {0.008 * 36:+.2f}) curv {res.get('curv', 0):+.3f} (want {0.016:+.3f})  {'ok' if ok else 'FAIL'}")
    # right boundary diverges (turn lane) while the left one stays parallel: heading must stay ~0
    ll = np.zeros((g.h, g.w), np.uint8)
    xs = np.linspace(3.0, 16.0, 60)
    for ys_ in (-1.5 + 0 * xs, 1.5 + 0.35 * (xs - 3.0)):
        u, v = g.ground_to_pix(xs, ys_)
        m = np.isfinite(u)
        cv2.polylines(ll, [np.stack([u[m], v[m]], 1).astype(np.int32)], False, 1, 3)
    est = LaneEstimator(g, a)
    res = est.update(np.zeros((g.h, g.w), np.uint8), ll, 0.0)
    ok = res["valid"] and abs(res["heading"]) < 0.06
    bad |= not ok
    print(f"diverging right line: heading {math.degrees(res['heading']):+.1f} deg (want ~0)  {'ok' if ok else 'FAIL'}")
    px = g.ground_to_pix(*g.pix_to_ground(200.0, 250.0))
    print(f"round trip pixel error {abs(px[0] - 200.0) + abs(px[1] - 250.0):.2e}")
    print("SELFTEST", "FAIL" if bad else "PASS")
    return 1 if bad else 0


class WheelOdom:
    """Integrates left/right wheel speeds (UDP JSON from the bridge, or CAN later) into a pose change.

    Pose is (x forward, y right, theta clockwise) accumulated since the last take(), in the frame the
    kart had then. Yaw rate = (v_left - v_right) / track, the same sign as the bridge."""

    def __init__(self, port, track, max_age=0.5):
        import json
        import socket
        import threading
        self.track, self.max_age = track, max_age
        self.lock = threading.Lock()
        self.pose = [0.0, 0.0, 0.0]
        self.last = None
        self.v = self.w = 0.0
        self.recv = 0.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.json = json
        threading.Thread(target=self.run, daemon=True).start()

    def feed(self, vl, vr, t):
        with self.lock:
            v, w = (vl + vr) / 2, (vl - vr) / self.track
            if self.last is not None and 0 < t - self.last <= self.max_age:
                dt = t - self.last
                th = self.pose[2] + 0.5 * w * dt  # midpoint heading over the step
                self.pose[0] += v * dt * math.cos(th)
                self.pose[1] += v * dt * math.sin(th)
                self.pose[2] += w * dt
            self.last, self.v, self.w, self.recv = t, v, w, time.time()

    def run(self):
        while True:
            try:
                d = self.json.loads(self.sock.recv(2048))
                self.feed(float(d["vl"]), float(d["vr"]), float(d["t"]))
            except (OSError, ValueError, KeyError):
                pass

    def take(self):
        """Pose change since the last call, or None while the stream is stale or absent."""
        with self.lock:
            if time.time() - self.recv > self.max_age:
                self.pose = [0.0, 0.0, 0.0]
                return None
            p, self.pose = tuple(self.pose), [0.0, 0.0, 0.0]
            return p


class RosIO:
    def __init__(self, image_topic, state_topic):
        import rclpy
        from std_msgs.msg import Float32MultiArray
        self.rclpy, self.msg = rclpy, Float32MultiArray
        rclpy.init()
        self.node = rclpy.create_node("seg_lane_node")
        self.pub = self.node.create_publisher(Float32MultiArray, state_topic, 10)
        self.latest = None
        if image_topic:
            from sensor_msgs.msg import Image
            self.node.create_subscription(Image, image_topic, self.on_image, 1)

    def on_image(self, m):
        buf = np.frombuffer(m.data, np.uint8)
        if m.encoding in ("bgr8", "rgb8"):
            img = buf.reshape(m.height, m.step // 3, 3)[:, :m.width]
            self.latest = img if m.encoding == "bgr8" else img[:, :, ::-1].copy()
        elif m.encoding in ("bgra8", "rgba8"):
            img = buf.reshape(m.height, m.step // 4, 4)[:, :m.width, :3]
            self.latest = img if m.encoding == "bgra8" else img[:, :, ::-1].copy()

    def frames(self):
        while self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest is not None:
                img, self.latest = self.latest, None
                yield "ros", img

    def publish(self, res):
        m = self.msg()
        m.data = [float(res["valid"]), res["offset"], res["heading"], res["look_y"], res["conf"]]
        self.pub.publish(m)
        self.rclpy.spin_once(self.node, timeout_sec=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["torch", "ncnn"], default="torch")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--folder")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--ros", action="store_true", help="publish --state-topic via rclpy")
    ap.add_argument("--ros-image", help="read frames from this sensor_msgs/Image topic instead of UDP")
    ap.add_argument("--odom-port", type=int, default=5603, help="UDP port of the bridge wheel speeds (0 = no odometry)")
    ap.add_argument("--track-width", type=float, default=0.9, help="m, wheel track (bridge --track)")
    ap.add_argument("--state-udp", default="127.0.0.1:5604", help="host:port, lane state as JSON datagrams for lane_drive.py ('' = off)")
    ap.add_argument("--state-topic", default="/lane/state")
    g = ap.add_argument_group("camera (must match the bridge)")
    g.add_argument("--cam-width", type=int, default=640)
    g.add_argument("--cam-height", type=int, default=360)
    g.add_argument("--cam-fov", type=float, default=70.4)
    g.add_argument("--cam-x", type=float, default=0.8, help="camera forward offset from kart origin, m (bridge --cam-x)")
    g.add_argument("--cam-height-m", type=float, default=1.5, help="camera height above the road, m (bridge --cam-z)")
    g.add_argument("--cam-pitch", type=float, default=0.0, help="degrees, positive = nose up (CARLA)")
    e = ap.add_argument_group("lane estimator")
    e.add_argument("--rows", default="3,4.5,6,8,10,13", help="ground distances (m) where the masks are sampled")
    e.add_argument("--lane-width", type=float, default=3.0, help="used when only one lane line is visible, m")
    e.add_argument("--min-lane", type=float, default=1.5)
    e.add_argument("--max-lane", type=float, default=8.0)
    e.add_argument("--max-y", type=float, default=7.0)
    e.add_argument("--max-edge", type=float, default=3.5, help="m, a drivable-mask edge farther right than this is not trusted as the lane's right boundary")
    e.add_argument("--curv-ridge", type=float, default=2.0, help="penalty that keeps the curvature estimate near 0 unless the data insist")
    e.add_argument("--max-curv", type=float, default=0.1, help="1/m, larger fitted curvatures are ignored")
    e.add_argument("--slope-gap", type=float, default=0.12, help="left/right boundary slopes differing more than this are not parallel: trust one side")
    e.add_argument("--max-heading", type=float, default=20.0, help="deg, larger lane headings are reported as lost")
    e.add_argument("--max-offset", type=float, default=2.0, help="m, larger lane offsets are reported as lost, not followed")
    e.add_argument("--gate", type=float, default=0.7, help="m, a line must be this close to the remembered boundary")
    e.add_argument("--max-line-angle", type=float, default=40.0, help="deg from forward; more transverse marks (stop lines) are dropped")
    e.add_argument("--min-blob-px", type=int, default=12)
    e.add_argument("--max-blob-width", type=float, default=2.5, help="m, lateral extent above which a mark is not a lane line")
    e.add_argument("--min-line-len", type=float, default=0.6, help="m")
    e.add_argument("--max-stripe-len", type=float, default=5.0, help="m, crosswalk stripes are short")
    e.add_argument("--stripe-gap", type=float, default=1.5, help="m, max lateral spacing of crosswalk stripes")
    e.add_argument("--min-side", type=int, default=2, help="rows needed to fit one lane boundary")
    e.add_argument("--line-hold", type=float, default=1.5, help="s to remember a boundary that vanished (dashed gaps)")
    e.add_argument("--line-hold-odom", type=float, default=4.0, help="same, when wheel odometry moves the remembered lines")
    e.add_argument("--min-conf", type=float, default=0.3)
    e.add_argument("--outlier", type=float, default=0.8, help="m, residual to drop a row from the fit")
    e.add_argument("--lookahead", type=float, default=6.0, help="m, x of the published lookahead point")
    e.add_argument("--alpha", type=float, default=0.5, help="EMA weight of the new estimate")
    e.add_argument("--hold", type=float, default=0.3, help="s to keep the last estimate when the lane is lost")
    o = ap.add_argument_group("output")
    o.add_argument("--no-display", action="store_true")
    o.add_argument("--save-dir")
    o.add_argument("--csv", help="write time, valid, offset, heading, lookahead_y, conf per frame")
    o.add_argument("--max-frames", type=int, default=0)
    o.add_argument("--selftest", action="store_true")
    # model arguments consumed by SegTorch / SegNcnn
    s = ap.add_argument_group("TwinLiteNet")
    from ml_camera_harness import MODELS_DIR
    s.add_argument("--seg-repo", default=str(MODELS_DIR / "twinlitenet/TwinLiteNet"))
    s.add_argument("--seg-pth")
    s.add_argument("--seg-param")
    s.add_argument("--seg-bin")
    s.add_argument("--seg-in", default="in0")
    s.add_argument("--seg-out-da", default="out0")
    s.add_argument("--seg-out-ll", default="out1")
    s.add_argument("--seg-width", type=int, default=640)
    s.add_argument("--seg-height", type=int, default=360)
    s.add_argument("--seg-threads", type=int, default=2)
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest(a))

    import ml_camera_harness as ml
    seg = ml.SegTorch(a) if a.backend == "torch" else ml.SegNcnn(a)
    ground = Ground(a.cam_width, a.cam_height, a.cam_fov, a.cam_x, a.cam_height_m, a.cam_pitch)
    est = LaneEstimator(ground, a)

    ros = RosIO(a.ros_image, a.state_topic) if (a.ros or a.ros_image) else None
    if a.ros_image:
        source = ros.frames()
    elif a.folder:
        source = ml.frames_folder(a.folder, a.loop)
    else:
        source = ml.frames_udp(a.port)

    odom = WheelOdom(a.odom_port, a.track_width) if a.odom_port else None
    import json
    import socket
    st_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if a.state_udp else None
    st_addr = (a.state_udp.rsplit(":", 1)[0], int(a.state_udp.rsplit(":", 1)[1])) if a.state_udp else None
    if a.save_dir:
        Path(a.save_dir).mkdir(parents=True, exist_ok=True)
    logf = open(a.csv, "w", newline="") if a.csv else None
    log = csv.writer(logf) if a.csv else None
    if log:
        log.writerow(["t", "valid", "offset_m", "heading_rad", "lookahead_y_m", "conf"])

    n, t0, t_prev, ms_avg = 0, time.time(), time.time(), 0.0
    for _, img in source:
        if img.shape[1] != a.cam_width or img.shape[0] != a.cam_height:
            img = cv2.resize(img, (a.cam_width, a.cam_height))
        t = time.perf_counter()
        da, ll = seg.infer(img)
        ms = (time.perf_counter() - t) * 1000
        ms_avg = ms if n == 0 else 0.9 * ms_avg + 0.1 * ms
        now = time.time()
        res = est.update(da, ll, now, odom.take() if odom else None)
        if ros:
            ros.publish(res)
        if log and n % 10 == 0:
            logf.flush()
        if st_sock:
            st_sock.sendto(json.dumps({"t": now, "valid": bool(res["valid"]), "offset": res["offset"], "heading": res["heading"],
                                       "look_y": res["look_y"], "curv": res.get("curv", 0.0), "conf": res["conf"], "lookahead": a.lookahead, "width": est.width,
                                       "crossing": bool(res.get("crossing")), "held": bool(res.get("held"))}).encode(), st_addr)
        if log:
            log.writerow([f"{now - t0:.3f}", int(res["valid"]), f"{res['offset']:.4f}", f"{res['heading']:.4f}",
                          f"{res['look_y']:.4f}", f"{res['conf']:.3f}"])
        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now
        frame = render(img, da, ll, est, res, fps, ms_avg)
        if a.save_dir:
            cv2.imwrite(str(Path(a.save_dir) / f"{n:06d}.jpg"), frame)
        if a.no_display:
            if n % 10 == 0:
                print(f"{n:5d} valid={res['valid']} offset={res['offset'] * 100:+.0f}cm "
                      f"heading={math.degrees(res['heading']):+.1f}deg conf={res['conf']:.2f} seg={ms_avg:.0f}ms")
        else:
            if n == 0:
                cv2.namedWindow("seg lane node", cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)  # no Qt toolbar, no QFont warnings
            cv2.imshow("seg lane node", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                cv2.imwrite(f"lane_{n:06d}.png", frame)
        n += 1
        if a.max_frames and n >= a.max_frames:
            break


if __name__ == "__main__":
    main()
