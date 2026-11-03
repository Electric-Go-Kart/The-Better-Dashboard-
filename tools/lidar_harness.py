#!/usr/bin/env python3
"""Desktop harness for the real lidar stack: STL-27L parser + masks + clustering + persistence + tracker.

Scans come from the CARLA bridge (UDP, default 5601) or from a built-in synthetic scene (--synthetic,
no CARLA needed). Each scan is re-encoded into real 47-byte STL-27L packets (range noise, min/max range,
CRC) and piped into tools/lidar_harness_cli (C++), which runs the production Backend/stl27lreader.cpp and
Backend/lidarpipeline.cpp. The harness draws the result top-down and scores it against CARLA ground truth.

    # build the C++ driver once (needs Qt Core + SerialPort)
    cmake -S tools -B build/lidar_harness -G Ninja -DCMAKE_PREFIX_PATH=<Qt>/gcc_64 && cmake --build build/lidar_harness

    python lidar_harness.py --synthetic                 # develop without CARLA
    python lidar_harness.py                             # live: carla_vesc_bridge.py running, UDP 5601 + 5602
    python lidar_harness.py --synthetic --no-display --score   # headless pass/fail, exit code 1 on failure

Camera fusion (spec 4.5 steps 3 and 5): boxes are turned into bearing wedges and matched to lidar clusters
inside the C++ pipeline. Box sources: --boxes gt (CARLA ground truth projected through the camera model,
default), --boxes yolo (YOLO-FastestV2 on the bridge camera stream, via ml_camera_harness.py), --boxes none.
Cameras: --cam-layout dual (two cameras at -/+ half the FOV, edges meeting at 0 deg) or single (one, yaw 0).

See tools/README.md. Needs: numpy opencv-python.
"""
import argparse
import json
import math
import os
import queue
import re
import select
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAYS = 2160                  # 21600 points/s at 10 Hz
STEP_DEG = 360.0 / RAYS
PKT_POINTS = 12
REV_S = 0.1

CLASS_COLORS = {"person": (60, 60, 255), "bike": (255, 160, 40), "vehicle": (60, 200, 60),
                "pole/barrier": (0, 200, 255), "cone": (0, 140, 255), "unknown": (200, 200, 200)}


# ---------------------------------------------------------------- geometry / config

def read_kart_geometry(path):
    """Reads the numeric `var name = value` lines of KartGeometry.js."""
    vals = {}
    for m in re.finditer(r"^var\s+(\w+)\s*=\s*([-\d.]+)", Path(path).read_text(), re.M):
        vals[m.group(1)] = float(m.group(2))
    length, width, behind = vals["kartLengthM"], vals["kartWidthM"], vals["lidarBehindCentreM"]
    return {"front": length / 2 + behind, "rear": length / 2 - behind, "half_width": width / 2,
            "range": vals.get("rangeM", 20.0)}


# ---------------------------------------------------------------- STL-27L encoder

def _crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        t.append(c)
    return t


CRC = _crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = CRC[c ^ b]
    return c


class Ring:
    """Latest range per angular bin. CARLA delivers partial rotations, a real STL-27L never does,
    so partial scans are accumulated and emitted as one full revolution."""

    def __init__(self, yaw_deg, mirrored):
        self.yaw, self.mirrored = yaw_deg, mirrored
        self.r = np.zeros(RAYS)
        self.t = np.full(RAYS, -1e9)

    def add(self, xy, now):
        if len(xy) == 0:
            return
        kart = np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))          # y right: clockwise from above
        sensor = (self.yaw - kart) if self.mirrored else (kart - self.yaw)
        idx = np.round((sensor % 360.0) / STEP_DEG).astype(int) % RAYS
        r = np.hypot(xy[:, 0], xy[:, 1])
        for i, rr in zip(idx, r):                                  # keep the nearest return per bin
            if self.t[i] != now or rr < self.r[i]:
                self.r[i], self.t[i] = rr, now

    def revolution(self, now, max_age):
        r = np.where(now - self.t <= max_age, self.r, 0.0)
        return r


def encode_revolution(ranges, t_ms, rng, args):
    """ranges: RAYS metres (0 = no return). Returns bytes of 180 STL-27L packets."""
    r = ranges.copy()
    hit = r > 0
    r[hit] += rng.normal(0.0, args.noise, hit.sum())
    if args.dropout > 0:
        r[rng.random(RAYS) < args.dropout] = 0
    r[(r < args.min_m) | (r > args.max_m)] = 0
    mm = np.clip(np.round(r * 1000), 0, 65535).astype(np.uint16)
    out = bytearray()
    for k in range(RAYS // PKT_POINTS):
        i0 = k * PKT_POINTS
        start = int(round(i0 * STEP_DEG * 100))
        end = int(round(((i0 + PKT_POINTS - 1) * STEP_DEG) * 100)) % 36000
        p = bytearray([0x54, 0x2C])
        p += int(3600).to_bytes(2, "little") + start.to_bytes(2, "little")
        for j in range(PKT_POINTS):
            p += int(mm[i0 + j]).to_bytes(2, "little") + bytes([200 if mm[i0 + j] else 0])
        p += end.to_bytes(2, "little") + int((t_ms + k * 0.55) % 65536).to_bytes(2, "little")
        p.append(crc8(p))
        out += p
    return bytes(out)


def corrupt(data, prob, rng):
    """Flips one byte in a fraction of the 47-byte packets and drops another fraction entirely."""
    if prob <= 0:
        return data
    out = bytearray()
    for i in range(0, len(data), 47):
        pkt = bytearray(data[i:i + 47])
        u = rng.random()
        if u < prob / 2:
            continue
        if u < prob:
            pkt[rng.integers(0, 47)] ^= 1 << int(rng.integers(0, 8))
        out += pkt
    return bytes(out)


# ---------------------------------------------------------------- synthetic CARLA-like scene

class Shape:
    def __init__(self, gid, cls, kind, size, path, structure=False):
        self.id, self.cls, self.kind, self.size, self.path, self.structure = gid, cls, kind, size, path, structure

    def state(self, t, dt=0.01):
        p0 = np.array(self.path(t)); p1 = np.array(self.path(t + dt))
        return p0, (p1 - p0) / dt

    @property
    def extent(self):
        return self.size if self.kind == "circle" else 0.5 * math.hypot(*self.size)


def default_scene():
    car_x = lambda t: -15 + ((4.0 * t) % 40)
    return [
        Shape(1, "person", "circle", 0.3, lambda t: (9.0, 3.0 + 2.5 * math.sin(0.5 * t))),
        Shape(2, "vehicle", "box", (4.2, 1.8), lambda t: (car_x(t), -6.0)),
        Shape(3, "pole/barrier", "circle", 0.1, lambda t: (6.0, 4.0)),
        Shape(4, "cone", "circle", 0.15, lambda t: (12.0, -3.0)),
        Shape(5, "vehicle", "box", (2.0, 1.0), lambda t: (-8.0, 7.0)),
        Shape(6, "pole/barrier", "box", (8.0, 0.3), lambda t: (0.0, -14.0), structure=True),
        # walks across the seam between the two cameras
        Shape(7, "person", "circle", 0.3, lambda t: (14.0, 3.0 * math.sin(0.35 * t))),
    ]


def raycast(shapes, t, yaw_deg=0.0):
    """Ideal 2160-ray scan from the origin. Returns xy points (hits only) and per-shape hit counts."""
    ang = np.radians(np.arange(RAYS) * STEP_DEG)                  # kart frame: clockwise = +y
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    best = np.full(RAYS, np.inf)
    owner = np.full(RAYS, -1)
    for si, s in enumerate(shapes):
        c, _ = s.state(t)
        if s.kind == "circle":
            b = d @ c
            disc = b * b - (c @ c - s.size ** 2)
            with np.errstate(invalid="ignore"):
                tt = b - np.sqrt(disc)
            ok = (disc >= 0) & (tt > 0)
        else:
            lx, ly = s.size[0] / 2, s.size[1] / 2
            o = -c
            tn, tf = np.full(RAYS, -np.inf), np.full(RAYS, np.inf)
            for axis, half in ((0, lx), (1, ly)):
                with np.errstate(divide="ignore", invalid="ignore"):
                    t1 = (-half - o[axis]) / d[:, axis]
                    t2 = (half - o[axis]) / d[:, axis]
                par = np.abs(d[:, axis]) < 1e-9
                inside = abs(o[axis]) <= half
                lo = np.where(par, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
                hi = np.where(par, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
                tn, tf = np.maximum(tn, lo), np.minimum(tf, hi)
            tt = tn
            ok = (tn <= tf) & (tf > 0) & (tn > 0)
        closer = ok & (tt < best)
        best[closer], owner[closer] = tt[closer], si
    hit = np.isfinite(best)
    xy = d[hit] * best[hit, None]
    counts = {s.id: int((owner == i).sum()) for i, s in enumerate(shapes)}
    return xy, counts


def synthetic_gt(shapes, t, counts):
    objs = []
    for s in shapes:
        (x, y), (vx, vy) = s.state(t)
        centre = math.hypot(x, y)
        objs.append({"id": s.id, "class": s.cls, "x": x, "y": y, "vx": float(vx), "vy": float(vy),
                     "d": max(0.0, centre - s.extent), "ext": s.extent,
                     "size": None if s.kind == "circle" else list(s.size), "pts": counts[s.id],
                     "structure": s.structure})
    return objs


# ---------------------------------------------------------------- cameras and boxes

class Cam:
    """Pinhole camera, yaw clockwise from kart forward (left camera negative)."""

    def __init__(self, yaw, fov, width):
        self.yaw, self.fov, self.w = yaw, fov, width
        self.fx = (width / 2) / math.tan(math.radians(fov / 2))

    def lo(self):
        return self.yaw - self.fov / 2

    def hi(self):
        return self.yaw + self.fov / 2

    def u(self, bearing):
        return self.w / 2 + self.fx * math.tan(math.radians(bearing - self.yaw))

    def sees(self, bearing):
        return self.lo() < bearing < self.hi()


def make_cams(args):
    if args.boxes == "none":
        return []
    if args.cam_layout == "single":
        return [Cam(0.0, args.cam_fov, args.cam_width)]
    return [Cam(-args.cam_fov / 2, args.cam_fov, args.cam_width), Cam(args.cam_fov / 2, args.cam_fov, args.cam_width)]


YOLO_CLASSES = ["person", "bike", "vehicle", "pole/barrier"]


def norm_class(c):
    """YOLO has no cone class; a cone can only come out as the closest class."""
    return "pole/barrier" if c == "cone" else c


def gt_boxes(gt, cams, rng, args):
    """Project ground-truth objects into each camera as the boxes a detector would give: visible
    angular span -> pixel columns, nearer objects hide farther ones (by centre bearing), optional
    jitter, misses and class confusion."""
    spans = []
    for g in gt:
        c = math.hypot(g["x"], g["y"])
        if g["x"] <= 0 or c < 0.3 or c > args.box_range:
            continue
        if g.get("size"):
            hx, hy = g["size"][0] / 2, g["size"][1] / 2
            b = [math.degrees(math.atan2(g["y"] + sy * hy, g["x"] + sx * hx)) for sx in (-1, 1) for sy in (-1, 1)]
            a0, a1 = min(b), max(b)
        else:
            half = math.degrees(math.asin(min(1.0, g.get("ext", 0.5) / c)))
            mid = math.degrees(math.atan2(g["y"], g["x"]))
            a0, a1 = mid - half, mid + half
        spans.append((c, a0, a1, g))
    spans.sort(key=lambda s: s[0])
    boxes, occ = [], []
    for _, a0, a1, g in spans:
        mid = (a0 + a1) / 2
        hidden = any(o0 <= mid <= o1 for o0, o1 in occ)
        occ.append((a0, a1))
        if hidden or g["class"] == "unknown" or rng.random() < args.box_miss:
            continue
        cls = norm_class(g["class"])
        if rng.random() < args.box_confuse:
            cls = str(rng.choice([c for c in YOLO_CLASSES if c != cls]))
        for ci, cam in enumerate(cams):
            lo, hi = max(a0, cam.lo() + 1e-6), min(a1, cam.hi() - 1e-6)
            if hi - lo < 0.3:
                continue
            u0 = cam.u(lo) + rng.normal(0, args.box_jitter)
            u1 = cam.u(hi) + rng.normal(0, args.box_jitter)
            boxes.append({"cam": ci, "u0": float(np.clip(u0, 0, cam.w)), "u1": float(np.clip(u1, 0, cam.w)),
                          "cls": cls, "conf": float(rng.uniform(0.55, 0.95))})
    return boxes


class YoloBoxes:
    """Runs the models from ml_camera_harness.py on the bridge's camera JPEG stream in a thread and keeps
    the latest boxes. Single camera only (the bridge streams one)."""

    def __init__(self, args, cam):
        import importlib.util
        import cv2
        self.cv2, self.cam = cv2, cam
        spec = importlib.util.spec_from_file_location("ml_camera_harness", Path(__file__).with_name("ml_camera_harness.py"))
        self.ml = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.ml)
        ns = argparse.Namespace(yolo_repo=str(self.ml.MODELS_DIR / "yolo/Yolo-FastestV2"), yolo_pth=args.yolo_pth,
                                yolo_data=None, device="cpu", yolo_conf=args.yolo_conf, yolo_param=args.yolo_param,
                                yolo_bin=args.yolo_bin, yolo_threads=1, yolo_in="input.1", yolo_out="output,375")
        self.net = self.ml.YoloNcnn(ns) if args.yolo_backend == "ncnn" else self.ml.YoloTorch(ns)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", args.cam_port))
        self.sock.settimeout(1.0)
        self.lock = threading.Lock()
        self.boxes, self.stamp, self.frames, self.ms = [], 0.0, 0, 0.0
        self.max_age = args.box_max_age
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        cv2, ml = self.cv2, self.ml
        while True:
            try:
                data = self.sock.recv(65535)
            except socket.timeout:
                continue
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            t = time.time()
            dets = self.net.infer(img)
            scale = self.cam.w / img.shape[1]
            out = []
            for score, cid, x, y, w, h in dets:
                label = ml.COCO_TO_BEV.get(cid)
                if label:
                    out.append({"cam": 0, "u0": x * scale, "u1": (x + w) * scale, "cls": label, "conf": float(score)})
            with self.lock:
                self.boxes, self.stamp, self.frames, self.ms = out, time.time(), self.frames + 1, (time.time() - t) * 1000

    def get(self):
        with self.lock:
            return list(self.boxes) if time.time() - self.stamp <= self.max_age else []


# ---------------------------------------------------------------- scoring

class Scorer:
    def __init__(self, gate_base, grace, vmin=0.5, verbose=False, cams=()):
        self.cams = cams
        self.cls_total = self.cls_assigned = self.cls_correct = 0
        self.fused_total = self.fused_hits = 0
        self.verbose = verbose
        self.timed = set()
        self.gate_base, self.grace, self.vmin = gate_base, grace, vmin
        self.tp = self.fp = self.fn = 0
        self.errs, self.verrs, self.lat = [], [], []
        self.streak, self.first_seen, self.last_track = {}, {}, {}
        self.switches = 0
        self.frames = 0

    def update(self, tracks, gt, range_m, frame):
        self.frames += 1
        visible = [g for g in gt if not g.get("structure") and g["d"] <= range_m and g.get("pts", 3) >= 3]
        seen_ids = {g["id"] for g in visible}
        for gid in list(self.streak):
            if gid not in seen_ids:
                self.streak.pop(gid); self.first_seen.pop(gid, None); self.last_track.pop(gid, None); self.timed.discard(gid)
        for g in visible:
            self.streak[g["id"]] = self.streak.get(g["id"], 0) + 1
            self.first_seen.setdefault(g["id"], frame)
        pairs = []
        for ti, t in enumerate(tracks):
            for gi, g in enumerate(visible):
                e = math.hypot(t["x"] - g["x"], t["y"] - g["y"])
                if e <= self.gate_base + g.get("ext", 0.5):
                    pairs.append((e, ti, gi))
        pairs.sort()
        tu, gu, matches = set(), set(), []
        for e, ti, gi in pairs:
            if ti in tu or gi in gu:
                continue
            tu.add(ti); gu.add(gi); matches.append((ti, gi, e))
        unmatched_tracks = [t for i, t in enumerate(tracks) if i not in tu]
        missed = [g for i, g in enumerate(visible) if i not in gu]
        for ti, gi, e in matches:
            g, t = visible[gi], tracks[ti]
            self.tp += 1
            self.errs.append(e)
            if math.hypot(g["vx"], g["vy"]) > self.vmin:
                self.verrs.append(math.hypot(t["vx"] - g["vx"], t["vy"] - g["vy"]))
            if self.cams and g["class"] != "unknown" and any(c.sees(math.degrees(math.atan2(g["y"], g["x"]))) for c in self.cams):
                self.cls_total += 1
                self.fused_total += 1
                self.fused_hits += t.get("src") == "fused"
                if t.get("cls", "unknown") != "unknown":
                    self.cls_assigned += 1
                    self.cls_correct += t["cls"] == norm_class(g["class"])
                    if self.verbose and t["cls"] != norm_class(g["class"]):
                        print(f"[scan {frame}] wrong class GT{g['id']} {g['class']} -> T{t['id']} {t['cls']}")
            prev = self.last_track.get(g["id"])
            if prev is not None and prev != t["id"]:
                self.switches += 1
                if self.verbose:
                    print(f"[scan {frame}] ID switch GT{g['id']}: T{prev} -> T{t['id']} at ({g['x']:.1f},{g['y']:.1f})")
            self.last_track[g["id"]] = t["id"]
            if g["id"] not in self.timed:
                self.timed.add(g["id"])
                f0 = self.first_seen[g["id"]]
                if f0 > 5:          # objects already present at start say nothing about latency
                    self.lat.append((frame - f0) * REV_S)
                    if self.verbose:
                        print(f"[scan {frame}] GT{g['id']} first matched {(frame - f0) * REV_S:.1f} s after entering")
        graced = [g for g in missed if self.streak.get(g["id"], 0) > self.grace]
        self.fn += len(graced)
        self.fp += len(unmatched_tracks)
        return {t["id"] for t in unmatched_tracks}, {visible[gi]["id"] for _, gi, _ in matches}, {g["id"] for g in graced}

    def summary(self):
        prec = self.tp / max(1, self.tp + self.fp)
        rec = self.tp / max(1, self.tp + self.fn)
        return {"frames": self.frames, "precision": prec, "recall": rec,
                "pos_err_mean_m": float(np.mean(self.errs)) if self.errs else float("nan"),
                "pos_err_p95_m": float(np.percentile(self.errs, 95)) if self.errs else float("nan"),
                "vel_err_rms": float(np.sqrt(np.mean(np.square(self.verrs)))) if self.verrs else float("nan"),
                "id_switches": self.switches,
                "class_coverage": self.cls_assigned / self.cls_total if self.cls_total else float("nan"),
                "class_accuracy": self.cls_correct / self.cls_assigned if self.cls_assigned else float("nan"),
                "fused_rate": self.fused_hits / self.fused_total if self.fused_total else float("nan"),
                "detect_latency_mean_s": float(np.mean(self.lat)) if self.lat else float("nan")}


# ---------------------------------------------------------------- display

def draw(frame_json, gt, fp_ids, hit_ids, miss_ids, hud, args, geom):
    import cv2
    size = 820
    scale = (size / 2 - 20) / (args.view_m)
    cx = cy = size // 2
    img = np.full((size, size, 3), 24, np.uint8)

    def px(x, y):
        return int(cx + y * scale), int(cy - x * scale)

    for rr in (5, 10, 20):
        cv2.circle(img, (cx, cy), int(rr * scale), (60, 60, 60), 1)
        cv2.putText(img, f"{rr} m", (cx + 4, cy - int(rr * scale) - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)
    cv2.rectangle(img, px(geom["front"], -geom["half_width"]), px(-geom["rear"], geom["half_width"]), (120, 120, 120), 1)
    cv2.circle(img, (cx, cy), 3, (255, 255, 255), -1)
    for cam in geom.get("cams", []):
        for edge in (cam.lo(), cam.hi()):
            e = math.radians(edge)
            cv2.line(img, (cx, cy), px(args.view_m * math.cos(e), args.view_m * math.sin(e)), (90, 60, 30), 1)
    for b in frame_json.get("boxes", []):
        col = (255, 220, 0) if b["cluster"] >= 0 else (0, 140, 255)
        for edge in (b["a0"], b["a1"]):
            e = math.radians(edge)
            cv2.line(img, (cx, cy), px(args.view_m * math.cos(e), args.view_m * math.sin(e)), col, 1)
        e = math.radians((b["a0"] + b["a1"]) / 2)
        cv2.putText(img, f"{b['cls']} {b['conf']:.2f}" + (f" m{b['match']:.2f}" if b["cluster"] >= 0 else " no lidar"),
                    px((args.view_m - 1.5) * math.cos(e), (args.view_m - 1.5) * math.sin(e)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    for x, y in frame_json["points"]:
        cv2.circle(img, px(x, y), 1, (150, 150, 150), -1)
    for c in frame_json["clusters"]:
        cv2.rectangle(img, px(c["x1"], c["y0"]), px(c["x0"], c["y1"]), (160, 90, 30), 1)
    for g in gt:
        if g["d"] > args.view_m + 3:
            continue
        col = CLASS_COLORS.get(g["class"], (200, 200, 200))
        rad = max(3, int(g.get("ext", 0.5) * scale))
        if g["id"] in miss_ids:
            col = (0, 0, 255)
        if g.get("size"):
            hx, hy = g["size"][0] / 2, g["size"][1] / 2
            cv2.rectangle(img, px(g["x"] + hx, g["y"] - hy), px(g["x"] - hx, g["y"] + hy), col, 1)
        else:
            cv2.circle(img, px(g["x"], g["y"]), rad, col, 1)
        cv2.putText(img, f"GT{g['id'] % 1000} {g['class']}", px(g["x"], g["y"] - 0.0), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    for t in frame_json["tracks"]:
        col = (0, 0, 255) if t["id"] in fp_ids else (60, 220, 60)
        p = px(t["x"], t["y"])
        cv2.drawMarker(img, p, col, cv2.MARKER_CROSS, 12, 2)
        label = f"T{t['id']}" + (f" {t['cls']} {t['cconf']:.2f}" if t.get("cls", "unknown") != "unknown" else "")
        label += " F" if t.get("src") == "fused" else ""
        cv2.putText(img, label, (p[0] + 8, p[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
        if math.hypot(t["vx"], t["vy"]) >= 0.3:
            cv2.arrowedLine(img, p, px(t["x"] + t["vx"], t["y"] + t["vy"]), (255, 255, 0), 2, tipLength=0.25)
    for i, line in enumerate(hud):
        cv2.putText(img, line, (8, 18 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
    return img


# ---------------------------------------------------------------- CLI process

class Cli:
    def __init__(self, args, geom, cams):
        cmd = [args.cli, "--yaw", str(args.mount_yaw), "--min-m", str(args.min_m), "--max-m", str(args.max_m),
               "--body", f"{geom['front']},{geom['rear']},{geom['half_width']}", "--range", str(geom["range"]),
               "--period-ms", str(REV_S * 1000)]
        if args.mirrored:
            cmd.append("--mirrored")
        for cam in cams:
            cmd += ["--camera", f"{cam.yaw},{cam.fov},{cam.w}"]
        for m in args.mask:
            cmd += ["--mask", m]
        for s in args.set:
            cmd += ["--set", s]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.out = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            try:
                self.out.put(json.loads(line))
            except ValueError:
                pass
        self.out.put(None)

    def send(self, data, kind=0):
        try:
            self.proc.stdin.write(bytes([kind]) + len(data).to_bytes(4, "little") + data)
            self.proc.stdin.flush()
        except BrokenPipeError:
            raise SystemExit("lidar_harness_cli exited unexpectedly")

    def close(self):
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        self.proc.wait(timeout=5)


# ---------------------------------------------------------------- sources

class LiveSource:
    """CARLA bridge: scans on UDP lidar-port (int16 cm x/y pairs), ground truth JSON on det-port."""

    def __init__(self, args):
        self.ls = self._bind(args.lidar_port)
        self.ds = self._bind(args.det_port)
        self.gt = []
        self.gt_time = 0.0

    @staticmethod
    def _bind(port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
        except OSError as e:
            raise SystemExit(f"cannot bind UDP {port} ({e}). Close the dash, or use --lidar-port/--det-port "
                             f"with the same ports on the bridge (--cam-host/--lidar-port/--det-port).")
        s.setblocking(False)
        return s

    def poll(self, timeout):
        """Returns list of xy arrays received within timeout."""
        scans = []
        r, _, _ = select.select([self.ls, self.ds], [], [], timeout)
        for s in r:
            try:
                while True:
                    data = s.recv(65535)
                    if s is self.ls:
                        scans.append(np.frombuffer(data, dtype="<i2").reshape(-1, 2).astype(float) / 100.0)
                    else:
                        try:
                            self.gt = json.loads(data).get("objects", [])
                            self.gt_time = time.time()
                        except ValueError:
                            pass
            except BlockingIOError:
                pass
        return scans

    def gt_snapshot(self):
        out = []
        for o in self.gt:
            o = dict(o)
            centre = math.hypot(o["x"], o["y"])
            o["ext"] = max(0.3, centre - o.get("d", centre))
            out.append(o)
        return out


# ---------------------------------------------------------------- main

def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--synthetic", action="store_true", help="built-in moving scene instead of CARLA")
    ap.add_argument("--cli", default=os.environ.get("LIDAR_HARNESS_CLI", str(ROOT / "build/lidar_harness/lidarHarnessCli")))
    ap.add_argument("--geometry", default=str(ROOT / "UI/Dashboard/dashboard/KartGeometry.js"))
    ap.add_argument("--lidar-port", type=int, default=5601)
    ap.add_argument("--det-port", type=int, default=5602, help="bridge ground-truth objects")
    ap.add_argument("--mount-yaw", type=float, default=0.0, help="deg, sensor 0 deg rotated clockwise from kart forward")
    ap.add_argument("--mirrored", action="store_true", help="sensor upside down, angles run counter-clockwise")
    ap.add_argument("--mask", action="append", default=[], metavar="FROM,TO", help="sensor-angle self-hit wedge, deg (repeatable)")
    cg = ap.add_argument_group("camera fusion")
    cg.add_argument("--boxes", choices=["gt", "yolo", "none"], default="gt")
    cg.add_argument("--cam-layout", choices=["dual", "single"], default="dual")
    cg.add_argument("--cam-fov", type=float, default=70.4, help="horizontal FOV of each camera, deg (must match the bridge for yolo)")
    cg.add_argument("--cam-width", type=int, default=640, help="image width, px (must match the bridge for yolo)")
    cg.add_argument("--cam-port", type=int, default=5600, help="bridge camera stream (yolo)")
    cg.add_argument("--box-range", type=float, default=30.0, help="gt boxes: farthest object a detector would see, m")
    cg.add_argument("--box-miss", type=float, default=0.0, help="gt boxes: fraction of detections missed")
    cg.add_argument("--box-confuse", type=float, default=0.0, help="gt boxes: fraction given a wrong class")
    cg.add_argument("--box-jitter", type=float, default=3.0, help="gt boxes: pixel noise on box edges")
    cg.add_argument("--box-max-age", type=float, default=0.5, help="yolo: drop boxes older than this, s")
    cg.add_argument("--yolo-backend", choices=["torch", "ncnn"], default="torch")
    cg.add_argument("--yolo-pth")
    cg.add_argument("--yolo-param")
    cg.add_argument("--yolo-bin")
    cg.add_argument("--yolo-conf", type=float, default=0.3)
    cg.add_argument("--min-class-acc", type=float, default=0.8)
    cg.add_argument("--min-class-cov", type=float, default=0.6)
    ap.add_argument("--noise", type=float, default=0.015, help="range noise sigma, m")
    ap.add_argument("--min-m", type=float, default=0.03)
    ap.add_argument("--max-m", type=float, default=25.0)
    ap.add_argument("--dropout", type=float, default=0.0, help="fraction of rays lost")
    ap.add_argument("--corrupt", type=float, default=0.0, help="fraction of packets dropped/bit-flipped (CRC test)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL", help="pipeline parameter, e.g. gateM=2")
    ap.add_argument("--duration", type=float, default=0, help="seconds (0 = until closed/Ctrl-C)")
    ap.add_argument("--realtime", action="store_true", help="pace the synthetic scene at 10 Hz (default when displaying)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--view-m", type=float, default=22.0)
    ap.add_argument("--save-dir", help="write annotated frames here")
    ap.add_argument("--score", action="store_true", help="print metrics at exit and fail on thresholds")
    ap.add_argument("--gate", type=float, default=1.0, help="match gate = this + object half-size, m")
    ap.add_argument("--grace", type=int, default=4, help="scans an object may be unmatched before it counts as missed")
    ap.add_argument("--min-precision", type=float, default=0.9)
    ap.add_argument("--min-recall", type=float, default=0.85)
    ap.add_argument("--max-id-switches", type=int, default=2)
    ap.add_argument("--max-pos-err", type=float, default=1.0)
    ap.add_argument("--verbose", action="store_true", help="print ID switches and detection events")
    ap.add_argument("--json-out", help="write the metrics summary here")
    return ap.parse_args()


def main():
    args = parse_args()
    if not Path(args.cli).exists():
        sys.exit(f"{args.cli} not found. Build it first (see the top of this file).")
    geom = read_kart_geometry(args.geometry)
    rng = np.random.default_rng(args.seed)
    if args.synthetic and args.boxes == "yolo":
        sys.exit("--boxes yolo needs camera frames from the bridge; use --boxes gt with --synthetic")
    cams = make_cams(args)
    geom["cams"] = cams
    yolo = YoloBoxes(args, cams[0]) if args.boxes == "yolo" else None
    if yolo and args.cam_layout != "single":
        sys.exit("--boxes yolo needs --cam-layout single (the bridge streams one camera)")
    cli = Cli(args, geom, cams)
    display = not args.no_display
    if display:
        import cv2
    if args.save_dir:
        import cv2
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
    scorer = Scorer(args.gate, args.grace, verbose=args.verbose, cams=cams)
    ring = Ring(args.mount_yaw, args.mirrored)
    shapes = default_scene() if args.synthetic else None
    live = None if args.synthetic else LiveSource(args)
    pace = args.realtime or (display and args.synthetic)

    history = {}          # scan index -> gt snapshot
    sent = 0
    t0 = time.time()
    next_rev = t0
    last_frame = None
    paused = False
    rate_t, rate_n, fps = t0, 0, 0.0
    print("geometry:", geom, flush=True)
    print("waiting for scans..." if live else "synthetic scene running", flush=True)

    def emit(ranges, gt):
        nonlocal sent
        data = encode_revolution(ranges, (sent * 100) % 65536, rng, args)
        data = corrupt(data, args.corrupt, rng)
        history[sent] = gt
        if args.boxes == "gt":
            boxes = gt_boxes(gt, cams, rng, args)
        else:
            boxes = yolo.get() if yolo else []
        if boxes:
            cli.send(json.dumps({"scan": sent, "boxes": boxes}).encode(), kind=1)
        cli.send(data)
        sent += 1

    def consume():
        nonlocal last_frame, rate_t, rate_n, fps
        while True:
            try:
                f = cli.out.get(timeout=0.002)
            except queue.Empty:
                return True
            if f is None:
                return False
            gt = history.pop(f["scan"], [])
            fp, hit, miss = scorer.update(f["tracks"], gt, geom["range"], f["scan"])
            rate_n += 1
            if time.time() - rate_t > 1.0:
                fps, rate_t, rate_n = rate_n / (time.time() - rate_t), time.time(), 0
            m = scorer.summary()
            hud = [f"scan {f['scan']}  pts {len(f['points'])}/{f['raw']}  tracks {len(f['tracks'])}  GT {len(gt)}  {fps:.1f} rev/s",
                   f"packets good {f['good']} bad {f['bad']}  boxes {len(f['boxes'])} (lidar-matched {sum(b['cluster'] >= 0 for b in f['boxes'])})"
                   + (f"  yolo {yolo.ms:.0f} ms" if yolo else ""),
                   f"prec {m['precision']:.2f} rec {m['recall']:.2f} err {m['pos_err_mean_m']:.2f} m  idsw {m['id_switches']}"
                   + (f"  cls cov {m['class_coverage']:.2f} acc {m['class_accuracy']:.2f}" if cams else ""),
                   "green T = track, red T = false positive, circle = GT (red = missed). q quit, space pause, s save"]
            last_frame = (f, gt, fp, hit, miss, hud)
            if display or args.save_dir:
                img = draw(f, gt, fp, hit, miss, hud, args, geom)
                if args.save_dir and f["scan"] % 5 == 0:
                    cv2.imwrite(str(Path(args.save_dir) / f"{f['scan']:06d}.png"), img)
                if display:
                    cv2.imshow("lidar harness", img)

    try:
        while True:
            now = time.time()
            elapsed = sent * REV_S if args.synthetic else now - t0
            if args.duration and elapsed >= args.duration:
                break
            if args.synthetic:
                if pace:
                    time.sleep(max(0, next_rev - time.time()))
                next_rev += REV_S
                sim_t = sent * REV_S
                xy, counts = raycast(shapes, sim_t)
                ring.add(xy, sim_t)
                emit(ring.revolution(sim_t, REV_S * 0.5), synthetic_gt(shapes, sim_t, counts))
            else:
                for scan in live.poll(max(0.0, next_rev - time.time())):
                    ring.add(scan, time.time())
                if time.time() >= next_rev:
                    next_rev += REV_S
                    if next_rev < time.time() - 1:
                        next_rev = time.time() + REV_S
                    rev = ring.revolution(time.time(), 0.25)
                    if np.count_nonzero(rev) and live.gt_time:
                        emit(rev, live.gt_snapshot())
                    elif np.count_nonzero(rev):
                        emit(rev, [])
            if not consume():
                break
            if display:
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q") or k == 27:
                    break
                if k == ord("s") and last_frame:
                    cv2.imwrite(f"lidar_{int(time.time())}.png", draw(*last_frame, args, geom))
                if k == ord(" "):
                    paused = not paused
                    while paused:
                        if cv2.waitKey(50) & 0xFF == ord(" "):
                            paused = False
    except KeyboardInterrupt:
        pass
    finally:
        # push one more empty-ish revolution so the last real one is flushed out of the reader
        if sent:
            emit(np.zeros(RAYS), [])
        cli.close()
        while True:
            try:
                f = cli.out.get(timeout=1)
            except queue.Empty:
                break
            if f is None:
                break
            gt = history.pop(f["scan"], [])
            scorer.update(f["tracks"], gt, geom["range"], f["scan"])
        if display:
            cv2.destroyAllWindows()

    m = scorer.summary()
    print("\n--- lidar harness summary ---")
    for k, v in m.items():
        print(f"{k:24s} {v:.3f}" if isinstance(v, float) else f"{k:24s} {v}")
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(m, indent=2))
    if args.score:
        fails = []
        if m["frames"] < 10:
            fails.append("fewer than 10 scans scored")
        if m["precision"] < args.min_precision:
            fails.append(f"precision {m['precision']:.2f} < {args.min_precision}")
        if m["recall"] < args.min_recall:
            fails.append(f"recall {m['recall']:.2f} < {args.min_recall}")
        if m["id_switches"] > args.max_id_switches:
            fails.append(f"id switches {m['id_switches']} > {args.max_id_switches}")
        if not m["pos_err_mean_m"] <= args.max_pos_err:
            fails.append(f"mean position error {m['pos_err_mean_m']:.2f} m > {args.max_pos_err}")
        if cams and args.boxes == "gt":
            if not m["class_coverage"] >= args.min_class_cov:
                fails.append(f"class coverage {m['class_coverage']:.2f} < {args.min_class_cov}")
            if not m["class_accuracy"] >= args.min_class_acc:
                fails.append(f"class accuracy {m['class_accuracy']:.2f} < {args.min_class_acc}")
        print("RESULT:", "FAIL: " + "; ".join(fails) if fails else "PASS")
        sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
