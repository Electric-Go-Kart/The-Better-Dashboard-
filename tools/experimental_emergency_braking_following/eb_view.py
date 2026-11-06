#!/usr/bin/env python3
"""Top-down view of what eb_drive.py sees and decides (run in .venv-ml, needs opencv).

    python eb_view.py                 # window; q or Esc quits
    python eb_view.py --save-dir runs/ped1/view     # also write a PNG every frame

Kart at the bottom, forward = up, lidar points grey, each track coloured by its matched class (grey = unknown)
with id, class, votes and velocity. The camera boxes from the YOLO node are drawn as wedges along the top: if a
wedge overlaps a track's bearing span the track gets that class. The red track is the one the planner is reacting to.
The green path is the lane centre the kart is steering for and the pale band is its swept corridor.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path

import cv2
import numpy as np

COLORS = {"person": (0, 140, 255), "bike": (255, 200, 0), "vehicle": (255, 120, 40), "pole/barrier": (200, 0, 200),
          "unknown": (160, 160, 160)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5608)
    ap.add_argument("--range", type=float, default=20.0, help="m shown ahead")
    ap.add_argument("--size", type=int, default=800, help="window height in px")
    ap.add_argument("--save-dir")
    ap.add_argument("--stale-after", type=float, default=1.0, help="local receive timeout, seconds")
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", a.port))
    sock.settimeout(0.5)
    if a.save_dir:
        Path(a.save_dir).mkdir(parents=True, exist_ok=True)
    H = a.size
    W = H // 2 + 200
    side = 10.0  # m shown each side
    ppm = H / (a.range + 3.0)
    ox, oy = W // 2 - 60, H - int(2.5 * ppm)  # kart (lidar) position in px

    def px(x, y):
        return int(ox + y * ppm), int(oy - x * ppm)

    n = 0
    print(f"waiting for planner debug data on UDP {a.port} ...")
    m = None
    received_at = None
    while True:
        try:
            m = json.loads(sock.recv(65535))
            received_at = time.monotonic()
        except socket.timeout:
            pass
        except ValueError:
            continue
        if m is None:
            continue
        age = time.monotonic() - received_at
        img = np.full((H, W, 3), 30, np.uint8)
        for r in range(5, int(a.range) + 1, 5):
            cv2.line(img, px(r, -side), px(r, side), (50, 50, 50), 1)
            cv2.putText(img, f"{r} m", px(r, -side + 0.2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)
        kw, front, rear, margin = m["kart"]
        # lane centre path and corridor
        lane = m.get("lane")
        if lane and lane.get("valid", True):
            o, h, k = lane["offset"], lane["heading"], lane.get("curv", 0.0)
            xs = np.linspace(0.0, a.range, 60)
            ys = (o + math.tan(h) * xs + 0.5 * k * xs * xs) * np.minimum(xs / 6.0, 1.0)
            half = kw / 2 + margin
            left = np.array([px(x, y - half) for x, y in zip(xs, ys)])
            right = np.array([px(x, y + half) for x, y in zip(xs, ys)])
            band = img.copy()
            cv2.fillPoly(band, [np.vstack([left, right[::-1]])], (70, 90, 60))
            img = cv2.addWeighted(band, 0.5, img, 0.5, 0)
            cv2.polylines(img, [np.array([px(x, y) for x, y in zip(xs, ys)])], False, (0, 200, 0), 1)
            if lane.get("width"):
                for s in (-1, 1):
                    cv2.polylines(img, [np.array([px(x, y + s * lane["width"] / 2) for x, y in zip(xs, ys)])], False, (200, 200, 200), 1)
        # kart
        cv2.rectangle(img, px(front, -kw / 2), px(-rear, kw / 2), (255, 255, 255), 1)
        # camera boxes as wedges
        fov = math.radians(m["fov"])
        for deg in (-m["fov"] / 2, m["fov"] / 2):
            t = math.radians(deg)
            cv2.line(img, px(0, 0), px(a.range * math.cos(t), a.range * math.sin(t)), (60, 60, 90), 1)
        for b in m["boxes"]:
            col = COLORS.get(b["cls"], COLORS["unknown"])
            for t in (b["lo"], b["hi"]):
                cv2.line(img, px(a.range - 2.0, (a.range - 2.0) * math.tan(t)), px(a.range, a.range * math.tan(t)), col, 2)
            mid = (b["lo"] + b["hi"]) / 2
            cv2.putText(img, f"yolo {b['cls']} {b['conf']:.2f}", px(a.range - 0.5, (a.range - 3.0) * math.tan(mid)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        for x, y in m["pts"]:
            cv2.circle(img, px(x, y), 1, (120, 120, 120), -1)
        for tr in m["tracks"]:
            col = COLORS.get(tr["cls"], COLORS["unknown"])
            for x, y in tr["pts"]:
                cv2.circle(img, px(x, y), 3, col, -1)
            cx, cy = tr["c"]
            if tr["threat"]:
                cv2.circle(img, px(cx, cy), 16, (0, 0, 255), 2)
            cv2.arrowedLine(img, px(cx, cy), px(cx + tr["v"][0], cy + tr["v"][1]), col, 1, tipLength=0.3)
            votes = " ".join(f"{k[:3]}{v:.1f}" for k, v in tr["votes"].items())
            cv2.putText(img, f"#{tr['id']} {tr['cls']} {votes}", (px(cx, cy)[0] + 8, px(cx, cy)[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
            # the bearing span the camera boxes are matched against
            for t in tr["bear"]:
                cv2.line(img, px(0, 0), px(3.0 * math.cos(t), 3.0 * math.sin(t)), col, 1)
        a_req = m["a_req"]
        lines = [f"{m['state']}", f"v {m['v']:.1f} -> {m['v_target']:.1f} m/s",
                 f"a_req {a_req:.1f}" if a_req is not None else "a_req -",
                 f"gap {m['gap']:.1f} m" if m["gap"] is not None else "gap -", f"nudge {m['nudge']:+.2f} m",
                 f"tracks {len(m['tracks'])}  boxes {len(m['boxes'])}",
                 f"link {'STALE' if age > a.stale_after else 'live'} {age:.2f} s"]
        if "steer" in m:
            lines.extend([f"{m.get('mode', '?')} / {m.get('control', '?')}",
                          f"recommend steer {m['steer']:+.2f}",
                          f"throttle {m['throttle']:.2f} brake {m['brake']:.2f}"])
        for i, s in enumerate(lines):
            c = (0, 0, 255) if m["state"] in ("AEB", "LIDAR-LOST") or age > a.stale_after else (230, 230, 230)
            cv2.putText(img, s, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1)
        cv2.imshow("eb view", img)
        if a.save_dir:
            cv2.imwrite(str(Path(a.save_dir) / f"{n:06d}.png"), img)
            n += 1
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break


if __name__ == "__main__":
    main()
