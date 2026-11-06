#!/usr/bin/env python3
"""YOLO-FastestV2 classifier for the emergency-braking stack (run in .venv-ml).

Reads the camera JPEGs the driver sends on UDP --port (5606, a second copy of the stream so it can run next
to seg_lane_node.py on 5600) and publishes the detected vehicles / people / bikes on UDP --out-port (5607) as JSON:
    {"t": ..., "w": 640, "h": 360, "boxes": [{"cls": "person", "conf": 0.7, "u0": .., "u1": .., "v0": .., "v1": ..}]}
Lidar decides WHERE things are and whether to brake; these boxes only tell it WHAT they are.

    python eb_vision_node.py --device cuda
    python eb_vision_node.py --backend ncnn --yolo-param y.param --yolo-bin y.bin
"""
import argparse
import json
import socket
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ml_camera_harness as ml  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=("torch", "ncnn"), default="torch")
    ap.add_argument("--device", default="cpu", help="torch device, e.g. cpu or cuda")
    ap.add_argument("--port", type=int, default=5606, help="UDP port of the camera stream")
    ap.add_argument("--camera", type=int, help="read directly from an OpenCV camera index, e.g. 0")
    ap.add_argument("--latest-frame", action="store_true", help="discard queued older UDP camera frames")
    ap.add_argument("--out-port", type=int, default=5607)
    ap.add_argument("--out-host", default="127.0.0.1")
    ap.add_argument("--yolo-repo", default=str(ml.MODELS_DIR / "yolo/Yolo-FastestV2"))
    ap.add_argument("--yolo-pth")
    ap.add_argument("--yolo-data")
    ap.add_argument("--yolo-param")
    ap.add_argument("--yolo-bin")
    ap.add_argument("--yolo-in", default="input.1")
    ap.add_argument("--yolo-out", default="output,375")
    ap.add_argument("--yolo-conf", type=float, default=0.3)
    ap.add_argument("--yolo-threads", type=int, default=1)
    ap.add_argument("--show", action="store_true", help="window with the camera frame and the boxes being sent")
    a = ap.parse_args()

    yolo = ml.YoloNcnn(a) if a.backend == "ncnn" else ml.YoloTorch(a)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.out_host, a.out_port)
    n, busy, t_rep = 0, 0.0, time.time()
    source = (ml.frames_camera(a.camera) if a.camera is not None else
              ml.frames_udp(a.port, latest=True) if a.latest_frame else ml.frames_udp(a.port))
    for _, img in source:
        t0 = time.perf_counter()
        dets = [d for d in yolo.infer(img) if d[1] in ml.COCO_TO_BEV]
        busy += time.perf_counter() - t0
        h, w = img.shape[:2]
        boxes = [{"cls": ml.COCO_TO_BEV[cid], "conf": round(float(score), 3), "u0": float(x), "u1": float(x + bw),
                  "v0": float(y), "v1": float(y + bh)} for score, cid, x, y, bw, bh in dets]
        sock.sendto(json.dumps({"t": time.time(), "w": w, "h": h, "boxes": boxes}).encode(), dest)
        n += 1
        if a.show:
            for bx in boxes:
                cv2.rectangle(img, (int(bx["u0"]), int(bx["v0"])), (int(bx["u1"]), int(bx["v1"])), (0, 255, 0), 2)
                cv2.putText(img, f"{bx['cls']} {bx['conf']:.2f}", (int(bx["u0"]), max(12, int(bx["v0"]) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("yolo boxes", img)
            cv2.waitKey(1)
        if time.time() - t_rep > 5.0:
            print(f"yolo {1000 * busy / n:5.1f} ms/frame, {n / (time.time() - t_rep):4.1f} fps, {len(boxes)} boxes last frame")
            n, busy, t_rep = 0, 0.0, time.time()
    source.close()


if __name__ == "__main__":
    main()
