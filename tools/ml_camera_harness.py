#!/usr/bin/env python3
"""Desktop test harness: feed camera frames to YOLO-FastestV2 and TwinLiteNet.

Standalone. Does not touch the dashboard. Frames come from the CARLA bridge's UDP JPEG
stream (default) or from a folder of images. Timings are desktop-relative only, measure
real speed on the Pi 5.

Backends:
  torch  run the .pth weights straight from the model repos (default paths below)
  ncnn   run converted .param/.bin files (the real deployment format)

    python ml_camera_harness.py --backend torch
    python ml_camera_harness.py --backend ncnn --yolo-param y.param --yolo-bin y.bin \
        --seg-param t.param --seg-bin t.bin

See tools/README.md. Needs: numpy opencv-python, plus torch (torch backend) or ncnn.
"""
import argparse
import glob
import importlib.util
import os
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# COCO ids (80-class YOLO-FastestV2) -> BEV classes of docs/ml-integration-spec.md
COCO_TO_BEV = {0: "person", 1: "bike", 3: "bike", 2: "vehicle", 5: "vehicle", 7: "vehicle",
               10: "pole/barrier", 11: "pole/barrier", 12: "pole/barrier"}
BEV_COLORS = {"person": (60, 60, 255), "bike": (255, 160, 40), "vehicle": (60, 200, 60),
              "pole/barrier": (0, 200, 255)}

MODELS_DIR = Path(os.environ.get("ML_MODELS_DIR", Path(__file__).resolve().parents[3] / "models"))


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def softmax(x, axis):
    e = np.exp(x - x.max(axis=axis, keepdims=True))
    return e / e.sum(axis=axis, keepdims=True)


def read_cfg(path):
    """Minimal reader for the Yolo-FastestV2 *.data file."""
    raw = {}
    for line in open(path):
        if "=" in line and not line.startswith("["):
            k, v = line.strip().split("=", 1)
            raw[k] = v
    na = int(raw["anchor_num"])
    return {"classes": int(raw["classes"]), "width": int(raw["width"]), "height": int(raw["height"]),
            "anchor_num": na,
            "anchors": np.array([float(a) for a in raw["anchors"].split(",")]).reshape(-1, na, 2)}


def decode_yolo(heads, cfg, conf_thr, nms_thr, img_w, img_h):
    """heads: list (stride 16 then 32) of (reg, obj, cls), already activated:
    reg (3*4,H,W) sigmoid, obj (3,H,W) sigmoid, cls (nc,H,W) softmax.
    Same math as handel_preds() in the Yolo-FastestV2 repo. Returns [(score, cls, x, y, w, h)]."""
    size_w, size_h = cfg["width"], cfg["height"]
    na = cfg["anchor_num"]
    boxes = []
    for i, (reg, obj, cls) in enumerate(heads):
        _, h, w = obj.shape
        reg = reg.reshape(na, 4, h, w)
        gy, gx = np.mgrid[0:h, 0:w]
        cls_p = cls.max(0)
        cls_id = cls.argmax(0)
        for a in range(na):
            score = obj[a] * cls_p
            ys, xs = np.where((obj[a] > conf_thr) & (score > conf_thr))
            if len(ys) == 0:
                continue
            r = reg[a][:, ys, xs]
            cx = (r[0] * 2 - 0.5 + gx[ys, xs]) * (size_w / w)
            cy = (r[1] * 2 - 0.5 + gy[ys, xs]) * (size_h / h)
            bw = (r[2] * 2) ** 2 * cfg["anchors"][i][a][0]
            bh = (r[3] * 2) ** 2 * cfg["anchors"][i][a][1]
            for k in range(len(ys)):
                boxes.append((float(score[ys[k], xs[k]]), int(cls_id[ys[k], xs[k]]),
                              cx[k] - bw[k] / 2, cy[k] - bh[k] / 2, bw[k], bh[k]))
    if not boxes:
        return []
    # class offset so NMS only suppresses within a class
    rects = [[float(b[2] + b[1] * 4096), float(b[3]), float(b[4]), float(b[5])] for b in boxes]
    keep = cv2.dnn.NMSBoxes(rects, [b[0] for b in boxes], 0.0, nms_thr)
    sx, sy = img_w / size_w, img_h / size_h
    return [(b[0], b[1], b[2] * sx, b[3] * sy, b[4] * sx, b[5] * sy)
            for b in (boxes[i] for i in np.array(keep).reshape(-1))]


class YoloTorch:
    def __init__(self, args):
        import torch
        self.torch = torch
        repo = Path(args.yolo_repo)
        sys.path.insert(0, str(repo))
        import model.detector  # repo module, needs the repo on sys.path
        sys.path.pop(0)
        self.cfg = read_cfg(args.yolo_data or repo / "data" / "coco.data")
        self.device = torch.device(args.device)
        # True skips the repo's backbone-only init file; the full checkpoint is loaded below
        net = model.detector.Detector(self.cfg["classes"], self.cfg["anchor_num"], True)
        net.load_state_dict(torch.load(args.yolo_pth or repo / "modelzoo" / "coco2017-0.241078ap-model.pth",
                                       map_location=self.device))
        self.net = net.to(self.device).eval()
        self.conf = args.yolo_conf

    def infer(self, bgr):
        torch = self.torch
        # the repo feeds the BGR image as is (no colour swap), scaled 0..1
        img = cv2.resize(bgr, (self.cfg["width"], self.cfg["height"]), interpolation=cv2.INTER_LINEAR)
        x = torch.from_numpy(img.transpose(2, 0, 1)[None].copy()).to(self.device).float() / 255.0
        with torch.no_grad():
            p = [t[0].cpu().numpy() for t in self.net(x)]
        heads = [(sigmoid(p[j]), sigmoid(p[j + 1]), softmax(p[j + 2], 0)) for j in (0, 3)]
        return decode_yolo(heads, self.cfg, self.conf, 0.4, bgr.shape[1], bgr.shape[0])


class SegTorch:
    def __init__(self, args):
        import torch
        self.torch = torch
        repo = Path(args.seg_repo)
        # loaded by path: both repos have a package called "model"
        spec = importlib.util.spec_from_file_location("twinlite_model", repo / "model" / "TwinLite.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self.device = torch.device(args.device)
        net = mod.TwinLiteNet()
        state = torch.load(args.seg_pth or repo / "pretrained" / "best.pth", map_location=self.device)
        state = {k.removeprefix("module."): v for k, v in state.items()}
        net.load_state_dict(state)
        self.net = net.to(self.device).eval()
        self.w, self.h = args.seg_width, args.seg_height

    def infer(self, bgr):
        torch = self.torch
        img = cv2.resize(bgr, (self.w, self.h))[:, :, ::-1]  # RGB
        x = torch.from_numpy(img.transpose(2, 0, 1)[None].copy()).to(self.device).float() / 255.0
        with torch.no_grad():
            da, ll = self.net(x)
        size = (bgr.shape[1], bgr.shape[0])
        return tuple(cv2.resize(t.argmax(1)[0].byte().cpu().numpy(), size, interpolation=cv2.INTER_NEAREST)
                     for t in (da, ll))


def load_ncnn(param, bin_, threads):
    import ncnn
    net = ncnn.Net()
    net.opt.num_threads = threads
    net.opt.use_vulkan_compute = False
    if net.load_param(param) != 0 or net.load_model(bin_) != 0:
        sys.exit(f"failed to load {param} / {bin_}")
    return net


def ncnn_mat(bgr, w, h, to_rgb):
    import ncnn
    pix = ncnn.Mat.PixelType.PIXEL_BGR2RGB if to_rgb else ncnn.Mat.PixelType.PIXEL_BGR
    mat = ncnn.Mat.from_pixels_resize(bgr, pix, bgr.shape[1], bgr.shape[0], w, h)
    mat.substract_mean_normalize([], [1 / 255.0] * 3)
    return mat


class YoloNcnn:
    """Expects the model exported with the repo's pytorch2onnx.py (sigmoid/softmax baked in,
    two outputs, each reg(12) + obj(3) + cls(nc) channels)."""

    def __init__(self, args):
        self.net = load_ncnn(args.yolo_param, args.yolo_bin, args.yolo_threads)
        self.cfg = read_cfg(args.yolo_data or Path(args.yolo_repo) / "data" / "coco.data")
        self.in_name = args.yolo_in
        self.out_names = [n for n in args.yolo_out.split(",") if n]
        self.conf = args.yolo_conf

    def infer(self, bgr):
        ex = self.net.create_extractor()
        ex.input(self.in_name, ncnn_mat(bgr, self.cfg["width"], self.cfg["height"], False))
        nc, na = self.cfg["classes"], self.cfg["anchor_num"]
        total = na * 4 + na + nc
        heads = []
        for name in self.out_names:
            _, out = ex.extract(name)
            a = np.array(out)
            if a.ndim == 4:
                a = a[0]
            if a.shape[-1] == total and a.shape[0] != total:  # (H, W, C) -> (C, H, W)
                a = a.transpose(2, 0, 1)
            if a.shape[0] != total:
                sys.exit(f"output {name} has shape {a.shape}, expected {total} channels")
            heads.append((a[:na * 4], a[na * 4:na * 5], a[na * 5:]))
        return decode_yolo(heads, self.cfg, self.conf, 0.4, bgr.shape[1], bgr.shape[0])


class SegNcnn:
    """TwinLiteNet: two heads, each (2, H, W) logits, class 1 = drivable / lane."""

    def __init__(self, args):
        self.net = load_ncnn(args.seg_param, args.seg_bin, args.seg_threads)
        self.w, self.h = args.seg_width, args.seg_height
        self.in_name, self.out_da, self.out_ll = args.seg_in, args.seg_out_da, args.seg_out_ll

    def infer(self, bgr):
        ex = self.net.create_extractor()
        ex.input(self.in_name, ncnn_mat(bgr, self.w, self.h, True))
        size = (bgr.shape[1], bgr.shape[0])
        masks = []
        for name in (self.out_da, self.out_ll):
            _, out = ex.extract(name)
            masks.append(cv2.resize(np.array(out).argmax(0).astype(np.uint8), size,
                                    interpolation=cv2.INTER_NEAREST))
        return tuple(masks)


def frames_udp(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(2.0)
    print(f"Waiting for JPEG frames on UDP {port} ...")
    while True:
        try:
            data = sock.recv(65535)
        except socket.timeout:
            print("no frames (is the bridge running with camera enabled?)")
            continue
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            yield "udp", img


def frames_folder(path, loop):
    files = sorted(f for ext in ("jpg", "jpeg", "png") for f in glob.glob(os.path.join(path, f"*.{ext}")))
    if not files:
        sys.exit(f"no images in {path}")
    while True:
        for f in files:
            img = cv2.imread(f)
            if img is not None:
                yield os.path.basename(f), img
        if not loop:
            return


def draw(img, dets, seg, label_of):
    out = img.copy()
    if seg is not None:
        da, ll = seg
        overlay = out.copy()
        overlay[da == 1] = (0, 200, 0)
        overlay[ll == 1] = (0, 0, 255)
        out = cv2.addWeighted(overlay, 0.4, out, 0.6, 0)
    for score, cid, x, y, w, h in dets:
        label = label_of(cid)
        color = BEV_COLORS.get(label, (255, 255, 0))
        cv2.rectangle(out, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
        cv2.putText(out, f"{label} {score:.2f}", (int(x), max(12, int(y) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["torch", "ncnn"], default="torch")
    ap.add_argument("--no-yolo", action="store_true")
    ap.add_argument("--no-seg", action="store_true")
    ap.add_argument("--device", default="cpu", help="torch device, e.g. cpu or cuda")
    src = ap.add_argument_group("input")
    src.add_argument("--port", type=int, default=5600, help="UDP port of the bridge camera stream")
    src.add_argument("--folder", help="read images from this folder instead of UDP")
    src.add_argument("--loop", action="store_true", help="repeat the folder forever")
    y = ap.add_argument_group("YOLO-FastestV2")
    y.add_argument("--yolo-repo", default=str(MODELS_DIR / "yolo/Yolo-FastestV2"))
    y.add_argument("--yolo-pth", help="default: <repo>/modelzoo/coco2017-0.241078ap-model.pth")
    y.add_argument("--yolo-data", help="*.data config, default: <repo>/data/coco.data")
    y.add_argument("--yolo-param", help="ncnn backend")
    y.add_argument("--yolo-bin", help="ncnn backend")
    y.add_argument("--yolo-in", default="input.1", help="ncnn input blob name")
    y.add_argument("--yolo-out", default="output,375", help="ncnn output blob names, stride 16 then 32")
    y.add_argument("--yolo-conf", type=float, default=0.3)
    y.add_argument("--yolo-threads", type=int, default=1)
    y.add_argument("--all-classes", action="store_true", help="show every COCO class, not just the BEV-mapped ones")
    s = ap.add_argument_group("TwinLiteNet")
    s.add_argument("--seg-repo", default=str(MODELS_DIR / "twinlitenet/TwinLiteNet"))
    s.add_argument("--seg-pth", help="default: <repo>/pretrained/best.pth")
    s.add_argument("--seg-param", help="ncnn backend")
    s.add_argument("--seg-bin", help="ncnn backend")
    s.add_argument("--seg-in", default="in0", help="ncnn input blob name")
    s.add_argument("--seg-out-da", default="out0", help="ncnn drivable-area output blob")
    s.add_argument("--seg-out-ll", default="out1", help="ncnn lane-line output blob")
    s.add_argument("--seg-width", type=int, default=640)
    s.add_argument("--seg-height", type=int, default=360)
    s.add_argument("--seg-threads", type=int, default=2)
    o = ap.add_argument_group("output")
    o.add_argument("--no-display", action="store_true", help="headless, print stats only")
    o.add_argument("--save-dir", help="write annotated frames here")
    o.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = never)")
    args = ap.parse_args()

    yolo = seg = None
    if args.backend == "torch":
        if not args.no_yolo:
            yolo = YoloTorch(args)
        if not args.no_seg:
            seg = SegTorch(args)
    else:
        if args.yolo_param and args.yolo_bin and not args.no_yolo:
            yolo = YoloNcnn(args)
        if args.seg_param and args.seg_bin and not args.no_seg:
            seg = SegNcnn(args)
    if not yolo and not seg:
        print("No model loaded, showing frames only.")

    coco_names = []
    names_file = Path(args.yolo_repo) / "data" / "coco.names"
    if names_file.exists():
        coco_names = [line.strip() for line in open(names_file)]

    def label_of(cid):
        if args.all_classes:
            return coco_names[cid] if cid < len(coco_names) else str(cid)
        return COCO_TO_BEV[cid]

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    source = frames_folder(args.folder, args.loop) if args.folder else frames_udp(args.port)
    n, t_y, t_s, t_report = 0, 0.0, 0.0, time.time()
    for _, img in source:
        dets, mask = [], None
        if yolo:
            t = time.perf_counter()
            dets = yolo.infer(img)
            t_y += time.perf_counter() - t
            if not args.all_classes:
                dets = [d for d in dets if d[1] in COCO_TO_BEV]
        if seg:
            t = time.perf_counter()
            mask = seg.infer(img)
            t_s += time.perf_counter() - t
        n += 1
        shown = draw(img, dets, mask, label_of)
        if args.save_dir:
            cv2.imwrite(os.path.join(args.save_dir, f"{n:06d}.jpg"), shown)
        if not args.no_display:
            cv2.imshow("ml_camera_harness (q to quit)", shown)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if time.time() - t_report >= 1.0 or args.max_frames:
            print(f"{img.shape[1]}x{img.shape[0]} frames={n} "
                  f"yolo={1000 * t_y / n:.1f}ms seg={1000 * t_s / n:.1f}ms "
                  f"dets={[(label_of(d[1]), round(d[0], 2)) for d in dets]}")
            t_report = time.time()
        if args.max_frames and n >= args.max_frames:
            break


if __name__ == "__main__":
    main()
