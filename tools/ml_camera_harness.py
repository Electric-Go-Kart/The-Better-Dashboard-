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
from collections import deque
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

    def infer(self, bgr, timings=None):
        start = time.perf_counter() if timings is not None else None
        ex = self.net.create_extractor()
        status = ex.input(self.in_name, ncnn_mat(bgr, self.cfg["width"], self.cfg["height"], False))
        if status != 0:
            raise RuntimeError(f"Failed to set YOLO input {self.in_name} (ncnn status {status})")
        if timings is not None:
            timings["yolo_preprocess"] = time.perf_counter() - start
        nc, na = self.cfg["classes"], self.cfg["anchor_num"]
        total = na * 4 + na + nc
        heads = []
        inference_time = postprocess_time = 0.0
        for name in self.out_names:
            start = time.perf_counter() if timings is not None else None
            status, out = ex.extract(name)
            if status != 0:
                raise RuntimeError(f"Failed to extract YOLO output {name} (ncnn status {status})")
            if timings is not None:
                inference_time += time.perf_counter() - start
                start = time.perf_counter()
            a = np.array(out)
            if a.ndim == 4:
                a = a[0]
            if a.shape[-1] == total and a.shape[0] != total:  # (H, W, C) -> (C, H, W)
                a = a.transpose(2, 0, 1)
            if a.shape[0] != total:
                sys.exit(f"output {name} has shape {a.shape}, expected {total} channels")
            heads.append((a[:na * 4], a[na * 4:na * 5], a[na * 5:]))
            if timings is not None:
                postprocess_time += time.perf_counter() - start
        start = time.perf_counter() if timings is not None else None
        result = decode_yolo(heads, self.cfg, self.conf, 0.4, bgr.shape[1], bgr.shape[0])
        if timings is not None:
            timings["yolo_ncnn_extract"] = inference_time
            timings["yolo_postprocess"] = postprocess_time + time.perf_counter() - start
        return result


class SegNcnn:
    """TwinLiteNet: two heads, each (2, H, W) logits, class 1 = drivable / lane."""

    def __init__(self, args):
        self.net = load_ncnn(args.seg_param, args.seg_bin, args.seg_threads)
        self.w, self.h = args.seg_width, args.seg_height
        self.in_name, self.out_da, self.out_ll = args.seg_in, args.seg_out_da, args.seg_out_ll
        self.letterbox = getattr(args, "seg_letterbox", False)

    def infer(self, bgr, timings=None):
        start = time.perf_counter() if timings is not None else None
        ex = self.net.create_extractor()
        if self.letterbox:
            ih, iw = bgr.shape[:2]
            scale = min(self.w / iw, self.h / ih)
            nw, nh = round(iw * scale), round(ih * scale)
            left = round((self.w - nw) / 2 - 0.1)
            top = round((self.h - nh) / 2 - 0.1)
            resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
            prepared = cv2.copyMakeBorder(resized, top, self.h - nh - top, left, self.w - nw - left,
                                          cv2.BORDER_CONSTANT, value=(114, 114, 114))
        else:
            prepared = bgr
        status = ex.input(self.in_name, ncnn_mat(prepared, self.w, self.h, True))
        if status != 0:
            raise RuntimeError(f"Failed to set segmentation input {self.in_name} (ncnn status {status})")
        if timings is not None:
            timings["seg_preprocess"] = time.perf_counter() - start
        size = (bgr.shape[1], bgr.shape[0])
        masks = []
        inference_time = postprocess_time = 0.0
        for name in (self.out_da, self.out_ll):
            start = time.perf_counter() if timings is not None else None
            status, out = ex.extract(name)
            if status != 0:
                raise RuntimeError(f"Failed to extract segmentation output {name} (ncnn status {status})")
            if timings is not None:
                inference_time += time.perf_counter() - start
                start = time.perf_counter()
            mask = np.array(out).argmax(0).astype(np.uint8)
            if self.letterbox:
                if mask.shape != (self.h, self.w):
                    raise ValueError(f"Expected segmentation output {(self.h, self.w)}, got {mask.shape}")
                mask = mask[top:top + nh, left:left + nw]
            masks.append(cv2.resize(mask, size,
                                    interpolation=cv2.INTER_NEAREST))
            if timings is not None:
                postprocess_time += time.perf_counter() - start
        if timings is not None:
            timings["seg_ncnn_extract"] = inference_time
            timings["seg_postprocess"] = postprocess_time
        return tuple(masks)


def frames_udp(port, latest=False):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(2.0)
    print(f"Waiting for JPEG frames on UDP {port} ...")
    try:
        while True:
            try:
                data = sock.recv(65535)
            except socket.timeout:
                print("no frames (is the bridge running with camera enabled?)")
                continue
            if latest:
                sock.setblocking(False)
                try:
                    while True:
                        data = sock.recv(65535)
                except BlockingIOError:
                    pass
                finally:
                    sock.settimeout(2.0)
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                yield "udp", img
    finally:
        sock.close()


def frames_camera(index):
    capture = cv2.VideoCapture(index)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"Could not open camera {index}")
        print(f"Reading camera {index} ...")
        while True:
            ok, img = capture.read()
            if not ok or img is None:
                raise RuntimeError(f"Failed to read frame from camera {index}")
            yield f"camera:{index}", img
    finally:
        capture.release()


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


def timed_frames(source):
    while True:
        start = time.perf_counter()
        try:
            frame = next(source)
        except StopIteration:
            return
        yield frame, time.perf_counter() - start


def print_profile(samples):
    if not samples:
        return
    print(f"Profile: last {len(samples)} frames (ms, median / p95)")
    for key in samples[0]:
        values = [sample[key] * 1000 for sample in samples]
        print(f"  {key:20s} {np.median(values):8.1f} / {np.percentile(values, 95):.1f}")


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
    src.add_argument("--camera", type=int, help="read directly from an OpenCV camera index, e.g. 0")
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
    s.add_argument("--seg-letterbox", action="store_true", help="letterbox input and remove padding from masks")
    s.add_argument("--seg-threads", type=int, default=2)
    o = ap.add_argument_group("output")
    o.add_argument("--no-display", action="store_true", help="headless, print stats only")
    o.add_argument("--save-dir", help="write annotated frames here")
    o.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = never)")
    o.add_argument("--profile-seg", action="store_true", help="report ncnn segmentation stage timings")
    o.add_argument("--profile-yolo", action="store_true", help="report ncnn YOLO stage timings")
    o.add_argument("--profile-warmup", type=int, default=5, help="frames to exclude from profiling (default: 5)")
    args = ap.parse_args()
    if args.camera is not None and args.folder:
        ap.error("--camera and --folder cannot be used together")
    if args.profile_warmup < 0:
        ap.error("--profile-warmup must be non-negative")
    if args.profile_seg and (args.backend != "ncnn" or args.no_seg or not args.seg_param or not args.seg_bin):
        ap.error("--profile-seg requires the ncnn segmentation model (--seg-param and --seg-bin)")
    if args.profile_yolo and (args.backend != "ncnn" or args.no_yolo or not args.yolo_param or not args.yolo_bin):
        ap.error("--profile-yolo requires the ncnn YOLO model (--yolo-param and --yolo-bin)")

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

    if args.camera is not None:
        source = frames_camera(args.camera)
    elif args.folder:
        source = frames_folder(args.folder, args.loop)
    else:
        source = frames_udp(args.port)
    n, t_y, t_s, t_report = 0, 0.0, 0.0, time.time()
    samples = deque(maxlen=1000)
    last_profile = time.monotonic()
    last_profile_frame = 0
    profiling = args.profile_seg or args.profile_yolo
    frames = timed_frames(source) if profiling else ((frame, 0.0) for frame in source)
    for (_, img), input_wait in frames:
        frame_start = time.perf_counter()
        timings = {"input_wait": input_wait} if profiling else None
        dets, mask = [], None
        if yolo:
            t = time.perf_counter()
            dets = yolo.infer(img, timings) if args.profile_yolo else yolo.infer(img)
            t_y += time.perf_counter() - t
            if not args.all_classes:
                dets = [d for d in dets if d[1] in COCO_TO_BEV]
        if seg:
            t = time.perf_counter()
            mask = seg.infer(img, timings) if args.profile_seg else seg.infer(img)
            t_s += time.perf_counter() - t
        n += 1
        render_start = time.perf_counter() if timings is not None else None
        shown = draw(img, dets, mask, label_of)
        if args.save_dir:
            cv2.imwrite(os.path.join(args.save_dir, f"{n:06d}.jpg"), shown)
        if not args.no_display:
            cv2.imshow("ml_camera_harness (q to quit)", shown)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if timings is not None:
            timings["render_output"] = time.perf_counter() - render_start
            timings["frame_total"] = input_wait + time.perf_counter() - frame_start
            if n > args.profile_warmup:
                samples.append(timings)
            if time.monotonic() - last_profile >= 5 or (args.max_frames and n >= args.max_frames):
                if samples:
                    print_profile(samples)
                    last_profile_frame = n
                last_profile = time.monotonic()
        if time.time() - t_report >= 1.0 or args.max_frames:
            print(f"{img.shape[1]}x{img.shape[0]} frames={n} "
                  f"yolo={1000 * t_y / n:.1f}ms seg={1000 * t_s / n:.1f}ms "
                  f"dets={[(label_of(d[1]), round(d[0], 2)) for d in dets]}")
            t_report = time.time()
        if args.max_frames and n >= args.max_frames:
            break
    source.close()
    if profiling and n != last_profile_frame:
        if samples:
            print_profile(samples)
        else:
            print("Profile: no measured frames; reduce --profile-warmup or run for longer.")


if __name__ == "__main__":
    main()
