#!/usr/bin/env python3
"""Parity + paper-metric check for TwinLiteNetPlus: PyTorch (upstream) vs patched PyTorch vs ncnn.

Run with tools/.venv-ml:
    # parity on a few images
    python parity_check.py --config nano --images /path/to/jpgs
    # paper metrics on BDD100K val (needs images/val, drivable_area_annotations/val, lane_line_annotations/val)
    python parity_check.py --config nano --bdd-root /data/bdd100k --limit 500

Reports:
  1. patched-vs-upstream torch logits (must be ~1e-5, proves the CAAM rewrite is equivalent)
  2. ncnn-vs-torch logits and argmax mask agreement (FP32 should be ~99.9+%)
  3. with labels: DA mIoU, LL accuracy, LL IoU using upstream val.py's preprocessing and formulas,
     calculated per image and averaged (equivalent to upstream with batch_size=1),
     for torch and ncnn, plus ncnn ms/frame.
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np
import torch

from export_ncnn import DEFAULT_REPO, load_model

W_, H_ = 640, 384


def letterbox(im, new_shape, color=(114, 114, 114)):
    """Same behaviour as upstream BDD100K.letterbox with default flags."""
    h, w = im.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (new_shape[1] - nw) / 2, (new_shape[0] - nh) / 2
    if (w, h) != (nw, nh):
        im = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    return cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)


def preprocess(path, h, w, expected_shape=None):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)
    if expected_shape is not None and img.shape[:2] != expected_shape:
        raise ValueError(f"{path}: expected image shape {expected_shape}, got {img.shape[:2]}")
    img = letterbox(img, (h, w))
    chw = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32) / 255.0
    return chw


class NcnnModel:
    def __init__(self, param, binf, threads):
        import ncnn
        self.ncnn = ncnn
        self.net = ncnn.Net()
        self.net.opt.num_threads = threads
        self.net.opt.use_vulkan_compute = False
        if self.net.load_param(param) != 0 or self.net.load_model(binf) != 0:
            raise RuntimeError(f"Failed to load ncnn model: {param}, {binf}")

    def __call__(self, chw):
        ex = self.net.create_extractor()
        if ex.input("in0", self.ncnn.Mat(chw).clone()) != 0:
            raise RuntimeError("Failed to set ncnn input in0")
        status_a, a = ex.extract("out0")
        status_b, b = ex.extract("out1")
        if status_a != 0 or status_b != 0:
            raise RuntimeError(f"Failed to extract ncnn outputs: out0={status_a}, out1={status_b}")
        return np.array(a), np.array(b)


def torch_forward(model, chw):
    with torch.no_grad():
        da, ll = model(torch.from_numpy(chw)[None])
    return da[0].numpy(), ll[0].numpy()


def set_caam(patched):
    import twinlite_patch
    from model.model import CAAM
    CAAM.forward = twinlite_patch.caam_forward if patched else CAAM._upstream_forward


class Metric:
    """Upstream metric formulas averaged per image (equivalent to validation batch size 1)."""

    def __init__(self):
        self.da_miou, self.ll_acc, self.ll_iou = [], [], []

    @staticmethod
    def cm(pred, gt):
        k = (gt >= 0) & (gt < 2)
        return np.bincount(2 * gt[k] + pred[k], minlength=4).reshape(2, 2).astype(np.float64)

    def add(self, da_pred, da_gt, ll_pred, ll_gt):
        c = self.cm(da_pred, da_gt)
        union = c.sum(1) + c.sum(0) - np.diag(c)
        iou = np.divide(np.diag(c), union, out=np.zeros_like(union), where=union != 0)
        self.da_miou.append(iou.mean())

        c = self.cm(ll_pred, ll_gt)
        tp, fp, fn, tn = c[1, 1], c[0, 1], c[1, 0], c[0, 0]
        self.ll_acc.append((tp / (tp + fn + 1e-12) + tn / (tn + fp + 1e-12)) / 2)
        union = c.sum(1) + c.sum(0) - np.diag(c)
        self.ll_iou.append(np.divide(np.diag(c), union, out=np.zeros_like(union), where=union != 0)[1])

    def summary(self):
        return (float(np.mean(self.da_miou)), float(np.mean(self.ll_acc)), float(np.mean(self.ll_iou)))


def load_gt(img_path, da_dir, ll_dir):
    name = os.path.splitext(os.path.basename(img_path))[0] + ".png"
    da_path, ll_path = os.path.join(da_dir, name), os.path.join(ll_dir, name)
    da = cv2.imread(da_path, 0)
    ll = cv2.imread(ll_path, 0)
    if da is None:
        raise FileNotFoundError(f"Missing or unreadable drivable-area label: {da_path}")
    if ll is None:
        raise FileNotFoundError(f"Missing or unreadable lane-line label: {ll_path}")
    # Upstream: resize to (640, 360), class 1 where value > 1
    da = (cv2.resize(da, (W_, 360)) > 1).astype(np.int64)
    ll = (cv2.resize(ll, (W_, 360)) > 1).astype(np.int64)
    return da, ll


def masks(da, ll):
    """Unpad 16:9 BDD masks and resize class predictions to the 640x360 labels."""
    result = []
    for output in (da, ll):
        if output.ndim != 3 or output.shape[0] != 2:
            raise ValueError(f"Expected segmentation output [2,H,W], got {output.shape}")
        h, w = output.shape[1:]
        scale = min(h / 720, w / 1280)
        nw, nh = round(1280 * scale), round(720 * scale)
        left, top = round((w - nw) / 2 - 0.1), round((h - nh) / 2 - 0.1)
        mask = output.argmax(0).astype(np.uint8)[top:top + nh, left:left + nw]
        result.append(cv2.resize(mask, (W_, 360), interpolation=cv2.INTER_NEAREST))
    return tuple(result)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=DEFAULT_REPO)
    ap.add_argument("--config", required=True)
    ap.add_argument("--weight")
    ap.add_argument("--height", type=int, default=H_)
    ap.add_argument("--width", type=int, default=W_)
    ap.add_argument("--param")
    ap.add_argument("--bin")
    ap.add_argument("--images", help="folder of jpg/png for parity (default: repo inference/images)")
    ap.add_argument("--bdd-root", help="BDD100K root containing images/val and the two annotation folders")
    ap.add_argument("--da-labels", help="directory of author-provided drivable-area PNG masks")
    ap.add_argument("--ll-labels", help="directory of author-provided lane-line PNG masks")
    ap.add_argument("--limit", type=int, default=0, help="max images (0 = all)")
    ap.add_argument("--threads", type=int, default=4)
    args = ap.parse_args()

    if bool(args.da_labels) != bool(args.ll_labels):
        ap.error("--da-labels and --ll-labels must be provided together")
    if args.bdd_root and (args.images or args.da_labels):
        ap.error("--bdd-root cannot be combined with --images or explicit label directories")
    if args.da_labels and not args.images:
        ap.error("explicit label directories require --images")
    if args.limit < 0:
        ap.error("--limit must be non-negative")
    if args.bdd_root:
        args.da_labels = os.path.join(args.bdd_root, "drivable_area_annotations", "val")
        args.ll_labels = os.path.join(args.bdd_root, "lane_line_annotations", "val")

    if args.height <= 0 or args.width <= 0:
        ap.error("--height and --width must be positive")

    stem = f"{args.config}_{args.height}x{args.width}"
    param = args.param or f"out/{stem}.ncnn.param"
    binf = args.bin or f"out/{stem}.ncnn.bin"

    if args.bdd_root:
        files = sorted(glob.glob(os.path.join(args.bdd_root, "images", "val", "*.jpg")))
    else:
        folder = args.images or os.path.join(args.repo, "inference", "images")
        files = sorted(f for ext in ("jpg", "jpeg", "png") for f in glob.glob(os.path.join(folder, f"*.{ext}")))
    if args.limit:
        files = files[:args.limit]
    if not files:
        sys.exit("no images found")
    print(f"{len(files)} images")
    if args.da_labels:
        for folder in (args.da_labels, args.ll_labels):
            if not os.path.isdir(folder):
                sys.exit(f"label directory not found: {folder}")
        for image in files:
            name = os.path.splitext(os.path.basename(image))[0] + ".png"
            for folder in (args.da_labels, args.ll_labels):
                path = os.path.join(folder, name)
                if not os.path.isfile(path):
                    sys.exit(f"missing label: {path}")

    torch.set_num_threads(args.threads)
    model = load_model(args.repo, args.config, args.weight, patched=False)
    ncnn_model = NcnnModel(param, binf, args.threads)

    expected_shape = (720, 1280) if args.da_labels else None
    inputs = [preprocess(f, args.height, args.width, expected_shape) for f in files[:8]]
    print(f"Parity check: {len(inputs)} images")

    # 1) upstream vs patched torch
    set_caam(False)
    ref = [torch_forward(model, x) for x in inputs]
    set_caam(True)
    pat = [torch_forward(model, x) for x in inputs]
    d = max(float(np.abs(a - b).max()) for r, p in zip(ref, pat) for a, b in zip(r, p))
    print(f"[1] upstream vs patched torch      max|logit diff| = {d:.2e}  {'OK' if d < 1e-3 else 'MISMATCH'}")

    # 2) torch vs ncnn
    dmax, agree_da, agree_ll = 0.0, [], []
    for x, p in zip(inputs, pat):
        n = ncnn_model(x)
        dmax = max(dmax, *(float(np.abs(a - b).max()) for a, b in zip(p, n)))
        mp, mn = masks(*p), masks(*n)
        agree_da.append((mp[0] == mn[0]).mean())
        agree_ll.append((mp[1] == mn[1]).mean())
    print(f"[2] torch vs ncnn                  max|logit diff| = {dmax:.2e}")
    print(f"    mask agreement  DA {np.mean(agree_da) * 100:.3f}%  LL {np.mean(agree_ll) * 100:.3f}%  "
          f"(min DA {np.min(agree_da) * 100:.2f}%, LL {np.min(agree_ll) * 100:.2f}%)")

    # timing
    x = inputs[0]
    for _ in range(3):
        ncnn_model(x)
    ts = []
    for _ in range(20):
        t = time.perf_counter()
        ncnn_model(x)
        ts.append((time.perf_counter() - t) * 1000)
    print(f"    ncnn {args.threads} threads: median {np.median(ts):.1f} ms  p95 {np.percentile(ts, 95):.1f} ms  (this machine)")

    # 3) paper metrics
    if args.da_labels:
        mt, mn = Metric(), Metric()
        t0 = time.time()
        for i, f in enumerate(files):
            gt = load_gt(f, args.da_labels, args.ll_labels)
            x = preprocess(f, args.height, args.width, expected_shape)
            da_t, ll_t = masks(*torch_forward(model, x))
            da_n, ll_n = masks(*ncnn_model(x))
            mt.add(da_t, gt[0], ll_t, gt[1])
            mn.add(da_n, gt[0], ll_n, gt[1])
            if (i + 1) % 100 == 0:
                print(f"    {i + 1}/{len(files)}  {time.time() - t0:.0f}s", flush=True)
        print(f"[3] Metrics on {len(mt.da_miou)} images (per-image average; upstream batch size 1)")
        print("                                   DA mIoU   LL Acc    LL IoU")
        for name, m in (("torch", mt), ("ncnn", mn)):
            a, b, c = m.summary()
            print(f"    {name:<30} {a * 100:7.2f}   {b * 100:7.2f}   {c * 100:7.2f}")


if __name__ == "__main__":
    main()
