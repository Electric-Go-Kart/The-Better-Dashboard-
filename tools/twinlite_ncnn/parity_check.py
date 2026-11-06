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
  3. with --bdd-root: DA mIoU, LL accuracy, LL IoU (the metrics reported in the TwinLiteNet / TwinLiteNetPlus
     papers, computed the way upstream val.py does: per-image, averaged, prediction cropped [12:-12]),
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


def preprocess(path, h, w):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)
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
        self.net.load_param(param)
        self.net.load_model(binf)

    def __call__(self, chw):
        ex = self.net.create_extractor()
        ex.input("in0", self.ncnn.Mat(chw).clone())
        _, a = ex.extract("out0")
        _, b = ex.extract("out1")
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
    """Per-image metrics exactly as upstream IOUEval.SegmentationMetric / utils.val."""

    def __init__(self):
        self.da_miou, self.ll_acc, self.ll_iou = [], [], []

    @staticmethod
    def cm(pred, gt):
        k = (gt >= 0) & (gt < 2)
        return np.bincount(2 * gt[k] + pred[k], minlength=4).reshape(2, 2).astype(np.float64)

    def add(self, da_pred, da_gt, ll_pred, ll_gt):
        c = self.cm(da_pred, da_gt)
        union = c.sum(1) + c.sum(0) - np.diag(c)
        iou = np.nan_to_num(np.diag(c) / union)
        self.da_miou.append(iou.mean())

        c = self.cm(ll_pred, ll_gt)
        tp, fp, fn, tn = c[1, 1], c[0, 1], c[1, 0], c[0, 0]
        self.ll_acc.append((tp / (tp + fn + 1e-12) + tn / (tn + fp + 1e-12)) / 2)
        union = c.sum(1) + c.sum(0) - np.diag(c)
        self.ll_iou.append(np.nan_to_num(np.diag(c) / union)[1])

    def summary(self):
        return (float(np.mean(self.da_miou)), float(np.mean(self.ll_acc)), float(np.mean(self.ll_iou)))


def load_gt(img_path):
    da = cv2.imread(img_path.replace("images", "drivable_area_annotations").replace("jpg", "png"), 0)
    ll = cv2.imread(img_path.replace("images", "lane_line_annotations").replace("jpg", "png"), 0)
    if da is None or ll is None:
        return None
    # Upstream: resize to (640, 360), class 1 where value > 1
    da = (cv2.resize(da, (W_, 360)) > 1).astype(np.int64)
    ll = (cv2.resize(ll, (W_, 360)) > 1).astype(np.int64)
    return da, ll


def masks(da, ll):
    return da.argmax(0)[12:-12], ll.argmax(0)[12:-12]


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
    ap.add_argument("--limit", type=int, default=0, help="max images (0 = all)")
    ap.add_argument("--threads", type=int, default=4)
    args = ap.parse_args()

    if (args.height, args.width) != (H_, W_) and args.bdd_root:
        sys.exit("BDD100K metrics assume 384x640 (labels are cropped [12:-12]); use the default size for them")

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

    torch.set_num_threads(args.threads)
    model = load_model(args.repo, args.config, args.weight, patched=False)
    ncnn_model = NcnnModel(param, binf, args.threads)

    inputs = [preprocess(f, args.height, args.width) for f in files[:8 if not args.bdd_root else 8]]

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
    if args.bdd_root:
        mt, mn = Metric(), Metric()
        t0 = time.time()
        for i, f in enumerate(files):
            gt = load_gt(f)
            if gt is None:
                continue
            x = preprocess(f, args.height, args.width)
            da_t, ll_t = masks(*torch_forward(model, x))
            da_n, ll_n = masks(*ncnn_model(x))
            mt.add(da_t, gt[0], ll_t, gt[1])
            mn.add(da_n, gt[0], ll_n, gt[1])
            if (i + 1) % 100 == 0:
                print(f"    {i + 1}/{len(files)}  {time.time() - t0:.0f}s", flush=True)
        print("[3] BDD100K val metrics            DA mIoU   LL Acc    LL IoU")
        for name, m in (("torch", mt), ("ncnn FP32", mn)):
            a, b, c = m.summary()
            print(f"    {name:<30} {a * 100:7.2f}   {b * 100:7.2f}   {c * 100:7.2f}")


if __name__ == "__main__":
    main()
