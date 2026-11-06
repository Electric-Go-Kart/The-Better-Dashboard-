#!/usr/bin/env python3
"""Export a TwinLiteNetPlus .pth (nano/small/...) to ncnn FP32 .param/.bin via PNNX.

Run with tools/.venv-ml (torch + pnnx):
    python export_ncnn.py --config nano --height 384 --width 640 --out out/
Outputs <out>/<config>_<H>x<W>.ncnn.param / .bin plus a .pt trace.
"""
import argparse
import os
import shutil
import sys

import torch

DEFAULT_REPO = "/home/joshua/Desktop/gokart/models/twin/TwinLiteNetPlus"


def load_model(repo, config, weight=None, patched=True):
    """patched=True swaps in the ncnn-friendly CAAM; patched=False restores upstream behaviour."""
    sys.path.insert(0, repo)
    from model.model import CAAM, TwinLiteNetPlus

    import twinlite_patch
    if not hasattr(CAAM, "_upstream_forward"):
        CAAM._upstream_forward = CAAM.forward
    CAAM.forward = twinlite_patch.caam_forward if patched else CAAM._upstream_forward

    model = TwinLiteNetPlus(argparse.Namespace(config=config))
    weight = weight or os.path.join(repo, "pretrained", f"{config}.pth")
    model.load_state_dict(torch.load(weight, map_location="cpu"))
    return model.eval()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=DEFAULT_REPO)
    ap.add_argument("--config", required=True, choices=["nano", "small", "medium", "large"])
    ap.add_argument("--weight")
    ap.add_argument("--height", type=int, default=384)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--out", default="out")
    args = ap.parse_args()

    import pnnx

    os.makedirs(args.out, exist_ok=True)
    model = load_model(args.repo, args.config, args.weight)
    x = torch.rand(1, 3, args.height, args.width)
    stem = f"{args.config}_{args.height}x{args.width}"
    pt = os.path.join(args.out, stem + ".pt")

    # Fixed shape, fp32 (fp16=False keeps weights comparable to PyTorch for parity checks)
    pnnx.export(model, pt, x, fp16=False)

    for ext in ("param", "bin"):
        src = os.path.join(args.out, f"{stem}.ncnn.{ext}")
        if not os.path.exists(src):
            sys.exit(f"missing {src}; check PNNX output above")
    print("wrote", os.path.join(args.out, stem + ".ncnn.param"))


if __name__ == "__main__":
    main()
