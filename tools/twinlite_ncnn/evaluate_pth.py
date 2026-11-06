#!/usr/bin/env python3
"""Evaluate original TwinLiteNetPlus PyTorch checkpoints using upstream batch metrics."""
import argparse
from pathlib import Path

import numpy as np
import torch

from export_ncnn import DEFAULT_REPO, load_model
from parity_check import Metric, load_gt, preprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", choices=("nano", "small", "medium", "large"), required=True)
    parser.add_argument("--repo", default=DEFAULT_REPO)
    parser.add_argument("--weight")
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--da-labels", type=Path, required=True)
    parser.add_argument("--ll-labels", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--limit", type=int, default=0, help="first N sorted images; 0 means all")
    args = parser.parse_args()
    if args.batch_size < 1 or args.threads < 1 or args.limit < 0:
        parser.error("batch size and threads must be positive; limit must be non-negative")
    if args.device == "cuda" and not torch.cuda.is_available():
        parser.error("CUDA is unavailable to PyTorch in this environment")
    images = sorted(args.images.glob("*.jpg"))
    if args.limit:
        images = images[:args.limit]
    if not images:
        parser.error(f"no JPG images in {args.images}")
    for image in images:
        for folder in (args.da_labels, args.ll_labels):
            if not (folder / (image.stem + ".png")).is_file():
                parser.error(f"missing label: {folder / (image.stem + '.png')}")

    torch.set_num_threads(args.threads)
    model = load_model(args.repo, args.config, args.weight, patched=False).to(args.device)
    sums = np.zeros(3)
    count = 0
    print(f"{args.config}: {len(images)} images, original PyTorch, {args.device}, batch size {args.batch_size}", flush=True)
    with torch.inference_mode():
        for start in range(0, len(images), args.batch_size):
            batch = images[start:start + args.batch_size]
            inputs = np.stack([preprocess(str(f), 384, 640, (720, 1280)) for f in batch])
            ground_truth = [load_gt(str(f), str(args.da_labels), str(args.ll_labels)) for f in batch]
            outputs = model(torch.from_numpy(inputs).to(args.device))
            predictions = [out.argmax(1)[:, 12:-12].cpu().numpy() for out in outputs]
            metric = Metric()
            metric.add(predictions[0], np.stack([gt[0] for gt in ground_truth]),
                       predictions[1], np.stack([gt[1] for gt in ground_truth]))
            sums += np.asarray(metric.summary()) * len(batch)
            count += len(batch)
            if start == 0 or count % 400 == 0 or count == len(images):
                print(f"{count}/{len(images)}", flush=True)
    da, acc, iou = sums / count * 100
    print(f"DA mIoU {da:.2f}%  Lane accuracy {acc:.2f}%  Lane IoU {iou:.2f}%", flush=True)


if __name__ == "__main__":
    main()
