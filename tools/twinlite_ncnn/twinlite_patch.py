"""ncnn-friendly CAAM for TwinLiteNetPlus.

The upstream patch_split/patch_recover use 5/6-D tensors, which ncnn cannot represent (max 4-D), so PNNX
silently drops those reshapes and the converted model is wrong. This replaces CAAM.forward with an
equivalent that only uses slicing, cat and <=4-D tensors. Call apply() after importing model.model.
Equivalence with the original is checked by parity_check.py (torch original vs torch patched).
"""
import torch
import torch.nn.functional as F


def _split(t, bins):
    """[B, C, H, W] -> [B, bh*bw, rH*rW, C] (same bin and pixel order as upstream patch_split)."""
    bh, bw = bins
    H, W = t.shape[2], t.shape[3]
    rH, rW = H // bh, W // bw
    parts = []
    for i in range(bh):
        for j in range(bw):
            p = t[:, :, i * rH:(i + 1) * rH, j * rW:(j + 1) * rW]
            parts.append(p.flatten(2).transpose(1, 2).unsqueeze(1))
    return torch.cat(parts, dim=1)


def _recover(t, bins, rH, rW):
    """[B, bh*bw, rH*rW, C] -> [B, C, H, W]."""
    bh, bw = bins
    rows = []
    for i in range(bh):
        cols = []
        for j in range(bw):
            p = t[:, i * bw + j]                      # [B, rH*rW, C]
            p = p.transpose(1, 2)                     # [B, C, rH*rW]
            cols.append(p.reshape(p.shape[0], p.shape[1], rH, rW))
        rows.append(torch.cat(cols, dim=3))
    return torch.cat(rows, dim=2)


def caam_forward(self, x):
    cam = self.conv_cam(x)
    cls_score = self.sigmoid(self.pool_cam(cam))
    residual = x

    H, W = x.shape[2], x.shape[3]
    rH, rW = H // self.bin_size[0], W // self.bin_size[1]
    B, K = cam.shape[0], cam.shape[1]

    cam = _split(cam, self.bin_size)                  # [B, N, rH*rW, K]
    x = _split(x, self.bin_size)                      # [B, N, rH*rW, C]

    bin_confidence = cls_score.view(B, K, -1).transpose(1, 2).unsqueeze(3)
    pixel_confidence = F.softmax(cam, dim=2)

    local_feats = torch.matmul(pixel_confidence.transpose(2, 3), x) * bin_confidence
    local_feats = self.gcn(local_feats)
    global_feats = self.fuse(local_feats)
    global_feats = self.relu(global_feats).repeat(1, x.shape[1], 1, 1)

    query = self.proj_query(x)
    key = self.proj_key(local_feats)
    value = self.proj_value(global_feats)

    aff_map = torch.matmul(query, key.transpose(2, 3))
    aff_map = F.softmax(aff_map, dim=-1)
    out = torch.matmul(aff_map, value)

    out = _recover(out, self.bin_size, rH, rW)
    return residual + self.conv_out(out)


def apply():
    from model.model import CAAM
    CAAM.forward = caam_forward
