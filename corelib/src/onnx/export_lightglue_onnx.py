#!/usr/bin/env python3
#
# Exports LightGlue (https://github.com/cvg/LightGlue, Apache-2.0) to an ONNX
# model following RTAB-Map's OnnxMatcher interface (see README.md).
#
# The model takes "kpts0" float32 [1,M,2] and "kpts1" float32 [1,N,2] (the
# keypoints normalized by RTAB-Map with the image size, as LightGlue does:
# (x - w/2) / (max(w,h)/2)), "desc0" float32 [1,M,D] and "desc1" float32
# [1,N,D], and returns "matches0" int64 [1,M] (index in the train keypoints or
# -1) and "mscores0" float32 [1,M] (confidence of each match). For the
# checkpoints trained with the keypoint scale and orientation (sift,
# doghardnet), the keypoint inputs have 4 columns: x, y, scale (OpenCV
# keypoint size) and orientation (radians), filled by RTAB-Map. The adaptive
# depth and width of LightGlue (early stopping, point pruning) are disabled: a
# static graph always runs all the layers.
#
# LightGlue should be installed (pip install git+https://github.com/cvg/LightGlue.git)
# or cloned and given with --lightglue-dir (its lightglue.py only needs torch).
# The weights of the given feature type are downloaded by LightGlue itself; a
# checkpoint of your own (e.g. trained with https://github.com/cvg/glue-factory)
# is given with --weights and --input-dim.
#
# Usage:
#   python3 export_lightglue_onnx.py --features superpoint --output superpoint_lightglue.onnx
#   python3 export_lightglue_onnx.py --features sift --output sift_lightglue.onnx
#   python3 export_lightglue_onnx.py --weights my_lightglue.pth --input-dim 256 --output my_lightglue.onnx
#
import argparse
import importlib
import os
import sys
import types

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn


def import_lightglue_module(lightglue_dir):
    """Imports lightglue.lightglue, without the package's __init__ (and its
    dependencies on kornia, opencv...) when a clone of the repository is given."""
    if lightglue_dir:
        package_dir = os.path.join(os.path.abspath(lightglue_dir), "lightglue")
        if not os.path.exists(os.path.join(package_dir, "lightglue.py")):
            raise FileNotFoundError("%s/lightglue.py not found, --lightglue-dir should be a clone of https://github.com/cvg/LightGlue" % package_dir)
        package = types.ModuleType("lightglue")
        package.__path__ = [package_dir]
        sys.modules["lightglue"] = package
    return importlib.import_module("lightglue.lightglue")


# ---------------------------------------------------------------------------
# Replacements of the LightGlue code for the exported model. They compute the
# same thing (the verification below compares the exported model with the
# original code).
# ---------------------------------------------------------------------------

def merge_heads(x):
    """[b,h,n,d] -> [b,n,h*d]."""
    b, h, n, d = x.shape
    return x.permute(0, 2, 1, 3).reshape(b, n, h * d)


def selfblock_forward_export(self, x, encoding, mask=None):
    """SelfBlock.forward with q, k and v taken as contiguous blocks of the
    Wqkv output (reordered by reorder_qkv_weights())."""
    lg = sys.modules[self.__module__]
    qkv = self.Wqkv(x)
    c = qkv.shape[-1] // 3
    q, k, v = (qkv[..., i * c:(i + 1) * c].unflatten(-1, (self.num_heads, -1)).permute(0, 2, 1, 3) for i in range(3))
    q = lg.apply_cached_rotary_emb(encoding, q)
    k = lg.apply_cached_rotary_emb(encoding, k)
    context = self.inner_attn(q, k, v, mask=mask)
    message = self.out_proj(merge_heads(context))
    return x + self.ffn(torch.cat([x, message], 2))


def reorder_qkv_weights(matcher, lg):
    """Reorders the rows of each SelfBlock's Wqkv from the (heads, dim, 3)
    interleaved order of LightGlue to contiguous q, k and v blocks, for
    selfblock_forward_export(). TensorRT fuses the projection and its split
    into q, k and v assuming contiguous blocks: with the interleaved order
    (unflatten(-1, (heads, -1, 3)) + select()), it silently computes wrong
    q, k and v (every keypoint then matches the same few ones)."""
    for block in matcher.modules():
        if isinstance(block, lg.SelfBlock):
            w, b = block.Wqkv.weight.data, block.Wqkv.bias.data
            block.Wqkv.weight.data = w.reshape(block.num_heads, -1, 3, w.shape[1]).permute(2, 0, 1, 3).reshape(w.shape).contiguous()
            block.Wqkv.bias.data = b.reshape(block.num_heads, -1, 3).permute(2, 0, 1).reshape(b.shape).contiguous()


def crossblock_forward_export(self, x0, x1, mask=None):
    """CrossBlock.forward with matmuls instead of einsums: exported as is, the
    einsums are ONNX Einsum nodes, about twice slower with ONNX Runtime on CPU
    and 25% on CUDA."""
    qk0, qk1 = self.map_(self.to_qk, x0, x1)
    v0, v1 = self.map_(self.to_v, x0, x1)
    qk0, qk1, v0, v1 = map(lambda t: t.unflatten(-1, (self.heads, -1)).permute(0, 2, 1, 3), (qk0, qk1, v0, v1))
    qk0, qk1 = qk0 * self.scale ** 0.5, qk1 * self.scale ** 0.5
    sim = torch.matmul(qk0, qk1.permute(0, 1, 3, 2))  # [b,h,i,j]
    if mask is not None:
        sim = sim.masked_fill(~mask, -float("inf"))
    attn01 = F.softmax(sim, dim=3)
    attn10 = F.softmax(sim.permute(0, 1, 3, 2), dim=3)  # [b,h,j,i]
    m0 = torch.matmul(attn01, v1)  # [b,h,i,d]
    m1 = torch.matmul(attn10, v0)  # [b,h,j,d]
    if mask is not None:
        m0, m1 = m0.nan_to_num(), m1.nan_to_num()
    m0, m1 = self.map_(merge_heads, m0, m1)
    m0, m1 = self.map_(self.to_out, m0, m1)
    x0 = x0 + self.ffn(torch.cat([x0, m0], 2))
    x1 = x1 + self.ffn(torch.cat([x1, m1], 2))
    return x0, x1


def sigmoid_log_double_softmax_export(sim, z0, z1):
    """sigmoid_log_double_softmax built by concatenation instead of in-place
    writes into a new tensor (same values): exported as is, the writes are
    ScatterND nodes, about 10% slower with ONNX Runtime on CUDA."""
    certainties = F.logsigmoid(z0) + F.logsigmoid(z1).permute(0, 2, 1)
    scores0 = F.log_softmax(sim, 2)
    scores1 = F.log_softmax(sim.permute(0, 2, 1), 2).permute(0, 2, 1)
    inner = scores0 + scores1 + certainties  # [b,m,n]
    col = F.logsigmoid(-z0)  # [b,m,1]
    row = F.logsigmoid(-z1).permute(0, 2, 1)  # [b,1,n]
    corner = torch.zeros_like(row[:, :, :1])  # [b,1,1]
    return torch.cat([torch.cat([inner, col], 2), torch.cat([row, corner], 2)], 1)  # [b,m+1,n+1]


def patch_for_export(lg, matcher):
    lg.SelfBlock.forward = selfblock_forward_export
    reorder_qkv_weights(matcher, lg)
    lg.CrossBlock.forward = crossblock_forward_export
    lg.sigmoid_log_double_softmax = sigmoid_log_double_softmax_export


class LightGlueOnnx(nn.Module):
    """kpts0/kpts1 [1,N,2] (x,y) or [1,N,4] (x,y,scale,orientation) -> matches0, mscores0."""

    def __init__(self, matcher, scale_ori):
        super().__init__()
        self.matcher = matcher
        self.scale_ori = scale_ori

    def features(self, kpts, desc):
        data = {"keypoints": kpts[..., :2], "descriptors": desc}
        if self.scale_ori:
            data["scales"] = kpts[..., 2]
            data["oris"] = kpts[..., 3]
        return data

    def forward(self, kpts0, kpts1, desc0, desc1):
        pred = self.matcher({"image0": self.features(kpts0, desc0), "image1": self.features(kpts1, desc1)})
        return pred["matches0"], pred["matching_scores0"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--features", default="superpoint", help="Feature type of a pretrained LightGlue: superpoint, disk, aliked, sift, doghardnet (default superpoint)")
    parser.add_argument("--weights", default="", help="Your own LightGlue checkpoint (*.pth) instead of a pretrained one (needs --input-dim)")
    parser.add_argument("--input-dim", type=int, default=0, help="Descriptor dimension of the checkpoint given with --weights")
    parser.add_argument("--scale-ori", action="store_true", help="The checkpoint given with --weights was trained with the keypoint scale and orientation")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--threshold", type=float, default=0.1, help="Match threshold inside the model (default 0.1)")
    parser.add_argument("--lightglue-dir", default="", help="Clone of https://github.com/cvg/LightGlue, when the package is not installed")
    parser.add_argument("--opset", type=int, default=18, help="ONNX opset version (default 18)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    lg = import_lightglue_module(args.lightglue_dir)

    # RTAB-Map normalizes the keypoints before calling the model.
    lg.normalize_keypoints = lambda kpts, size=None: kpts

    conf = dict(
        depth_confidence=-1,   # no early stopping
        width_confidence=-1,   # no point pruning
        filter_threshold=args.threshold,
        flash=False,
        mp=False,
    )
    if args.weights:
        if args.input_dim <= 0:
            parser.error("--input-dim is required with --weights")
        print("Loading LightGlue checkpoint %s (%d-d descriptors%s)" % (args.weights, args.input_dim, ", scale and orientation" if args.scale_ori else ""))
        matcher = lg.LightGlue(features=None, input_dim=args.input_dim, add_scale_ori=args.scale_ori, weights=args.weights, **conf).eval()
    else:
        print("Loading LightGlue for %s features" % args.features)
        matcher = lg.LightGlue(features=args.features, **conf).eval()
    model = LightGlueOnnx(matcher, bool(matcher.conf.add_scale_ori)).eval()
    dim = matcher.conf.input_dim
    columns = 4 if matcher.conf.add_scale_ori else 2

    def random_keypoints(n):
        kpts = torch.rand(1, n, columns) * 2 - 1
        if columns == 4:
            kpts[..., 2] = torch.rand(1, n) * 10 + 1   # scale: keypoint size in pixels
            kpts[..., 3] = torch.rand(1, n) * 2 * np.pi  # orientation in radians
        return kpts

    torch.manual_seed(0)
    m, n = 512, 384
    kpts0 = random_keypoints(m)
    kpts1 = random_keypoints(n)
    desc0 = F.normalize(torch.randn(1, m, dim), dim=-1)
    desc1 = F.normalize(torch.randn(1, n, dim), dim=-1)

    # Reference outputs of the original code for the verification, on other
    # counts than the export ones and with a train set holding part of the
    # query one so that there are matches to compare.
    check_kpts1 = torch.cat([kpts0[:, :300].clone(), random_keypoints(200)], 1)
    check_kpts1[:, :300, :2] += 0.001
    check_desc1 = torch.cat([desc0[:, :300], F.normalize(torch.randn(1, 200, dim), dim=-1)], 1)
    check_inputs = {"kpts0": kpts0, "kpts1": check_kpts1, "desc0": desc0, "desc1": check_desc1}
    with torch.no_grad():
        ref = [t.numpy() for t in model(**check_inputs)]

    patch_for_export(lg, matcher)

    print("Exporting to %s (descriptor dimension %d, keypoints with %d columns)" % (args.output, dim, columns))
    with torch.no_grad():
        n0, n1 = torch.export.Dim("num_keypoints0"), torch.export.Dim("num_keypoints1")
        torch.onnx.export(
            model,
            (kpts0, kpts1, desc0, desc1),
            args.output,
            input_names=["kpts0", "kpts1", "desc0", "desc1"],
            output_names=["matches0", "mscores0"],
            dynamic_shapes=({1: n0}, {1: n1}, {1: n0}, {1: n1}),
            opset_version=args.opset,
            dynamo=True,
            external_data=False,
        )

    import onnx
    onnx.checker.check_model(onnx.load(args.output))
    print("Exported %s (%.1f MB)" % (args.output, os.path.getsize(args.output) / 1e6))

    if args.no_check:
        return
    try:
        import onnxruntime as ort
    except ImportError:
        print("onnxruntime is not installed, skipping the verification")
        return
    session = ort.InferenceSession(args.output, providers=["CPUExecutionProvider"])
    print("Model inputs:  %s" % ", ".join("%s %s %s" % (i.name, i.type, i.shape) for i in session.get_inputs()))
    print("Model outputs: %s" % ", ".join("%s %s %s" % (o.name, o.type, o.shape) for o in session.get_outputs()))
    out = session.run(None, {k: v.numpy() for k, v in check_inputs.items()})
    for name, r, o in zip(["matches0", "mscores0"], ref, out):
        if r.shape != o.shape:
            raise RuntimeError("%s: torch %s vs onnx %s" % (name, r.shape, o.shape))
    # The 300 planted correspondences must be found by both; the other query
    # keypoints have near-tied random candidates whose argmax float noise can
    # flip, so they are not compared index by index.
    planted = np.arange(300)
    recall_ref = float((ref[0][0, :300] == planted).mean())
    recall_onnx = float((out[0][0, :300] == planted).mean())
    score_diff = float(np.abs(ref[1] - out[1]).max())
    same = int((ref[0][0] == out[0][0]).sum())
    print("Verified: planted matches found by the original code %.1f%%, by onnxruntime %.1f%%; "
          "max match score difference %g; %d/%d match indices identical" % (100 * recall_ref, 100 * recall_onnx, score_diff, same, m))
    if score_diff > 1e-2:
        raise RuntimeError("Match scores differ between the original code and onnxruntime")
    if recall_onnx < recall_ref - 0.05 or recall_onnx < 0.9:
        raise RuntimeError("The exported model finds fewer of the planted matches than the original code")


if __name__ == "__main__":
    main()
