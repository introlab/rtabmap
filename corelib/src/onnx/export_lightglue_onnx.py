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
# Export-friendly replacements of the LightGlue code that the TorchScript ONNX
# exporter does not handle with dynamic keypoint counts: Tensor.unflatten()
# with a -1 size loses the tensor rank, and the exporter then resolves the
# negative dimensions, flatten() ranges and slice ends that follow against a
# wrong rank or shape. They compute the same thing (each was checked to be
# needed by exporting without it, and the verification below compares the
# exported model with the original code).
# ---------------------------------------------------------------------------

def unflatten_export(self, dim, sizes):
    dim = dim % self.dim()
    sizes = list(sizes)
    if -1 in sizes:
        known = 1
        for s in sizes:
            if s != -1:
                known *= s
        sizes[sizes.index(-1)] = self.shape[dim] // known
    return self.reshape(tuple(self.shape[:dim]) + tuple(sizes) + tuple(self.shape[dim + 1:]))


def merge_heads_export(x):
    """[b,h,n,d] -> [b,n,h*d] with an explicit reshape."""
    b, h, n, d = x.shape
    return x.permute(0, 2, 1, 3).reshape(b, n, h * d)


def selfblock_forward_export(self, x, encoding, mask=None):
    """SelfBlock.forward with permutes instead of transposes (permute needs no
    rank inference from the exporter)."""
    lg = sys.modules[self.__module__]
    qkv = self.Wqkv(x)
    qkv = qkv.unflatten(-1, (self.num_heads, -1, 3)).permute(0, 2, 1, 3, 4)
    q, k, v = qkv.select(4, 0), qkv.select(4, 1), qkv.select(4, 2)
    q = lg.apply_cached_rotary_emb(encoding, q)
    k = lg.apply_cached_rotary_emb(encoding, k)
    context = self.inner_attn(q, k, v, mask=mask)
    # reshape rather than flatten(start_dim=2): flatten's end_dim=-1 is
    # normalized by the exporter with its wrong rank
    message = self.out_proj(merge_heads_export(context))
    return x + self.ffn(torch.cat([x, message], 2))


def crossblock_forward_export(self, x0, x1, mask=None):
    """CrossBlock.forward with matmuls instead of einsums."""
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
    m0, m1 = self.map_(merge_heads_export, m0, m1)
    m0, m1 = self.map_(self.to_out, m0, m1)
    x0 = x0 + self.ffn(torch.cat([x0, m0], 2))
    x1 = x1 + self.ffn(torch.cat([x1, m1], 2))
    return x0, x1


def sigmoid_log_double_softmax_export(sim, z0, z1):
    """sigmoid_log_double_softmax built by concatenation instead of in-place
    writes into a new tensor (same values)."""
    certainties = F.logsigmoid(z0) + F.logsigmoid(z1).permute(0, 2, 1)
    scores0 = F.log_softmax(sim, 2)
    scores1 = F.log_softmax(sim.permute(0, 2, 1), 2).permute(0, 2, 1)
    inner = scores0 + scores1 + certainties  # [b,m,n]
    col = F.logsigmoid(-z0)  # [b,m,1]
    row = F.logsigmoid(-z1).permute(0, 2, 1)  # [b,1,n]
    corner = torch.zeros_like(row[:, :, :1])  # [b,1,1]
    return torch.cat([torch.cat([inner, col], 2), torch.cat([row, corner], 2)], 1)  # [b,m+1,n+1]


def filter_matches_export(scores, th):
    """filter_matches with the slice ends taken from the traced shape: the
    exporter resolves the literal -1 ends of the original against the wrong
    shape it believes."""
    m = scores.shape[1] - 1
    n = scores.shape[2] - 1
    inner = scores[:, :m, :n]
    max0, max1 = inner.max(2), inner.max(1)
    m0, m1 = max0.indices, max1.indices
    indices0 = torch.arange(m0.shape[1], device=m0.device)[None]
    indices1 = torch.arange(m1.shape[1], device=m1.device)[None]
    mutual0 = indices0 == m1.gather(1, m0)
    mutual1 = indices1 == m0.gather(1, m1)
    max0_exp = max0.values.exp()
    zero = max0_exp.new_tensor(0)
    mscores0 = torch.where(mutual0, max0_exp, zero)
    mscores1 = torch.where(mutual1, mscores0.gather(1, m1), zero)
    valid0 = mutual0 & (mscores0 > th)
    valid1 = mutual1 & valid0.gather(1, m1)
    m0 = torch.where(valid0, m0, -1)
    m1 = torch.where(valid1, m1, -1)
    return m0, m1, mscores0, mscores1


def register_transpose_diagnostic():
    """Reports which tensor a failing transpose export comes from (the
    exporter's own error doesn't say)."""
    try:
        from torch.onnx._internal.torchscript_exporter import symbolic_opset9 as opset9
    except ImportError:
        from torch.onnx import symbolic_opset9 as opset9

    def transpose_diagnostic(g, self, dim0, dim1):
        try:
            return opset9.transpose(g, self, dim0, dim1)
        except Exception:
            print("transpose export failed on %s (dims %s, %s) produced by %s in %s" % (
                self.type(), dim0, dim1, self.node().kind(), self.node().scopeName()))
            raise

    torch.onnx.register_custom_op_symbolic("aten::transpose", transpose_diagnostic, 9)


PATCHES = {
    "unflatten": lambda lg: setattr(torch.Tensor, "unflatten", unflatten_export),
    "self_block": lambda lg: setattr(lg.SelfBlock, "forward", selfblock_forward_export),
    "cross_block": lambda lg: setattr(lg.CrossBlock, "forward", crossblock_forward_export),
    "double_softmax": lambda lg: setattr(lg, "sigmoid_log_double_softmax", sigmoid_log_double_softmax_export),
    "filter_matches": lambda lg: setattr(lg, "filter_matches", filter_matches_export),
}


def patch_for_export(lg, skip=()):
    for name, apply in PATCHES.items():
        if name not in skip:
            apply(lg)
    register_transpose_diagnostic()


def dump_intermediate_shapes(path, inputs, scopes, top_level=False):
    """Debugging aid: runs the model with every intermediate tensor of the
    given scopes (and of the top-level forward with top_level) as an output
    and prints their shapes."""
    import onnx
    import onnxruntime as ort
    model = onnx.load(path)
    model = onnx.shape_inference.infer_shapes(model)
    known = {vi.name: vi for vi in list(model.graph.value_info)}
    names = []
    for node in model.graph.node:
        for out in node.output:
            selected = any(s in out for s in scopes) or (top_level and out.startswith("/matcher/") and out.count("/") == 2)
            if out and selected and out in known:
                model.graph.output.append(known[out])
                names.append(out)
    session = ort.InferenceSession(model.SerializeToString(), providers=["CPUExecutionProvider"])
    results = session.run(names, {k: v.numpy() for k, v in inputs.items()})
    print("Intermediate shapes (name: runtime shape):")
    for name, value in zip(names, results):
        print("  %s: %s" % (name, list(value.shape)))


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
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    parser.add_argument("--skip-patches", default="", help="Debugging: comma-separated export substitutions to leave out, among " + ", ".join(PATCHES))
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

    patch_for_export(lg, skip=[s for s in args.skip_patches.split(",") if s])

    print("Exporting to %s (descriptor dimension %d, keypoints with %d columns)" % (args.output, dim, columns))
    with torch.no_grad():
        torch.onnx.export(
            model,
            (kpts0, kpts1, desc0, desc1),
            args.output,
            input_names=["kpts0", "kpts1", "desc0", "desc1"],
            output_names=["matches0", "mscores0"],
            dynamic_axes={
                "kpts0": {1: "num_keypoints0"},
                "kpts1": {1: "num_keypoints1"},
                "desc0": {1: "num_keypoints0"},
                "desc1": {1: "num_keypoints1"},
                "matches0": {1: "num_keypoints0"},
                "mscores0": {1: "num_keypoints0"},
            },
            opset_version=args.opset,
            do_constant_folding=True,
            dynamo=False,
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
            dump_intermediate_shapes(args.output, check_inputs, ["log_assignment"], top_level=True)
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
