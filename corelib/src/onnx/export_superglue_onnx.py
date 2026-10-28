#!/usr/bin/env python3
#
# Exports SuperGlue (https://github.com/magicleap/SuperGluePretrainedNetwork,
# non-commercial license) to an ONNX model following RTAB-Map's OnnxMatcher
# interface (see README.md), from a clone of the repository (--superglue-dir,
# which holds models/superglue.py and models/weights/superglue_{indoor,outdoor}.pth).
#
# The model takes "kpts0" float32 [1,M,2] and "kpts1" float32 [1,N,2] (the
# keypoints normalized by RTAB-Map with the image size the LightGlue way,
# (x - w/2) / (max(w,h)/2); SuperGlue's own normalization, by 0.7 max(w,h),
# is a constant rescaling done inside the model), "desc0" float32 [1,M,256]
# and "desc1" float32 [1,N,256] (SuperPoint descriptors), "scores0" float32
# [1,M] and "scores1" float32 [1,N] (the keypoint detection scores, which
# SuperGlue's keypoint encoder takes), and returns "matches0" int64 [1,M]
# (index in the train keypoints or -1) and "mscores0" float32 [1,M].
# The Sinkhorn iterations are unrolled in the graph (--iterations).
#
# SuperGlue was trained with the magicleap SuperPoint: use it with
# superpoint_magicleap.onnx (LightGlue-ONNX releases) or SuperPointTorch.
#
# Usage:
#   python3 export_superglue_onnx.py --superglue-dir ~/SuperGluePretrainedNetwork --weights indoor --output superglue_indoor.onnx
#
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn


def attention_export(query, key, value):
    """SuperGlue's attention() with matmuls instead of einsums: [b,d,h,n] tensors."""
    dim = query.shape[1]
    q = query.permute(0, 2, 3, 1)  # [b,h,n,d]
    k = key.permute(0, 2, 1, 3)    # [b,h,d,m]
    v = value.permute(0, 2, 3, 1)  # [b,h,m,d]
    scores = torch.matmul(q, k) / dim ** 0.5
    prob = F.softmax(scores, 3)
    out = torch.matmul(prob, v)    # [b,h,n,d]
    return out.permute(0, 3, 1, 2), prob


def log_sinkhorn_iterations(Z, log_mu, log_nu, iters):
    u, v = torch.zeros_like(log_mu), torch.zeros_like(log_nu)
    for _ in range(iters):
        u = log_mu - torch.logsumexp(Z + v.unsqueeze(1), dim=2)
        v = log_nu - torch.logsumexp(Z + u.unsqueeze(2), dim=1)
    return Z + u.unsqueeze(2) + v.unsqueeze(1)


def log_optimal_transport(scores, alpha, iters):
    """SuperGlue's log_optimal_transport with the sizes from the traced shape."""
    b, m, n = scores.shape
    one = scores.new_tensor(1.0)
    ms, ns = m * one, n * one
    bins0 = alpha.expand(b, m, 1)
    bins1 = alpha.expand(b, 1, n)
    alpha = alpha.expand(b, 1, 1)
    couplings = torch.cat([torch.cat([scores, bins0], 2), torch.cat([bins1, alpha], 2)], 1)
    norm = -(ms + ns).log()
    log_mu = torch.cat([norm.expand(m), ns.log()[None] + norm])
    log_nu = torch.cat([norm.expand(n), ms.log()[None] + norm])
    log_mu, log_nu = log_mu[None].expand(b, -1), log_nu[None].expand(b, -1)
    Z = log_sinkhorn_iterations(couplings, log_mu, log_nu, iters)
    return Z - norm


class SuperGlueOnnx(nn.Module):
    def __init__(self, superglue, iterations, threshold):
        super().__init__()
        self.superglue = superglue
        self.iterations = iterations
        self.threshold = threshold

    def forward(self, kpts0, kpts1, desc0, desc1, scores0, scores1):
        sg = self.superglue
        # LightGlue normalization (half the largest side) to SuperGlue's (0.7 x largest side)
        k0 = kpts0 * (0.5 / 0.7)
        k1 = kpts1 * (0.5 / 0.7)
        d0 = desc0.permute(0, 2, 1)  # [1,256,M]
        d1 = desc1.permute(0, 2, 1)
        d0 = d0 + sg.kenc(k0, scores0)
        d1 = d1 + sg.kenc(k1, scores1)
        d0, d1 = sg.gnn(d0, d1)
        mdesc0, mdesc1 = sg.final_proj(d0), sg.final_proj(d1)
        scores = torch.matmul(mdesc0.permute(0, 2, 1), mdesc1) / sg.config["descriptor_dim"] ** 0.5  # [1,M,N]
        scores = log_optimal_transport(scores, sg.bin_score, self.iterations)

        m = scores.shape[1] - 1
        n = scores.shape[2] - 1
        inner = scores[:, :m, :n]
        max0, max1 = inner.max(2), inner.max(1)
        indices0, indices1 = max0.indices, max1.indices
        arange0 = torch.arange(m, device=scores.device)[None]
        arange1 = torch.arange(n, device=scores.device)[None]
        mutual0 = arange0 == indices1.gather(1, indices0)
        mutual1 = arange1 == indices0.gather(1, indices1)
        zero = scores.new_tensor(0.0)
        mscores0 = torch.where(mutual0, max0.values.exp(), zero)
        valid0 = mutual0 & (mscores0 > self.threshold)
        matches0 = torch.where(valid0, indices0, -1)
        return matches0, mscores0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--superglue-dir", required=True, help="Clone of https://github.com/magicleap/SuperGluePretrainedNetwork (models/superglue.py, models/weights/*.pth)")
    parser.add_argument("--weights", default="indoor", choices=["indoor", "outdoor"], help="Pretrained weights (default indoor)")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--iterations", type=int, default=20, help="Sinkhorn iterations, unrolled in the graph (default 20, rtabmap's PyMatcher/Iterations default; SuperGlue's own default is 100)")
    parser.add_argument("--threshold", type=float, default=0.2, help="Match threshold inside the model (default 0.2, SuperGlue's)")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    sys.path.insert(0, os.path.abspath(args.superglue_dir))
    import models.superglue as sg_module
    print("Loading SuperGlue (%s weights) from %s" % (args.weights, args.superglue_dir))
    superglue = sg_module.SuperGlue({"weights": args.weights, "sinkhorn_iterations": args.iterations, "match_threshold": args.threshold}).eval()
    model = SuperGlueOnnx(superglue, args.iterations, args.threshold).eval()

    torch.manual_seed(0)
    m, n = 512, 384
    def random_inputs(m, n):
        return {
            "kpts0": torch.rand(1, m, 2) * 2 - 1,
            "kpts1": torch.rand(1, n, 2) * 2 - 1,
            "desc0": F.normalize(torch.randn(1, m, 256), dim=-1),
            "desc1": F.normalize(torch.randn(1, n, 256), dim=-1),
            "scores0": torch.rand(1, m),
            "scores1": torch.rand(1, n),
        }
    inputs = random_inputs(m, n)

    # Reference of the original code on the check inputs (query keypoints 0..299
    # planted in the train set), before the export substitutions.
    check = random_inputs(m, 500)
    check["kpts1"][:, :300] = check["kpts0"][:, :300] + 0.001
    check["desc1"][:, :300] = check["desc0"][:, :300]
    check["scores1"][:, :300] = check["scores0"][:, :300]
    def original(inputs):
        # SuperGlue's own forward: pixel keypoints and an image whose size gives
        # the same normalization as the LightGlue-normalized inputs
        w, h = 640, 480
        half = max(w, h) / 2.0
        center = torch.tensor([w / 2.0, h / 2.0])
        data = {
            "image0": torch.zeros(1, 1, h, w), "image1": torch.zeros(1, 1, h, w),
            "keypoints0": inputs["kpts0"] * half + center, "keypoints1": inputs["kpts1"] * half + center,
            "descriptors0": inputs["desc0"].permute(0, 2, 1), "descriptors1": inputs["desc1"].permute(0, 2, 1),
            "scores0": inputs["scores0"], "scores1": inputs["scores1"],
        }
        pred = superglue(data)
        return [pred["matches0"].numpy(), pred["matching_scores0"].numpy()]
    with torch.no_grad():
        ref = original(check)

    sg_module.attention = attention_export

    print("Exporting to %s (%d Sinkhorn iterations, threshold %g)" % (args.output, args.iterations, args.threshold))
    with torch.no_grad():
        torch.onnx.export(
            model,
            tuple(inputs.values()),
            args.output,
            input_names=list(inputs.keys()),
            output_names=["matches0", "mscores0"],
            dynamic_axes={
                "kpts0": {1: "num_keypoints0"}, "kpts1": {1: "num_keypoints1"},
                "desc0": {1: "num_keypoints0"}, "desc1": {1: "num_keypoints1"},
                "scores0": {1: "num_keypoints0"}, "scores1": {1: "num_keypoints1"},
                "matches0": {1: "num_keypoints0"}, "mscores0": {1: "num_keypoints0"},
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
    out = session.run(None, {k: v.numpy() for k, v in check.items()})
    for name, r, o in zip(["matches0", "mscores0"], ref, out):
        if r.shape != o.shape:
            raise RuntimeError("%s: torch %s vs onnx %s" % (name, r.shape, o.shape))
    planted = np.arange(300)
    recall_ref = float((ref[0][0, :300] == planted).mean())
    recall_onnx = float((out[0][0, :300] == planted).mean())
    score_diff = float(np.abs(ref[1] - out[1]).max())
    print("Verified: planted matches found by the original code %.1f%%, by onnxruntime %.1f%%; max match score difference %g"
          % (100 * recall_ref, 100 * recall_onnx, score_diff))
    if score_diff > 1e-2:
        raise RuntimeError("Match scores differ between the original code and onnxruntime")
    if recall_onnx < recall_ref - 0.05 or recall_onnx < 0.9:
        raise RuntimeError("The exported model finds fewer of the planted matches than the original code")


if __name__ == "__main__":
    main()
