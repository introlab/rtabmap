#!/usr/bin/env python3
#
# Exports XFeat (https://github.com/verlab/accelerated_features, Apache-2.0,
# CVPR 2024) to an ONNX model following RTAB-Map's OnnxDetector interface (see
# README.md), from a clone of the repository (--model-dir) and its weights
# (weights/xfeat.pt in the clone):
#   git clone https://github.com/verlab/accelerated_features.git
#
# The model takes a grayscale image "image" float32 [1,1,H,W] in [0,1] (H and
# W dynamic unless --height/--width are given; XFeat averages the channels
# anyway) and returns "keypoints" float32 [1,N,2] (x,y), "scores" float32
# [1,N] and "descriptors" float32 [1,N,64]. It reproduces XFeat's sparse
# detectAndCompute(): keypoint heatmap, NMS, score = heatmap x reliability,
# top-k, descriptors interpolated from the 1/8-resolution feature map and
# L2-normalized. The one difference is how a size that is not a multiple of
# 32 is handled: XFeat resizes the image down, this model zero-pads it (like
# DISK) and drops the keypoints of the padding, so the keypoints stay in the
# image's coordinates. XFeat has no LightGlue checkpoint: match its
# descriptors with the nearest neighbor matchers (Vis/CorNNType 0-5).
#
# Usage:
#   python3 export_xfeat_onnx.py --model-dir accelerated_features --output xfeat.onnx
#
import argparse
import os
import sys
import types

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn


def unfold2d_export(self, x, ws=2):
    """Replaces XFeatModel._unfold2d (space to depth): the original uses
    Tensor.unfold(), which the ONNX exporter only supports for static sizes.
    Same output: channel = c * ws^2 + i * ws + j for the pixel (i, j) of each
    ws x ws block."""
    b, c, h, w = x.shape
    x = x.reshape(b, c, h // ws, ws, w // ws, ws)
    x = x.permute(0, 1, 3, 5, 2, 4)
    return x.reshape(b, c * ws * ws, h // ws, w // ws)


def interpolate_sparse(x, pos, h, w, mode):
    """XFeat's InterpolateSparse2d: samples x [1,C,h',w'] at the pixel positions
    pos [1,N,2] (x,y) of an h x w image (normalized with W-1, H-1 and
    align_corners=False, as in the original), returns [1,N,C]."""
    # The size as a tensor built from the traced shape, so that the export
    # stays dynamic (new_tensor([w - 1, h - 1]) would bake the export size in).
    one = pos.new_tensor(1.0)
    grid = 2.0 * (pos / torch.stack([one * (w - 1), one * (h - 1)])) - 1.0
    sampled = F.grid_sample(x, grid.unsqueeze(-2), mode=mode, align_corners=False)
    return sampled.permute(0, 2, 3, 1).squeeze(-2)


class XFeatOnnx(nn.Module):
    def __init__(self, net, threshold, max_keypoints, interpolation):
        super().__init__()
        self.net = net
        self.threshold = threshold
        self.max_keypoints = max_keypoints
        self.interpolation = interpolation

    def forward(self, image):
        h = image.shape[2]
        w = image.shape[3]
        # The network needs sizes divisible by 32: zero-pad bottom/right (XFeat
        # resizes instead, see the header).
        pad_h = (32 - h % 32) % 32
        pad_w = (32 - w % 32) % 32
        x = F.pad(image, (0, pad_w, 0, pad_h))
        hp = h + pad_h
        wp = w + pad_w

        feats, logits, reliability = self.net(x)  # [1,64,h/8,w/8], [1,65,h/8,w/8], [1,1,h/8,w/8]
        feats = F.normalize(feats, dim=1)

        # Keypoint heatmap at full resolution (get_kpts_heatmap): softmax, drop
        # the dustbin, depth to space.
        scores = F.softmax(logits, 1)[:, :64]
        b, _, hc, wc = scores.shape
        heat = scores.permute(0, 2, 3, 1).reshape(b, hc, wc, 8, 8)
        heat = heat.permute(0, 1, 3, 2, 4).reshape(b, 1, hc * 8, wc * 8)

        # NMS (kernel 5) and threshold, within the image (not the padding)
        local_max = F.max_pool2d(heat, 5, stride=1, padding=2)
        keep = (heat == local_max) & (heat > self.threshold)
        keep = keep[0, 0, :h, :w]
        idxs = torch.nonzero(keep)  # (y,x)
        keypoints = idxs.flip(1).float()[None]  # [1,N,2] (x,y)
        heat_scores = heat[0, 0, :h, :w][keep][None]  # nearest sample of the heatmap = its value

        # score = heatmap x reliability (bilinear at the keypoint, maps cover the padded image)
        rel = interpolate_sparse(reliability, keypoints, hp, wp, "bilinear").squeeze(-1)
        kscores = heat_scores * rel  # [1,N]

        if self.max_keypoints > 0:
            kscores, order = torch.sort(kscores[0], descending=True)
            kscores = kscores[: self.max_keypoints][None]
            keypoints = keypoints[0][order][: self.max_keypoints][None]

        descriptors = interpolate_sparse(feats, keypoints, hp, wp, self.interpolation)  # [1,N,64]
        descriptors = F.normalize(descriptors, dim=-1).reshape(1, -1, 64)  # declared [1,N,64]
        return keypoints, kscores, descriptors


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-dir", required=True, help="Clone of https://github.com/verlab/accelerated_features (holds modules/model.py and weights/xfeat.pt)")
    parser.add_argument("--weights", default="", help="Weights file (default <model-dir>/weights/xfeat.pt)")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--threshold", type=float, default=0.05, help="Detection threshold on the keypoint heatmap (default 0.05, XFeat's)")
    parser.add_argument("--max-keypoints", type=int, default=4096, help="Keep at most this many keypoints, by score (default 4096, XFeat's; 0: all)")
    parser.add_argument("--interpolation", default="bicubic", choices=["bicubic", "bilinear"], help="Descriptor sampling (default bicubic, XFeat's; bilinear is faster on GPU providers)")
    parser.add_argument("--height", type=int, default=0, help="Fixed input height (default 0: dynamic)")
    parser.add_argument("--width", type=int, default=0, help="Fixed input width (default 0: dynamic)")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    model_dir = os.path.abspath(args.model_dir)
    weights = args.weights or os.path.join(model_dir, "weights", "xfeat.pt")
    sys.path.insert(0, model_dir)
    from modules.model import XFeatModel
    print("Loading XFeat from %s" % weights)
    net = XFeatModel()
    net.load_state_dict(torch.load(weights, map_location="cpu", weights_only=True))
    net = net.eval()
    net._unfold2d = types.MethodType(unfold2d_export, net)
    model = XFeatOnnx(net, args.threshold, args.max_keypoints, args.interpolation).eval()

    height = args.height if args.height > 0 else 480
    width = args.width if args.width > 0 else 640
    dynamic = args.height <= 0 or args.width <= 0
    torch.manual_seed(0)
    image = torch.rand(1, 1, height, width)

    dynamic_axes = {
        "keypoints": {1: "num_keypoints"},
        "scores": {1: "num_keypoints"},
        "descriptors": {1: "num_keypoints"},
    }
    if dynamic:
        dynamic_axes["image"] = {2: "height", 3: "width"}

    print("Exporting to %s (input %s)" % (args.output, "[1,1,H,W] dynamic" if dynamic else "[1,1,%d,%d]" % (height, width)))
    with torch.no_grad():
        torch.onnx.export(
            model,
            (image,),
            args.output,
            input_names=["image"],
            output_names=["keypoints", "scores", "descriptors"],
            dynamic_axes=dynamic_axes,
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
    # A size that is not a multiple of 32 when dynamic, to exercise the padding.
    # A blocky synthetic image (upsampled low resolution noise) has corners and
    # edges everywhere, uniform noise would give almost no keypoint.
    check_h = height + (7 if dynamic else 0)
    check_w = width + (13 if dynamic else 0)
    check_image = F.interpolate(torch.rand(1, 1, check_h // 24, check_w // 24), size=(check_h, check_w), mode="nearest")
    with torch.no_grad():
        ref = [t.numpy() for t in model(check_image)]
    out = session.run(None, {"image": check_image.numpy()})
    # The keypoint order can differ (unstable sort of equal scores): compare
    # them by position.
    def by_position(kpts, scores, desc):
        order = np.lexsort((kpts[0, :, 0], kpts[0, :, 1]))
        return [kpts[:, order], scores[:, order], desc[:, order]]
    if ref[0].shape == out[0].shape:
        ref = by_position(*ref)
        out = by_position(*out)
    for name, r, o in zip(["keypoints", "scores", "descriptors"], ref, out):
        if r.shape != o.shape:
            raise RuntimeError("%s: torch %s vs onnx %s" % (name, r.shape, o.shape))
        diff = np.abs(r - o).max() if r.size else 0.0
        # keypoints are exact, scores and descriptors within float drift
        tol = 1e-3 if name == "keypoints" else 1e-2 * max(1.0, float(np.abs(r).max()) if r.size else 1.0)
        print("  %s %s max difference torch/onnx = %g (tolerance %g)" % (name, o.shape, diff, tol))
        if diff > tol:
            raise RuntimeError("%s differs between torch and onnxruntime" % name)
    print("Verified: %d keypoints on a %dx%d random image" % (out[0].shape[1], check_image.shape[3], check_image.shape[2]))


if __name__ == "__main__":
    main()
