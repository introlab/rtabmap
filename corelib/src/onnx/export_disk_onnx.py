#!/usr/bin/env python3
#
# Exports DISK (https://github.com/cvlab-epfl/disk, Apache-2.0) to an ONNX model
# following RTAB-Map's OnnxDetector interface (see README.md), from kornia's
# implementation and pretrained weights (the ones cvg/LightGlue uses):
#   pip install kornia
#
# The model takes an RGB image "image" float32 [1,3,H,W] in [0,1] (H and W
# dynamic unless --height/--width are given; RTAB-Map replicates its grayscale
# image on the 3 channels) and returns "keypoints" float32 [1,N,2] (x,y),
# "scores" float32 [1,N] and "descriptors" float32 [1,N,128]. The padding to a
# multiple of 16, the NMS, the detection threshold and the optional top-k are
# inside the model. Pair it with disk_lightglue.onnx (LightGlue-ONNX releases,
# or export_lightglue_onnx.py --features disk).
#
# Usage:
#   python3 export_disk_onnx.py --output disk.onnx
#   python3 export_disk_onnx.py --weights epipolar --output disk_epipolar.onnx
#
import argparse
import os

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn


class DiskOnnx(nn.Module):
    """Wraps kornia's DISK with an export-friendly detection (same rules as
    kornia.feature.DISK.forward: zero-padding to a multiple of 16, maps cropped
    back, strict local maxima of the heatmap in a window_size window above the
    threshold, descriptors read at the integer keypoint positions and
    L2-normalized)."""

    def __init__(self, disk, window_size, threshold, max_keypoints):
        super().__init__()
        self.disk = disk
        self.window_size = window_size
        self.threshold = threshold
        self.max_keypoints = max_keypoints

    def forward(self, image):
        h = image.shape[2]
        w = image.shape[3]
        # The U-Net needs sizes divisible by 16: zero-pad bottom/right, like kornia.
        pad_h = (16 - h % 16) % 16
        pad_w = (16 - w % 16) % 16
        x = F.pad(image, (0, pad_w, 0, pad_h))
        heatmaps, descriptors = self.disk.heatmap_and_dense_descriptors(x)
        heatmaps = heatmaps[:, :, :h, :w]
        descriptors = descriptors[:, :, :h, :w]

        heat = heatmaps[0, 0]
        local_max = F.max_pool2d(heatmaps, self.window_size, stride=1, padding=self.window_size // 2)[0, 0]
        keep = (heat == local_max) & (heat > self.threshold)
        idxs = torch.nonzero(keep)  # [N,2] (y,x)
        scores = heat[keep]
        if self.max_keypoints > 0:
            scores, order = torch.sort(scores, descending=True)
            scores = scores[: self.max_keypoints]
            idxs = idxs[order][: self.max_keypoints]
        keypoints = idxs.flip(1).float()  # (x,y)

        # Descriptor at each keypoint pixel: gather on the flattened map.
        flat = descriptors[0].reshape(self.disk.desc_dim, -1)  # [128, H*W]
        desc = flat[:, idxs[:, 0] * w + idxs[:, 1]]  # [128, N]
        desc = F.normalize(desc, p=2, dim=0).transpose(0, 1)
        return keypoints[None], scores[None], desc[None]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="depth", help="kornia pretrained checkpoint, 'depth' (default, the one LightGlue was trained with) or 'epipolar', or a path to a DISK checkpoint (*.pth)")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--window-size", type=int, default=5, help="Non-maximum suppression window in pixels (default 5)")
    parser.add_argument("--threshold", type=float, default=0.0, help="Detection threshold on the heatmap (default 0)")
    parser.add_argument("--max-keypoints", type=int, default=0, help="Keep at most this many keypoints, by score (default 0: all)")
    parser.add_argument("--height", type=int, default=0, help="Fixed input height (default 0: dynamic)")
    parser.add_argument("--width", type=int, default=0, help="Fixed input width (default 0: dynamic)")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    import kornia
    if os.path.exists(args.weights):
        print("Loading DISK checkpoint %s" % args.weights)
        disk = kornia.feature.DISK()
        state = torch.load(args.weights, map_location="cpu", weights_only=True)
        disk.load_state_dict(state["extractor"] if "extractor" in state else state)
    else:
        print("Loading kornia's pretrained DISK '%s'" % args.weights)
        disk = kornia.feature.DISK.from_pretrained(args.weights)
    disk = disk.eval()
    model = DiskOnnx(disk, args.window_size, args.threshold, args.max_keypoints).eval()

    height = args.height if args.height > 0 else 480
    width = args.width if args.width > 0 else 640
    dynamic = args.height <= 0 or args.width <= 0
    torch.manual_seed(0)
    image = torch.rand(1, 3, height, width)

    dynamic_axes = {
        "keypoints": {1: "num_keypoints"},
        "scores": {1: "num_keypoints"},
        "descriptors": {1: "num_keypoints"},
    }
    if dynamic:
        dynamic_axes["image"] = {2: "height", 3: "width"}

    print("Exporting to %s (input %s)" % (args.output, "[1,3,H,W] dynamic" if dynamic else "[1,3,%d,%d]" % (height, width)))
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
    # A size that is not a multiple of 16 when dynamic, to exercise the padding.
    # A blocky synthetic image (upsampled low resolution noise) has corners and
    # edges everywhere, uniform noise would give almost no keypoint.
    check_h = height + (7 if dynamic else 0)
    check_w = width + (13 if dynamic else 0)
    check_image = F.interpolate(torch.rand(1, 3, check_h // 24, check_w // 24), size=(check_h, check_w), mode="nearest")
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
