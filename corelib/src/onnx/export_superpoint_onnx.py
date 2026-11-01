#!/usr/bin/env python3
#
# Exports SuperPoint to an ONNX model following RTAB-Map's OnnxDetector
# interface (see README.md), from either:
#  - rpautrat/SuperPoint (MIT): https://github.com/rpautrat/SuperPoint
#      --model-dir <clone> --weights <clone>/weights/superpoint_v6_from_tf.pth
#  - magicleap/SuperPointPretrainedNetwork (non-commercial license):
#      https://github.com/magicleap/SuperPointPretrainedNetwork
#      --model-dir <clone> --weights <clone>/superpoint_v1.pth
#
# The model takes a grayscale image "image" float32 [1,1,H,W] in [0,1] (H and W
# dynamic unless --height/--width are given) and returns "keypoints" float32
# [1,N,2] (x,y), "scores" float32 [1,N] and "descriptors" float32 [1,N,256].
# NMS, detection threshold, border removal and the optional top-k are inside
# the model.
#
# Note that LightGlue's SuperPoint checkpoint (superpoint_lightglue.onnx) was
# trained on the descriptors of the magicleap weights: the rpautrat weights
# give different descriptors and should not be paired with it (see README.md).
#
# Usage:
#   python3 export_superpoint_onnx.py --model-dir ~/SuperPoint --weights ~/SuperPoint/weights/superpoint_v6_from_tf.pth --output superpoint.onnx
#
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn


def batched_nms(scores, nms_radius):
    """Fast NMS by max pooling (from rpautrat/SuperPoint, same as LightGlue's SuperPoint)."""
    assert nms_radius >= 0

    def max_pool(x):
        return F.max_pool2d(x, kernel_size=nms_radius * 2 + 1, stride=1, padding=nms_radius)

    zeros = torch.zeros_like(scores)
    max_mask = scores == max_pool(scores)
    for _ in range(2):
        supp_mask = max_pool(max_mask.float()) > 0
        supp_scores = torch.where(supp_mask, zeros, scores)
        new_max_mask = supp_scores == max_pool(supp_scores)
        max_mask = max_mask | (new_max_mask & (~supp_mask))
    return torch.where(max_mask, scores, zeros)


def sample_descriptors(keypoints, descriptors, s=8):
    """Bilinear interpolation of the dense descriptors at the keypoints (from rpautrat/SuperPoint)."""
    b, c, h, w = descriptors.shape
    # The size as a tensor built from the traced shape, so that the export
    # stays dynamic (new_tensor([w, h]) would bake the export size in).
    one = keypoints.new_tensor(1.0)
    keypoints = (keypoints + 0.5) / (torch.stack([one * w, one * h]) * s)
    keypoints = keypoints * 2 - 1  # normalize to (-1, 1)
    descriptors = F.grid_sample(descriptors, keypoints.view(b, 1, -1, 2), mode="bilinear", align_corners=False)
    descriptors = F.normalize(descriptors.reshape(b, c, -1), p=2, dim=1)
    return descriptors


class SuperPointOnnx(nn.Module):
    """Wraps the network with an export-friendly post-processing."""

    def __init__(self, net, flavor, nms_radius, threshold, remove_borders, max_keypoints):
        super().__init__()
        self.net = net
        self.flavor = flavor
        self.nms_radius = nms_radius
        self.threshold = threshold
        self.remove_borders = remove_borders
        self.max_keypoints = max_keypoints
        self.stride = 8
        self.descriptor_dim = 256

    def dense(self, image):
        """Returns the detector logits [1,65,H/8,W/8] and the normalized dense descriptors [1,256,H/8,W/8]."""
        if self.flavor == "rpautrat":
            features = self.net.backbone(image)
            logits = self.net.detector(features)
            descriptors = F.normalize(self.net.descriptor(features), p=2, dim=1)
        else:  # magicleap: forward() returns semi (logits) and the normalized descriptors
            logits, descriptors = self.net(image)
        return logits, descriptors

    def forward(self, image):
        logits, dense = self.dense(image)

        # Decode the detection scores: softmax, drop the dustbin, depth to space
        scores = F.softmax(logits, 1)[:, :-1]
        b, _, h, w = scores.shape
        s = self.stride
        scores = scores.permute(0, 2, 3, 1).reshape(b, h, w, s, s)
        scores = scores.permute(0, 1, 3, 2, 4).reshape(b, 1, h * s, w * s)
        scores = batched_nms(scores, self.nms_radius)[:, 0]  # max pooling on a 4-D tensor for the export

        # Discard keypoints near the image borders (a mask rather than in-place
        # slices, which don't export)
        if self.remove_borders > 0:
            pad = self.remove_borders
            height = scores.shape[1]
            width = scores.shape[2]
            ys = torch.arange(height, device=scores.device)[:, None]
            xs = torch.arange(width, device=scores.device)[None, :]
            inside = (ys >= pad) & (ys < height - pad) & (xs >= pad) & (xs < width - pad)
            scores = torch.where(inside[None], scores, torch.zeros_like(scores))

        # Extract the keypoints (batch of 1)
        scores = scores[0]
        keep = scores > self.threshold
        idxs = torch.nonzero(keep)  # [N,2] (y,x)
        keypoint_scores = scores[keep]
        if self.max_keypoints > 0:
            keypoint_scores, order = torch.sort(keypoint_scores, descending=True)
            keypoint_scores = keypoint_scores[: self.max_keypoints]
            idxs = idxs[order][: self.max_keypoints]
        keypoints = idxs.flip(1).float()  # (y,x) -> (x,y)

        descriptors = sample_descriptors(keypoints[None], dense, s)  # [1,256,N]
        # reshape with the known dimension so that the model declares [1,N,256]
        descriptors = descriptors.transpose(1, 2).reshape(1, -1, self.descriptor_dim)
        return keypoints[None], keypoint_scores[None], descriptors


def load_network(model_dir, weights, flavor):
    sys.path.insert(0, os.path.abspath(model_dir))
    if flavor == "rpautrat":
        from superpoint_pytorch import SuperPoint
        net = SuperPoint()
    else:
        from demo_superpoint import SuperPointNet
        net = SuperPointNet()
    state = torch.load(weights, map_location="cpu", weights_only=True)
    net.load_state_dict(state)
    return net.eval()


def detect_flavor(model_dir):
    if os.path.exists(os.path.join(model_dir, "superpoint_pytorch.py")):
        return "rpautrat"
    if os.path.exists(os.path.join(model_dir, "demo_superpoint.py")):
        return "magicleap"
    raise FileNotFoundError("Neither superpoint_pytorch.py (rpautrat/SuperPoint) nor demo_superpoint.py "
                            "(magicleap/SuperPointPretrainedNetwork) found in %s" % model_dir)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model-dir", required=True, help="Directory holding superpoint_pytorch.py (rpautrat) or demo_superpoint.py (magicleap)")
    parser.add_argument("--weights", required=True, help="Weights file (*.pth)")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--nms-radius", type=int, default=4, help="Non-maximum suppression radius in pixels (default 4)")
    parser.add_argument("--threshold", type=float, default=0.005, help="Detection threshold (default 0.005)")
    parser.add_argument("--remove-borders", type=int, default=4, help="Discard keypoints closer than this to the image borders (default 4)")
    parser.add_argument("--max-keypoints", type=int, default=0, help="Keep at most this many keypoints, by score (default 0: all)")
    parser.add_argument("--height", type=int, default=0, help="Fixed input height (default 0: dynamic)")
    parser.add_argument("--width", type=int, default=0, help="Fixed input width (default 0: dynamic)")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    flavor = detect_flavor(args.model_dir)
    print("Loading %s SuperPoint from %s" % (flavor, args.weights))
    net = load_network(args.model_dir, args.weights, flavor)
    model = SuperPointOnnx(net, flavor, args.nms_radius, args.threshold, args.remove_borders, args.max_keypoints).eval()

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
    # another size than the export one when dynamic, to be sure the shapes are not baked in
    # A blocky synthetic image (upsampled low resolution noise) has corners and
    # edges everywhere, uniform noise would give almost no keypoint.
    check_h = height + (64 if dynamic else 0)
    check_w = width + (32 if dynamic else 0)
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
