#! /usr/bin/env python3
#
# Export ALIKED (https://github.com/Shiaoming/ALIKED, BSD-3-Clause; implementation
# of the lightglue package) to ONNX for RTAB-Map's ONNX Runtime ALIKED detector.
#
# Requirements: pip install lightglue onnx onnxscript
#
#   python3 rtabmap_export_aliked_onnx.py --width 640 --height 400            # aliked-n16_640x400.onnx
#   python3 rtabmap_export_aliked_onnx.py --model aliked-t16 --width 640 --height 480 --top_k 2048
#
# The model has a fixed input size (the camera's) and returns a fixed number of
# keypoint candidates, which keeps every shape static (best for TensorRT):
#   image       [1,1,H,W] float32  grayscale image in [0,1]
# Outputs, sorted by decreasing score:
#   keypoints   [1,K,2]   float32  sub-pixel (x,y) in pixels
#   scores      [1,K]     float32  detection score (keep those > detection threshold, ALIKED's default is 0.2)
#   descriptors [1,K,D]   float32  L2-normalized descriptors (D=128 for aliked-n16/n32, 64 for aliked-t16)
#
# Same results as lightglue.ALIKED, except that two of its operations are
# rewritten with standard ONNX operators: the deformable convolutions
# (torchvision's deform_conv2d, as grid_sample + 1x1 convolution) and the
# keypoint detection (DKD, with fixed top-k instead of a variable-size threshold mask).

import argparse
import types

import torch
import torch.nn.functional as F
from lightglue import ALIKED
from lightglue.aliked import DeformableConv2d, simple_nms


def _deform_conv_forward(self, x):
    # DeformableConv2d.forward() (stride 1, dilation 1, no mask in ALIKED) with
    # torchvision.ops.deform_conv2d replaced by grid_sample + 1x1 convolution.
    assert not self.mask
    b, c, h, w = x.shape
    max_offset = max(h, w) / 4.0
    offset = self.offset_conv(x).clamp(-max_offset, max_offset)
    weight, bias = self.regular_conv.weight, self.regular_conv.bias
    o, _, kh, kw = weight.shape
    pad = self.padding[0] if isinstance(self.padding, (tuple, list)) else self.padding
    ys, xs = torch.meshgrid(torch.arange(h, dtype=x.dtype), torch.arange(w, dtype=x.dtype), indexing="ij")
    ki, kj = torch.meshgrid(torch.arange(kh, dtype=x.dtype) - pad, torch.arange(kw, dtype=x.dtype) - pad, indexing="ij")
    off = offset.view(b, kh * kw, 2, h, w)  # (dy, dx) for each kernel position
    py = ys[None, None] + ki.reshape(1, -1, 1, 1) + off[:, :, 0]
    px = xs[None, None] + kj.reshape(1, -1, 1, 1) + off[:, :, 1]
    grid = torch.stack((2 * px / (w - 1) - 1, 2 * py / (h - 1) - 1), -1)
    sampled = F.grid_sample(x, grid.reshape(b, kh * kw * h, w, 2), mode="bilinear", padding_mode="zeros", align_corners=True)
    sampled = sampled.view(b, c, kh * kw, h, w).reshape(b, c * kh * kw, h, w)
    out = F.conv2d(sampled, weight.reshape(o, c * kh * kw, 1, 1))
    return out if bias is None else out + bias.view(1, -1, 1, 1)


class AlikedOnnx(torch.nn.Module):
    def __init__(self, aliked, width, height, top_k):
        super().__init__()
        self.aliked = aliked
        self.top_k = top_k
        for module in aliked.modules():
            if isinstance(module, DeformableConv2d):
                module.forward = types.MethodType(_deform_conv_forward, module)
        dkd = aliked.dkd
        self.radius = dkd.radius
        self.temperature = dkd.temperature
        self.register_buffer("hw_grid", dkd.hw_grid.clone())  # (kernel**2) x 2, (x, y) offsets
        r = self.radius
        border = torch.zeros(1, 1, height, width)
        border[:, :, r:height - r, r:width - r] = 1
        self.register_buffer("border", border)  # DKD's "remove border"

    def forward(self, image):
        a = self.aliked
        feature_map, score_map = a.extract_dense_map(image.expand(-1, 3, -1, -1))
        _, _, h, w = score_map.shape
        r = self.radius
        # DKD, top-k mode with sub-pixel refinement
        nms_scores = simple_nms(score_map, self.radius) * self.border
        top = torch.topk(nms_scores.view(1, -1), self.top_k)
        idx = top.indices[0]
        x_i = (idx % w).to(score_map.dtype)
        y_i = torch.div(idx, w, rounding_mode="trunc").to(score_map.dtype)
        # local (2r+1)^2 score patches around each candidate, zero padded like nn.Unfold
        padded = F.pad(score_map, (r, r, r, r))
        wp = w + 2 * r
        dxy = self.hw_grid.to(torch.int64)  # (x, y) offsets in [-r, r]
        patch_idx = (idx[:, None] // w + r + dxy[None, :, 1]) * wp + (idx[:, None] % w + r + dxy[None, :, 0])
        patch_scores = padded.view(-1)[patch_idx.view(-1)].view(self.top_k, -1)
        max_v = patch_scores.max(dim=1).values[:, None]
        x_exp = ((patch_scores - max_v) / self.temperature).exp()
        xy_residual = x_exp @ self.hw_grid / x_exp.sum(dim=1)[:, None]
        keypoints_xy = torch.stack((x_i, y_i), 1) + xy_residual
        wh = torch.tensor([w - 1, h - 1], dtype=score_map.dtype)
        keypoints_norm = keypoints_xy / wh * 2 - 1
        descriptors, _ = a.desc_head(feature_map, [keypoints_norm])
        keypoints = wh * (keypoints_norm + 1) / 2.0
        return keypoints[None], top.values, descriptors[0][None]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="aliked-n16", help="aliked-t16, aliked-n16 (default), aliked-n16rot, aliked-n32")
    parser.add_argument("--width", type=int, required=True, help="image width (pixels)")
    parser.add_argument("--height", type=int, required=True, help="image height (pixels)")
    parser.add_argument("--top_k", type=int, default=1024, help="keypoint candidates returned (default 1024)")
    parser.add_argument("-o", "--output", help="output file (default: <model>_<width>x<height>.onnx)")
    parser.add_argument("--opset", type=int, default=18)
    args = parser.parse_args()
    output = args.output or "%s_%dx%d.onnx" % (args.model, args.width, args.height)

    aliked = ALIKED(model_name=args.model, max_num_keypoints=args.top_k).eval()
    model = AlikedOnnx(aliked, args.width, args.height, args.top_k).eval()
    example = torch.rand(1, 1, args.height, args.width)
    with torch.no_grad():
        torch.onnx.export(
            model, (example,), output,
            input_names=["image"], output_names=["keypoints", "scores", "descriptors"],
            opset_version=args.opset, dynamo=True, external_data=False,
        )
    print("Exported %s (%s, %dx%d, %d keypoint candidates, %d-D descriptors)"
          % (output, args.model, args.width, args.height, args.top_k, aliked.desc_head.sf_conv.out_channels))


if __name__ == "__main__":
    main()
