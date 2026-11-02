#! /usr/bin/env python3
#
# Export ALIKED (https://github.com/Shiaoming/ALIKED, BSD-3-Clause; implementation
# of the lightglue package) to an ONNX model following RTAB-Map's OnnxDetector
# interface (see README.md).
#
# Requirements: pip install lightglue onnx onnxscript (onnxruntime to verify the export),
# or a clone of https://github.com/cvg/LightGlue given with --lightglue-dir (its
# aliked.py and utils.py need torch, torchvision, kornia and opencv). The
# weights are downloaded by lightglue.
#
# The model takes a grayscale image "image" float32 [1,1,H,W] in [0,1] (H and W
# dynamic unless --height/--width are given) and returns "keypoints" float32
# [1,N,2] (sub-pixel x,y), "scores" float32 [1,N] and "descriptors" float32
# [1,N,D] (L2-normalized, D=128, 64 for aliked-t16). The padding to a multiple
# of 32, the NMS, the detection threshold and the optional top-k are inside the
# model. Pair it with aliked_lightglue.onnx (export_lightglue_onnx.py --features
# aliked).
#
# Like lightglue.ALIKED, the keypoints are the local maxima above --threshold
# (at most --max-keypoints of them, by score), or with --threshold 0 the
# --max-keypoints strongest ones: with --height/--width too, every shape is
# then static (best for TensorRT). The scores are the score map interpolated
# at the sub-pixel keypoints.
#
# Same results as lightglue.ALIKED, except that the operations that don't
# export are rewritten with standard ONNX operators and the image size as a
# tensor: the deformable convolutions (torchvision's deform_conv2d, as
# grid_sample + 1x1 convolution), the keypoint detection (DKD, without the
# Python loops) and the descriptor head (SDDH, without einsum). The
# verification compares the exported model with lightglue.ALIKED.
#
# Usage:
#   python3 export_aliked_onnx.py --output aliked.onnx
#   python3 export_aliked_onnx.py --model aliked-t16 --output aliked-t16.onnx
#   python3 export_aliked_onnx.py --threshold 0 --max-keypoints 1024 --width 640 --height 480 --output aliked_640x480.onnx

import argparse
import importlib
import os
import sys
import types

import numpy as np
import torch
import torch.nn.functional as F


def import_aliked_module(lightglue_dir):
    """Imports lightglue.aliked, without the package's __init__ (and the other
    extractors) when a clone of the repository is given."""
    if lightglue_dir:
        package_dir = os.path.join(os.path.abspath(lightglue_dir), "lightglue")
        if not os.path.exists(os.path.join(package_dir, "aliked.py")):
            raise FileNotFoundError("%s/aliked.py not found, --lightglue-dir should be a clone of https://github.com/cvg/LightGlue" % package_dir)
        package = types.ModuleType("lightglue")
        package.__path__ = [package_dir]
        sys.modules["lightglue"] = package
    return importlib.import_module("lightglue.aliked")


def _image_size(x):
    """(h, w) of a [B,C,H,W] tensor as an int64 tensor, for the index
    arithmetic: the ONNX translation of the exporter doesn't take the
    symbolic sizes of x.shape there."""
    return torch.tensor(x.shape[2:])


def _deform_conv_forward(self, x):
    # DeformableConv2d.forward() (stride 1, dilation 1, no mask in ALIKED) with
    # torchvision.ops.deform_conv2d replaced by grid_sample + 1x1 convolution.
    assert not self.mask
    b, c, h, w = x.shape
    hw = _image_size(x).to(x.dtype)
    max_offset = hw.max() / 4.0
    offset = self.offset_conv(x).clamp(-max_offset, max_offset)
    bias = self.regular_conv.bias
    kh, kw = self.regular_conv.kernel_size
    pad = self.padding[0] if isinstance(self.padding, (tuple, list)) else self.padding
    ys, xs = torch.meshgrid(torch.arange(h, dtype=x.dtype), torch.arange(w, dtype=x.dtype), indexing="ij")
    ki, kj = torch.meshgrid(torch.arange(kh, dtype=x.dtype) - pad, torch.arange(kw, dtype=x.dtype) - pad, indexing="ij")
    off = offset.view(b, kh * kw, 2, h, w)  # (dy, dx) for each kernel position
    py = ys[None, None] + ki.reshape(1, -1, 1, 1) + off[:, :, 0]
    px = xs[None, None] + kj.reshape(1, -1, 1, 1) + off[:, :, 1]
    grid = torch.stack((2 * px / (hw[1] - 1) - 1, 2 * py / (hw[0] - 1) - 1), -1)
    sampled = F.grid_sample(x, grid.reshape(b, kh * kw * h, w, 2), mode="bilinear", padding_mode="zeros", align_corners=True)
    sampled = sampled.view(b, c, kh * kw, h, w).reshape(b, c * kh * kw, h, w)
    out = F.conv2d(sampled, self.weight_1x1)
    return out if bias is None else out + bias.view(1, -1, 1, 1)


class AlikedOnnx(torch.nn.Module):
    def __init__(self, aliked_module, aliked, threshold, max_keypoints):
        super().__init__()
        self.aliked = aliked
        self.simple_nms = aliked_module.simple_nms
        self.threshold = threshold
        self.max_keypoints = max_keypoints
        for module in aliked.modules():
            if isinstance(module, aliked_module.DeformableConv2d):
                # the kernel as a 1x1 convolution on the sampled positions,
                # computed once here
                weight = module.regular_conv.weight.detach()
                module.register_buffer("weight_1x1", weight.reshape(weight.shape[0], -1, 1, 1).clone())
                module.forward = types.MethodType(_deform_conv_forward, module)
        sddh = aliked.desc_head
        assert not sddh.mask and not sddh.conv2D
        dkd = aliked.dkd
        self.radius = dkd.radius
        self.temperature = dkd.temperature
        self.register_buffer("hw_grid", dkd.hw_grid.clone())  # (kernel**2) x 2, (x, y) offsets

    def dense_map(self, image):
        # ALIKED.extract_dense_map() with the padding to a multiple of 32
        # (replicated borders, split on both sides like InputPadder) computed
        # from the image size.
        a = self.aliked
        h, w = image.shape[2], image.shape[3]
        pad_h = (32 - h % 32) % 32
        pad_w = (32 - w % 32) % 32
        top = pad_h // 2
        left = pad_w // 2
        image = F.pad(image, (left, pad_w - left, top, pad_h - top), mode="replicate")
        x1 = a.block1(image)
        x2 = a.block2(a.pool2(x1))
        x3 = a.block3(a.pool4(x2))
        x4 = a.block4(a.pool4(x3))
        x1 = a.gate(a.conv1(x1))
        x2 = a.gate(a.conv2(x2))
        x3 = a.gate(a.conv3(x3))
        x4 = a.gate(a.conv4(x4))
        x1234 = torch.cat([x1, a.upsample2(x2), a.upsample8(x3), a.upsample32(x4)], dim=1)
        score_map = torch.sigmoid(a.score_head(x1234))
        feature_map = F.normalize(x1234, p=2, dim=1)
        return feature_map[:, :, top:top + h, left:left + w], score_map[:, :, top:top + h, left:left + w]

    def describe(self, feature_map, keypoints):
        # SDDH.forward() for one image, keypoints in pixels: patches gathered
        # with flat indices, the einsum as a matrix product.
        sddh = self.aliked.desc_head
        _, c, h, w = feature_map.shape
        hw = _image_size(feature_map)
        wh = hw.flip(0).to(keypoints.dtype) - 1
        max_offset = hw.max().to(keypoints.dtype) / 4.0
        # -1 for the number of keypoints n: a 0 in an ONNX Reshape copies the input dimension
        k = sddh.kernel_size
        p = sddh.n_pos
        flat = feature_map[0].reshape(c, -1)
        if k > 1:
            # get_patches(): the k x k patch whose corner is around the keypoint
            # pixel (truncated first, like SDDH), clamped in the image
            corner = (keypoints.long() - k / 2 + 1).long()
            cx = torch.minimum(corner[:, 0].clamp(min=0), hw[1] - 1 - k)
            cy = torch.minimum(corner[:, 1].clamp(min=0), hw[0] - 1 - k)
            r = torch.arange(k)
            idx = (cy[:, None, None] + r[None, :, None]) * hw[1] + (cx[:, None, None] + r[None, None, :])
            patch = flat[:, idx.reshape(-1)].reshape(c, -1, k, k).permute(1, 0, 2, 3)
        else:
            kl = keypoints.long()
            patch = flat[:, kl[:, 1] * hw[1] + kl[:, 0]].permute(1, 0).reshape(-1, c, 1, 1)
        offset = sddh.offset_conv(patch).clamp(-max_offset, max_offset)  # [n, 2*p, 1, 1]
        offset = offset[:, :, 0, 0].reshape(-1, 2, p).permute(0, 2, 1)  # [n, p, 2]
        pos = 2.0 * (keypoints[:, None, :] + offset) / wh - 1
        features = F.grid_sample(feature_map, pos.reshape(1, -1, 1, 2), mode="bilinear", align_corners=True)
        features = features.reshape(c, -1, p, 1).permute(1, 0, 2, 3)  # [n, c, p, 1]
        features = torch.selu(sddh.sf_conv(features))[..., 0]  # [n, c, p]
        # einsum("ncp,pcd->nd", features, agg_weights)
        descriptors = features.permute(0, 2, 1).reshape(-1, p * c) @ sddh.agg_weights.reshape(p * c, -1)
        return F.normalize(descriptors, p=2.0, dim=1)

    def forward(self, image):
        feature_map, score_map = self.dense_map(image.expand(-1, 3, -1, -1))
        _, _, h, w = score_map.shape
        hw = _image_size(score_map)
        r = self.radius
        # DKD: NMS, border removal, threshold or top-k, sub-pixel refinement
        ys = torch.arange(h)
        xs = torch.arange(w)
        inside = ((ys >= r) & (ys < hw[0] - r))[:, None] & ((xs >= r) & (xs < hw[1] - r))[None, :]
        nms_scores = (self.simple_nms(score_map, r)[0, 0] * inside).reshape(-1)
        if self.threshold > 0:
            idx = torch.nonzero(nms_scores > self.threshold)[:, 0]
            if self.max_keypoints > 0:
                order = torch.sort(nms_scores[idx], descending=True)[1]
                idx = idx[order][: self.max_keypoints]
        else:
            idx = torch.topk(nms_scores, self.max_keypoints).indices
        x_i = (idx % hw[1]).to(score_map.dtype)
        y_i = torch.div(idx, hw[1], rounding_mode="trunc").to(score_map.dtype)
        # local (2r+1)^2 score patches around each keypoint, zero padded like nn.Unfold
        padded = F.pad(score_map, (r, r, r, r)).reshape(-1)
        wp = hw[1] + 2 * r
        dxy = self.hw_grid.to(torch.int64)  # (x, y) offsets in [-r, r]
        patch_idx = (idx[:, None] // hw[1] + r + dxy[None, :, 1]) * wp + (idx[:, None] % hw[1] + r + dxy[None, :, 0])
        patch_scores = padded[patch_idx.reshape(-1)].reshape(-1, dxy.shape[0])
        max_v = patch_scores.max(dim=1).values[:, None]
        x_exp = ((patch_scores - max_v) / self.temperature).exp()
        xy_residual = x_exp @ self.hw_grid / x_exp.sum(dim=1)[:, None]
        keypoints = torch.stack((x_i, y_i), 1) + xy_residual
        # score map interpolated at the keypoints, like DKD
        wh = hw.flip(0).to(score_map.dtype) - 1
        grid = (keypoints / wh * 2 - 1).reshape(1, 1, -1, 2)
        scores = F.grid_sample(score_map, grid, mode="bilinear", align_corners=True)[0, 0, 0]
        descriptors = self.describe(feature_map, keypoints)
        # the descriptor size declared in the model
        dim = self.aliked.desc_head.sf_conv.out_channels
        return keypoints[None], scores[None], descriptors.reshape(1, -1, dim)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="aliked-n16", help="aliked-t16, aliked-n16 (default), aliked-n16rot, aliked-n32")
    parser.add_argument("--lightglue-dir", default="", help="Clone of https://github.com/cvg/LightGlue, when the package is not installed")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--threshold", type=float, default=0.2, help="Detection threshold (default 0.2, ALIKED's; 0: the --max-keypoints strongest keypoints)")
    parser.add_argument("--max-keypoints", type=int, default=0, help="Keep at most this many keypoints, by score (default 0: all)")
    parser.add_argument("--height", type=int, default=0, help="Fixed input height (default 0: dynamic)")
    parser.add_argument("--width", type=int, default=0, help="Fixed input width (default 0: dynamic)")
    parser.add_argument("--opset", type=int, default=18, help="ONNX opset version (default 18)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()
    if args.threshold <= 0 and args.max_keypoints <= 0:
        parser.error("--threshold 0 needs --max-keypoints")

    aliked_module = import_aliked_module(args.lightglue_dir)
    print("Loading lightglue's ALIKED '%s'" % args.model)
    aliked = aliked_module.ALIKED(model_name=args.model).eval()
    model = AlikedOnnx(aliked_module, aliked, args.threshold, args.max_keypoints).eval()

    height = args.height if args.height > 0 else 480
    width = args.width if args.width > 0 else 640
    dynamic = args.height <= 0 or args.width <= 0
    torch.manual_seed(0)
    image = torch.rand(1, 1, height, width)

    # torch.export infers the dynamic number of keypoints, only the image size is declared
    dynamic_shapes = ({2: torch.export.Dim("height"), 3: torch.export.Dim("width")},) if dynamic else None

    print("Exporting to %s (input %s)" % (args.output, "[1,1,H,W] dynamic" if dynamic else "[1,1,%d,%d]" % (height, width)))
    with torch.no_grad():
        torch.onnx.export(
            model,
            (image,),
            args.output,
            input_names=["image"],
            output_names=["keypoints", "scores", "descriptors"],
            dynamic_shapes=dynamic_shapes,
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
    # References: the PyTorch model exported, and the original lightglue.ALIKED
    # (torchvision's deform_conv2d, its own DKD and SDDH) with the same
    # detection settings, so the comparison covers the rewritten operations
    # too. When dynamic, sizes that are not multiples of 32 (to exercise the
    # padding) and a portrait one. A blocky synthetic image (upsampled low
    # resolution noise) has corners and edges everywhere.
    reference = aliked_module.ALIKED(model_name=args.model, detection_threshold=args.threshold,
                                     max_num_keypoints=args.max_keypoints if args.max_keypoints > 0 else -1).eval()
    sizes = [(height + 7, width + 13), (width + 5, height + 3)] if dynamic else [(height, width)]

    # The keypoint order can differ, and float drift can add or remove the
    # local maxima right at the threshold or tied in the NMS: each keypoint of
    # onnxruntime is paired with the closest reference keypoint, 99% of them
    # must be found, with the same score and descriptor.
    def compare(label, ref, out):
        ref_kpts, out_kpts = ref[0][0], out[0][0]
        if abs(len(ref_kpts) - len(out_kpts)) > len(ref_kpts) // 100:
            raise RuntimeError("keypoints: %s %d vs onnx %d" % (label, len(ref_kpts), len(out_kpts)))
        if len(out_kpts) == 0:
            return
        dist = np.linalg.norm(out_kpts[:, None, :] - ref_kpts[None, :, :], axis=2)
        closest = dist.argmin(axis=1)
        paired = dist[np.arange(len(out_kpts)), closest] < 1e-2  # sub-pixel keypoints within float drift
        print("  keypoints %s: %d/%d of onnx found in %s (%d)" % (out[0].shape, paired.sum(), len(out_kpts), label, len(ref_kpts)))
        if paired.sum() < len(out_kpts) * 99 // 100:
            raise RuntimeError("keypoints differ between %s and onnxruntime" % label)
        for name, r, o in [("scores", ref[1][0], out[1][0]), ("descriptors", ref[2][0], out[2][0])]:
            diff = np.abs(r[closest[paired]] - o[paired]).max()
            tol = 1e-2
            print("  %s %s max difference %s/onnx = %g (tolerance %g)" % (name, o.shape, label, diff, tol))
            if diff > tol:
                raise RuntimeError("%s differs between %s and onnxruntime" % (name, label))

    for check_h, check_w in sizes:
        check_image = F.interpolate(torch.rand(1, 1, check_h // 24, check_w // 24), size=(check_h, check_w), mode="nearest")
        with torch.no_grad():
            exported = [t.numpy() for t in model(check_image)]
            pred = reference({"image": check_image})
        original = [pred["keypoints"].numpy(), pred["keypoint_scores"].numpy(), pred["descriptors"].numpy()]
        out = session.run(None, {"image": check_image.numpy()})
        compare("torch", exported, out)
        compare("lightglue.ALIKED", original, out)
        print("Verified: %d keypoints on a %dx%d random image" % (out[0].shape[1], check_w, check_h))


if __name__ == "__main__":
    main()
