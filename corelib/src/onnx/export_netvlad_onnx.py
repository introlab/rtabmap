#!/usr/bin/env python3
#
# Exports NetVLAD (VGG16 + NetVLAD + PCA whitening, the Pittsburgh 30k weights
# of the original MATLAB release converted by netvlad_tf_open and hosted by
# hloc) to an ONNX model following RTAB-Map's OnnxDescriptor interface (see
# README.md). The network definition and the parsing of the MATLAB weights
# follow hloc's extractor (https://github.com/cvg/Hierarchical-Localization,
# Apache-2.0); torch, torchvision and scipy are needed.
#
# The model takes an RGB image "image" float32 [1,3,H,W] in [0,1] (H and W
# dynamic unless --height/--width are given; the mean subtraction is inside)
# and returns "descriptor" float32 [1,D], L2-normalized, with D = --dim
# (default 4096). The PCA components are ordered, so --dim keeps the leading
# ones: the whitening matrix is 32768 x D floats on top of the 59 MB VGG16
# backbone, i.e. 596 MB at 4096, 193 MB at 1024, 126 MB at 512.
#
# Usage:
#   python3 export_netvlad_onnx.py --download --weights Pitts30K_struct.mat --output netvlad.onnx --dim 1024
#
import argparse
import os
import sys

import numpy as np
import torch
import torch.nn.functional as F
import torchvision.models as models
from torch import nn

# The VGG16 + NetVLAD + whitening models of https://www.di.ens.fr/willow/research/netvlad/
# (vd16_pitts30k_conv5_3_vlad_preL2_intra_white, the authors' best model, and
# vd16_tokyoTM_conv5_3_vlad_preL2_intra_white), converted from MATLAB class
# objects to plain structs and hosted by hloc. Only these converted files can
# be read here: the project page's own .mat files need MATLAB to be converted.
CHECKPOINT_URLS = {
    "Pitts30K": "https://cvg-data.inf.ethz.ch/hloc/netvlad/Pitts30K_struct.mat",
    "TokyoTM": "https://cvg-data.inf.ethz.ch/hloc/netvlad/TokyoTM_struct.mat",
}


class NetVLADLayer(nn.Module):
    def __init__(self, input_dim=512, K=64):
        super().__init__()
        self.score_proj = nn.Conv1d(input_dim, K, kernel_size=1, bias=False)
        self.centers = nn.Parameter(torch.empty([input_dim, K]))
        self.output_dim = input_dim * K

    def forward(self, x):  # [1,C,N]
        scores = F.softmax(self.score_proj(x), dim=1)  # [1,K,N]
        diff = x.unsqueeze(2) - self.centers.unsqueeze(0).unsqueeze(3)  # [1,C,K,N]
        desc = (scores.unsqueeze(1) * diff).sum(dim=3)  # [1,C,K]
        desc = F.normalize(desc, dim=1)  # intra-normalization
        desc = desc.reshape(1, self.output_dim)
        return F.normalize(desc, dim=1)


class NetVLAD(nn.Module):
    """VGG16 up to conv5_3 + NetVLAD + whitening, weights from the MATLAB struct."""

    def __init__(self, mat_path, dim):
        super().__init__()
        from scipy.io import loadmat
        backbone = list(models.vgg16().children())[0]
        self.backbone = nn.Sequential(*list(backbone.children())[:-2])  # no last ReLU + MaxPool
        self.netvlad = NetVLADLayer()
        mat = loadmat(mat_path, struct_as_record=False, squeeze_me=True)
        layers = mat["net"].layers
        for layer, mat_layer in zip(self.backbone.children(), layers):
            if isinstance(layer, nn.Conv2d):
                w = torch.tensor(mat_layer.weights[0]).float().permute([3, 2, 0, 1])
                b = torch.tensor(mat_layer.weights[1]).float()
                layer.weight = nn.Parameter(w)
                layer.bias = nn.Parameter(b)
        score_w = torch.tensor(layers[30].weights[0]).float().permute([1, 0]).unsqueeze(-1)
        center_w = -torch.tensor(layers[30].weights[1]).float()  # stored negated in the MATLAB code
        self.netvlad.score_proj.weight = nn.Parameter(score_w)
        self.netvlad.centers = nn.Parameter(center_w)
        w = torch.tensor(layers[33].weights[0]).float().squeeze().permute([1, 0])  # [4096, 32768]
        b = torch.tensor(np.asarray(layers[33].weights[1]).squeeze()).float()
        full_dim = w.shape[0]
        if dim <= 0 or dim > full_dim:
            dim = full_dim
        self.whiten = nn.Linear(self.netvlad.output_dim, dim)
        self.whiten.weight = nn.Parameter(w[:dim].contiguous())
        self.whiten.bias = nn.Parameter(b[:dim].contiguous())
        self.dim = dim
        mean = np.asarray(mat["net"].meta.normalization.averageImage, dtype=np.float32).reshape(-1)[:3]
        self.register_buffer("mean", torch.tensor(mean).view(1, 3, 1, 1))

    def forward(self, image):  # [1,3,H,W] in [0,1]
        x = torch.clamp(image * 255.0, 0.0, 255.0) - self.mean
        x = self.backbone(x)  # [1,512,h,w]
        b, c = x.shape[0], x.shape[1]
        x = x.reshape(b, c, -1)
        x = F.normalize(x, dim=1)
        desc = self.netvlad(x)
        desc = self.whiten(desc)
        return F.normalize(desc, dim=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", required=True, help="MATLAB weights file (Pitts30K_struct.mat or TokyoTM_struct.mat)")
    parser.add_argument("--model", choices=sorted(CHECKPOINT_URLS.keys()), default="Pitts30K",
                        help="Which weights --download fetches: " + ", ".join("%s (%s)" % (k, v) for k, v in sorted(CHECKPOINT_URLS.items())) + " (default Pitts30K)")
    parser.add_argument("--download", action="store_true", help="Download the --model weights (555 MB) to --weights when the file is missing")
    parser.add_argument("--output", required=True, help="Output ONNX file")
    parser.add_argument("--dim", type=int, default=4096, help="Descriptor dimension, the leading PCA components (default 4096, the full whitening)")
    parser.add_argument("--height", type=int, default=0, help="Fixed input height (default 0: dynamic)")
    parser.add_argument("--width", type=int, default=0, help="Fixed input width (default 0: dynamic)")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version (default 17)")
    parser.add_argument("--no-check", action="store_true", help="Don't verify the exported model with onnxruntime")
    args = parser.parse_args()

    if not os.path.exists(args.weights):
        if not args.download:
            raise FileNotFoundError("%s not found, add --download to fetch it" % args.weights)
        print("Downloading %s to %s" % (CHECKPOINT_URLS[args.model], args.weights))
        torch.hub.download_url_to_file(CHECKPOINT_URLS[args.model], args.weights)

    print("Loading NetVLAD from %s" % args.weights)
    model = NetVLAD(args.weights, args.dim).eval()
    print("Descriptor dimension %d" % model.dim)

    height = args.height if args.height > 0 else 480
    width = args.width if args.width > 0 else 640
    dynamic = args.height <= 0 or args.width <= 0
    torch.manual_seed(0)
    image = torch.rand(1, 3, height, width)
    dynamic_axes = {}
    if dynamic:
        dynamic_axes["image"] = {2: "height", 3: "width"}

    print("Exporting to %s (input %s)" % (args.output, "[1,3,H,W] dynamic" if dynamic else "[1,3,%d,%d]" % (height, width)))
    with torch.no_grad():
        torch.onnx.export(
            model,
            (image,),
            args.output,
            input_names=["image"],
            output_names=["descriptor"],
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
    check_h = height + (32 if dynamic else 0)
    check_w = width + (48 if dynamic else 0)
    check_image = F.interpolate(torch.rand(1, 3, check_h // 16, check_w // 16), size=(check_h, check_w), mode="bilinear", align_corners=False)
    with torch.no_grad():
        ref = model(check_image).numpy()
    out = session.run(None, {"image": check_image.numpy()})[0]
    if ref.shape != out.shape:
        raise RuntimeError("descriptor: torch %s vs onnx %s" % (ref.shape, out.shape))
    diff = float(np.abs(ref - out).max())
    norm = float(np.linalg.norm(out))
    print("Verified: descriptor %s, max difference torch/onnx = %g, norm = %g" % (out.shape, diff, norm))
    if diff > 1e-3 or abs(norm - 1.0) > 1e-3:
        raise RuntimeError("The exported model differs from the PyTorch one")


if __name__ == "__main__":
    main()
