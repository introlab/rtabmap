#! /usr/bin/env python3
#
# Export LightGlue (https://github.com/cvg/LightGlue, Apache-2.0) to ONNX for
# RTAB-Map's ONNX Runtime matcher (Vis/CorNNType 10, LightGlue/ModelPath).
#
# Requirements: pip install lightglue onnx onnxscript
#
#   python3 rtabmap_export_lightglue_onnx.py                       # superpoint_lightglue.onnx
#   python3 rtabmap_export_lightglue_onnx.py --features disk -o disk_lightglue.onnx
#
# The exported graph always runs all transformer layers: LightGlue's adaptive
# depth (early stop) and width (point pruning) depend on the data, which a
# static ONNX graph cannot express.
#
# Inputs (batch size 1, any number of keypoints M and N):
#   kpts0 [1,M,2] float32  keypoints of image 0, normalized like LightGlue's
#   kpts1 [1,N,2] float32  normalize_keypoints(): (kpt - size/2) / (max(w,h)/2)
#   desc0 [1,M,D] float32  descriptors, L2-normalized
#   desc1 [1,N,D] float32
# Outputs:
#   matches0 [1,M] int64   index in image 1 of the mutual best match, -1 if none
#   mscores0 [1,M] float32 matching score of that match (0 if none)
# RTAB-Map keeps matches with mscores0 > LightGlue/Threshold (LightGlue's
# filter_threshold), which is the same as LightGlue's filter_matches().

import argparse
import types

import torch
from lightglue import LightGlue
from lightglue.lightglue import apply_cached_rotary_emb


def _self_block_forward(self, x, encoding, mask=None):
    # Same as lightglue's SelfBlock.forward(), with q, k and v computed by three
    # linear layers instead of one Wqkv whose output interleaves them: TensorRT
    # (10.16, ONNX Runtime's TensorRT execution provider) computes wrong values
    # for the unflatten(..., 3) + [..., i] slicing of the interleaved output.
    q, k, v = (proj(x).unflatten(-1, (self.num_heads, -1)).transpose(1, 2) for proj in (self.Wq, self.Wk, self.Wv))
    q = apply_cached_rotary_emb(encoding, q)
    k = apply_cached_rotary_emb(encoding, k)
    context = self.inner_attn(q, k, v, mask=mask)
    message = self.out_proj(context.transpose(1, 2).flatten(start_dim=-2))
    return x + self.ffn(torch.cat([x, message], -1))


def split_qkv(lightglue):
    for layer in lightglue.transformers:
        block = layer.self_attn
        weight, bias = block.Wqkv.weight, block.Wqkv.bias  # output features ordered (head, dim, 3)
        for i, name in enumerate(("Wq", "Wk", "Wv")):
            linear = torch.nn.Linear(weight.shape[1], weight.shape[0] // 3, bias=bias is not None)
            with torch.no_grad():
                linear.weight.copy_(weight[i::3])
                if bias is not None:
                    linear.bias.copy_(bias[i::3])
            setattr(block, name, linear)
        block.forward = types.MethodType(_self_block_forward, block)


class LightGlueOnnx(torch.nn.Module):
    def __init__(self, lightglue):
        super().__init__()
        self.lg = lightglue

    def forward(self, kpts0, kpts1, desc0, desc1):
        lg = self.lg
        desc0 = lg.input_proj(desc0)
        desc1 = lg.input_proj(desc1)
        encoding0 = lg.posenc(kpts0)
        encoding1 = lg.posenc(kpts1)
        for i in range(lg.conf.n_layers):
            desc0, desc1 = lg.transformers[i](desc0, desc1, encoding0, encoding1)
        scores, _ = lg.log_assignment[lg.conf.n_layers - 1](desc0, desc1)
        # filter_matches() without the threshold
        max0, max1 = scores[:, :-1, :-1].max(2), scores[:, :-1, :-1].max(1)
        m0, m1 = max0.indices, max1.indices
        indices0 = torch.arange(m0.shape[1], device=m0.device)[None]
        mutual0 = indices0 == m1.gather(1, m0)
        mscores0 = torch.where(mutual0, max0.values.exp(), torch.zeros_like(max0.values))
        matches0 = torch.where(mutual0, m0, torch.full_like(m0, -1))
        return matches0, mscores0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--features", default="superpoint",
                        help="LightGlue weights: superpoint (default), disk, aliked, sift, doghardnet")
    parser.add_argument("-o", "--output", help="output file (default: <features>_lightglue.onnx)")
    parser.add_argument("--opset", type=int, default=18)
    args = parser.parse_args()
    output = args.output or "%s_lightglue.onnx" % args.features

    # flash=False: plain attention exports to standard ONNX operators
    lightglue = LightGlue(features=args.features, flash=False, depth_confidence=-1, width_confidence=-1).eval()
    split_qkv(lightglue)
    model = LightGlueOnnx(lightglue).eval()
    dim = lightglue.conf.input_dim
    m, n = 512, 384  # example sizes, both are exported as dynamic
    example = (torch.rand(1, m, 2) * 2 - 1, torch.rand(1, n, 2) * 2 - 1,
               torch.nn.functional.normalize(torch.randn(1, m, dim), dim=-1),
               torch.nn.functional.normalize(torch.randn(1, n, dim), dim=-1))
    M = torch.export.Dim("M", min=2, max=8192)
    N = torch.export.Dim("N", min=2, max=8192)
    with torch.no_grad():
        torch.onnx.export(
            model, example, output,
            input_names=["kpts0", "kpts1", "desc0", "desc1"],
            output_names=["matches0", "mscores0"],
            dynamic_shapes={"kpts0": {1: M}, "kpts1": {1: N}, "desc0": {1: M}, "desc1": {1: N}},
            opset_version=args.opset,
            dynamo=True,
            external_data=False,
        )
    print("Exported %s (LightGlue %s weights, %d-D descriptors, %d layers)"
          % (output, args.features, dim, lightglue.conf.n_layers))


if __name__ == "__main__":
    main()
