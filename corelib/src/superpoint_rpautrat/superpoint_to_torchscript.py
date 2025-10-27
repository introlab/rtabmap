#!/usr/bin/env python3
"""
Convert PyTorch weights to TorchScript format for C++ usage.

Copy and run this script in "~/Superpoint" to expor
"""

import argparse
import os

import torch
import torch.nn as nn
from superpoint_pytorch import SuperPoint


def wrap_model(model: nn.Module):
    """
    Simple wrapper to fix SuperPoint input format for TorchScript.
    Easier to call from C++ code since the input isn't a dictionary.
    """
    class Wrapper(nn.Module):
        def __init__(self, net: nn.Module):
            super().__init__()
            self.net = net

        def forward(self, x: torch.Tensor):
            # SuperPoint expects {"image": tensor} but TorchScript doesn't like dict indexing
            out = self.net.forward({"image": x})
            
            # Return the format expected by C++ code: keypoints, scores, descriptors
            # For single batch item, take the first (and only) element
            keypoints = out["keypoints"][0] if out["keypoints"] else torch.empty(0, 2)
            scores = out["keypoint_scores"][0] if out["keypoint_scores"] else torch.empty(0)
            descriptors = out["descriptors"][0] if out["descriptors"] else torch.empty(0, 256)
            
            return (keypoints, scores, descriptors)
    
    return Wrapper(model)


@torch.no_grad()
def main():
    parser = argparse.ArgumentParser(description="Convert SuperPoint weights to TorchScript")
    parser.add_argument("--weights", required=True, help="Path to weights file")
    parser.add_argument("--output", required=True, help="Output TorchScript file")
    args = parser.parse_args()

    # Check if weights are already TorchScript
    try:
        scripted = torch.jit.load(args.weights, map_location="cpu")
        scripted.eval()
        torch.jit.save(scripted, args.output)
        print(f"Converted TorchScript file: {args.output}")
        return
    except:
        pass

    # Load SuperPoint model and weights
    model = SuperPoint().eval()
    
    # Load weights without forcing CPU location to allow CUDA usage
    weights = torch.load(args.weights, map_location=None)
    if isinstance(weights, dict) and "state_dict" in weights:
        weights = weights["state_dict"]
    
    model.load_state_dict(weights, strict=False)
    
    wrapped = wrap_model(model)
    dummy = torch.randn(1, 1, 288, 1920, device="cuda")  # Dummy input, grayscale, using cuda.
    
    # Convert to TorchScript using trace (SuperPoint has dynamic behavior that scripting can't handle)
    print("Using torch.jit.trace (SuperPoint has dynamic behavior)...")
    scripted = torch.jit.trace(wrapped, (dummy,), strict=False)
    print("Successfully traced SuperPoint model")
    
    # Save output
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    torch.jit.save(scripted, args.output)
    print(f"Converted SuperPoint weights to TorchScript: {args.output}")


if __name__ == "__main__":
    main()
