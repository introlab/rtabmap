#!/usr/bin/env python3
"""
Convert PyTorch weights to TorchScript format for C++ usage.
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

def generate_model(
    weights_path: str,
    output_path: str,
    cuda: bool,
    nms_radius: int,
    threshold: float,
    width: int,
    height: int,
):
    # Check if weights are already TorchScript
    try:
        scripted = torch.jit.load(weights_path, map_location="cpu")
        scripted.eval()
        torch.jit.save(scripted, output_path)
        print(f"Converted TorchScript file: {output_path}")
        return
    except:
        pass

    device = "cuda" if cuda else "cpu"

    # Load SuperPoint model and weights
    model = SuperPoint(
        nms_radius=nms_radius,
        threshold=threshold,
    ).eval().to(device)
    
    # Load weights without forcing CPU location to allow CUDA usage
    weights = torch.load(weights_path, map_location=None)
    if isinstance(weights, dict) and "state_dict" in weights:
        weights = weights["state_dict"]
    
    model.load_state_dict(weights, strict=False)
    
    wrapped = wrap_model(model)
    dummy = torch.randn(1, 1, height, width, device=device)  # Dummy input, grayscale, using cuda.
    
    # Convert to TorchScript using trace (SuperPoint has dynamic behavior that scripting can't handle)
    print("Using torch.jit.trace (SuperPoint has dynamic behavior)...")
    scripted = torch.jit.trace(wrapped, (dummy,), strict=False)
    print("Successfully traced SuperPoint model")
    
    # Save output
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    torch.jit.save(scripted, output_path)
    print(f"Converted SuperPoint weights to TorchScript: {output_path}")



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert SuperPoint weights to TorchScript")
    parser.add_argument("--weights", required=True, help="Path to weights file")
    parser.add_argument("--output", required=True, help="Output TorchScript file")
    parser.add_argument("--cuda", action="store_true", help="Use CUDA")
    parser.add_argument("--width", type=int, default=1920, help="Width of the input image")
    parser.add_argument("--height", type=int, default=288, help="Height of the input image")
    parser.add_argument("--nms_radius", type=int, default=4, help="NMS radius")
    parser.add_argument("--threshold", type=float, default=0.005, help="Confidence threshold")
    args = parser.parse_args()
    print(f"Generating model from weights: {args.weights} to output: {args.output}")


    
    # INSERT_YOUR_CODE
    print("============================================================")
    print(" superpoint_to_torchscript.py - MAIN ENTRYPOINT")
    print("============================================================")
    print(" Arguments received for model generation:")
    print("  --weights:    ", args.weights)
    print("  --output:     ", args.output)
    print("  --cuda:       ", args.cuda)
    print("  --width:      ", args.width)
    print("  --height:     ", args.height)
    print("  --nms_radius: ", args.nms_radius)
    print("  --threshold:  ", args.threshold)
    print("------------------------------------------------------------")
    print("   Did you mean to run this as a standalone script?")
    print("   If you see this, then the script is being used!")
    print("   (This block intentionally verbose for debugging.)")
    print("------------------------------------------------------------")
    print(" Python version:", __import__("sys").version)
    try:
        import torch
        print(" Torch version:", torch.__version__)
        print("   CUDA available:", torch.cuda.is_available())
        print("   CUDA devices:  ", torch.cuda.device_count() if torch.cuda.is_available() else 0)
    except Exception as e:
        print(" ERROR: Unable to import torch: ", e)
    try:
        import os
        print(" Current working dir:", os.getcwd())
    except Exception as e:
        print(" ERROR: Unable to import os: ", e)
    print("------------------------------------------------------------")
    print(" WARNING: This script may be called programmatically by RTAB-Map!")
    print("   If you see this output in your logs, model export should work.")
    print("============================================================")
    import time
    time.sleep(0.5)


    
    generate_model(
        weights_path=args.weights,
        output_path=args.output,
        cuda=args.cuda,
        nms_radius=args.nms_radius,
        threshold=args.threshold,
        width=args.width,
        height=args.height,
    )
