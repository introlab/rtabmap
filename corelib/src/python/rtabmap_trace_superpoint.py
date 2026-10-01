"""Trace MagicLeap's SuperPoint pretrained net to TorchScript for rtabmap.

Usage:
    python rtabmap_trace_superpoint.py [--weights superpoint_v1.pth]
                                       [--output  superpoint_v1.pt]
                                       [--model-dir <dir containing demo_superpoint.py>]

`--model-dir` is prepended to sys.path so `from demo_superpoint import ...`
resolves. Defaults to the directory of `--weights` (or cwd).
"""

import argparse
import os
import sys

import torch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights",   default="superpoint_v1.pth")
    parser.add_argument("--output",    default="superpoint_v1.pt")
    parser.add_argument("--model-dir", default=None,
            help="directory containing demo_superpoint.py (default: dir of --weights, then cwd)")
    args = parser.parse_args()

    candidates = [args.model_dir, os.path.dirname(os.path.abspath(args.weights)), os.getcwd()]
    for path in candidates:
        if path and path not in sys.path:
            sys.path.insert(0, path)

    from demo_superpoint import SuperPointNet  # noqa: E402

    model = SuperPointNet()
    model.load_state_dict(torch.load(args.weights, map_location="cpu"))
    model.eval()
    example = torch.rand(1, 1, 640, 480)
    traced_script_module = torch.jit.trace(model, example, check_trace=False)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    traced_script_module.save(args.output)
    print(f"Saved TorchScript: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
