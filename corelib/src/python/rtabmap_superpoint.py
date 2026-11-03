#! /usr/bin/env python3
#
# Drop this file in the root folder of SuperPoint git: https://github.com/magicleap/SuperPointPretrainedNetwork
# To use with rtabmap:
#   --Vis/FeatureType 15 --Kp/DetectorStrategy 15 --PyDetector/Path "~/SuperPointPretrainedNetwork/rtabmap_superpoint.py"
#

import random
import numpy as np
import torch
import os

#import sys
#import os
#print(os.sys.path)
#print(sys.version)

from demo_superpoint import SuperPointFrontend

torch.set_grad_enabled(False)

# How the descriptors are read from the network's coarse (1/8) descriptor map:
#   "demo"      (default) as magicleap's demo_superpoint.py, like the maps made
#               with this script so far.
#   "lightglue" at the centers of the 8x8 cells, as LightGlue's SuperPoint, so
#               the descriptors are the ones of rtabmap's OnnxDetector
#               (superpoint.onnx of LightGlue-ONNX) and SuperPointTorch, and the
#               ones SuperPoint-LightGlue was trained with.
# Both work, but their descriptors of a same keypoint differ (cosine ~0.975),
# so they match each other poorly: don't mix them in a map. Can also be set
# with the RTABMAP_SUPERPOINT_SAMPLING environment variable.
DESCRIPTOR_SAMPLING = os.environ.get('RTABMAP_SUPERPOINT_SAMPLING', 'demo')

device = 'cpu'
superpoint = []
coarse_desc = [None]  # last coarse descriptor map of the network (forward hook)

script_dir = os.path.dirname(os.path.abspath(__file__))

def init(cuda):
    #print("SuperPoint python init()")
    
    global device
    device = 'cuda' if torch.cuda.is_available() and cuda else 'cpu'
    
    weights_abs_path = os.path.join(script_dir, "superpoint_v1.pth")
    
    # This class runs the SuperPoint network and processes its outputs.
    global superpoint
    superpoint = SuperPointFrontend(weights_path=weights_abs_path,
                          nms_dist=4,
                          conf_thresh=0.015,
                          nn_thresh=1,
                          cuda=cuda)

    if DESCRIPTOR_SAMPLING not in ('demo', 'lightglue'):
        raise ValueError('Unknown DESCRIPTOR_SAMPLING "%s" (demo or lightglue)' % DESCRIPTOR_SAMPLING)
    if DESCRIPTOR_SAMPLING == 'lightglue':
        # Keep the coarse descriptor map of SuperPointFrontend.run() to sample it
        # again. run() calls net.forward() directly, so forward hooks don't fire.
        forward = superpoint.net.forward
        def forward_keeping_descriptors(x):
            outputs = forward(x)
            coarse_desc[0] = outputs[1]
            return outputs
        superpoint.net.forward = forward_keeping_descriptors

def sample_descriptors_lightglue(pts, height, width, s=8):
    # LightGlue's SuperPoint sample_descriptors(): keypoint (x,y) relative to the
    # centers of the s x s cells, bilinear with align_corners=True.
    keypoints = torch.from_numpy(np.ascontiguousarray(pts[:2].T)).float().to(coarse_desc[0].device)
    keypoints = keypoints - s / 2 + 0.5
    keypoints = keypoints / torch.tensor([width - s / 2 - 0.5, height - s / 2 - 0.5], device=keypoints.device)
    keypoints = keypoints * 2 - 1
    desc = torch.nn.functional.grid_sample(coarse_desc[0], keypoints.view(1, 1, -1, 2), mode='bilinear', align_corners=True)
    desc = torch.nn.functional.normalize(desc.reshape(desc.shape[1], -1), p=2, dim=0)
    return desc.cpu().numpy()

def detect(imageBuffer):
    #print("SuperPoint python detect()")
    global device
    image = np.asarray(imageBuffer)
    image = (image.astype('float32') / 255.)
   
    global superpoint
    pts, desc, heatmap = superpoint.run(image)
    if DESCRIPTOR_SAMPLING == 'lightglue' and pts.shape[1] > 0:
        desc = sample_descriptors_lightglue(pts, image.shape[0], image.shape[1])
    # return float: Kpts:Nx3, Desc:NxDim
    # use copy to make sure memory is correctly re-ordered
    pts = np.float32(np.transpose(pts)).copy()
    desc = np.float32(np.transpose(desc)).copy()
    
    # rtabmap expects format:
    #   pts:         array Nx3 (type=11 or float)
    #   descriptors: array NxDIM 35x256 (type=11 or float)
    return pts, desc


if __name__ == '__main__':
    #test
    init(False)
    detect(np.random.rand(640,480)*255)
