#! /usr/bin/env python3
#
# Drop this file in the root folder of SuperPoint git: https://github.com/rpautrat/SuperPoint
# To use with rtabmap:
#   --Vis/FeatureType 15 --Kp/DetectorStrategy 15 --PyDetector/Path "~/SuperPoint/rtabmap_superpoint_rpautrat.py"
#

import torch
import numpy as np
from superpoint_pytorch import SuperPoint

device = 'cpu'
superpoint = []

def init(cuda):
    global device
    device = 'cuda' if torch.cuda.is_available() and cuda else 'cpu'

    global superpoint
    # default config
    superpoint = SuperPoint().eval()
    
    # Load weights directly to target device
    superpoint.load_state_dict(torch.load('weights/superpoint_v6_from_tf.pth', map_location=device))
    
    # Move the model to the target device
    superpoint.to(device)

def detect(imageBuffer):
    global superpoint
    global device
    
    image = np.asarray(imageBuffer)
    image = (image.astype('float32') / 255.)
    image_tensor = torch.from_numpy(image[None, None]).float().to(device)
    # Result: (1, 1, H, W) - PyTorch tensor on correct device (CPU or GPU).
    
    with torch.no_grad():
        pred = superpoint({'image': image_tensor})
    
    # Extract keypoints and descriptors
    keypoints = pred['keypoints'][0].cpu().numpy()  # Shape: (N, 2)
    keypoints_response = pred['keypoint_scores'][0].cpu().numpy()  # Shape: (N,)
    # Convert keypoints to the format expected by PyDetector: (N, 3) with [x, y, response]
    keypoints_with_response = np.column_stack([keypoints, keypoints_response]).astype(np.float32)

    descriptors = pred['descriptors'][0].cpu().numpy()  # Shape: (N, descriptor_dim)    
    
    return keypoints_with_response, descriptors