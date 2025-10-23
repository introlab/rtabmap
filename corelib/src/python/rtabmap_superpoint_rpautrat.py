#! /usr/bin/env python3
#
# Drop this file in the root folder of SuperPoint git: https://github.com/rpautrat/SuperPoint
# To use with rtabmap:
#   --Vis/FeatureType 15 --Kp/DetectorStrategy 15 --PyDetector/Path "~/SuperPoint/rtabmap_superpoint_rpautrat.py"
#
import numpy as np
import os

# Defer importing Pytorch to work around the GIL issue
torch = None
superpoint = None
device = 'cpu'

def init(cuda):
    global torch, superpoint, device

    # GIL is now initialized
    import torch
    from superpoint_pytorch import SuperPoint

    superpoint = SuperPoint().eval()

    # set up device, gpu or cpu depending on the availability and the user's choice
    device = 'cuda' if torch.cuda.is_available() and cuda else 'cpu'
    
    # Load weights directly to target device
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    weights_path = os.path.join(script_dir, 'weights', 'superpoint_v6_from_tf.pth')
    
    # Load model weights with proper error handling
    try:
        state_dict = torch.load(weights_path, map_location=device, weights_only=True)
        superpoint.load_state_dict(state_dict)
    except Exception as e:
        print(f"Error loading weights: {e}")
        raise
    
    # Move the model to the target device
    superpoint.to(device)
    
    # Ensure model is in eval mode for inference
    superpoint.eval()

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
    keypoints_response = pred['keypoint_scores'][0].cpu().numpy()
    keypoints_with_response = np.column_stack([keypoints, keypoints_response]).astype(np.float32)
    # Result: (N, 3) with [x, y, response]

    descriptors = pred['descriptors'][0].cpu().numpy()  
    # Result: (N, descriptor_dim)    

    print("keypoints shape: ", keypoints_with_response.shape)
    print("descriptors shape: ", descriptors.shape)
    desc = np.float32(descriptors).copy()
    pts = np.float32(keypoints_with_response).copy()
    print("pts shape: ", pts.shape)
    print("desc shape: ", desc.shape)

    # Debug output
    print(f"DEBUG: Image shape: {image.shape}")
    print(f"DEBUG: Found {len(keypoints_with_response)} keypoints")
    if len(keypoints_with_response) > 0:
        print(f"DEBUG: First keypoint: {keypoints_with_response[0]}")
        print(f"DEBUG: Keypoint range: x=[{keypoints_with_response[:, 0].min():.2f}, {keypoints_with_response[:, 0].max():.2f}], y=[{keypoints_with_response[:, 1].min():.2f}, {keypoints_with_response[:, 1].max():.2f}]")
        print(f"DEBUG: Response range: [{keypoints_with_response[:, 2].min():.4f}, {keypoints_with_response[:, 2].max():.4f}]")
    print(f"DEBUG: Descriptor shape: {descriptors.shape}")
    print(f"DEBUG: Descriptor range: [{descriptors.min():.4f}, {descriptors.max():.4f}]")
    print(f"DEBUG: Descriptor norms: min={np.linalg.norm(descriptors, axis=1).min():.4f}, max={np.linalg.norm(descriptors, axis=1).max():.4f}")
    
    return pts, desc