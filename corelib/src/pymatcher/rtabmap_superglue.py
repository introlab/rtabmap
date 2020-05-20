#! /usr/bin/env python3
#
# Drop this file in the root folder of SuperGlue git: https://github.com/magicleap/SuperGluePretrainedNetwork
# To use with rtabmap:
#   --Vis/CorNNType 6 --SuperGlue/Path "~/SuperGluePretrainedNetwork/rtabmap_superglue.py"
#

import random
import numpy as np
import torch

#import sys
#import os
#print(os.sys.path)
#print(sys.version)

from models.matching import SuperGlue

torch.set_grad_enabled(False)

device = 'cpu'
superglue = []

def init(descriptorDim, matchThreshold, iterations, cuda, model):
    print("SuperGlue python init()")
    # Load the SuperGlue model.
    global device
    device = 'cuda' if torch.cuda.is_available() and cuda else 'cpu'
    assert model == "indoor" or model == "outdoor", "Available models for SuperGlue are 'indoor' or 'outdoor'"
    config = {
        'superglue': {
            'weights': model,
            'sinkhorn_iterations': iterations,
            'match_threshold': matchThreshold,
            'descriptor_dim' : descriptorDim
        }
    }
    global superglue
    superglue = SuperGlue(config.get('superglue', {})).eval().to(device)


def match(kptsFrom, kptsTo, scoresFrom, scoresTo, descriptorsFrom, descriptorsTo, imageWidth, imageHeight):
    #print("SuperGlue python match()")
    global device
    kptsFrom = np.asarray(kptsFrom)
    kptsFrom = kptsFrom[None, :, :]
    kptsTo = np.asarray(kptsTo)
    kptsTo = kptsTo[None, :, :]
    scoresFrom = np.asarray(scoresFrom)
    scoresFrom = scoresFrom[None, :]
    scoresTo = np.asarray(scoresTo)
    scoresTo = scoresTo[None, :]
    descriptorsFrom = np.transpose(np.asarray(descriptorsFrom))
    descriptorsFrom = descriptorsFrom[None, :, :]
    descriptorsTo = np.transpose(np.asarray(descriptorsTo))
    descriptorsTo = descriptorsTo[None, :, :]
      
    data = {
       'image0': torch.rand(1, 1, imageHeight, imageWidth).to(device),
       'image1': torch.rand(1, 1, imageHeight, imageWidth).to(device),
       'scores0': torch.from_numpy(scoresFrom).to(device),
       'scores1': torch.from_numpy(scoresTo).to(device),
       'keypoints0': torch.from_numpy(kptsFrom).to(device),
       'keypoints1': torch.from_numpy(kptsTo).to(device),
       'descriptors0': torch.from_numpy(descriptorsFrom).to(device),
       'descriptors1': torch.from_numpy(descriptorsTo).to(device),
    }
    

    global superglue
    results = superglue(data)

    matches0 = results['matches0'].to('cpu').numpy()
  
    matchesFrom = np.nonzero(matches0!=-1)[1]
    matchesTo = matches0[np.nonzero(matches0!=-1)]
       
    matchesArray = np.stack((matchesFrom, matchesTo), axis=1);
    
    return matchesArray


if __name__ == '__main__':
    #test
    init(256, 0.2, 20, True, 'indoor')
    match([[1, 2], [1,3]], [[1, 3], [1,2]], [1, 3], [1,3], np.full((2, 256), 1),np.full((2, 256), 1), 640, 480)
