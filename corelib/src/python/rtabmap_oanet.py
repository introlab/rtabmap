#! /usr/bin/env python3
#
# Drop this file in the "demo" folder of OANet git: https://github.com/zjhthu/OANet
# To use with rtabmap:
#   --Vis/CorNNType 6 --PyMatcher/Path ~/OANet/demo/rtabmap_oanet.py --PyMatcher/Model ~/OANet/model/gl3d/sift-4000/model_best.pth
#

import sys    
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/../core')
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
    
#print(os.sys.path)
#print(sys.version)

import numpy as np
from learnedmatcher import LearnedMatcher

lm = None

def init(descriptorDim, matchThreshold, iterations, cuda, model_path):
    print("OANet python init()")
    global lm
    lm = LearnedMatcher(model_path, inlier_threshold=1, use_ratio=0, use_mutual=0)


def match(kptsFrom, kptsTo, scoresFrom, scoresTo, descriptorsFrom, descriptorsTo, imageWidth, imageHeight):
    #print("OANet python match()")
    
    kpt1 = np.asarray(kptsFrom)
    kpt2 = np.asarray(kptsTo)
    desc1 = np.asarray(descriptorsFrom)
    desc2 = np.asarray(descriptorsTo)
      
    global lm
    matches, _, _ = lm.infer([kpt1, kpt2], [desc1, desc2])
    return matches


if __name__ == '__main__':
    #test
    init(128, 0.2, 20, False, True)
    match([[1, 2], [1,3], [4,6]], [[1, 3], [1,2], [16,2]], [1, 3,6], [1,3,5], np.full((3, 128), 1), np.full((3, 128), 1), 640, 480)
