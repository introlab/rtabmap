#! /usr/bin/env python3
#
# Drop this file in the "python" folder of NetVLAD git (tensorflow-v1 used): https://github.com/uzh-rpg/netvlad_tf_open/
# Updated to work with https://github.com/uzh-rpg/netvlad_tf_open/pull/9
# To use with rtabmap:
#   --Mem/GlobalDescriptorStrategy 1 --Kp/TfIdfLikelihoodUsed false --Mem/RehearsalSimilarity 1 --PyDescriptor/Dim 128 --PyDescriptor/Path ~/netvlad_tf_open/python/rtabmap_netvlad.py
#

import sys    
import os
import numpy as np
import time
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
    
#print(os.sys.path)
#print(sys.version)

import tensorflow as tf
import netvlad_tf.net_from_mat as nfm
import netvlad_tf.nets as nets

image_batch = None
net_out = None
saver = None
sess = None
dim = 4096

def init(descriptorDim):
    print("NetVLAD python init()")
    global image_batch
    global net_out
    global saver
    global sess
    global dim
    
    dim = descriptorDim

    tf.compat.v1.disable_eager_execution()
    tf.compat.v1.reset_default_graph()

    image_batch = tf.compat.v1.placeholder(
        dtype=tf.float32, shape=[None, None, None, 3])

    net_out = nets.vgg16NetvladPca(image_batch)
    saver = tf.compat.v1.train.Saver()

    sess = tf.compat.v1.Session()
    saver.restore(sess, nets.defaultCheckpoint())


def extract(image):
    print(f"NetVLAD python extract{image.shape}")
    global image_batch
    global net_out
    global sess
    global dim

    if(image.shape[2] == 1):
        image = np.dstack((image, image, image))

    batch = np.expand_dims(image, axis=0)
    result = sess.run(net_out, feed_dict={image_batch: batch})
    
    # All that needs to be done (only valid for NetVLAD+whitening networks!)
    # to reduce the dimensionality of the NetVLAD representation below 4096 to D
    # is to keep the first D dimensions and L2-normalize.
    if(result.shape[1] > dim):
        v = result[:, :dim]
        result = v/np.linalg.norm(v)

    return np.float32(result)


if __name__ == '__main__':
    #test
    img = np.zeros([100,100,3],dtype=np.uint8)
    img.fill(255)
    init(128)
    descriptor = extract(img)
    print(descriptor.shape)
    print(descriptor)