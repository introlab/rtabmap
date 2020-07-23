#! /usr/bin/env python3
#
# Drop this file in the "python" folder of NetVLAD git (tensorflow-v1 used): https://github.com/uzh-rpg/netvlad_tf_open/
# To use with rtabmap:
#   --PyDescriptor/Dim 128 --PyDescriptor/Path ~/netvlad_tf_open/python/rtabmap_netvlad.py
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

    tf.reset_default_graph()

    image_batch = tf.placeholder(
        dtype=tf.float32, shape=[None, None, None, 3])

    net_out = nets.vgg16NetvladPca(image_batch)
    saver = tf.train.Saver()

    sess = tf.Session()
    saver.restore(sess, nets.defaultCheckpoint())


def extract(image):
    print("NetVLAD python extract()")
    global image_batch
    global net_out
    global sess
    global dim
    
    print(image.shape)
    
    batch = np.expand_dims(image, axis=0)
    result = sess.run(net_out, feed_dict={image_batch: batch})
    result = result[:,:dim]
    
    return result


if __name__ == '__main__':
    #test
    img = np.zeros([100,100,3],dtype=np.uint8)
    img.fill(255)
    print(extract(img))
