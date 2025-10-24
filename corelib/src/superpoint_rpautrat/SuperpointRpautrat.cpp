/**
 * SuperPoint implementation based on the PyTorch version by Rémi Pautrat, Paul-Edouard Sarlin
 * Adapted for RTAB-Map integration
 */

#include "SuperpointRpautrat.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace rtabmap
{

    SuperPoint::SuperPoint()
        : conv1a(torch::nn::Conv2dOptions(1, 64, 3).stride(1).padding(1)),
          bn1a(torch::nn::BatchNorm2dOptions(64).eps(0.001)),
          conv1b(torch::nn::Conv2dOptions(64, 64, 3).stride(1).padding(1)),
          bn1b(torch::nn::BatchNorm2dOptions(64).eps(0.001)),

          conv2a(torch::nn::Conv2dOptions(64, 64, 3).stride(1).padding(1)),
          bn2a(torch::nn::BatchNorm2dOptions(64).eps(0.001)),
          conv2b(torch::nn::Conv2dOptions(64, 64, 3).stride(1).padding(1)),
          bn2b(torch::nn::BatchNorm2dOptions(64).eps(0.001)),

          conv3a(torch::nn::Conv2dOptions(64, 128, 3).stride(1).padding(1)),
          bn3a(torch::nn::BatchNorm2dOptions(128).eps(0.001)),
          conv3b(torch::nn::Conv2dOptions(128, 128, 3).stride(1).padding(1)),
          bn3b(torch::nn::BatchNorm2dOptions(128).eps(0.001)),

          conv4a(torch::nn::Conv2dOptions(128, 128, 3).stride(1).padding(1)),
          bn4a(torch::nn::BatchNorm2dOptions(128).eps(0.001)),
          conv4b(torch::nn::Conv2dOptions(128, 128, 3).stride(1).padding(1)),
          bn4b(torch::nn::BatchNorm2dOptions(128).eps(0.001)),

          convDetectorA(torch::nn::Conv2dOptions(128, 256, 3).stride(1).padding(1)),
          bnDetectorA(torch::nn::BatchNorm2dOptions(256).eps(0.001)),
          convDetectorB(torch::nn::Conv2dOptions(256, 65, 1).stride(1).padding(0)),
          bnDetectorB(torch::nn::BatchNorm2dOptions(65).eps(0.001)),

          convDescriptorA(torch::nn::Conv2dOptions(128, 256, 3).stride(1).padding(1)),
          bnDescriptorA(torch::nn::BatchNorm2dOptions(256).eps(0.001)),
          convDescriptorB(torch::nn::Conv2dOptions(256, 256, 1).stride(1).padding(0)),
          bnDescriptorB(torch::nn::BatchNorm2dOptions(256).eps(0.001))
    {

        register_module("conv1a", conv1a);
        register_module("bn1a", bn1a);
        register_module("conv1b", conv1b);
        register_module("bn1b", bn1b);

        register_module("conv2a", conv2a);
        register_module("bn2a", bn2a);
        register_module("conv2b", conv2b);
        register_module("bn2b", bn2b);

        register_module("conv3a", conv3a);
        register_module("bn3a", bn3a);
        register_module("conv3b", conv3b);
        register_module("bn3b", bn3b);

        register_module("conv4a", conv4a);
        register_module("bn4a", bn4a);
        register_module("conv4b", conv4b);
        register_module("bn4b", bn4b);

        register_module("convDetectorA", convDetectorA);
        register_module("bnDetectorA", bnDetectorA);
        register_module("convDetectorB", convDetectorB);
        register_module("bnDetectorB", bnDetectorB);

        register_module("convDescriptorA", convDescriptorA);
        register_module("bnDescriptorA", bnDescriptorA);
        register_module("convDescriptorB", convDescriptorB);
        register_module("bnDescriptorB", bnDescriptorB);

    }

    

    std::vector<torch::Tensor> SuperPoint::forward(torch::Tensor input)
    {
        // 3 Backbone layers conv -> batchnorm -> relu activation -> max pool
        input = torch::relu(bn1a(conv1a(input)));
        input = torch::relu(bn1b(conv1b(input)));
        input = torch::max_pool2d(input, 2, 2);

        input = torch::relu(bn2a(conv2a(input)));
        input = torch::relu(bn2b(conv2b(input)));
        input = torch::max_pool2d(input, 2, 2);

        input = torch::relu(bn3a(conv3a(input)));
        input = torch::relu(bn3b(conv3b(input)));

        input = torch::relu(bn4a(conv4a(input)));
        input = torch::relu(bn4b(conv4b(input)));

        auto features = input;

        // Detector Head
        auto cDetectorA = torch::relu(bnDetectorA(convDetectorA(features)));
        auto semi_dense = convDetectorB(cDetectorA);  // [B, 65, H/8, W/8]
        
        // Process detection scores
        semi_dense = torch::softmax(semi_dense, 1);
        semi_dense = semi_dense.slice(1, 0, 64);  // Remove "no keypoint" channel -> [B, 64, H/8, W/8]
        
        int b = semi_dense.size(0);
        int h = semi_dense.size(2);  // H/8
        int w = semi_dense.size(3);  // W/8
        
        // Reshape to full resolution
        semi_dense = semi_dense.permute({0, 2, 3, 1});  // [B, H/8, W/8, 64]
        semi_dense = semi_dense.contiguous().view({b, h, w, 8, 8});  // [B, H/8, W/8, 8, 8]
        semi_dense = semi_dense.permute({0, 1, 3, 2, 4});  // [B, H/8, 8, W/8, 8]
        semi_dense = semi_dense.contiguous().view({b, h * 8, w * 8});  // [B, H, W]
        
        // TODO: Apply NMS to the detection scores
        
        // Descriptor Head  
        auto cDescriptorA = torch::relu(bnDescriptorA(convDescriptorA(features)));
        auto desc = convDescriptorB(cDescriptorA);  // [B, 256, H/8, W/8]

        // Normalize descriptors
        auto dn = torch::norm(desc, 2, 1);
        desc = desc.div(torch::unsqueeze(dn, 1));
    }

    cv::Mat SPDetector::compute(const std::vector<cv::KeyPoint> &keypoints)
    {

    }
}