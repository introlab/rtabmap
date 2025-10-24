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
    input = torch::max_pool2d(input, 2, 2);

    input = torch::relu(bn4a(conv4a(input)));
    input = torch::relu(bn4b(conv4b(input)));

    auto features = input;

    // Detector Head
    auto cDetectorA = torch::relu(bnDetectorA(convDetectorA(features)));
    auto scores = convDetectorB(cDetectorA);  // [B, 65, H/8, W/8]
    
    // Process detection scores
    scores = torch::softmax(scores, 1);
    scores = scores.slice(1, 0, 64);  // Remove "no keypoint" channel -> [B, 64, H/8, W/8]
    
    int b = scores.size(0);
    int h = scores.size(2);  // H/8
    int w = scores.size(3);  // W/8
    
    // Reshape to full resolution
    scores = scores.permute({0, 2, 3, 1});  // [B, H/8, W/8, 64]
    scores = scores.contiguous().view({b, h, w, 8, 8});  // [B, H/8, W/8, 8, 8]
    scores = scores.permute({0, 1, 3, 2, 4});  // [B, H/8, 8, W/8, 8]
    scores = scores.contiguous().view({b, h * 8, w * 8});  // [B, H, W]
            
    // Descriptor Head  
    auto cDescriptorA = torch::relu(bnDescriptorA(convDescriptorA(features)));
    auto desc = convDescriptorB(cDescriptorA);  // [B, 256, H/8, W/8]

    std::vector<torch::Tensor> ret;
    ret.push_back(scores);
    ret.push_back(desc);

    return ret;
}

SPDetector::SPDetector(const std::string & modelPath, float threshold, bool nms, int minDistance, bool cuda) :
		threshold_(threshold),
		nms_(nms),
		minDistance_(minDistance),
		detected_(false)
{
	UDEBUG("modelPath=%s thr=%f nms=%d cuda=%d", modelPath.c_str(), threshold, nms?1:0, cuda?1:0);
	if(modelPath.empty())
	{
		UERROR("Model's path is empty!");
		return;
	}
	std::string path = uReplaceChar(modelPath, '~', UDirectory::homeDir());
	if(!UFile::exists(path))
	{
		UERROR("Model's path \"%s\" doesn't exist!", path.c_str());
		return;
	}
	model_ = std::make_shared<SuperPoint>();
	torch::load(model_, uReplaceChar(path, '~', UDirectory::homeDir()));

	if(cuda && !torch::cuda::is_available())
	{
		UWARN("Cuda option is enabled but torch doesn't have cuda support on this platform, using CPU instead.");
	}
	cuda_ = cuda && torch::cuda::is_available();
	torch::Device device(cuda_?torch::kCUDA:torch::kCPU);
	model_->to(device);
}

cv::Mat SPDetector::compute(const std::vector<cv::KeyPoint> &keypoints)
{
	if(!detected_)
	{
		UERROR("SPDetector has been reset before extracting the descriptors! detect() should be called before compute().");
		return cv::Mat();
	}
    if(keypoints.empty())
	{
		return cv::Mat();
	}

    // Get dimensions
    auto b = desc_.size(0);  // batch size (should be 1)
    auto c = desc_.size(1);  // channels (256 for SuperPoint)
    auto h = desc_.size(2);  // height/8
    auto w = desc_.size(3);  // width/8
    
    float s = 8;
    cv::Mat kpt_mat(keypoints.size(), 2, CV_32F);
    for (size_t i = 0; i < keypoints.size(); i++) {
        kpt_mat.at<float>(i, 0) = keypoints[i].pt.y - s/2 + 0.5;  // y
        kpt_mat.at<float>(i, 1) = keypoints[i].pt.x - s/2 + 0.5;  // x
    }
    
    // Convert to torch tensor
    auto fkpts = torch::from_blob(kpt_mat.data, {(long int)keypoints.size(), 2}, torch::kFloat);
    
    // Create grid for grid_sampler (matching SuperPoint.cc approach)
    auto grid = torch::zeros({1, 1, fkpts.size(0), 2}).to(desc_.device());
    grid[0][0].slice(1, 0, 1) = 2.0 * fkpts.slice(1, 1, 2) / (w*s - s/2 - 0.5) - 1;  // x
    grid[0][0].slice(1, 1, 2) = 2.0 * fkpts.slice(1, 0, 1) / (h*s - s/2 - 0.5) - 1;  // y
    
    // Grid sample (bilinear interpolation) - match Python: align_corners=False
    auto sampled = torch::grid_sampler(desc_, grid, 0, 0, false);
    
    // Normalize descriptors (L2 normalization)
    auto normalized = torch::nn::functional::normalize(
        sampled.reshape({desc_.size(1), -1}), 
        torch::nn::functional::NormalizeFuncOptions().dim(0)
    );
    
    // Transpose to [n_keypoints, 256]
    normalized = normalized.transpose(0, 1).contiguous();
    
    // Move to CPU if needed
    if(cuda_)
        normalized = normalized.to(torch::kCPU);
    
    // Convert to OpenCV Mat
    cv::Mat desc_mat(cv::Size(normalized.size(1), normalized.size(0)), CV_32FC1, normalized.data_ptr<float>());
    
    return desc_mat.clone();
}

std::vector<cv::KeyPoint> SPDetector::detect(const cv::Mat &img, const cv::Mat & mask)
{
    torch::NoGradGuard no_grad_guard;
    auto x = torch::from_blob(img.data, {1, 1, img.rows, img.cols}, torch::kByte);
    x = x.to(torch::kFloat) / 255;

    torch::Device device(cuda_?torch::kCUDA:torch::kCPU);
    x = x.set_requires_grad(false);
    
    // run the model
    auto out = model_->forward(x.to(device));
    auto scores = out[0];
    desc_ = out[1];

    if(nms_)
    {
        auto options = torch::nn::functional::MaxPool2dFuncOptions(minDistance_*2+1).stride(1).padding(minDistance_);
        auto options_r1 = torch::nn::functional::MaxPool2dFuncOptions(3).stride(1).padding(1);

        auto zeros = torch::zeros_like(scores);
        auto max_mask = scores == torch::nn::functional::max_pool2d(scores, options);
        auto max_mask_r1 = scores == torch::nn::functional::max_pool2d(scores, options_r1);
        for(size_t i=0; i<2; i++)
        {
            auto supp_mask = torch::nn::functional::max_pool2d(max_mask.to(torch::kF32), options) > 0;
            auto supp_scores = torch::where(supp_mask, zeros, scores);
            auto new_max_mask = supp_scores == torch::nn::functional::max_pool2d(supp_scores, options);
            max_mask = max_mask | (new_max_mask & (~supp_mask) & max_mask_r1);
        }
        prob_ = torch::where(max_mask, scores, zeros).squeeze(0);
    }
    else
    {
        prob_ = scores.squeeze(0);
    }

    auto kpts = (prob_ > threshold_);
    kpts = torch::nonzero(kpts);  // [n_keypoints, 2]  (y, x)

    auto kpts_cpu = kpts.to(torch::kCPU);
    auto prob_cpu = prob_.to(torch::kCPU);

    std::vector<cv::KeyPoint> keypoints;
    for(int i=0; i<kpts_cpu.size(0); i++)
    {
        if(mask.empty() || mask.at<unsigned char>(kpts_cpu[i][0].item<int>(), kpts_cpu[i][1].item<int>()) != 0)
        {
            float response = prob_cpu[kpts_cpu[i][0]][kpts_cpu[i][1]].item<float>();
            keypoints.emplace_back(cv::KeyPoint(kpts_cpu[i][1].item<float>(), kpts_cpu[i][0].item<float>(), 8, -1, response));
        }
    }

    detected_ = true;
    return keypoints;
}
} // namespace rtabmap