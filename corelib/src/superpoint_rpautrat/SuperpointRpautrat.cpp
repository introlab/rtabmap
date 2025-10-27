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
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace rtabmap
{

SuperPointRpautratModel::SuperPointRpautratModel()
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

std::vector<torch::Tensor> SuperPointRpautratModel::forward(torch::Tensor input)
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

SPDetectorRpautrat::SPDetectorRpautrat(const std::string & modelPath, float threshold, bool nms, int minDistance, bool cuda) :
		threshold_(threshold),
		nms_(nms),
		minDistance_(minDistance),
		detected_(false)
{
	UDEBUG("modelPath=%s thr=%f nms=%d minDistance=%d cuda=%d", modelPath.c_str(), threshold, nms?1:0, minDistance, cuda?1:0);
	UWARN("Initializing SuperPoint Rpautrat detector with model: %s", modelPath.c_str());
	UWARN("SuperPoint Rpautrat parameters: threshold=%.3f, nms=%s, minDistance=%d", threshold, nms?"true":"false", minDistance);
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
	// Load TorchScript model
	model_ = torch::jit::load(uReplaceChar(path, '~', UDirectory::homeDir()));
	model_.eval();

	if(cuda && !torch::cuda::is_available())
	{
		UWARN("Cuda option is enabled but torch doesn't have cuda support on this platform, using CPU instead.");
	}
	cuda_ = cuda && torch::cuda::is_available();
	torch::Device device(cuda_?torch::kCUDA:torch::kCPU);
	model_.to(device);
}

SPDetectorRpautrat::~SPDetectorRpautrat()
{
}

cv::Mat SPDetectorRpautrat::compute(const std::vector<cv::KeyPoint> &keypoints)
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

    // RTAB-Map may have applied additional filtering (like top-K) to the keypoints
    // We need to match the input keypoints to our stored descriptors
    torch::Tensor filtered_descriptors = torch::zeros({(long int)keypoints.size(), 256}, desc_.options());
    
    UWARN("COMPUTE: Input keypoints: %ld, Stored desc tensor: [%ld, %ld]", 
          keypoints.size(), desc_.size(0), desc_.size(1));
    
    // Get the stored keypoints for matching
    auto stored_keypoints_cpu = keypoints_tensor_.to(torch::kCPU);
    
    int num_stored_keypoints = stored_keypoints_cpu.size(0);
    UWARN("COMPUTE: Stored keypoints: %d", num_stored_keypoints);
    
    // Pre-extract all coordinates for efficient matching
    float * kp_data = stored_keypoints_cpu.data_ptr<float>();
    
    for(size_t i = 0; i < keypoints.size(); i++) {
        float x = keypoints[i].pt.x;
        float y = keypoints[i].pt.y;
        
        // Find matching descriptor by coordinate
        for(int j = 0; j < num_stored_keypoints; j++) {
            float stored_x = kp_data[j * 2 + 0];   // x coordinate
            float stored_y = kp_data[j * 2 + 1];   // y coordinate
            
            // Match by coordinates only
            float dx = x - stored_x;
            float dy = y - stored_y;
            float distSq = dx * dx + dy * dy;
            
            // Use tight tolerance for coordinates
            if(distSq < 1.0f) {
                filtered_descriptors[i] = desc_[j];
                break;
            }
        }
    }

    
    auto normalized = filtered_descriptors;
    
    // Move to CPU if needed
    if(cuda_)
        normalized = normalized.to(torch::kCPU);
    
    // Convert to OpenCV Mat
    cv::Mat desc_mat(cv::Size(normalized.size(1), normalized.size(0)), CV_32FC1, normalized.data_ptr<float>());
    UWARN("COMPUTE: Final output descriptor mat: %d rows x %d cols", desc_mat.rows, desc_mat.cols);
    
    return desc_mat.clone();
}

std::vector<cv::KeyPoint> SPDetectorRpautrat::detect(const cv::Mat &img, const cv::Mat & mask)
{
    torch::NoGradGuard no_grad_guard;
    auto x = torch::from_blob(img.data, {1, 1, img.rows, img.cols}, torch::kByte);
    x = x.to(torch::kFloat) / 255;

    torch::Device device(cuda_?torch::kCUDA:torch::kCPU);
    x = x.set_requires_grad(false).to(device);
    model_.to(device);
    
    // run the model
    auto outputs = model_.forward({x}).toTuple();
    keypoints_tensor_ = outputs->elements()[0].toTensor();  // [N, 2] keypoint coordinates
    auto scores_tensor = outputs->elements()[1].toTensor();    // [N] keypoint scores
    desc_ = outputs->elements()[2].toTensor();              // [N, 256] descriptors
    
    UWARN("DETECT: Model output - Keypoints: [%ld, %ld], Scores: [%ld], Descriptors: [%ld, %ld]", 
          keypoints_tensor_.size(0), keypoints_tensor_.size(1),
          scores_tensor.size(0), desc_.size(0), desc_.size(1));
    
    // Convert to CPU for processing
    auto keypoints_cpu = keypoints_tensor_.to(torch::kCPU);
    auto scores_cpu = scores_tensor.to(torch::kCPU);
    
    // Apply threshold filtering
    std::vector<cv::KeyPoint> keypoints;
    for(int i = 0; i < keypoints_cpu.size(0); i++) {
        float score = scores_cpu[i].item<float>();
        if(score > threshold_) {
            float x = keypoints_cpu[i][0].item<float>();  // x coordinate
            float y = keypoints_cpu[i][1].item<float>();  // y coordinate
            
            // Check mask if provided
            if(mask.empty() || mask.at<unsigned char>((int)y, (int)x) != 0) {
                keypoints.emplace_back(cv::KeyPoint(x, y, 8, -1, score));
            }
        }
    }
    UWARN("DETECT: After threshold (%.3f) filtering: %ld keypoints", threshold_, keypoints.size());
    
    // Apply NMS if enabled - use simple 1D NMS for keypoint arrays
    if(nms_ && keypoints.size() > 1) {
        // Convert minDistance from int to float
        float minDistNms = (float)minDistance_;
        
        // Simple NMS: remove keypoints that are too close to each other
        // Keep track of which keypoints to keep
        std::vector<bool> keep_mask(keypoints.size(), true);
        
        for(size_t i = 0; i < keypoints.size(); i++) {
            if(!keep_mask[i]) continue; // Already suppressed
            
            for(size_t j = i + 1; j < keypoints.size(); j++) {
                if(!keep_mask[j]) continue; // Already suppressed
                
                float dist = std::sqrt(std::pow(keypoints[i].pt.x - keypoints[j].pt.x, 2) + 
                                      std::pow(keypoints[i].pt.y - keypoints[j].pt.y, 2));
                if(dist < minDistNms) {
                    // Keep the one with higher response, suppress the other
                    if(keypoints[i].response > keypoints[j].response) {
                        keep_mask[j] = false;
                    } else {
                        keep_mask[i] = false;
                        break; // Current keypoint is suppressed, move to next
                    }
                }
            }
        }
        
        // Filter keypoints and corresponding descriptors
        std::vector<cv::KeyPoint> filtered_keypoints;
        long int num_kept_keypoints = std::count(keep_mask.begin(), keep_mask.end(), true);
        torch::Tensor filtered_keypoints_tensor = torch::zeros({num_kept_keypoints, 2}, keypoints_tensor_.options());
        torch::Tensor filtered_descriptors = torch::zeros({num_kept_keypoints, 256}, desc_.options());
        
        int filtered_idx = 0;
        for(size_t i = 0; i < keypoints.size(); i++) {
            if(keep_mask[i]) {
                filtered_keypoints.push_back(keypoints[i]);
                filtered_keypoints_tensor[filtered_idx] = keypoints_tensor_[i];
                filtered_descriptors[filtered_idx] = desc_[i];
                filtered_idx++;
            }
        }
        
        // Update the stored tensors to maintain correspondence
        keypoints = filtered_keypoints;
        keypoints_tensor_ = filtered_keypoints_tensor;
        desc_ = filtered_descriptors;
        
        UWARN("DETECT: After 1D NMS (minDist=%.1f): %ld keypoints", minDistNms, keypoints.size());
    }

    detected_ = true;
    UWARN("DETECT: Final keypoint count: %ld", keypoints.size());
    return keypoints;
}
} // namespace rtabmap