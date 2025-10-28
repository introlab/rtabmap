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

    // TODO: load the model on the first frame if it isn't already instead of in the constructor
    // this way if odom already has features we don't need to load the model at all, slight optimization
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
    
    // Get the stored keypoints for matching
    auto stored_keypoints_cpu = keypoints_tensor_.to(torch::kCPU);
    
    int num_stored_keypoints = stored_keypoints_cpu.size(0);
    
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
    
    auto outputs = model_.forward({x}).toTuple();
    keypoints_tensor_ = outputs->elements()[0].toTensor();  // [N, 2] keypoint coordinates
    auto scores_tensor = outputs->elements()[1].toTensor();    // [N] keypoint scores
    desc_ = outputs->elements()[2].toTensor();              // [N, 256] descriptors    

    // Convert to CPU for processing
    auto keypoints_cpu = keypoints_tensor_.to(torch::kCPU);
    auto scores_cpu = scores_tensor.to(torch::kCPU);

    std::vector<cv::KeyPoint> filtered_keypoints;
    std::vector<int64_t> keep_indices_vec;
    
    // Apply mask filtering
    for(int i = 0; i < keypoints_cpu.size(0); i++) {
        float score = scores_cpu[i].item<float>();
        float x = keypoints_cpu[i][0].item<float>();  // x coordinate
        float y = keypoints_cpu[i][1].item<float>();  // y coordinate
            
        // Check mask if provided
        if(mask.empty() || mask.at<unsigned char>((int)y, (int)x) != 0) {
            keep_indices_vec.push_back(i);
            filtered_keypoints.emplace_back(cv::KeyPoint(x, y, 8, -1, score));
        }
    }
    
    // Update the stored tensors to maintain correspondence
    // This way if keypoints are re-ordered, we can still match kpts->descs in the compute step
    auto keep_indices = torch::from_blob(keep_indices_vec.data(), {(long int)keep_indices_vec.size()}, torch::kLong);
    keep_indices = keep_indices.to(keypoints_tensor_.device());
    auto filtered_keypoints_tensor = keypoints_tensor_.index_select(0, keep_indices);
    auto filtered_descriptors = desc_.index_select(0, keep_indices);
    
    keypoints_tensor_ = filtered_keypoints_tensor;
    desc_ = filtered_descriptors;

    detected_ = true;
    return filtered_keypoints;
}
} // namespace rtabmap