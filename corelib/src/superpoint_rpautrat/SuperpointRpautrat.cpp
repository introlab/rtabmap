/**
 * SuperPoint implementation based on the PyTorch version by Rémi Pautrat, Paul-Edouard Sarlin
 * Adapted for RTAB-Map integration
 */

#include "SuperpointRpautrat.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/PythonInterface.h>
#include <pybind11/embed.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace rtabmap
{

// Run the python script to export the SuperPoint model file with the desired parameters
static std::string exportSuperPointTorchScript(
    const std::string & superpointDir,
    const int & width,
    const int & height,
    const float & threshold,
    const int & nms_radius,
    const int & max_keypoints,
    const bool & cuda)
{
    // Resolve paths
    const std::string srcDir = UDirectory::getDir(__FILE__); // folder of this .cpp at build time
    const std::string srcScript = srcDir + "/superpoint_to_torchscript.py";
    const std::string dstScript = superpointDir + "/superpoint_to_torchscript.py";
    const std::string weights   = superpointDir + "/weights/superpoint_v6_from_tf.pth";
    const std::string output    = superpointDir + "/weights/superpoint_v6_generated.pt";
    
    // Sanity checks
    if(!UFile::exists(srcScript)) {
        UERROR("Torchscript conversion script not found: %s", srcScript.c_str());
        return "";
    }
    if(!UFile::exists(weights)) {
        UERROR("Weights not found: %s", weights.c_str());
        return "";
    }
    
    // Copy script over (overwrite unconditionally)
    try {
        UFile::copy(srcScript, dstScript);
    } catch(const std::exception &e) {
        UERROR("Copy failed: %s -> %s (%s)", srcScript.c_str(), dstScript.c_str(), e.what());
        return "";
    }

    std::string cuda_flag = cuda ? "--cuda" : "";

    // Build CLI command (set PYTHONPATH so imports resolve)
    std::stringstream cmd;
    cmd << "PYTHONPATH=\"" << superpointDir << "\" "
        << "python3 \"" << dstScript << "\""
        << " --width " << width
        << " --height " << height
        << " --weights \"" << weights << "\""
        << " --output \"" << output << "\""
        << " --threshold " << threshold
        << " --nms_radius " << nms_radius
        << " --max_keypoints " << max_keypoints << " "
        << cuda_flag;
        
    UINFO("Executing: %s", cmd.str().c_str());
    const int code = std::system(cmd.str().c_str());
    if(code != 0) {
        UERROR("superpoint_to_torchscript.py failed (exit=%d)", code);
        return "";
    }
    
    // clean up the temporary script
	UFile::erase(dstScript);

	return output;
}

SPDetectorRpautrat::SPDetectorRpautrat(std::string superpointDir, float threshold, bool nms, int minDistance, bool cuda) :
		superpointDir_(superpointDir),
        threshold_(threshold),
		nms_(nms),
		minDistance_(minDistance),
		detected_(false)
{
	if(cuda && !torch::cuda::is_available())
	{
		UWARN("Cuda option is enabled but torch doesn't have cuda support on this platform, using CPU instead.");
	}
	cuda_ = cuda && torch::cuda::is_available();
    
    if(!UFile::exists(superpointDir_)) {
        UERROR("Superpoint directory not found: %s", superpointDir_.c_str());
    }
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
    if(!detected_)
    {
        UWARN("DEBUG: first call to detect(), initailizing model...");

        pybind11::gil_scoped_acquire acquire; // required before any Python C-API calls
        PyRun_SimpleString("import sys");
        std::string cmd_add_path = "sys.path.insert(0, \"" + superpointDir_ + "\")";
        PyRun_SimpleString(cmd_add_path.c_str());
        
        // effectively disable nms if it is not enabled by setting radius to 0
        int nms_radius = nms_ ? minDistance_ : 0;

        std::string modelPath = exportSuperPointTorchScript(
            superpointDir_,
            img.cols,
            img.rows,
            nms_radius, 
            threshold_, 
            1000, 
            cuda_
        );

        UDEBUG("modelPath=%s thr=%f nms=%d minDistance=%d cuda=%d", modelPath.c_str(), threshold_, nms_?1:0, minDistance_, cuda_?1:0);
        UWARN("Initializing SuperPoint Rpautrat detector with model: %s", modelPath.c_str());
        UWARN("SuperPoint Rpautrat parameters: threshold=%.3f, nms=%s, minDistance=%d", threshold_, nms_?"true":"false", minDistance_);
        if(modelPath.empty())
        {
            UERROR("Model's path is empty! The model was not exported correctly.");
            return std::vector<cv::KeyPoint>();
        }
        if(!UFile::exists(modelPath))
        {
            UERROR("Model's path \"%s\" doesn't exist!", modelPath.c_str());
            return std::vector<cv::KeyPoint>();
        }

        // Load TorchScript model
        model_ = torch::jit::load(modelPath);
        model_.eval();

        // Move the model to the appropriate device
        torch::Device device(cuda_?torch::kCUDA:torch::kCPU);
	    model_.to(device);
    }
    
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
    int total_before_mask = keypoints_cpu.size(0);

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

    // Log counts
    UINFO("SuperPoint Rpautrat: keypoints before mask=%d, after mask=%zu", total_before_mask, filtered_keypoints.size());

    detected_ = true;
    return filtered_keypoints;
}
} // namespace rtabmap