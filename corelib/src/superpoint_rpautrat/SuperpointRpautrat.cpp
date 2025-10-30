/**
 * SuperPoint implementation based on the PyTorch version by Rémi Pautrat, Paul-Edouard Sarlin
 * Adapted for RTAB-Map integration
 */

#include "SuperpointRpautrat.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <pybind11/embed.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <sstream>
#include "superpoint_to_torchscript_py.h"

namespace rtabmap
{

// Run the python script to export the SuperPoint model file with the desired parameters
static std::string exportSuperPointTorchScript(
    const std::string & superpointWeightsPath,
    const int & width,
    const int & height,
    const float & threshold,
    const int & nms_radius,
    const bool & cuda)
{
	// Resolve paths (no dependency on source tree)
	const std::string file_content = uHex2Str(SUPERPOINT_TO_TORCHSCRIPT_PY);
	const std::string dstScript = std::string("/tmp/superpoint_to_torchscript.py");
	const std::string weightsPath   = superpointWeightsPath;
	const std::string output    = std::string("/tmp/superpoint_v6_generated.pt");
    
	// Sanity checks
    if(!UFile::exists(weightsPath)) {
        UERROR("Weights not found: %s", weightsPath.c_str());
        return "";
    }

	// Generate the temporary script, overwrite if it already exists
	{
		std::ofstream ofs(dstScript.c_str(), std::ios::out | std::ios::trunc);
		if(!ofs.is_open())
		{
			UERROR("Failed to open destination script for writing: %s", dstScript.c_str());
			return "";
		}
		ofs << file_content;
		if(!ofs.good())
		{
			UERROR("Failed to write embedded script to: %s", dstScript.c_str());
			return "";
		}
	}
    
    // Execute the script inside the embedded Python interpreter (cross-platform, safer)
    try
    {
        pybind11::gil_scoped_acquire acquire;

        // Add both the weights directory and its parent (repo root) to sys.path
        // This assumes a certain direcotry structure: /SuperPoint/weights/superpoint_v6_from_tf.pth
        std::string weightsDir = UDirectory::getDir(weightsPath);
        std::string repoRoot = UDirectory::getDir(weightsDir);
        auto sys = pybind11::module_::import("sys");
        auto sysPath = sys.attr("path").cast<pybind11::list>();
        sysPath.append(repoRoot);
        sysPath.append(weightsDir);
        
        // Build sys.argv for the script
        pybind11::list argv;
        argv.append(dstScript);
        argv.append("--width");       
        argv.append(std::to_string(width));
        argv.append("--height");      
        argv.append(std::to_string(height));
        argv.append("--weights");     
        argv.append(weightsPath);
        argv.append("--output");      
        argv.append(output);
        argv.append("--threshold");   
        argv.append(std::to_string(threshold));
        argv.append("--nms_radius");  
        argv.append(std::to_string(nms_radius));
        if(cuda) { 
            argv.append("--cuda"); 
        }
        sys.attr("argv") = argv;

        // Run the script as __main__
        auto runpy = pybind11::module_::import("runpy");
        runpy.attr("run_path")(dstScript, pybind11::arg("run_name") = "__main__");
    }
    // pybind11 throws std::exception for RuntimeError
    catch (const std::exception &e)
    {
        UERROR("Python export failed: %s", e.what());
        UFile::erase(dstScript);
        return "";
    }
    
    // clean up the temporary script
	UFile::erase(dstScript);

	return output;
}

SPDetectorRpautrat::SPDetectorRpautrat(std::string superpointWeightsPath, float threshold, bool nms, int minDistance, bool cuda) :
		device_(torch::kCPU),
		superpointWeightsPath_(superpointWeightsPath),
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
    
    if(!UFile::exists(superpointWeightsPath_)) {
        UERROR("Superpoint weights not found: %s", superpointWeightsPath_.c_str());
    }

    // Update device based on cuda availability
    device_ = torch::Device(cuda_ ? torch::kCUDA : torch::kCPU);
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
    float * kp_data = stored_keypoints_cpu.data_ptr<float>();
    
    for(size_t i = 0; i < keypoints.size(); i++) {
        float x = keypoints[i].pt.x;
        float y = keypoints[i].pt.y;
        
        // Find matching descriptor by coordinate
        for(int j = 0; j < num_stored_keypoints; j++) {
            float stored_x = kp_data[j * 2 + 0];
            float stored_y = kp_data[j * 2 + 1];

            float dx = x - stored_x;
            float dy = y - stored_y;
            float distSq = dx * dx + dy * dy;
            
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
    // On first frame, run a trace of the model with the desired parameters and load the model file
    if(!detected_)
    {        
        // effectively disable nms if it is not enabled by setting radius to 0
        int nms_radius = nms_ ? minDistance_ : 0;
        
        std::string modelPath = exportSuperPointTorchScript(
            superpointWeightsPath_,
            img.cols,
            img.rows,
            threshold_,
            nms_radius,
            cuda_
        );
        
        UDEBUG("Initializing SuperPoint Rpautrat detector with model: %s", modelPath.c_str());
        UDEBUG("modelPath=%s thr=%f nms=%d minDistance=%d cuda=%d", modelPath.c_str(), threshold_, nms_?1:0, minDistance_, cuda_?1:0);
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
        model_.eval(); // put in evaluation mode
        model_.to(device_);
    }
    
    // format the input tensor for the model
    torch::NoGradGuard no_grad_guard;
    auto x = torch::from_blob(img.data, {1, 1, img.rows, img.cols}, torch::kByte);
    x = x.to(torch::kFloat) / 255;
    x = x.set_requires_grad(false).to(device_);
    
    auto outputs = model_.forward({x}).toTuple();
    keypoints_tensor_ = outputs->elements()[0].toTensor();  // [N, 2] keypoint coordinates
    auto scores_tensor = outputs->elements()[1].toTensor(); // [N] keypoint scores
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