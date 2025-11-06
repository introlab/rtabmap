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
    const std::string & superpointModelPath,
    const std::string & outputDir,
    const int & width,
    const int & height,
    const float & threshold,
    const int & nms_radius,
    const bool & cuda)
{
    // Validate output directory is explicitly set and exists
    if(outputDir.empty())
	{
        UERROR("Output directory is not set.");
		return std::string("");
	}
	if(!UDirectory::exists(outputDir))
	{
		UERROR("Output directory does not exist: %s", outputDir.c_str());
		return std::string("");
	}

    // Resolve paths (no dependency on source tree)
	const std::string weightsPath = superpointWeightsPath;
    const std::string modelPath = superpointModelPath;
	const std::string output = std::string(outputDir + "/superpoint_v6_from_tf.pt");
    
	// Sanity checks
    if(!UFile::exists(weightsPath)) {
        UERROR("Weights not found: %s", weightsPath.c_str());
        return "";
    }
    if(!UFile::exists(modelPath)) {
        UERROR("Model not found: %s", modelPath.c_str());
        return "";
    }
    
    // Execute the script inside the embedded Python interpreter
    try
    {
        pybind11::gil_scoped_acquire acquire;

        // set sys.path to the location of the model definition so it can be imported
        std::string modelDir = UDirectory::getDir(modelPath);
        auto sys = pybind11::module_::import("sys");
        auto sysPath = sys.attr("path").cast<pybind11::list>();
        sysPath.append(modelDir);
        
        // Build sys.argv for the script
        pybind11::list argv;
        argv.append("superpoint_to_torchscript.py");  // Use the model filename as script name
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

        // Run the script
        auto main_module = pybind11::module_::import("__main__");
        main_module.attr("__name__") = "__main__";

        pybind11::exec(uHex2Str(SUPERPOINT_TO_TORCHSCRIPT_PY));
        pybind11::exec("main()");
    }
    // pybind11 throws std::exception for RuntimeError
    catch (const std::exception &e)
    {
        UERROR("Python export failed: %s", e.what());
        return "";
    }
    
	return output;
}

SPDetectorRpautrat::SPDetectorRpautrat(std::string superpointWeightsPath, std::string superpointModelPath, std::string outputDir, float threshold, bool nms, int minDistance, bool cuda) :
		device_(torch::kCPU),
		superpointWeightsPath_(superpointWeightsPath),
        superpointModelPath_(superpointModelPath),
        outputDir_(outputDir),
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

    // These should have the same size
    UASSERT(static_cast<size_t>(desc_.size(0)) == keypoints.size());

    // Move to CPU and return descriptors computed in the forward pass
    torch::Tensor desc_cpu = desc_.to(torch::kCPU);
    cv::Mat desc_mat(cv::Size(desc_cpu.size(1), desc_cpu.size(0)), CV_32FC1, desc_cpu.data_ptr<float>());
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
            superpointModelPath_,
            outputDir_,
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