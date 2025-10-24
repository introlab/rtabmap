/**
 * SuperPoint implementation based on the PyTorch version by Rémi Pautrat, Paul-Edouard Sarlin
 * Adapted for RTAB-Map integration
 */

#ifndef SUPERPOINT_RPAUTRAT_H
#define SUPERPOINT_RPAUTRAT_H

#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace rtabmap
{

// Configuration structure for SuperPoint (only configurable parameters)
struct SuperPointConfig {
    float detection_threshold = 0.005f;  // Detection threshold
    int max_num_keypoints = -1;           // -1 means no limit
    int nms_radius = 4;                  // NMS radius
    int remove_borders = 4;               // Remove keypoints near borders
    bool cuda = false;                    // Use CUDA if available
    
    SuperPointConfig() = default;
};

struct SuperPoint : torch::nn::Module {
    SuperPoint();
    
    std::vector<torch::Tensor> forward(torch::Tensor x);

    // Backbone: 3 layers matching Python implementation
    // Layer 1: 1 -> 64 -> 64 (with MaxPool)
    torch::nn::Conv2d conv1a;  // 1 -> 64
    torch::nn::Conv2d conv1b;  // 64 -> 64
    
    // Layer 2: 64 -> 128 -> 128 (with MaxPool)  
    torch::nn::Conv2d conv2a;  // 64 -> 128
    torch::nn::Conv2d conv2b;  // 128 -> 128
    
    // Layer 3: 128 -> 256 -> 256 (no MaxPool)
    torch::nn::Conv2d conv3a;  // 128 -> 256
    torch::nn::Conv2d conv3b;  // 256 -> 256
    
    // Detector head: 256 -> 256 -> 65
    torch::nn::Conv2d convDetectorA;   // 256 -> 256
    torch::nn::Conv2d convDetectorB;  // 256 -> 65
    
    // Descriptor head: 256 -> 256 -> 256  
    torch::nn::Conv2d convDescriptorA;  // 256 -> 256
    torch::nn::Conv2d convDescriptorB;  // 256 -> 256
}

class SPDetector {
    public:
        SPDetector(const std::string & modelPath, const SuperPointConfig& config = SuperPointConfig());
        virtual ~SPDetector();
        std::vector<cv::KeyPoint> detect(const cv::Mat &img, const cv::Mat & mask = cv::Mat());
        cv::Mat compute(const std::vector<cv::KeyPoint> &keypoints);
    
        void setThreshold(float threshold) {config_.detection_threshold = threshold;}
        void setMaxKeypoints(int max_kpts) {config_.max_num_keypoints = max_kpts;}
        void setNMSRadius(int radius) {config_.nms_radius = radius;}
        void setRemoveBorders(int borders) {config_.remove_borders = borders;}
    
    private:
        std::shared_ptr<SuperPoint> model_;
        SuperPointConfig config_;
        torch::Tensor prob_;
        torch::Tensor desc_;
        bool detected_;
    };    
}



#endif // SUPERPOINT_RPAUTRAT_H
