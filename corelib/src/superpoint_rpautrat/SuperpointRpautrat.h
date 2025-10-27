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

struct SuperPointRpautratModel : torch::nn::Module {
    SuperPointRpautratModel();
    
    std::vector<torch::Tensor> forward(torch::Tensor x);

    // Backbone: 4 layers with BatchNorm
    // Layer 1: 1 -> 64 -> 64 (with MaxPool)
    torch::nn::Conv2d conv1a;  // 1 -> 64
    torch::nn::BatchNorm2d bn1a;
    torch::nn::Conv2d conv1b;  // 64 -> 64
    torch::nn::BatchNorm2d bn1b;
    
    // Layer 2: 64 -> 64 -> 64 (with MaxPool)  
    torch::nn::Conv2d conv2a;  // 64 -> 64
    torch::nn::BatchNorm2d bn2a;
    torch::nn::Conv2d conv2b;  // 64 -> 64
    torch::nn::BatchNorm2d bn2b;
    
    // Layer 3: 64 -> 128 -> 128
    torch::nn::Conv2d conv3a;  // 64 -> 128
    torch::nn::BatchNorm2d bn3a;
    torch::nn::Conv2d conv3b;  // 128 -> 128
    torch::nn::BatchNorm2d bn3b;
    
    // Layer 4: 128 -> 128 -> 128 (no MaxPool)
    torch::nn::Conv2d conv4a;  // 128 -> 128
    torch::nn::BatchNorm2d bn4a;
    torch::nn::Conv2d conv4b;  // 128 -> 128
    torch::nn::BatchNorm2d bn4b;
    
    // Detector head: 128 -> 256 -> 65
    torch::nn::Conv2d convDetectorA;   // 128 -> 256
    torch::nn::BatchNorm2d bnDetectorA;
    torch::nn::Conv2d convDetectorB;  // 256 -> 65
    torch::nn::BatchNorm2d bnDetectorB;
    
    // Descriptor head: 128 -> 256 -> 256  
    torch::nn::Conv2d convDescriptorA;  // 128 -> 256
    torch::nn::BatchNorm2d bnDescriptorA;
    torch::nn::Conv2d convDescriptorB;  // 256 -> 256
    torch::nn::BatchNorm2d bnDescriptorB;
};

class SPDetectorRpautrat {
    public:
        SPDetectorRpautrat(const std::string & modelPath, float threshold = 0.005f, bool nms = true, int nmsRadius = 4, bool cuda = false);
        virtual ~SPDetectorRpautrat();
        std::vector<cv::KeyPoint> detect(const cv::Mat &img, const cv::Mat & mask = cv::Mat());
        cv::Mat compute(const std::vector<cv::KeyPoint> &keypoints);
    
        void setThreshold(float threshold) {threshold_ = threshold;}
        void setNMS(bool enabled) {nms_ = enabled;}
        void setMinDistance(int minDistance) {minDistance_ = minDistance;}
    
    private:
        torch::jit::script::Module model_;
        torch::Tensor prob_;
        torch::Tensor desc_;
        torch::Tensor keypoints_tensor_;  // Store original keypoints from model
        
        float threshold_;
        bool nms_;
        int minDistance_;
        bool cuda_;
        
        bool detected_;
    };    
}



#endif // SUPERPOINT_RPAUTRAT_H
