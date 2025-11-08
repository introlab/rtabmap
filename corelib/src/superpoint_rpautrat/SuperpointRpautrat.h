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

class SPDetectorRpautrat {
    public:
        SPDetectorRpautrat(
            std::string superpointWeightsPath, 
            std::string superpointModelPath,
            std::string outputDir, 
            float threshold = 0.005f, 
            bool nms = true, 
            int nmsRadius = 4, 
            bool cuda = false
        );
        virtual ~SPDetectorRpautrat();
        std::vector<cv::KeyPoint> detect(const cv::Mat &img, const cv::Mat & mask = cv::Mat());
        cv::Mat compute(const std::vector<cv::KeyPoint> &keypoints);
    
    private:
        torch::jit::script::Module model_;
        torch::Device device_;
        torch::Tensor desc_;
        torch::Tensor keypoints_tensor_;
        
        std::string superpointWeightsPath_;
        std::string superpointModelPath_;
        std::string outputDir_;
        float threshold_;
        bool nms_;
        int minDistance_;
        bool cuda_;
        
        bool detected_;
    };    
}



#endif // SUPERPOINT_RPAUTRAT_H
