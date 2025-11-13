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
            bool cuda = false,
            int maxFeatures = 1000,
            bool ssc = false
        );
        virtual ~SPDetectorRpautrat();
        std::vector<cv::KeyPoint> detect(const cv::Mat &img, const cv::Mat & mask = cv::Mat());
        cv::Mat compute(const std::vector<cv::KeyPoint> &keypoints);
        
        // Setters for post-processing parameters that don't require model reinitialization
        void setMaxFeatures(int maxFeatures) { maxFeatures_ = maxFeatures; }
        void setSSC(bool ssc) { ssc_ = ssc; }
    
    private:
        torch::jit::script::Module model_;
        torch::Device device_;
        cv::Mat desc_;
        
        std::string superpointWeightsPath_;
        std::string superpointModelPath_;
        std::string outputDir_;
        float threshold_;
        bool nms_;
        int minDistance_;
        bool cuda_;
        int maxFeatures_;
        bool ssc_;
        
        bool detected_;
    };    
}



#endif // SUPERPOINT_RPAUTRAT_H
