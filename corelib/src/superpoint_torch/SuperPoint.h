/**
 * Original code from https://github.com/KinglittleQ/SuperPoint_SLAM
 */

#ifndef SUPERPOINT_H
#define SUPERPOINT_H


#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include <vector>

#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif


namespace rtabmap
{

struct SuperPoint : torch::nn::Module {
  SuperPoint();

  std::vector<torch::Tensor> forward(torch::Tensor x);


  torch::nn::Conv2d conv1a;
  torch::nn::Conv2d conv1b;

  torch::nn::Conv2d conv2a;
  torch::nn::Conv2d conv2b;

  torch::nn::Conv2d conv3a;
  torch::nn::Conv2d conv3b;

  torch::nn::Conv2d conv4a;
  torch::nn::Conv2d conv4b;

  torch::nn::Conv2d convPa;
  torch::nn::Conv2d convPb;

  // descriptor
  torch::nn::Conv2d convDa;
  torch::nn::Conv2d convDb;

};

class SPDetector {
public:
    SPDetector(const std::string & modelPath, float threshold = 0.2f, bool nms = true, int minDistance = 4, bool cuda = false);
    virtual ~SPDetector();
    std::vector<cv::KeyPoint> detect(const cv::Mat &img, const cv::Mat & mask = cv::Mat());
    cv::Mat compute(const std::vector<cv::KeyPoint> &keypoints);

    void setThreshold(float threshold) {threshold_ = threshold;}
    void SetNMS(bool enabled) {nms_ = enabled;}
    void setMinDistance(float minDistance) {minDistance_ = minDistance;}

private:
    std::shared_ptr<SuperPoint> model_;
    torch::Tensor prob_;
    torch::Tensor desc_;

    float threshold_;
    bool nms_;
    int minDistance_;
    bool cuda_;

    bool detected_;
};

}

#endif
