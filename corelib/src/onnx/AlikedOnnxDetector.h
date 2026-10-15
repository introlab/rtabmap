/**
 * ONNX Runtime interface for ALIKED (https://github.com/Shiaoming/ALIKED):
 * the model is exported with corelib/src/python/rtabmap_export_aliked_onnx.py
 */

#ifndef ALIKEDONNXDETECTOR_H
#define ALIKEDONNXDETECTOR_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "rtabmap/core/rtabmap_core_export.h"
#include <memory>
#include <string>
#include <vector>

namespace rtabmap
{

class OnnxSession;

// Keypoints and descriptors of Kp/DetectorStrategy=17 (provider: OnnxSession::Provider)
class RTABMAP_CORE_EXPORT AlikedOnnxDetector
{
public:
	AlikedOnnxDetector(
			const std::string & modelPath,
			float threshold,
			int provider,
			bool tensorRTFp16,
			const std::string & tensorRTCachePath);
	virtual ~AlikedOnnxDetector();

	bool isValid() const {return session_.get() != 0;}

	// Detects keypoints (response = ALIKED score) and computes their descriptors
	// (one row each, CV_32F). The model has a fixed input size: an image as large
	// as several model inputs side by side (a multi-camera image) is processed one
	// camera at a time. Keypoints outside the mask (if not empty) are dropped.
	void detect(const cv::Mat & image, const cv::Mat & mask,
			std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors);

	const std::string & modelPath() const {return modelPath_;}
	float threshold() const {return threshold_;}
	int provider() const {return provider_;}
	bool tensorRTFp16() const {return tensorRTFp16_;}
	const std::string & tensorRTCachePath() const {return tensorRTCachePath_;}
	int descriptorDim() const {return descriptorDim_;}
	cv::Size inputSize() const {return inputSize_;}

private:
	std::string modelPath_;
	float threshold_;
	int provider_;
	bool tensorRTFp16_;
	std::string tensorRTCachePath_;
	cv::Size inputSize_;
	int candidates_;
	int descriptorDim_;
	std::shared_ptr<OnnxSession> session_;
};

}

#endif
