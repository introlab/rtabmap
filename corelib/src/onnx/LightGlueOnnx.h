/**
 * ONNX Runtime interface for LightGlue (https://github.com/cvg/LightGlue):
 * the model is exported with corelib/src/python/rtabmap_export_lightglue_onnx.py
 */

#ifndef LIGHTGLUEONNX_H
#define LIGHTGLUEONNX_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "rtabmap/core/rtabmap_core_export.h"
#include <memory>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
}

namespace rtabmap
{

// Feature matcher of Vis/CorNNType=10
class RTABMAP_CORE_EXPORT LightGlueOnnx
{
public:
	enum Provider {kCPU=0, kCUDA=1, kTensorRT=2};

	LightGlueOnnx(
			const std::string & modelPath,
			float threshold,
			int provider,
			bool tensorRTFp16,
			const std::string & tensorRTCachePath,
			int tensorRTMaxKeypoints);
	virtual ~LightGlueOnnx();

	bool isValid() const {return session_.get() != 0;}

	// Same convention as PyMatcher::match(): query/train keypoints in pixels of
	// an image of size imageSize (for multi-camera, the cameras' images side by
	// side), float descriptors with one row per keypoint.
	std::vector<cv::DMatch> match(
			const cv::Mat & descriptorsQuery,
			const cv::Mat & descriptorsTrain,
			const std::vector<cv::KeyPoint> & keypointsQuery,
			const std::vector<cv::KeyPoint> & keypointsTrain,
			const cv::Size & imageSize);

	const std::string & modelPath() const {return modelPath_;}
	float threshold() const {return threshold_;}
	int provider() const {return provider_;} // as requested
	int activeProvider() const {return activeProvider_;} // after falling back to what is available
	bool tensorRTFp16() const {return tensorRTFp16_;}
	const std::string & tensorRTCachePath() const {return tensorRTCachePath_;}
	int tensorRTMaxKeypoints() const {return tensorRTMaxKeypoints_;}
	int descriptorDim() const {return descriptorDim_;}

private:
	std::string modelPath_;
	float threshold_;
	int provider_;
	int activeProvider_;
	bool tensorRTFp16_;
	std::string tensorRTCachePath_;
	int tensorRTMaxKeypoints_;
	int descriptorDim_;
	std::unique_ptr<Ort::Env> env_;
	std::unique_ptr<Ort::Session> session_;
};

}

#endif
