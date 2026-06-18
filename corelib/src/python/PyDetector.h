/**
 * Python interface for python local feature detectors like:
 *  - SuperPoint: https://github.com/magicleap/SuperPointPretrainedNetwork
 */

#ifndef PYDETECTOR_H
#define PYDETECTOR_H

#include <rtabmap/core/Features2d.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "rtabmap/core/PythonInterface.h"
#include "rtabmap/core/rtabmap_core_export.h"
#include <Python.h>

namespace rtabmap
{

class RTABMAP_CORE_EXPORT PyDetector : public Feature2D
{
public:
	PyDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~PyDetector();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeaturePyDetector;}
	// PyDetector hands off CUDA usage to the python script -- the C++
	// wrapper can't introspect it, so we report capability is "possible"
	// whenever the python interpreter is built in. The actual hardware
	// presence + the script's own decision is out of our hands.
	virtual bool isGpuAvailable() const override {return true;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat());
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
  PyObject * pModule_;
  PyObject * pFunc_;
  std::string path_;
  bool cuda_;
  cv::Mat descriptors_;
};

}

#endif
