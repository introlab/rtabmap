/**
 * Python interface for python matchers like:
 *  - SuperGlue: https://github.com/magicleap/SuperGluePretrainedNetwork
 *  - OANET https://github.com/zjhthu/OANet
 */

#ifndef PYMATCHER_H
#define PYMATCHER_H

#include <rtabmap/core/Features2d.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "rtabmap/core/PythonInterface.h"
#include <Python.h>

namespace rtabmap
{

class PyDetector : public Feature2D
{
public:
	PyDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~PyDetector();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeaturePyDetector;}

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
