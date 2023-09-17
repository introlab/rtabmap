/**
 * Python interface for python matchers like:
 *  - SuperGlue: https://github.com/magicleap/SuperGluePretrainedNetwork
 *  - OANET https://github.com/zjhthu/OANet
 */

#ifndef PYMATCHER_H
#define PYMATCHER_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include "rtabmap/core/PythonInterface.h"
#include <vector>
#include <Python.h>

namespace rtabmap
{

class PyMatcher
{
public:
  PyMatcher(const std::string & pythonMatcherPath,
		  float matchThreshold = 0.2f,
		  int iterations = 20,
		  bool cuda = true,
		  const std::string & model = "indoor");
  virtual ~PyMatcher();

  const std::string & path() const {return path_;}
  float matchThreshold() const {return matchThreshold_;}
  int iterations() const {return iterations_;}
  bool cuda() const {return cuda_;}
  const std::string & model() const {return model_;}

  std::vector<cv::DMatch> match(
		  const cv::Mat & descriptorsQuery,
		  const cv::Mat & descriptorsTrain,
		  const std::vector<cv::KeyPoint> & keypointsQuery,
		  const std::vector<cv::KeyPoint> & keypointsTrain,
		  const cv::Size & imageSize);

private:
  PyObject * pModule_;
  PyObject * pFunc_;
  std::string path_;
  float matchThreshold_;
  int iterations_;
  bool cuda_;
  std::string model_;
};

}

#endif
