/**
 * Python interface for SuperGlue: https://github.com/magicleap/SuperGluePretrainedNetwork
 */

#ifndef SUPERGLUE_H
#define SUPERGLUE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>

#include <Python.h>

namespace rtabmap
{

class SuperGlue
{
public:
  SuperGlue(const std::string & supergluePythonPath, float matchThreshold = 0.2f, int iterations = 20, bool cuda = false);
  virtual ~SuperGlue();

  const std::string & path() const {return path_;}
  float matchThreshold() const {return matchThreshold_;}
  int iterations() const {return iterations_;}
  bool cuda() const {return cuda_;}

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
};

}

#endif
