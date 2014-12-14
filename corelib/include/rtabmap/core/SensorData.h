/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/Transform.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
class RTABMAP_EXP SensorData
{
public:
	SensorData(); // empty constructor
	SensorData(const cv::Mat & image, int id = 0);

	// Metric constructor
	SensorData(const cv::Mat & image,
		  const cv::Mat & depthOrRightImage,
		  float fx,
		  float fyOrBaseline,
		  float cx,
		  float cy,
		  const Transform & localTransform,
		  const Transform & pose,
		  float poseVariance,
		  int id = 0);

	// Metric constructor + 2d laser scan
	SensorData(const cv::Mat & laserScan,
		  const cv::Mat & image,
		  const cv::Mat & depthOrRightImage,
		  float fx,
		  float fyOrBaseline,
		  float cx,
		  float cy,
		  const Transform & localTransform,
		  const Transform & pose,
		  float poseVariance,
		  int id = 0);

	virtual ~SensorData() {}

	bool isValid() const {return !_image.empty();}

	// use isValid() instead
	RTABMAP_DEPRECATED(bool empty() const, "Use !isValid() instead.");

	const cv::Mat & image() const {return _image;}
	int id() const {return _id;}
	void setId(int id) {_id = id;}

	bool isMetric() const {return !_depthOrRightImage.empty() || _fx != 0.0f || _fyOrBaseline != 0.0f || !_pose.isNull();}
	void setPose(const Transform & pose, float variance) {_pose = pose; _poseVariance=variance;}
	cv::Mat depth() const {return (_depthOrRightImage.type()==CV_32FC1 || _depthOrRightImage.type()==CV_16UC1)?_depthOrRightImage:cv::Mat();}
	cv::Mat rightImage() const {return _depthOrRightImage.type()==CV_8UC1?_depthOrRightImage:cv::Mat();}
	const cv::Mat & depthOrRightImage() const {return _depthOrRightImage;}
	const cv::Mat & laserScan() const {return _laserScan;}
	float fx() const {return _fx;}
	float fy() const {return (_depthOrRightImage.type()==CV_8UC1)?0:_fyOrBaseline;}
	float cx() const {return _cx;}
	float cy() const {return _cy;}
	float baseline() const {return _depthOrRightImage.type()==CV_8UC1?_fyOrBaseline:0;}
	float fyOrBaseline() const {return _fyOrBaseline;}
	const Transform & pose() const {return _pose;}
	const Transform & localTransform() const {return _localTransform;}
	float poseVariance() const {return _poseVariance;}

	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors)
	{
		_keypoints = keypoints;
		_descriptors = descriptors;
	}
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	const cv::Mat & descriptors() const {return _descriptors;}

private:
	cv::Mat _image;
	int _id;

	// Metric stuff
	cv::Mat _depthOrRightImage;
	cv::Mat _laserScan;
	float _fx;
	float _fyOrBaseline;
	float _cx;
	float _cy;
	Transform _pose;
	Transform _localTransform;
	float _poseVariance;

	// features
	std::vector<cv::KeyPoint> _keypoints;
	cv::Mat _descriptors;
};

}


#endif /* SENSORDATA_H_ */
