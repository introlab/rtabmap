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
		  const cv::Mat & depth,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0);

	// Metric constructor + 2d depth
	SensorData(const cv::Mat & image,
		  const cv::Mat & depth,
		  const cv::Mat & depth2d,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id = 0);

	virtual ~SensorData() {}

	bool isValid() const {return !_image.empty();}

	// use isValid() instead
	RTABMAP_DEPRECATED(bool empty() const, "Use !isValid() instead.");

	const cv::Mat & image() const {return _image;}
	int id() const {return _id;};

	bool isMetric() const {return !_depth.empty() || _fx != 0.0f || _fy != 0.0f || !_pose.isNull();}
	void setPose(const Transform & pose) {_pose = pose;}
	const cv::Mat & depth() const {return _depth;}
	const cv::Mat & depth2d() const {return _depth2d;}
	float depthFx() const {return _fx;}
	float depthFy() const {return _fy;}
	float depthCx() const {return _cx;}
	float depthCy() const {return _cy;}
	const Transform & pose() const {return _pose;}
	const Transform & localTransform() const {return _localTransform;}

private:
	cv::Mat _image;
	int _id;

	// Metric stuff
	cv::Mat _depth;
	cv::Mat _depth2d;
	float _fx;
	float _fy;
	float _cx;
	float _cy;
	Transform _pose;
	Transform _localTransform;
};

}


#endif /* SENSORDATA_H_ */
