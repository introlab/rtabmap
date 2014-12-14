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


#include "rtabmap/core/SensorData.h"
#include "rtabmap/utilite/ULogger.h"

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
SensorData::SensorData() :
	_image(cv::Mat()),
	_id(0),
	_fx(0.0f),
	_fyOrBaseline(0.0f),
	_cx(0.0f),
	_cy(0.0f),
	_localTransform(Transform::getIdentity()),
	_poseVariance(1.0f)
{
}

SensorData::SensorData(const cv::Mat & image,
	  int id) :
	_image(image),
	_id(id),
	_fx(0.0f),
	_fyOrBaseline(0.0f),
	_cx(0.0f),
	_cy(0.0f),
	_localTransform(Transform::getIdentity()),
	_poseVariance(1.0f)
{
	UASSERT(image.type() == CV_8UC1 || // Mono
			image.type() == CV_8UC3);  // RGB
}

	// Metric constructor
SensorData::SensorData(const cv::Mat & image,
		  const cv::Mat & depthOrRightImage,
		  float fx,
		  float fyOrBaseline,
		  float cx,
		  float cy,
		  const Transform & localTransform,
		  const Transform & pose,
		  float poseVariance,
		  int id) :
	_image(image),
	_id(id),
	_depthOrRightImage(depthOrRightImage),
	_fx(fx),
	_fyOrBaseline(fyOrBaseline),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform),
	_poseVariance(poseVariance)
{
	UASSERT(image.type() == CV_8UC1 || // Mono
			image.type() == CV_8UC3);  // RGB
	UASSERT(depthOrRightImage.type() == CV_32FC1 || // Depth in meter
			depthOrRightImage.type() == CV_16UC1 || // Depth in millimetre
			depthOrRightImage.type() == CV_8U);     // Right stereo image
	UASSERT(!depthOrRightImage.empty() && _fx>0.0f && _fyOrBaseline>0.0f && _cx>=0.0f && _cy>=0.0f);
	UASSERT(!_localTransform.isNull());
}

	// Metric constructor + 2d depth
SensorData::SensorData(const cv::Mat & laserScan,
		  const cv::Mat & image,
		  const cv::Mat & depthOrRightImage,
		  float fx,
		  float fyOrBaseline,
		  float cx,
		  float cy,
		  const Transform & localTransform,
		  const Transform & pose,
		  float poseVariance,
		  int id) :
	_image(image),
	_id(id),
	_depthOrRightImage(depthOrRightImage),
	_laserScan(laserScan),
	_fx(fx),
	_fyOrBaseline(fyOrBaseline),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform),
	_poseVariance(poseVariance)
{
	UASSERT(_laserScan.empty() || _laserScan.type() == CV_32FC2);
	UASSERT(image.type() == CV_8UC1 || // Mono
			image.type() == CV_8UC3);  // RGB
	UASSERT(depthOrRightImage.type() == CV_32FC1 || // Depth in meter
			depthOrRightImage.type() == CV_16UC1 || // Depth in millimetre
			depthOrRightImage.type() == CV_8U);     // Right stereo image
	UASSERT(!depthOrRightImage.empty() && _fx>0.0f && _fyOrBaseline>0.0f && _cx>=0.0f && _cy>=0.0f);
	UASSERT(!_localTransform.isNull());
}

bool SensorData::empty() const
{
	return _image.empty();
}

} // namespace rtabmap

